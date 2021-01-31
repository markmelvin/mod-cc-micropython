import uasyncio as asyncio
# NOTE: We use pyb.Timer and not machine.Timer so we can specify one of the high-speed timers
from pyb import LED, Timer
from machine import Pin, UART
import time

from primitives.switch import Switch
from py_cc.cc_classes import CCVersion
from py_cc.cc_protocol import CCActuator, CCAssignment, set_log_level, LOGLEVEL_DEBUG, LOGLEVEL_INFO
from py_cc.cc_constants import CC_BAUD_RATE_FALLBACK, CC_ACTUATOR_MOMENTARY, \
                               CC_MODE_MOMENTARY, CC_MODE_TOGGLE, CC_MODE_TRIGGER, CC_MODE_OPTIONS, CC_MODE_TAP_TEMPO, \
                               CC_EV_UPDATE, CC_EV_ASSIGNMENT, CC_EV_UNASSIGNMENT, CC_EV_MASTER_RESETED, CC_EV_SET_VALUE

from py_cc.control_chain import ControlChainSlaveDevice, convert_to_ms, convert_from_ms, RX_BUFFER_SIZE

DEVICE_NAME = "Audiofab Footswitch"
DEVICE_URL = "https://github.com/markmelvin/mod-cc-micropython"

FIRMWARE_MAJOR       = 0
FIRMWARE_MINOR       = 4
FIRMWARE_MICRO       = 0

BAUD_RATE       = CC_BAUD_RATE_FALLBACK
TIMER_NUMBER    = 2         # Timer2 is one of the high-speed timers
UART_NUMBER     = 6         # UART6 is on PC6/7
SERIAL_TX_EN    = 'PA10'    # Serial "is transmitting" pin

# Footswitch inputs
FOOTSWITCH_DIOS = ['PB3', 'PB5', 'PB4', 'PB10']
# Corresponding indicator LED outputs
INDICATOR_DIOS = ['PA8', 'PA9', 'PA7', 'PA6']
assert len(FOOTSWITCH_DIOS) == len(INDICATOR_DIOS)

FOOTSWITCH_ACTUATOR_MIN = 0.0
FOOTSWITCH_ACTUATOR_MAX = 1.0

TAP_TEMPO_ON_MS = 50
TAP_TEMPO_DEFAULT_MIN_INTERVAL_MS = 100
TAP_TEMPO_DEFAULT_MAX_INTERVAL_MS = 3000


# Asyncio debugging heartbeat. Flash the onboard LED regularly.
def heartbeat():
    pin = Pin('PA5',  Pin.OUT)
    while True:
        pin.on()
        await asyncio.sleep_ms(250)
        pin.off()
        await asyncio.sleep_ms(500)

asyncio.create_task(heartbeat())

class MomentaryButton:
    def __init__(self, dio):
        self.value = False
        self.last_pressed = time.ticks_ms()
        self.interval_ms = 0
        self.min_interval_ms = TAP_TEMPO_DEFAULT_MIN_INTERVAL_MS
        self.max_interval_ms = TAP_TEMPO_DEFAULT_MAX_INTERVAL_MS
        # Use an underlying asyncio Switch to automatically debounce the input
        pin = Pin(dio, Pin.IN, Pin.PULL_UP)
        self.switch = Switch(pin)
        self.switch.close_func(self.set_value, (True,))
        self.switch.open_func(self.set_value, (False,))

    def set_value(self, value):
        if value and not self.is_pressed:
            # Rising edge
            now = time.ticks_ms()

            # Update the running average interval between button presses
            delta = time.ticks_diff(now, self.last_pressed)
            if delta >= self.min_interval_ms and delta < self.max_interval_ms:
                if self.interval_ms > 0:
                    self.interval_ms = (self.interval_ms + delta) / 2.0
                else:
                    self.interval_ms = delta
            elif delta >= self.max_interval_ms:
                self.interval_ms = 0

            self.last_pressed = now

        self.value = True if value else False

    def set_interval_limits(self, min_interval_ms, max_interval_ms):
        self.min_interval_ms = min_interval_ms
        self.max_interval_ms = max_interval_ms

    @property
    def is_pressed(self,):
        return self.value == True

    @property
    def tap_tempo_ms(self,):
        if self.interval_ms < self.min_interval_ms or self.interval_ms > self.max_interval_ms:
            return self.max_interval_ms
        return self.interval_ms


class Indicator:
    def __init__(self, dio, interval_ms=1000):
        self.pin = Pin(dio,  Pin.OUT)
        self.interval_ms = interval_ms
        self.value = False
        self.blink_on_time_ms = 0
        asyncio.create_task(self.run())

    async def run(self):
        while True:
            if self.blink_on_time_ms <= 0:
                await asyncio.sleep_ms(100)
            else:
                if self.is_on:
                    # Turn off for the off duration
                    self.set_value(False)
                    await asyncio.sleep_ms(int(max(self.interval_ms - self.blink_on_time_ms, 0)))
                else:
                    # Turn on for the blink duration
                    self.set_value(True)
                    await asyncio.sleep_ms(self.blink_on_time_ms)

    def set_value(self, value):
        if value != self.value:
            self.last_change = time.ticks_ms()
            self.pin.on() if value else self.pin.off()
            self.value = True if value else False

    def on(self,):
        self.set_value(True)

    def off(self,):
        self.set_value(False)

    def set_blink_rate(self, on_time_ms, interval_ms=1000, value=True):
        self.blink_on_time_ms = on_time_ms
        self.set_interval(interval_ms)
        self.set_value(value)

    def set_interval(self, interval_ms):
        self.interval_ms = interval_ms

    @property
    def is_on(self,):
        return self.value == True

    @property
    def is_off(self,):
        return self.value == False

    @property
    def is_blinking(self,):
        return self.blink_on_time_ms > 0


class FootswitchActuator:
    '''
    A footswitch-style actuator that has a momentary pushbutton and associated LED indicator.
    '''
    def __init__(self, id, name, button_dio, led_dio):
        self.id = id
        self.cc_actuator = CCActuator(id, name, CC_ACTUATOR_MOMENTARY,
                                      FOOTSWITCH_ACTUATOR_MIN, FOOTSWITCH_ACTUATOR_MAX,
                                      CC_MODE_TOGGLE | CC_MODE_TRIGGER | CC_MODE_OPTIONS | CC_MODE_TAP_TEMPO | CC_MODE_MOMENTARY,
                                      max_assignments=1)
        self._button = MomentaryButton(button_dio)
        self._indicator = Indicator(led_dio)
        # Local tap tempo. Mod must set this value initially to avoid feedback loops.
        self.tap_tempo_ms = None

    async def update(self, ticks_ms):
        # Update the underlying actuator state with current button state
        if self.actuator.assignment is not None:
            # Is this assigned to TAP_TEMPO?
            if self.tap_tempo_ms and (self.actuator.assignment.mode & CC_MODE_TAP_TEMPO):
                self.actuator.value = convert_from_ms(self.button.tap_tempo_ms, self.actuator.assignment.unit)
                return

        # Otherwise set value based on simple switch limits
        self.actuator.value = FOOTSWITCH_ACTUATOR_MAX if self.button.is_pressed else FOOTSWITCH_ACTUATOR_MIN

    def update_assignment(self, assignment):
        if assignment is None:
            self.tap_tempo_ms = None
            self.indicator.set_blink_rate(0, value=False)
            self.button.set_interval_limits(TAP_TEMPO_DEFAULT_MIN_INTERVAL_MS, TAP_TEMPO_DEFAULT_MAX_INTERVAL_MS)
            return

        if assignment.mode & (CC_MODE_TRIGGER | CC_MODE_OPTIONS):
            self.indicator.set_value(True)

        elif assignment.mode & CC_MODE_TOGGLE:
            self.indicator.set_value(assignment.value)

        elif assignment.mode & CC_MODE_TAP_TEMPO:
            # Update interval limits on tap tempo
            # NOTE: We want to calculate the minimum interval with the maximum tempo (and vice versa)
            self.button.set_interval_limits(
                convert_to_ms(assignment.max, assignment.unit),
                convert_to_ms(assignment.min, assignment.unit),
            )
            interval_ms = convert_to_ms(assignment.value, assignment.unit)
            self.tap_tempo_ms = interval_ms
            self.button.interval_ms = interval_ms
            self.indicator.set_blink_rate(TAP_TEMPO_ON_MS, interval_ms=interval_ms)

        elif assignment.mode & CC_MODE_MOMENTARY:
            self.indicator.set_value(assignment.value)

    @property
    def actuator(self,):
        return self.cc_actuator

    @property
    def button(self,):
        return self._button

    @property
    def indicator(self,):
        return self._indicator


class Footswitch:
    def __init__(self,):
        # Pulse LED on evaluation board on startup
        self.actuators = []
        self.connected = False

        # Configure actuators
        for i in range(len(FOOTSWITCH_DIOS)):
            self.actuators.append(FootswitchActuator(i, "Foot #%d" % (i+1), FOOTSWITCH_DIOS[i], INDICATOR_DIOS[i]))

        # Create the underlying Control Chain device slave
        timer = Timer(TIMER_NUMBER)
        uart = UART(UART_NUMBER, BAUD_RATE, bits=8, parity=None, stop=1, rxbuf=RX_BUFFER_SIZE)
        tx_en = Pin(SERIAL_TX_EN, Pin.OUT)

        # The control chain device will operate in a background thread and call our
        # event callback whever something we need to know about occurs
        self.device = ControlChainSlaveDevice(DEVICE_NAME, DEVICE_URL, CCVersion(FIRMWARE_MAJOR, FIRMWARE_MINOR, FIRMWARE_MICRO),
                                              [w.actuator for w in self.actuators], timer, uart, tx_en, events_callback=self.events_cb)

    def run(self,):
        '''The main processing loop that runs forever'''
        print("Starting footswitch firmware!")

        while True:
            now = time.ticks_ms()
            if not self.connected:
                if self.device.is_connected:
                    self.connected = True
                    self.force_led_pattern([0]*len(self.actuators))
                else:
                    # Indicate to user we're waiting to handshake
                    for actuator in self.actuators:
                        actuator.indicator.on()
                        await asyncio.sleep_ms(100)
                        actuator.indicator.off()

            else:
                # Update the actuator state
                for i, actuator in enumerate(self.actuators):
                    await actuator.update(now)

                await asyncio.sleep_ms(0)

    def reset(self,):
        # Turn off all LEDs
        self.force_led_pattern([0]*len(self.actuators))
        # TODO: What else do we do here?

    def force_led_pattern(self, pattern):
        for i in range(min(len(pattern), len(self.actuators))):
            self.actuators[i].indicator.set_blink_rate(0)
            self.actuators[i].indicator.set_value(True if pattern[i] else False)

    def events_cb(self, event):
        '''
        Event callback from the Control Chain driver
        '''
        print("Got event: ", event.__dict__)
        if event.id == CC_EV_UPDATE or event.id == CC_EV_ASSIGNMENT:
            if isinstance(event.data, CCAssignment):
                if event.data.id in self.device.assignments:
                    print("Assignment: ", event.data.__dict__)
                    self.actuators[event.data.actuator_id].update_assignment(event.data)

        elif event.id == CC_EV_UNASSIGNMENT:
            print("Unassignment: ", event.data)
            self.actuators[event.data].update_assignment(None)

        elif event.id == CC_EV_SET_VALUE:
            if isinstance(event.data, CCMsgSetValue):
                print("Set value: ", event.data.__dict__)
                self.actuators[event.data.actuator_id].update_assignment(self.device.assignments.get(event.data.assignment_id))

        elif event.id == CC_EV_MASTER_RESETED:
            self.reset()