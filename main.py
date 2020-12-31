# micropython-based footswitch using the moddevices Arduino shield
# attached to an STM32 Nucleo development board (NUCLEO-F-446RE)
import machine
import pyb
import time
import sys
import math
import gc

sys.path.append('./')

from py_cc.cc_protocol import CCActuator, CCAssignment, set_log_level, LOGLEVEL_DEBUG, LOGLEVEL_INFO
from py_cc.cc_constants import CC_BAUD_RATE_FALLBACK, CC_ACTUATOR_MOMENTARY, \
                               CC_MODE_MOMENTARY, CC_MODE_TOGGLE, CC_MODE_TRIGGER, CC_MODE_OPTIONS, CC_MODE_TAP_TEMPO, \
                               CC_EV_UPDATE, CC_EV_ASSIGNMENT, CC_EV_UNASSIGNMENT, CC_EV_MASTER_RESETED, CC_EV_SET_VALUE

from utils.thread import Thread
from utils.control_chain import ControlChainSlaveDevice

DEVICE_NAME = "Audiofab Footswitch"
DEVICE_URL = "https://github.com/markmelvin/mod-cc-micropython"

BAUD_RATE       = CC_BAUD_RATE_FALLBACK
LED_DIO         = 'PA5'
TIMER_NUMBER    = 2         # Timer2 is one of the high-speed timers
UART_NUMBER     = 6         # UART6 is on PC6/7
SERIAL_TX_EN    = 'PA10'    # Serial "is transmitting" pin

BUTTON_DEBOUNCE_CYCLES = 10
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


def convert_to_ms(value, source_units):
    units_lower = source_units.lower()
    if units_lower == "bpm":
        return 60000.0 / max(value, 0.1)
    elif units_lower == "hz":
        return 1000.0 / max(value, 0.1)
    elif units_lower == "s":
        return value * 1000.0
    elif units_lower == "ms":
        return value
    return 0.0

def convert_from_ms(value, desired_units):
    units_lower = desired_units.lower()
    if units_lower == "bpm":
        return 60000.0 * value
    elif units_lower == "hz":
        return 1000.0 * value
    elif units_lower == "s":
        return value / 1000.0
    elif units_lower == "ms":
        return value
    return 0.0


class MomentaryButton:
    def __init__(self, dio):
        self.pin = machine.Pin(dio, machine.Pin.IN, machine.Pin.PULL_UP)
        self.value = False
        self.last_pressed = time.ticks_ms()
        self.interval_ms = 0
        self.min_interval_ms = TAP_TEMPO_DEFAULT_MIN_INTERVAL_MS
        self.max_interval_ms = TAP_TEMPO_DEFAULT_MAX_INTERVAL_MS

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
    def pin_value(self,):
        return self.pin.value()

    @property
    def is_pressed(self,):
        return self.value == True

    @property
    def tap_tempo_ms(self,):
        if self.interval_ms < self.min_interval_ms or self.interval_ms > self.max_interval_ms:
            return self.max_interval_ms
        return self.interval_ms


class Indicator:
    def __init__(self, dio, on_time_ms=-1, off_time_ms=-1):
        self.pin = machine.Pin(dio,  machine.Pin.OUT)
        self.value = False
        self.blink = False
        self.last_change = time.ticks_ms()
        self.on_time_ms = on_time_ms
        self.off_time_ms = off_time_ms

    def set_value(self, value, reset_blink=True):
        if reset_blink:
            self.disable_blink()

        if value != self.value:
            self.last_change = time.ticks_ms()
            self.pin.on() if value else self.pin.off()
            self.value = True if value else False

    def set_blink(self, value):
        if self.is_off:
            # Indicator is not supposed to be on, so do nothing
            return

        if value != self.blink:
            # When self.blink is true, the indicator is forced off
            self.blink = True if value else False
            self.last_change = time.ticks_ms()
            self.pin.on() if not self.blink else self.pin.off()

    def on(self,):
        self.set_value(True)

    def off(self,):
        self.set_value(False)

    def set_blink_rate(self, on_time_ms, off_time_ms, value=True):
        self.on_time_ms = on_time_ms
        self.off_time_ms = off_time_ms
        self.set_value(value, reset_blink=False)

    def disable_blink(self,):
        self.on_time_ms = -1
        self.off_time_ms = -1
        self.blink = False

    @property
    def is_on(self,):
        return self.value == True

    @property
    def is_off(self,):
        return self.value == False

    @property
    def is_blinking(self,):
        return self.blink == True


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

    def update(self, ticks_ms):
        # Update the underlying actuator state with current button state
        if self.actuator.assignment is not None:
            # Is this assigned to TAP_TEMPO?
            if self.actuator.assignment.mode & CC_MODE_TAP_TEMPO:
                self.actuator.value = convert_from_ms(self.button.tap_tempo_ms, self.actuator.assignment.units)
                return

        # Otherwise set value based on simple switch limits
        self.actuator.value = FOOTSWITCH_ACTUATOR_MAX if self.button.is_pressed else FOOTSWITCH_ACTUATOR_MIN

    def update_assignment(self, assignment):
        if assignment is None:
            self.indicator.set_value(False)
            self.button.set_interval_limits(TAP_TEMPO_DEFAULT_MIN_INTERVAL_MS, TAP_TEMPO_DEFAULT_MAX_INTERVAL_MS)
            return

        if assignment.mode & (CC_MODE_TRIGGER | CC_MODE_OPTIONS):
            self.indicator.set_value(True)

        elif assignment.mode & CC_MODE_TOGGLE:
            self.indicator.set_value(assignment.value)

        elif assignment.mode & CC_MODE_TAP_TEMPO:
            # Update interval limits on tap tempo
            self.button.set_interval_limits(
                convert_to_ms(assignment.min, assignment.unit),
                convert_to_ms(assignment.max, assignment.unit),
            )
            interval_ms = convert_to_ms(assignment.value, assignment.unit)
            self.indicator.set_blink_rate(TAP_TEMPO_ON_MS, interval_ms - TAP_TEMPO_ON_MS)

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


class DIOMonitor:
    BUTTON_DEBOUNCE_TIME_MS = 50

    def __init__(self, footswitches):
        self.footswitches = footswitches
        self.last_tick = time.ticks_ms()
        # Store a list of [last_pin_value, last_transition_time] for each button
        self._debounces = [[False, self.last_tick] for w in self.footswitches]
        self._monitor = Thread(target=self.monitor)
        self._monitor.start()

    def monitor(self,):
        '''
        Runs in a background thread to asynchronously debounce the footswitch buttons.
        '''
        while True:
            now = time.ticks_ms()

            # Debounce the buttons
            for i, footswitch in enumerate(self.footswitches):
                cur_value = footswitch.button.pin_value == 0    # inputs are active low
                if cur_value != self._debounces[i][0]:
                    self._debounces[i][1] = now
                else:
                    # If button held or released for more than the debounce time
                    # and the pin value is different than the button's current value
                    # update the button's debounced value
                    if cur_value != footswitch.button.is_pressed and \
                            time.ticks_diff(now, self._debounces[i][1]) > self.BUTTON_DEBOUNCE_TIME_MS:
                        footswitch.button.set_value(cur_value)
                self._debounces[i][0] = cur_value

            # Handle any LED blinking
            for i, footswitch in enumerate(self.footswitches):
                # Only blink when the indicator is on
                if footswitch.indicator.is_on:
                    if not footswitch.indicator.is_blinking and footswitch.indicator.on_time_ms > 0 and \
                            time.ticks_diff(now, footswitch.indicator.last_change) > footswitch.indicator.on_time_ms:
                        footswitch.indicator.set_blink(True)
                    elif footswitch.indicator.is_blinking and footswitch.indicator.off_time_ms > 0 and \
                            time.ticks_diff(now, footswitch.indicator.last_change) > footswitch.indicator.off_time_ms:
                        footswitch.indicator.set_blink(False)

            self.last_tick = time.ticks_ms()


class Footswitch:
    COUNT_RATE_MS = 150

    def __init__(self):
        self.last_tick = time.ticks_ms()
        self.last_update = self.last_tick
        self.connected = False
        self.actuators = []

        # Configure actuators
        for i in range(len(FOOTSWITCH_DIOS)):
            self.actuators.append(FootswitchActuator(i, "Foot #%d" % (i+1), FOOTSWITCH_DIOS[i], INDICATOR_DIOS[i]))

        # Create the underlying Control Chain device slave
        # NOTE: We use pyb.Timer here and not machine.Timer so we can specify one of the high-speed timers
        timer = pyb.Timer(TIMER_NUMBER)
        uart = machine.UART(UART_NUMBER, BAUD_RATE, bits=8, parity=None, stop=1, rxbuf=256)
        tx_en = machine.Pin(SERIAL_TX_EN, machine.Pin.OUT)
        self.device = ControlChainSlaveDevice(DEVICE_NAME, DEVICE_URL, [w.actuator for w in self.actuators],
                                              timer, uart, tx_en,
                                              events_callback=self.events_cb)

        # LED pattern to show while the footswitch is trying to connect to the Mod
        # (this will be updated in the mainloop until connected)
        self.led_pattern_override = [1] + [0]*(len(self.actuators) - 1)

        # Fire up the IO monitor background thread
        self.button_monitor = DIOMonitor(self.actuators)

    def process(self,):
        now = time.ticks_ms()

        # Tell the device to process any messages in its receive queue
        # (MUST be called once in the main loop, ideally at the start)
        self.device.process_messages()

        if not self.connected:
            # Handle the transition from handshaking to connected (clearing the LEDs)
            if self.device.is_connected:
                self.connected = True
                # Turn off all LEDs
                self.led_pattern_override = [0]*len(self.actuators)
                self.force_led_pattern(self.led_pattern_override)
            else:
                if time.ticks_diff(now, self.last_update) > self.COUNT_RATE_MS:
                    # Advance the LED pattern indicating the footswitch is still connecting
                    # Rotate the list in a circular manner
                    self.led_pattern_override = self.led_pattern_override[-1:] + self.led_pattern_override[:-1]
                    self.last_update = now
                    self.force_led_pattern(self.led_pattern_override)

        # Update the actuator state
        for i, actuator in enumerate(self.actuators):
            actuator.update(now)

        self.last_tick = now

        # Tell the device to update itself based on any actuator state changes
        # (MUST be called once in the main loop, ideally at the end after updating any actuator values)
        self.device.update()

    def reset(self,):
        # Turn off all LEDs
        self.force_led_pattern([0]*len(self.actuators))
        # TODO: What else do we do here?

    def force_led_pattern(self, pattern):
        for i in range(min(len(pattern), len(self.actuators))):
            self.actuators[i].indicator.set_value(True if pattern[i] else False)

    def events_cb(self, event):
        print("Got event: ", event.__dict__)
        if event.id == CC_EV_UPDATE or event.id == CC_EV_ASSIGNMENT:
            if isinstance(event.data, CCAssignment):
                print(event.data.__dict__)
                self.actuators[event.data.actuator_id].update_assignment(event.data)

        elif event.id == CC_EV_UNASSIGNMENT:
            self.actuators[event.data].update_assignment(None)

        elif event.id == CC_EV_SET_VALUE:
            if isinstance(event.data, CCMsgSetValue):
                print(event.data.__dict__)
                self.actuators[event.data.actuator_id].update_assignment(self.device.assignments.get(event.data.assignment_id))

        elif event.id == CC_EV_MASTER_RESETED:
            self.reset()


def main():
    # LED on evaluation board
    led = machine.Pin(LED_DIO, machine.Pin.OUT)

    # Flash LED on startup
    led.on()
    time.sleep_ms(500)
    led.off()

    print("Starting footswitch firmware!")
    footswitch = Footswitch()

    while True:
        footswitch.process()


if __name__ == "__main__":
    set_log_level(LOGLEVEL_INFO)
    gc.collect()
    main()
