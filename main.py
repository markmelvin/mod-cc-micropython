# micropython-based footswitch using the moddevices Arduino shield
# attached to an STM32 Nucleo development board (NUCLEO-F-446RE)
import pyb
import machine
import time
import sys
import math
import gc

sys.path.append('./')

from py_cc.cc_protocol import CCActuator, CCAssignment, set_log_level, LOGLEVEL_DEBUG, LOGLEVEL_INFO
from py_cc.cc_constants import CC_BAUD_RATE_FALLBACK, CC_ACTUATOR_MOMENTARY, \
                               CC_MODE_TOGGLE, CC_MODE_TRIGGER, CC_MODE_OPTIONS, \
                               CC_ACTUATOR_CONTINUOUS, CC_MODE_REAL, CC_MODE_FEEDBACK, \
                               CC_EV_UPDATE, CC_EV_ASSIGNMENT, CC_EV_UNASSIGNMENT

from utils.thread import Thread
from utils.control_chain import ControlChainSlaveDevice

DEVICE_NAME = "Audiofab Footswitch"
DEVICE_URL = "http://audiofab.com"

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

FOOTSWITCH_ACTUATOR_MIN = 0.0
FOOTSWITCH_ACTUATOR_MAX = 1.0


class UpDownCounter:
    DIR_DOWN = 0
    DIR_UP = 1

    def __init__(self, min, max):
        self.value = 0
        self.minimum = min
        self.maximum = max
        self.direction = self.DIR_UP

    def tick(self,):
        self.value = self.value + (1 if self.direction == self.DIR_UP else -1)
        if self.value >= self.maximum:
            self.direction = self.DIR_DOWN
        elif self.value <= self.minimum:
            self.direction = self.DIR_UP


class HandshakingIndicatorDriver:
    MIN = 0
    MAX = 512   # Set according to loop speed (how fast .tick() is called)

    def __init__(self, indicators):
        self.indicators = indicators
        self.width = (self.MAX - self.MIN) / len(self.indicators)
        self.counter = UpDownCounter(min=self.MIN, max=self.MAX)

    def tick(self,):
        self.counter.tick()
        self.update_indicators()

    def update_indicators(self,):
        # Sweep an LED back and forth
        for i, indicator in enumerate(self.indicators):
            if self.counter.value >= (i * self.width) and self.counter.value <= ((i + 1) * self.width):
                indicator.on()
            else:
                indicator.off()


class MomentaryButton:
    def __init__(self, dio):
        self.pin = machine.Pin(dio, machine.Pin.IN, machine.Pin.PULL_UP)
        self.value = False
        self.last_pressed = time.ticks_ms()
        self.interval_ms = 0

    def update(self, value):
        if value and not self.is_pressed:
            # Rising edge
            now = time.ticks_ms()
            self.interval_ms = time.ticks_diff(now, self.last_pressed)
            self.last_pressed = now

        self.value = True if value else False

    @property
    def pin_value(self,):
        return self.pin.value()

    @property
    def is_pressed(self,):
        return self.value is True

    @property
    def tap_tempo(self,):
        # Return BPM
        return 60 * 1000 / self.interval_ms


class Footswitch:
    def __init__(self):
        self.last_tick = time.ticks_ms()
        self.connected = False
        self.switches = []
        self.indicators = []

        # Configure actuators
        actuators = []
        for i, dio in enumerate(FOOTSWITCH_DIOS):
            # Configure the DIOs
            self.switches.append(MomentaryButton(dio))
            # Add it as an actuator
            actuators.append(CCActuator(i,
                                        "Foot #%d" % (i+1),
                                        CC_ACTUATOR_MOMENTARY,
                                        FOOTSWITCH_ACTUATOR_MIN, FOOTSWITCH_ACTUATOR_MAX,
                                        CC_MODE_TOGGLE | CC_MODE_TRIGGER | CC_MODE_OPTIONS,
                                        max_assignments=1))
            # Add a corresponding LED (the "ID" of the actuator corresponds to the index
            # of its switch and indicator in their respective arrays)
            self.indicators.append(machine.Pin(INDICATOR_DIOS[i], machine.Pin.OUT))

        self.state_indicator = HandshakingIndicatorDriver(self.indicators)

        # Create the underlying Control Chain device slave
        # NOTE: We use pyb.Timer here and not machine.Timer so we can specify one of the high-speed timers
        timer = pyb.Timer(TIMER_NUMBER)
        uart = machine.UART(UART_NUMBER, BAUD_RATE, bits=8, parity=None, stop=1, rxbuf=256)
        tx_en = machine.Pin(SERIAL_TX_EN, machine.Pin.OUT)
        self.device = ControlChainSlaveDevice(DEVICE_NAME, DEVICE_URL, actuators,
                                              timer, uart, tx_en,
                                              events_callback=self.events_cb)

        self.button_monitor = Thread(target=self.monitor_buttons)
        self.button_monitor.start()


    def events_cb(self, event):
        # print("Got event: ", event.__dict__)
        if event.id == CC_EV_UPDATE or event.id == CC_EV_ASSIGNMENT:
            if isinstance(event.data, CCAssignment):
                if event.data.id in self.device.assignments:
                    self.update_indicator_for_assignment(event.data)

        elif event.id == CC_EV_UNASSIGNMENT:
            self.indicators[event.data].off()

    def mainloop(self,):
        now = time.ticks_ms()

        # Tell the device to process any messages in its receive queue
        # (MUST be called once in the main loop, ideally at the start)
        self.device.process_messages()

        # Handle the transition from handshaking to connected (clearing the LEDs)
        if not self.connected and self.device.is_connected:
            self.connected = True
            # Turn off all LEDs
            for indicator in self.indicators:
                indicator.off()
            # We're connected so let the actuator state control the indicators
            self.state_indicator = None

        # Process the switch values and update their underlying indicator values
        for i, switch in enumerate(self.switches):
            self.device.actuators[i].value = FOOTSWITCH_ACTUATOR_MAX if switch.is_pressed else FOOTSWITCH_ACTUATOR_MIN

        # Update the indicators, when appropriate
        if self.state_indicator is not None:
            self.state_indicator.tick()

        self.last_tick = now

        # Tell the device to update itself based on any actuator state changes
        # (MUST be called once in the main loop, ideally at the end after updating any actuator values)
        self.device.update()

    def monitor_buttons(self,):
        '''
        Runs in a background thread to debounce the footswtich buttons
        '''
        button_history = [
            [1]*BUTTON_DEBOUNCE_CYCLES for w in self.switches
        ]

        while True:
            for i, switch in enumerate(self.switches):
                button_history[i].append(switch.pin_value)
                if len(button_history[i]) > BUTTON_DEBOUNCE_CYCLES:
                    _ = button_history[i].pop(0)
                if 1 not in button_history[i]:
                    self.switches[i].update(True)
                elif 0 not in button_history[i]:
                    self.switches[i].update(False)
                # else leave state as it was
            time.sleep_ms(1)

    def update_indicator_for_assignment(self, assignment):
        if assignment.mode & (CC_MODE_TRIGGER | CC_MODE_OPTIONS):
            self.indicators[assignment.actuator_id].on()

        elif assignment.mode & CC_MODE_TOGGLE:
            if assignment.value:
                self.indicators[assignment.actuator_id].on()
            else:
                self.indicators[assignment.actuator_id].off()


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
        footswitch.mainloop()


if __name__ == "__main__":
    set_log_level(LOGLEVEL_INFO)
    gc.collect()
    main()
