# micropython-based footswitch using the moddevices Arduino shield
# attached to an STM32 Nucleo development board (NUCLEO-F-446RE)
import pyb
import machine
import time
import sys
import _thread
from ucollections import deque

sys.path.append('./')

from py_cc.cc_protocol import CCDevice, CCSlave, CCActuator, \
                              set_log_level, LOGLEVEL_DEBUG, LOGLEVEL_INFO
from py_cc.cc_constants import CC_BAUD_RATE_FALLBACK, CC_ACTUATOR_MOMENTARY, \
                               CC_MODE_TOGGLE, CC_MODE_TRIGGER, CC_MODE_OPTIONS

BAUD_RATE       = CC_BAUD_RATE_FALLBACK
LED_PIN         = 1
TIMER_NUMBER    = 2         # Timer2 is one of the high-speed timers
UART_NUMBER     = 6         # UART6 is on PC6/7
SERIAL_TX_EN    = 'PA10'

class Thread:
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None):
        self.target = target
        self.args = args
        self.kwargs = {} if kwargs is None else kwargs

    def start(self):
        _thread.start_new_thread(self.run, ())

    def run(self):
        self.target(*self.args, **self.kwargs)


class Footswitch:
    def __init__(self, number_of_footswitches=4):
        # Serial TX enable pin
        self.tx_en = machine.Pin(SERIAL_TX_EN, machine.Pin.OUT)
        self.tx_en.off()

        # Configure actuators
        actuators = []
        for i in range(number_of_footswitches):
            actuators.append(CCActuator("Foot #%d" % (i+1),
                                        CC_ACTUATOR_MOMENTARY,
                                        0.0, 1.0,
                                        CC_MODE_TOGGLE | CC_MODE_TRIGGER | CC_MODE_OPTIONS,
                                        max_assignments=1))

        # Device and control chain slave
        self.device = CCDevice("Python Footswitch!", "http://audiofab.com", actuators=actuators)
        self.cc_slave = CCSlave(self.response_cb, self.events_cb, pyb.Timer(TIMER_NUMBER), self.device)
        self.send_queue = deque((), 50)
        self.receive_queue = deque((), 50)
        self.serial_monitor = Thread(target=self.handle_serial_traffic)
        self.serial_monitor.start()

    def response_cb(self, data):
        self.send_queue.append(data)

    def events_cb(self, event):
        print(event.__dict__)

    def process(self,):
        while len(self.receive_queue) > 0:
            self.cc_slave.protocol.receive_data(self.receive_queue.popleft())
        self.cc_slave.process()

    def handle_serial_traffic(self,):
        uart = machine.UART(UART_NUMBER, BAUD_RATE, bits=8, parity=None, stop=1, rxbuf=256)
        while True:
            # Check for new bytes
            while (uart.any() > 0):
                b = uart.read()
                # print("Received: %s" % [w for w in b])
                self.receive_queue.append(b)

            # Service any queued messages to be sent
            if len(self.send_queue) > 0:
                # Enable serial TX
                self.tx_en.on()
                time.sleep_us(100)
                buf = self.send_queue.popleft()
                bytes_left = len(buf)
                while bytes_left > 0:
                    print("Sending: %s" % [w for w in buf[-bytes_left:]])
                    bytes_left -= uart.write(bytes(buf[-bytes_left:]))
                self.tx_en.off()

def main():
    led = pyb.LED(LED_PIN)

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
    main()
