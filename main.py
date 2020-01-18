# micropython-based footswitch using the moddevices Arduino shield
# attached to an STM32 Nucleo development board (NUCLEO-F-446RE)
import pyb
import machine
import time
import sys
import _thread
import math
import gc
from ucollections import deque

sys.path.append('./')

from py_cc.cc_protocol import CCDevice, CCSlave, CCActuator, \
                              set_log_level, LOGLEVEL_DEBUG, LOGLEVEL_INFO
from py_cc.cc_constants import CC_BAUD_RATE_FALLBACK, CC_ACTUATOR_MOMENTARY, \
                               CC_MODE_TOGGLE, CC_MODE_TRIGGER, CC_MODE_OPTIONS, \
                               CC_ACTUATOR_CONTINUOUS, CC_MODE_REAL, CC_SYNC_BYTE

BAUD_RATE       = CC_BAUD_RATE_FALLBACK
LED_PIN         = 1
TIMER_NUMBER    = 2         # Timer2 is one of the high-speed timers
UART_NUMBER     = 6         # UART6 is on PC6/7
SERIAL_TX_EN    = 'PA10'

FOOTSWITCH_DIOS = ['PB3', 'PB4', 'PB5', 'PB6']
BUTTON_DEBOUNCE_CYCLES = 10

RX_BUFFER_SIZE = 2048
TX_BUFFER_SIZE = 128


class Thread:
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None):
        self.target = target
        self.args = args
        self.kwargs = {} if kwargs is None else kwargs

    def start(self):
        _thread.start_new_thread(self.run, ())

    def run(self):
        self.target(*self.args, **self.kwargs)


class SerialManager:
    def __init__(self, send_queue, receive_queue):
        # Serial TX enable pin
        self.tx_en = machine.Pin(SERIAL_TX_EN, machine.Pin.OUT)
        self.tx_en.off()

        self.send_queue = send_queue
        self.receive_queue = receive_queue

        self.lock = _thread.allocate_lock()
        self.serial_monitor = Thread(target=self.handle_serial_traffic)
        self.serial_monitor.start()
        # RX circular buffer management
        self.start_ptr = 0   # start of used data in the buffer
        self.end_ptr = 0     # start of free space in the buffer

    def handle_serial_traffic(self,):
        uart = machine.UART(UART_NUMBER, BAUD_RATE, bits=8, parity=None, stop=1, rxbuf=256)
        # Allocate fixed buffers once upfront and use memoryviews
        # to save on allocations and reduce fragmentation
        _rx_buffer = bytearray(RX_BUFFER_SIZE)
        _tx_buffer = bytearray(TX_BUFFER_SIZE)
        # We *always* transmit a sync byte, so stick it in there now
        _tx_buffer[0] = CC_SYNC_BYTE

        RX_BUF = memoryview(_rx_buffer)
        TX_BUF = memoryview(_tx_buffer)

        while True:
            # Check for new bytes
            avail = uart.any()
            while (avail > 0):
                free_space = (RX_BUFFER_SIZE - self.start_ptr - self.end_ptr)
                print(avail, self.start_ptr, self.end_ptr, len(self.receive_queue))
                # Read at most half the number of free bytes in the circular buffer
                if avail > free_space // 2:
                    _len = uart.readinto(RX_BUF[self.end_ptr:], free_space // 2)
                else:
                    _len = uart.readinto(RX_BUF[self.end_ptr:])
                with self.lock:
                    if _len:
                        print("Read %d bytes!" % _len)
                        overflow = RX_BUFFER_SIZE - (self.end_ptr + _len)
                        if overflow > 0:
                            # Split into memory views that don't cross buffer boundary
                            self.receive_queue.append((self.end_ptr, RX_BUF[self.end_ptr:]))
                            self.end_ptr = overflow + 1
                            self.receive_queue.append((0, RX_BUF[0:self.end_ptr]))
                        else:
                            self.receive_queue.append((self.end_ptr, RX_BUF[self.end_ptr:self.end_ptr + _len]))
                            self.end_ptr += _len
                avail = uart.any()

            # Service any queued messages to be sent
            if len(self.send_queue) > 0:
                # Enable serial TX
                self.tx_en.on()
                time.sleep_us(100)
                message = self.send_queue.popleft()
                num_tx_bytes = message.get_tx_bytes(TX_BUF[1:])
                bytes_left = num_tx_bytes + 1   # +1 for the sync byte
                while bytes_left > 0:
                    print("Sending: %s" % [w for w in TX_BUF[-bytes_left:]])
                    bytes_left -= uart.write(TX_BUF[-bytes_left:])
                self.tx_en.off()
                del message

    def free(self, idx, _len):
        with self.lock:
            # Adjust start pointer based on memory we're freeing
            overflow = RX_BUFFER_SIZE - (self.start_ptr + _len)
            if overflow > 0:
                self.start_ptr = overflow
            else:
                self.start_ptr += _len


class Footswitch:
    def __init__(self, led):
        self.led = led
        self.analog = 0.0
        self.last_tick = time.ticks_ms()

        self.switches = []
        # Configure actuators
        actuators = []
        for i, dio in enumerate(FOOTSWITCH_DIOS):
            # Configure the DIO
            self.switches.append(machine.Pin(dio, machine.Pin.IN, machine.Pin.PULL_UP))
            # Add it as an actuator
            actuators.append(CCActuator(i,
                                        "Foot #%d" % (i+1),
                                        CC_ACTUATOR_MOMENTARY,
                                        0.0, 1.0,
                                        CC_MODE_TOGGLE | CC_MODE_TRIGGER | CC_MODE_OPTIONS,
                                        max_assignments=1))

        # Append a simulated analog signal (continuous sine wave)
        actuators.append(CCActuator(len(FOOTSWITCH_DIOS),
                                    "Sine Wave",
                                    CC_ACTUATOR_CONTINUOUS,
                                    0.0, 1.0,
                                    CC_MODE_REAL,
                                    max_assignments=1))

        # Device and control chain slave
        self.device = CCDevice("Python Footswitch!", "http://audiofab.com", actuators=actuators)
        self.cc_slave = CCSlave(self.response_cb, self.events_cb, pyb.Timer(TIMER_NUMBER), self.device)
        self.send_queue = deque((), 100)
        self.receive_queue = deque((), 100)
        self.footswitch_values = [False]*len(self.switches)

        self.serial = SerialManager(self.send_queue, self.receive_queue)
        self.button_monitor = Thread(target=self.monitor_buttons)
        self.button_monitor.start()

    def response_cb(self, message):
        self.send_queue.append(message)

    def events_cb(self, event):
        print("Got Event: ", event.__dict__)

    def process(self,):
        now = time.ticks_ms()

        self.process_recv_queue()

        for i, pressed in enumerate(self.footswitch_values):
            self.device.actuators[i].value = 1.0 if pressed else 0.0

        self.analog += 0.01
        if self.analog > math.pi:
            self.analog = 0.0
        self.device.actuators[len(self.footswitch_values)].value = math.sin(self.analog)

        if time.ticks_diff(now, self.last_tick) > 1000:
            print(gc.mem_free())
            self.last_tick = now

        self.cc_slave.process()

    def process_recv_queue(self,):
        while len(self.receive_queue) > 0:
            start_index, data = self.receive_queue.popleft()
            messages = self.cc_slave.protocol.receive_data(data)
            if messages is not None:
                for msg in messages:
                    self.cc_slave.handle_message(msg)

            # Free data in circular buffer
            self.serial.free(start_index, len(data))

    def monitor_buttons(self,):
        button_history = [
            [1]*BUTTON_DEBOUNCE_CYCLES for w in self.switches
        ]

        while True:
            for i, switch in enumerate(self.switches):
                button_history[i].append(switch.value())
                if len(button_history[i]) > BUTTON_DEBOUNCE_CYCLES:
                    _ = button_history[i].pop(0)
                if 1 not in button_history[i]:
                    self.footswitch_values[i] = True
                elif 0 not in button_history[i]:
                    self.footswitch_values[i] = False
                # else leave state as it was
            time.sleep_ms(1)


def main():
    led = pyb.LED(LED_PIN)

    # Flash LED on startup
    led.on()
    time.sleep_ms(500)
    led.off()

    print("Starting footswitch firmware!")
    footswitch = Footswitch(led)

    while True:
        footswitch.process()


if __name__ == "__main__":
    set_log_level(LOGLEVEL_INFO)
    gc.collect()
    main()
