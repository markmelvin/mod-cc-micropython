import _thread
import time
import machine
from ucollections import deque

from .thread import Thread

from py_cc.cc_protocol import CCDevice, CCSlave
from py_cc.cc_constants import LISTENING_REQUESTS

RX_BUFFER_SIZE = 2048
TX_BUFFER_SIZE = 128


class ControlChainSlaveDevice:
    def __init__(self, name, url, actuators, timer, uart, tx_en, events_callback=None):
        self.events_cb = events_callback

        # Serial TX enable pin
        self.uart = uart
        self.tx_en = tx_en
        self.tx_en.off()

        self.send_queue = deque((), 50)
        self.recv_queue = deque((), 50)

        self.cc_device = CCDevice(name, url, actuators=actuators)
        self.cc_slave = CCSlave(self.on_response, self.on_event, timer, self.cc_device)

        self.lock = _thread.allocate_lock()
        self.start_ptr = 0   # start of used data in the buffer
        self.end_ptr = 0     # start of free space in the buffer
        self.serial_monitor = Thread(target=self.handle_serial_traffic)
        self.serial_monitor.start()

    def handle_serial_traffic(self,):
        # Allocate fixed buffers once up-front and use memoryviews
        # to save on allocations and reduce memory fragmentation
        _rx_buffer = bytearray(RX_BUFFER_SIZE)
        _tx_buffer = bytearray(TX_BUFFER_SIZE)

        RX_BUF = memoryview(_rx_buffer)
        TX_BUF = memoryview(_tx_buffer)

        while True:
            # Check for new bytes
            bytes_avail =  self.uart.any()
            if bytes_avail > 0:
                with self.lock:
                    if self.start_ptr > self.end_ptr:
                        free_space = (self.start_ptr - self.end_ptr)
                    else:
                        free_space = (RX_BUFFER_SIZE - (self.end_ptr - self.start_ptr))

                    if free_space < bytes_avail:
                        #TODO - handle failure in a robust manner
                        print("ERROR: Losing %d bytes of data!" % (bytes_avail - free_space))

                    bytes_to_read = min(bytes_avail, free_space)
                    overflow = (self.end_ptr + bytes_to_read) - RX_BUFFER_SIZE
                    if overflow > 0:
                        # Split into memory views that don't cross buffer boundary
                        buf = RX_BUF[self.end_ptr:]
                        self._fill(buf)
                        self.recv_queue.append((self.end_ptr, buf))

                        # Update final end pointer
                        self.end_ptr = overflow
                        buf = RX_BUF[0:self.end_ptr]
                        self._fill(buf)
                        self.recv_queue.append((0, buf))

                    else:
                        buf = RX_BUF[self.end_ptr:self.end_ptr + bytes_to_read]
                        self._fill(buf)
                        self.recv_queue.append((self.end_ptr, buf))
                        self.end_ptr += bytes_to_read
                        if self.end_ptr == RX_BUFFER_SIZE:
                            self.end_ptr = 0

            # Service any queued messages to be sent
            if len(self.send_queue) > 0:
                # Enable serial TX
                self.tx_en.on()
                # TODO - Figure out what this should be...
                time.sleep_us(300)
                message = self.send_queue.popleft()
                num_tx_bytes = message.get_tx_bytes(TX_BUF[0:])
                bytes_written = 0
                while bytes_written < num_tx_bytes:
                    # print("Sending: %s" % [w for w in TX_BUF[bytes_written:num_tx_bytes]])
                    bytes_written += self.uart.write(TX_BUF[bytes_written:num_tx_bytes])
                self.tx_en.off()
                del message

    def _fill(self, buffer):
        length = len(buffer)
        remaining = length
        while remaining > 0:
            _read = self.uart.readinto(buffer[length-remaining:], remaining)
            remaining -= _read

    def free(self, idx, _len):
        with self.lock:
            # Adjust start pointer based on memory we're freeing
            overflow = (self.start_ptr + _len) - RX_BUFFER_SIZE
            if overflow >= 0:
                self.start_ptr = overflow
            else:
                self.start_ptr += _len

    # Callbacks
    def on_response(self, message):
        self.send_queue.append(message)

    def on_event(self, event):
        if self.events_cb is not None:
            self.events_cb(event)

    #########################
    # User-exposed API calls
    #########################
    def process_messages(self,):
        '''
        Process the receive queue and handle any queued messages found in it.
        '''
        while len(self.recv_queue) > 0:
            start_index, data = self.recv_queue.popleft()
            messages = self.cc_slave.protocol.receive_data(data)
            # Free data in circular buffer
            self.free(start_index, len(data))
            if messages is not None:
                for msg in messages:
                    self.cc_slave.handle_message(msg)

    def update(self,):
        '''
        Handle any actuator state changes and fire events as appropriate.
        '''
        self.cc_slave.process()

    @property
    def is_connected(self,):
        return self.cc_slave.comm_state == LISTENING_REQUESTS

    @property
    def assignments(self,):
        return self.cc_device.assignments

    @property
    def actuators(self,):
        return self.cc_device.actuators
