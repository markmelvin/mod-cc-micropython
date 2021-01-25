import uasyncio as asyncio
from primitives.queue import Queue, QueueEmpty, QueueFull
import time
import machine

from py_cc.cc_protocol import CCDevice, CCSlave
from py_cc.cc_constants import LISTENING_REQUESTS

RX_BUFFER_SIZE = 64
TX_BUFFER_SIZE = 128

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
        return 60000.0 / max(value, 0.1)
    elif units_lower == "hz":
        return 1000.0 / max(value, 0.1)
    elif units_lower == "s":
        return value / 1000.0
    elif units_lower == "ms":
        return value
    return 0.0

class ControlChainSlaveDevice:
    def __init__(self, name, url, fw_version, actuators, timer, uart, tx_en, events_callback=None):
        self.events_cb = events_callback

        self.uart = uart

        # Serial TX enable pin
        self.tx_en = tx_en
        self.tx_en.off()

        self.send_queue = Queue(maxsize=50)
        self.event_queue = Queue(maxsize=50)

        self.cc_device = CCDevice(name, url, fw_version, actuators=actuators)
        self.cc_slave = CCSlave(self.on_response, self.on_event, timer, self.cc_device)

        # Allocate fixed buffers once up-front and use memoryviews
        # to save on allocations and reduce memory fragmentation
        self._rx_buffer = bytearray(RX_BUFFER_SIZE)
        self._tx_buffer = bytearray(TX_BUFFER_SIZE)

        self.RX_BUF = memoryview(self._rx_buffer)
        self.TX_BUF = memoryview(self._tx_buffer)

        # Hook up our callback to the UART receive interrupt
        # TODO: This may not be fully public/finished API but seems to work well for my STM32 board
        #self.uart.irq(handler, trigger, hard)  # Not properly documented in uPython
        # TODO: machine.UART.IRQ_RXIDLE seems specific to STM32?
        self.uart.irq(self.on_serial_rx_data, machine.UART.IRQ_RXIDLE, False)

        asyncio.create_task(self.handle_serial_tx())
        asyncio.create_task(self.cc_slave_process())
        asyncio.create_task(self.process_events())

    def on_serial_rx_data(self, *args, **kwargs):
        '''
        Callback called as an ISR whenever there are 1-8 characters waiting in the RX buffer.
        '''
        bytes_avail =  self.uart.any()
        if bytes_avail > RX_BUFFER_SIZE:
            #TODO - handle failure in a robust manner
            print("ERROR: Losing %d bytes of data!" % (bytes_avail - RX_BUFFER_SIZE))

        buffer = self.RX_BUF[0:bytes_avail]
        length = len(buffer)
        remaining = length
        while remaining > 0:
            _read = self.uart.readinto(buffer[length-remaining:], remaining)
            remaining -= _read
        messages = self.cc_slave.protocol.receive_data(buffer)
        for message in messages:
            self.cc_slave.handle_message(message)

    async def handle_serial_tx(self,):
        while True:
            # Service any queued messages to be sent
            message = await self.send_queue.get()
            # Enable serial TX
            self.tx_en.on()
            # TODO - Figure out what this should be...
            time.sleep_us(300)
            num_tx_bytes = message.get_tx_bytes(self.TX_BUF[0:])
            bytes_written = 0
            while bytes_written < num_tx_bytes:
                # print("Sending: %s" % [w for w in self.TX_BUF[bytes_written:num_tx_bytes]])
                bytes_written += self.uart.write(self.TX_BUF[bytes_written:num_tx_bytes])
            self.tx_en.off()
            del message

    # Callbacks
    def on_response(self, message):
        try:
            self.send_queue.put_nowait(message)
        except QueueFull:
            print("Send queue full! Dropping message!", message.__dict__)

    def on_event(self, event):
        try:
            self.event_queue.put_nowait(event)
        except QueueFull:
            print("Event queue full! Dropping event!", event.__dict__)

    async def cc_slave_process(self,):
        '''
        Advance the control chain slave state machines.
        '''
        while True:
            self.cc_slave.process()
            await asyncio.sleep_ms(0)

    async def process_events(self,):
        '''
        Asynchronously process the event queue and handle any queued messages found in it,
        and advance the control chain slave state machines.
        '''
        while True:
            # Process any events in the event queue
            event = await self.event_queue.get()
            if self.events_cb is not None:
                self.events_cb(event)
            await asyncio.sleep_ms(0)

    #########################
    # User-exposed API calls
    #########################
    @property
    def is_connected(self,):
        return self.cc_slave.comm_state == LISTENING_REQUESTS

    @property
    def assignments(self,):
        return self.cc_device.assignments

    @property
    def actuators(self,):
        return self.cc_device.actuators
