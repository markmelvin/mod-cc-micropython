## WARNING: This file may be severely out of date (and may be deleted at some point)

import time
import sys
import _thread
import ustruct as struct
from ucollections import deque

sys.path.append('../')

from py_cc.cc_protocol import *
from py_cc.cc_constants import *


class Thread:
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None):
        self.target = target
        self.args = args
        self.kwargs = {} if kwargs is None else kwargs

    def start(self):
        _thread.start_new_thread(self.run, ())

    def run(self):
        self.target(*self.args, **self.kwargs)

class Timer(Thread):
    """Call a function after a specified number of seconds:
            t = Timer(30.0, f, args=None, kwargs=None)
            t.start()
            t.cancel()     # stop the timer's action if it's still waiting
    """
    def __init__(self, interval, function, args=None, kwargs=None):
        Thread.__init__(self)
        self.interval = interval
        self.function = function
        self.args = args if args is not None else []
        self.kwargs = kwargs if kwargs is not None else {}

    def cancel(self):
        """Stop the timer if it hasn't finished yet."""
        raise NotImplementedError("cancel not implemented")

    def run(self):
        time.sleep_us(self.interval)
        self.function(*self.args, **self.kwargs)


def timer_set(value, callback):
    # print("Timer: %f" % value)
    t = Timer(value, callback)
    t.start()


CC_MASTER_STATE_IDLE                    = 0
CC_MASTER_STATE_WAITING_FOR_RESET       = 1
CC_MASTER_STATE_WAITING_FOR_HANDSHAKE   = 2
CC_MASTER_STATE_HANDSHAKING             = 3
CC_MASTER_STATE_WAITING_FOR_DEV_DESCR   = 4
CC_MASTER_STATE_PAIRED                  = 5

class CCMaster:
    ADVERTIZING_INTERVAL_MS = 5000
    PING_INTERVAL_MS        = 100

    '''Class to spoof the Mod DuoX to simulate handshaking with a device'''
    def __init__(self,):
        self.state = CC_MASTER_STATE_IDLE
        self.stop_request = False
        self.thread = Thread(target=self.run)
        self.send_data = None
        self.device_ids = {}
        self.receive_queue = deque((), 50)
        self.event_queue = deque((), 50)
        self.adv_time = time.ticks_ms()
        self.last_ping = None

    def start(self,):
        self.thread.start()

    def stop(self,):
        self.stop_request = True

    def _send(self, data):
        if self.send_data is not None and callable(self.send_data):
            self.send_data([CC_SYNC_BYTE] + data)

    def sync(self,):
        self._send(CCMsgSync(CC_BROADCAST_ADDRESS, {'type' : CC_SYNC_SETUP_CYCLE}).get_tx_bytes())
        self.state = CC_MASTER_STATE_WAITING_FOR_RESET

    def run(self,):
        while not self.stop_request:
            now = time.ticks_ms()
            if self.state == CC_MASTER_STATE_IDLE:
                self.sync()

            elif self.state == CC_MASTER_STATE_PAIRED:
                if self.last_ping is not None and time.ticks_diff(now, self.last_ping) > self.PING_INTERVAL_MS:
                    for device_id in self.device_ids.keys():
                        # print("Pinging paired devices...")
                        self._send(CCMsgSync(device_id, {'type' : CC_SYNC_REGULAR_CYCLE}).get_tx_bytes())
                    self.last_ping = now

            # Service the message queues
            if len(self.receive_queue) > 0:
                self.handle_response(self.receive_queue.popleft())

            if len(self.event_queue) > 0:
                self.handle_event(self.event_queue.popleft())

            # If we have not paired wih anything, send out a sync setup
            # every ADVERTIZING_INTERVAL_MS
            if self.state not in [CC_MASTER_STATE_IDLE, CC_MASTER_STATE_PAIRED] and \
                    time.ticks_diff(now, self.adv_time) > self.ADVERTIZING_INTERVAL_MS:
                self.state = CC_MASTER_STATE_IDLE
                self.adv_time = now

    def queue_msg(self, data):
        self.receive_queue.append(data)

    def queue_event(self, data):
        self.event_queue.append(data)

    def handle_event(self, event):
        print("CCMaster got event: ", event.__dict__)
        if self.state == CC_MASTER_STATE_WAITING_FOR_RESET and event.id == CC_EV_MASTER_RESETED:
            self._send(CCMsgSync(CC_BROADCAST_ADDRESS, {'type' : CC_SYNC_HANDSHAKE_CYCLE}).get_tx_bytes())
            self.state = CC_MASTER_STATE_WAITING_FOR_HANDSHAKE
            self.adv_time = time.ticks_ms()
        if self.state == CC_MASTER_STATE_HANDSHAKING and \
                event.id == CC_EV_HANDSHAKE_FAILED and event.data == CC_HANDSHAKE_OK and \
                    len(self.device_ids) > 0:
            for device_id in self.device_ids.keys():
                self._send(CCMsgDeviceDescriptorRequest(device_id, {'type' : CC_DEVICE_DESC_REQ}).get_tx_bytes())
            self.state = CC_MASTER_STATE_WAITING_FOR_DEV_DESCR


    def handle_response(self, data):
        print("CCMaster got response: ", data)
        if len(data) > 1:
            sync, address, command , data_len_lsb, data_len_msb, *_data = data
            crc = _data.pop(-1)
            if self.state == CC_MASTER_STATE_WAITING_FOR_HANDSHAKE and command == CC_CMD_HANDSHAKE:
                message = CCMsgHandshake(CC_BROADCAST_ADDRESS, _data)
                self.state = CC_MASTER_STATE_HANDSHAKING
                # Reply to device with new device ID
                device_id = self.assign_next_device_id(message)
                print("Assigning device id %d" % device_id)
                reply = CCMsgHandshakeStatus(CC_BROADCAST_ADDRESS,
                                                {'random_id' : message.handshake.random_id,
                                                 'status' : CC_HANDSHAKE_OK,
                                                 'assigned_device_id' : device_id,
                                                #  'channel' : 0,   # NOT SUPPORTED IN CURRENT PROTOCOL!
                                                 })
                self._send(reply.get_tx_bytes())

            if self.state == CC_MASTER_STATE_WAITING_FOR_DEV_DESCR and command == CC_CMD_DEV_DESCRIPTOR and \
                    address in self.device_ids:
                self._send(CCMsgDeviceDescriptorRequest(address, {'type' : CC_DEVICE_DESC_ACK}).get_tx_bytes())
                self.state = CC_MASTER_STATE_PAIRED
                self.last_ping = time.ticks_ms()

            if self.state == CC_MASTER_STATE_PAIRED and command == CC_CMD_CHAIN_SYNC and \
                    address in self.device_ids:
                message = CCMsgSync(address, _data)
                if message.type == CC_SYNC_REGULAR_CYCLE:
                    print("CCMaster: heartbeat message from device %d" % address)


    def assign_next_device_id(self, handshake):
        next_id = 1
        try:
            next_id = max(self.device_ids) + 1
        except ValueError:
            pass
        self.device_ids[next_id] = handshake
        return next_id


if __name__ == "__main__":
    set_log_level(LOGLEVEL_INFO)
    RUNTIME_SECS = 10

    master = CCMaster()

    # Configure actuators
    actuators = []
    for i in range(4):
        actuators.append(CCActuator("Foot #%d" % (i+1),
                                    CC_ACTUATOR_MOMENTARY,
                                    0.0, 1.0,
                                    CC_MODE_TOGGLE | CC_MODE_TRIGGER | CC_MODE_OPTIONS,
                                    max_assignments=1))

    # Device and control chain slave
    device = CCDevice("Python Footswitch!", "http://audiofab.com", actuators=actuators)


    slave = CCSlave(master.queue_msg, master.queue_event, timer_set, device)
    master.send_data = slave.protocol.receive_data
    master.start()

    start_time = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_time) < RUNTIME_SECS*1000:
        time.sleep(0.01)
        slave.process()

    master.stop()
