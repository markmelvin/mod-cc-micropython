import urandom as random
from ucollections import deque
import micropython
import time

random.seed(1)
micropython.alloc_emergency_exception_buf(100)

from .cc_classes import *

_loglevel = LOGLEVEL_INFO

def log(level, msg, *args):
    if level >= _loglevel:
        msg = str(msg)
        if len(args):
            msg = msg % args
        print(msg)

def set_log_level(level):
    global _loglevel
    _loglevel = int(level)

def decode_message(device_id, command, data):
    '''Factory function to create a message from the given data'''
    log(LOGLEVEL_DEBUG, "decode_message: %d, %d, %s", device_id, command, data)
    commands_to_classes = {
        CC_CMD_CHAIN_SYNC       : CCMsgSync,
        CC_CMD_HANDSHAKE        : CCMsgHandshakeStatus,
        CC_CMD_DEV_DESCRIPTOR   : CCMsgDeviceDescriptorRequest,
        CC_CMD_DEV_CONTROL      : CCMsgDeviceControl,
        CC_CMD_ASSIGNMENT       : CCMsgAssignment,
        CC_CMD_UNASSIGNMENT     : CCMsgUnassignment,
    }
    if command in commands_to_classes:
        try:
            return commands_to_classes[command](device_id, data)
        except:
            log(LOGLEVEL_INFO, "An error occurred decoding message (%s).", str(data))
    return None

class CCProtocol:
    '''
    Class encapsulating the Control Chain protocol. This protocol consists
    of messages that start with a sync byte, then is followed by a message
    header, optional data payload, and a CRC calculated over the message
    header and data payload. This class supports receiving messages that may
    be split across multiple calls to the receive_data(data) method.
    '''
    def __init__(self, address=CC_BROADCAST_ADDRESS):
        self.address = address
        self.cur_header = [0]*CC_MSG_HEADER_SIZE
        self._reset(full_reset=True)

    def _reset(self, full_reset=False):
        self.msg_state = MSG_STATE_IDLE
        for i in range(CC_MSG_HEADER_SIZE):
            self.cur_header[i] = 0
        self.cur_device_id = -1
        self.cur_command = -1
        self.cur_data_size = 0
        self.cur_msg_data = []
        if full_reset:
            self.bytes_received = 0
            self.good_message_received = False

    def receive_data(self, data):
        '''
        Re-entrant method that receives an array or list of data bytes
        and advances the protocol state machine accordingly.
        '''
        messages = []
        log(LOGLEVEL_DEBUG, "CCProtocol.receive_data(): %s \n(%s)", [w for w in data], self.__dict__)
        for i in range(len(data)):
            b = data[i]

            if self.msg_state > MSG_STATE_IDLE and self.msg_state < MSG_STATE_READ_DATA:
                self.cur_header[self.msg_state - 1] = b

            if self.msg_state == MSG_STATE_IDLE:
                # sync byte
                if b == CC_SYNC_BYTE:
                    log(LOGLEVEL_DEBUG, "got sync byte!\n")
                    self.msg_state = MSG_STATE_READ_ADDRESS

            elif self.msg_state == MSG_STATE_READ_ADDRESS:
                # device _id
                # check if it's messaging this device or is a broadcast message
                if b == CC_BROADCAST_ADDRESS or self.address == b or self.address == CC_BROADCAST_ADDRESS:
                    self.cur_device_id = b
                    log(LOGLEVEL_DEBUG, "cur_device_id=%s\n", self.cur_device_id)
                    self.msg_state = MSG_STATE_READ_COMMAND
                else:
                    # message is not for us
                    self._reset()

            elif self.msg_state == MSG_STATE_READ_COMMAND:
                # command
                self.cur_command = b
                log(LOGLEVEL_DEBUG, "cur_command=%s\n", self.cur_command)
                self.msg_state = MSG_STATE_READ_DATALEN_LSB

            elif self.msg_state == MSG_STATE_READ_DATALEN_LSB:
                # data size LSB
                self.cur_data_size = b
                self.msg_state = MSG_STATE_READ_DATALEN_MSB

            elif self.msg_state == MSG_STATE_READ_DATALEN_MSB:
                # data size MSB
                self.cur_data_size = (b << 8) | self.cur_data_size
                log(LOGLEVEL_DEBUG, "cur_data_size=%s\n", self.cur_data_size)
                self.msg_state = MSG_STATE_READ_CRC if self.cur_data_size == 0 else MSG_STATE_READ_DATA

            elif self.msg_state == MSG_STATE_READ_DATA:
                # data
                self.cur_msg_data.append(b)
                log(LOGLEVEL_DEBUG, "cur_data=%s\n", self.cur_msg_data)

                if len(self.cur_msg_data) == self.cur_data_size:
                    self.msg_state = MSG_STATE_READ_CRC

            elif self.msg_state == MSG_STATE_READ_CRC:
                # crc
                if crc8(self.cur_header + self.cur_msg_data) == b:
                    log(LOGLEVEL_DEBUG, "crc=%s\n", b)
                    message = decode_message(self.cur_device_id, self.cur_command, self.cur_msg_data)
                    if message is not None:
                        messages.append(message)
                        if message.command != CC_CMD_CHAIN_SYNC:
                            log(LOGLEVEL_INFO, "Received message: %s", message)
                        self.good_message_received = True
                    else:
                        log(LOGLEVEL_WARNING, "Unknown message ignored (command=0x%02X, data=%s)", self.cur_command, self.cur_msg_data)
                        # unknown message - ignore it
                else:
                    log(LOGLEVEL_ERROR, "!! Error, invalid crc detected! (command=0x%02X, data=%s)", self.cur_command, self.cur_msg_data)
                self._reset()
            self.bytes_received += 1

            # return error if no valid message was received after CC_PROTOCOL_MAX_UNKNOWN_BYTES bytes
            if not self.good_message_received and self.bytes_received >= CC_PROTOCOL_MAX_UNKNOWN_BYTES:
                self._reset(full_reset=True)

        return messages

CC_UPDATES_QUEUE_SIZE   = 50

class CCSlave:
    def __init__(self, response_cb, events_cb, timer, device):
        self.response_cb = response_cb
        self.events_cb = events_cb
        self.us_timer = timer
        # Allocate a bound method reference for use in a callback
        # (see https://docs.micropython.org/en/latest/reference/isr_rules.html#creation-of-python-objects
        #  and https://forum.micropython.org/viewtopic.php?f=2&t=4027&p=23118#p23118)
        self.on_timer_ref = self.on_timer
        self.device = device
        self.updates_queue = deque((), CC_UPDATES_QUEUE_SIZE)
        self.sync_counter = 0
        self.handshake_attempts = 0
        self.handshake_timeout = 0
        self.dev_desc_timeout = 0
        self.comm_state = WAITING_SYNCING
        self.protocol = CCProtocol()

    def clear_updates(self,):
        while len(self.updates_queue) > 0:
            self.updates_queue.popleft()

    def process(self,):
        for actuator in self.device.actuators:
            if actuator.check_for_updates():
                # append update to be sent
                self.updates_queue.append(CCUpdate(actuator.assignment.id, actuator.assignment.value))
                # Broadcast an update event
                self.raise_event(CCEvent(CC_EV_UPDATE, actuator.assignment))

    def timer_set(self, value):
        self.us_timer.init(prescaler=83, period=value, callback=self.timer_cb)

    def timer_cb(self, t):
        self.us_timer.callback(None)
        micropython.schedule(self.on_timer_ref, 0)

    def on_timer(self, _):
        # TODO: [future/optimization] the update message shouldn't be built in the interrupt
        # handler, the time of the frame is being wasted with processing. ideally it has to be
        # cached in the main loop and the interrupt handler is only used to queue the message
        # (send command)
        # log(LOGLEVEL_DEBUG, "In on_timer: %s", len(self.updates_queue))
        if len(self.updates_queue) == 0:
            # the device cannot stay so long time without say hey to mod, it's very needy
            self.sync_counter += 1
            if self.sync_counter >= I_AM_ALIVE_PERIOD:
                self.send_message(CCMsgSync(self.protocol.address, {'type' : CC_SYNC_REGULAR_CYCLE}))
                self.sync_counter = 0
        else:
            # Send the next update
            self.send_message(self.get_updates_message())
            self.sync_counter = 0

    def get_updates_message(self,):
        count = len(self.updates_queue)
        if count > MAX_UPDATES_PER_FRAME:
            count = MAX_UPDATES_PER_FRAME
        return CCMsgUpdates(self.protocol.address,
                            {'updates' : [self.updates_queue.popleft() for i in range(count)]})

    def send_message(self, message):
        if message.command != CC_CMD_CHAIN_SYNC:
            log(LOGLEVEL_INFO, "Sending message: %s", message.__dict__)
        # send sync byte plus message
        self.response_cb(message)

    def raise_event(self, event):
        self.events_cb(event)

    def handle_message(self, message):
        if message is None or self.device is None:
            return

        if isinstance(message, CCMsgSync):
            if message.type == CC_SYNC_SETUP_CYCLE:
                self.clear_updates()
                self.device.clear_assignments()

                self.raise_event(CCEvent(CC_EV_MASTER_RESETED, 0))
                self.comm_state = WAITING_SYNCING

        if self.comm_state == WAITING_SYNCING:
            if isinstance(message, CCMsgSync):
                # check if it's handshake sync cycle
                if message.type == CC_SYNC_HANDSHAKE_CYCLE:
                    # generate handshake
                    handshake = CCHandshake(random.getrandbits(16),
                                            CCVersion(CC_PROTOCOL_MAJOR, CC_PROTOCOL_MINOR, 0),
                                            CCVersion(CC_FIRMWARE_MAJOR, CC_FIRMWARE_MINOR,CC_FIRMWARE_MICRO))
                    # delay the message before send it (the delay value is based on the random id)
                    # this delay should minimize the chance of handshake conflicting
                    # since multiple devices can be connected at the same time
                    time.sleep_us(handshake.get_delay_us())

                    # send message
                    self.send_message(CCMsgHandshake(self.protocol.address, {'handshake' : handshake}))
                    self.device.handshake = handshake
                    self.comm_state = WAITING_HANDSHAKE

        elif self.comm_state == WAITING_HANDSHAKE:
            if isinstance(message, CCMsgHandshakeStatus):
                # check whether master replied to this device
                if self.device.handshake.random_id == message.random_id:
                    # WHY? Is this just because there is no HANDSHAKE_SUCCEEDED event?
                    self.raise_event(CCEvent(CC_EV_HANDSHAKE_FAILED, message.status))

                    # TODO: check status
                    # TODO: handle channel
                    self.protocol.address = message.assigned_device_id
                    log(LOGLEVEL_INFO, "We were assigned device address %d!", self.protocol.address)
                    self.comm_state = WAITING_DEV_DESCRIPTOR
                    self.handshake_attempts = 0
                    self.handshake_timeout = 0

                else:
                    # if doesn't receive handshake reply in 3 attempts returns to previous state
                    self.handshake_attempts += 1
                    if self.handshake_attempts >= 3:
                        self.handshake_attempts = 0
                        self.comm_state = WAITING_SYNCING
            else:
                self.handshake_timeout += 1
                if self.handshake_timeout >= 200:
                    self.handshake_timeout = 0
                    self.comm_state = WAITING_SYNCING

        elif self.comm_state in (WAITING_DEV_DESCRIPTOR, WAITING_DEV_DESCRIPTOR_ACK):
            if isinstance(message, CCMsgDeviceDescriptorRequest):
                if message.type == CC_DEVICE_DESC_REQ and self.comm_state == WAITING_DEV_DESCRIPTOR:
                    # build and send device descriptor message
                    self.send_message(CCMsgDeviceDescriptor(self.protocol.address, {'device' : self.device}))
                    self.comm_state = WAITING_DEV_DESCRIPTOR_ACK

                elif message.type == CC_DEVICE_DESC_ACK and self.comm_state == WAITING_DEV_DESCRIPTOR_ACK:
                    # device descriptor was successfully delivered
                    self.comm_state = LISTENING_REQUESTS
                    log(LOGLEVEL_INFO, "Successfully paired with master!")
                    self.dev_desc_timeout = 0
            else:
                self.dev_desc_timeout += 1
                if self.dev_desc_timeout >= 200:
                    self.dev_desc_timeout = 0
                    self.comm_state = WAITING_SYNCING

        elif self.comm_state == LISTENING_REQUESTS:
            if isinstance(message, CCMsgSync) and message.type == CC_SYNC_REGULAR_CYCLE:
                # device id is used to define the communication frame
                # timer is reset each regular sync message
                self.timer_set(self.protocol.address * CC_FRAME_PERIOD)

            elif isinstance(message, CCMsgDeviceControl):
                # device disabled
                if message.enable:
                    self.raise_event(CCEvent(CC_EV_DEVICE_DISABLED, CC_UPDATE_REQUIRED))

                    # FIXME: properly disable device
                    while True:
                        pass

            elif isinstance(message, CCMsgAssignment):
                self.device.add_assignment(message.assignment)
                self.raise_event(CCEvent(CC_EV_ASSIGNMENT, message.assignment))
                self.send_message(CCMsgAssignmentAcknowledge(self.protocol.address, CC_CMD_ASSIGNMENT, None))

            elif isinstance(message, CCMsgUnassignment):
                actuator_id = self.device.remove_assignment(message.assignment_id)
                self.raise_event(CCEvent(CC_EV_UNASSIGNMENT, actuator_id))
                self.send_message(CCMsgAssignmentAcknowledge(self.protocol.address, CC_CMD_UNASSIGNMENT, None))
