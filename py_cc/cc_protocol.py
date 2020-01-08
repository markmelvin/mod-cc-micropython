import urandom as random
import ustruct as struct
from ucollections import deque
import micropython
import time

from .crc8 import crc8
from .cc_constants import *

random.seed(1)
micropython.alloc_emergency_exception_buf(100)

SIZEOF_FLOAT = struct.calcsize('f')
SIZEOF_USHORT = struct.calcsize('H')
SIZEOF_UINT = struct.calcsize('I')

LOGLEVEL_CRITICAL   = 50
LOGLEVEL_ERROR      = 40
LOGLEVEL_WARNING    = 30
LOGLEVEL_INFO       = 20
LOGLEVEL_DEBUG      = 10
LOGLEVEL_NOTSET     = 0

_loglevel = LOGLEVEL_INFO

############################################
# Helper functions
############################################
def get_sized_string(data):
    size = data[0]
    return bytes(data[1:size+1]).decode()

def get_serialized_string(s, max_size=255):
    size = min(len(s), max_size)
    return [size] + [w for w in ("%s" % s[:size]).encode()]

def log(level, msg, *args):
    if level >= _loglevel:
        msg = str(msg)
        if len(args):
            msg = msg % args
        print(msg)

def set_log_level(level):
    global _loglevel
    _loglevel = int(level)


############################################
# Utility classes
############################################
class CCVersion:
    def __init__(self, major, minor, micro):
        self.major = major
        self.minor = minor
        self.micro = micro

    def get_string(self,):
        s = '%d' % self.major
        for version in [self.minor, self.micro]:
            if version is not None:
                s += '.%d' % version
        return s

    def __repr__(self,):
        return self.get_string()


class CCListItem:
    def __init__(self, label, value):
        self.label = label
        self.value = value


class CCAssignment:
    def __init__(self, _id, actuator_id, label, value, _min, _max, _def,
                 mode, steps, unit, list_items=None):
        self.id = _id
        self.actuator_id = actuator_id
        self.label = label
        self.value = value
        self.min = _min
        self.max = _max
        self._def = _def
        self.mode = mode
        self.steps = steps
        self.unit = unit
        self.list_items = [] if list_items is None else list_items


class CCUpdate:
    def __init__(self, assignment_id, value):
        self.assignment_id = assignment_id
        self.value = value


class CCEvent:
    def __init__(self, _id, data):
        self.id = _id
        self.data = data


class CCHandshake:
    def __init__(self, random_id, protocol_version, firmware_version):
        self.random_id = random_id
        self.protocol_version = protocol_version
        self.firmware_version = firmware_version

    def get_delay_us(self,):
        return int( ((self.random_id % HANDSHAKES_PERIOD) / HANDSHAKE_SIZE) * HANDSHAKE_SIZE )


class CCActuator:
    def __init__(self, name, _type, _min, _max, supported_modes, max_assignments=None):
        self.name = name
        self.min = _min
        self.max = _max
        self.type = _type
        self.supported_modes = supported_modes
        self.max_assignments = max_assignments
        self.value = 0.0
        self.lock = False
        self.assignments = []


class CCDevice:
    def __init__(self, name, uri, actuators=None):
        self.name = name
        self.uri = uri
        self.actuators = [] if actuators is None else actuators
        self.handshake = None
        self.assignments = []

    def add_actuator(self, actuator):
        pass

    def add_assignment(self, assignment):
        pass

    def remove_assignment(self, assignment_id):
        pass

    def clear_assignments(self,):
        pass

    def __repr__(self,):
        return self.__dict__


############################################
# Message classes
############################################
class CCMsg:
    '''Base class for a Control Chain message.'''
    def __init__(self, device_id, data):
        '''
        Create a message. If data is a dictionary, those key/value
        pairs will be set on the object.
        Otherwise, it is assumed to be an array of bytes and will be
        passed to self._decode(data) to decode the object structure
        from the raw bytes.
        '''
        self.device_id = device_id
        self.command = self.get_command()
        if type(data) is dict:
            _ = [setattr(self, key, value) for key, value in data.items()]
        else:
            self._decode(data)

    def get_command(self,):
        raise NotImplementedError("Not implemented!")

    def _decode(self, data):
        '''
        Populate this class from its internal data bytes array
        '''
        pass

    def _get_data_payload(self,):
        '''
        Get the data payload for this command
        '''
        return []

    def get_tx_bytes(self,):
        '''
        Get the data bytes to be transmitted on the wire for this message
        '''
        buffer = []
        data = self._get_data_payload()

        # Calculate header
        buffer.append(self.device_id)
        buffer.append(self.command)
        data_len = len(data)
        buffer.append((data_len >> 0) & 0xFF)
        buffer.append((data_len >> 8) & 0xFF)

        # Add data
        if data_len > 0:
            buffer.extend(data)

        # calculate crc
        buffer.append(crc8(buffer))
        return buffer


class CCMsgHandshakeStatus(CCMsg):
    def get_command(self,):
        return CC_CMD_HANDSHAKE

    def _decode(self, data):
        offset = SIZEOF_USHORT
        assert len(data) == (offset + 2)
        self.random_id = struct.unpack('H', bytes(data))[0]
        self.status = data[offset]
        self.assigned_device_id = data[offset+1]
        # self.channel = data[offset+2]     # THIS IS NOT SUPPORTED IN THE CURRENT PROTOCOL!

    def _get_data_payload(self,):
        return [w for w in struct.pack('H', self.random_id)] + \
                    [self.status,
                     self.assigned_device_id,
                     #self.channel,         # THIS IS NOT SUPPORTED IN THE CURRENT PROTOCOL!
                    ]


class CCMsgHandshake(CCMsg):
    def get_command(self,):
        return CC_CMD_HANDSHAKE

    def _decode(self, data):
        has_protocol_micro = (len(data) == SIZEOF_USHORT + 6)
        offset = SIZEOF_USHORT
        assert len(data) == (offset + (6 if has_protocol_micro else 5))
        random_id = struct.unpack('H', bytes(data))[0]
        protocol_major = data[offset]
        protocol_minor = data[offset+1]
        if has_protocol_micro:
            protocol_micro = data[offset+2]
            offset += 3
        else:
            protocol_micro = None
            offset += 2
        firmware_major = data[offset]
        firmware_minor = data[offset+1]
        firmware_micro = data[offset+2]
        self.handshake = CCHandshake(random_id,
                                     CCVersion(protocol_major, protocol_minor, protocol_micro),
                                     CCVersion(firmware_major, firmware_minor, firmware_micro))

    def _get_data_payload(self,):
        return [w for w in struct.pack('H', self.handshake.random_id)] + \
                    [self.handshake.protocol_version.major,
                     self.handshake.protocol_version.minor,
                     #self.handshake.protocol_version.micro,    # The C version doesn't send this
                     self.handshake.firmware_version.major,
                     self.handshake.firmware_version.minor,
                     self.handshake.firmware_version.micro]


class CCMsgDeviceControl(CCMsg):
    def get_command(self,):
        return CC_CMD_DEV_CONTROL

    def _decode(self, data):
        assert len(data) == 1
        self.enable = bool(data[0])

    def _get_data_payload(self,):
        return [int(self.enable)]


class CCMsgDeviceDescriptorRequest(CCMsg):
    def get_command(self,):
        return CC_CMD_DEV_DESCRIPTOR

    def _decode(self, data):
        assert len(data) == 1
        self.type = data[0]

    def _get_data_payload(self,):
        return [self.type]


class CCMsgAssignment(CCMsg):
    def get_command(self,):
        return CC_CMD_ASSIGNMENT

    def _decode(self, data):
        _id = data[0]
        actuator_id = data[1]
        offset = 2
        label = get_sized_string(data[offset:])
        offset += len(label)+1
        value = struct.unpack('f', bytes(data[offset:]))[0]
        offset += SIZEOF_FLOAT
        _min = struct.unpack('f', bytes(data[offset:]))[0]
        offset += SIZEOF_FLOAT
        _max = struct.unpack('f', bytes(data[offset:]))[0]
        offset += SIZEOF_FLOAT
        _def = struct.unpack('f', bytes(data[offset:]))[0]
        offset += SIZEOF_FLOAT
        mode = struct.unpack('I', bytes(data[offset:]))[0]
        offset += SIZEOF_UINT
        steps = struct.unpack('H', bytes(data[offset:]))[0]
        offset += SIZEOF_USHORT
        unit = get_sized_string(data[offset:])
        offset += len(unit)+1
        list_items = []
        num_items = data[offset]
        offset += 1
        for i in range(num_items):
            item_label = get_sized_string(data[offset:])
            offset += len(item_label)+1
            item_value = struct.unpack('f', bytes(data[offset:]))[0]
            offset += SIZEOF_FLOAT
            list_items.append(CCListItem(item_label, item_value))

        self.assignment = CCAssignment(_id, actuator_id, label, value,
                                       _min, _max, _def, mode, steps,
                                       unit, list_items=list_items)

    def _get_data_payload(self,):
        buffer = [self.assignment.id, self.assignment.actuator_id] + \
        get_serialized_string(self.assignment.label, max_size=16) + \
        [w for w in struct.pack('f', self.assignment.value)] + \
        [w for w in struct.pack('f', self.assignment.min)] + \
        [w for w in struct.pack('f', self.assignment.max)] + \
        [w for w in struct.pack('f', self.assignment._def)] + \
        [w for w in struct.pack('I', self.assignment.mode)] + \
        [w for w in struct.pack('H', self.assignment.steps)] + \
        get_serialized_string(self.assignment.unit, max_size=16) + \
        [len(self.assignment.list_items)]
        for item in self.assignment.list_items:
            buffer += get_serialized_string(item.label, max_size=16) + \
                      [w for w in struct.pack('f', item.value)]
        return buffer


class CCMsgUnassignment(CCMsg):
    def get_command(self,):
        return CC_CMD_UNASSIGNMENT

    def _decode(self, data):
        assert len(data) == 1
        self.assignment_id = data[0]

    def _get_data_payload(self,):
        return [self.assignment_id]


class CCMsgSync(CCMsg):
    def get_command(self,):
        return CC_CMD_CHAIN_SYNC

    def _decode(self, data):
        assert len(data) == 1
        self.type = data[0]

    def _get_data_payload(self,):
        return [self.type]


class CCMsgUpdates(CCMsg):
    def get_command(self,):
        return CC_CMD_DATA_UPDATE

    def _decode(self, data):
        self.updates = []
        num_updates = data[0]
        offset = 1
        for i in range(num_updates):
            assignment_id = data[offset]
            value = struct.unpack('f', bytes(data[offset+1:]))[0]
            offset += 1 + SIZEOF_FLOAT
            self.updates.append(CCUpdate(assignment_id, value))

    def _get_data_payload(self,):
        data = [len(self.updates)]

        for update in self.updates:
            data.append(update.assignment_id)
            data.extend([w for w in struct.pack('f', update.value)])
        return data


class CCMsgAssignmentAcknowledge(CCMsg):
    def __init__(self, device_id, assignment_cmd_type, data):
        self.cmd_type = assignment_cmd_type
        super().__init__(device_id, data)

    def get_command(self,):
        return self.cmd_type


class CCMsgDeviceDescriptor(CCMsg):
    def get_command(self,):
        return CC_CMD_DEV_DESCRIPTOR

    def _decode(self, data):
        offset = 0
        uri = get_sized_string(data[offset:])
        offset += len(uri)+1
        name = get_sized_string(data[offset:])
        offset += len(name)+1
        num_actuators = data[offset]
        offset += 1
        actuators = []
        for i in range(num_actuators):
            actuator_name = get_sized_string(data[offset:])
            offset += len(actuator_name)+1
            supported_modes = struct.unpack('I', bytes(data[offset:]))[0]
            offset += SIZEOF_UINT
            max_assignments = data[offset]
            offset += 1
            actuators.append(CCActuator(actuator_name,
                                        CC_ACTUATOR_TYPE_UNKNOWN,       # Unknown for this type of message
                                        0.0, 1.0, supported_modes,      # Unknown for this type of message
                                        max_assignments=max_assignments))
        self.device = CCDevice(name, uri, actuators=actuators)

    def _get_data_payload(self,):
        # serialize uri
        data = get_serialized_string(self.device.uri)
        # serialize name
        data.extend(get_serialized_string(self.device.name))
        # number of actuators
        data.append(len(self.device.actuators))

        # serialize actuators data
        for actuator in self.device.actuators:
            # actuator name
            data.extend(get_serialized_string(actuator.name, max_size=16))
            # supported modes
            data.extend([w for w in struct.pack('I', actuator.supported_modes)])
            data.append(actuator.max_assignments)

        return data


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
            print("An error occurred decoding message (%s)." % str(data))
    return None


class CCProtocol:
    '''
    Class encapsulating the Control Chain protocol. This protocol consists
    of messages that start with a sync byte, then is followed by a message
    header, optional data payload, and a CRC calculated over the message
    header and data payload. This class supports receiving messages that may
    be split across multiple calls to the receive_data(data) method.
    '''
    def __init__(self, address=CC_BROADCAST_ADDRESS, message_rcv_cb=None):
        self.message_rcv_cb = message_rcv_cb
        self.address = address
        self._reset(full_reset=True)

    def _reset(self, full_reset=False):
        self.msg_state = MSG_STATE_IDLE
        self.cur_header = [0]*CC_MSG_HEADER_SIZE
        self.cur_device_id = -1
        self.cur_command = -1
        self.cur_data_size = -1
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
        log(LOGLEVEL_DEBUG, "CCProtocol.receive_data(): %s \n(%s)", data, self.__dict__)
        for i in range(len(data)):
            b = data[i]

            if self.msg_state > MSG_STATE_IDLE and self.msg_state < MSG_STATE_READ_DATA:
                self.cur_header[self.msg_state - 1] = b

            if self.msg_state == MSG_STATE_IDLE:
                # sync byte
                if b == CC_SYNC_BYTE:
                    self.msg_state = MSG_STATE_READ_ADDRESS

            elif self.msg_state == MSG_STATE_READ_ADDRESS:
                # device _id
                # check if it's messaging this device or is a broadcast message
                if b == CC_BROADCAST_ADDRESS or self.address == b or self.address == CC_BROADCAST_ADDRESS:
                    self.cur_device_id = b
                    self.msg_state = MSG_STATE_READ_COMMAND
                else:
                    # message is not for us
                    self._reset()

            elif self.msg_state == MSG_STATE_READ_COMMAND:
                # command
                self.cur_command = b
                self.msg_state = MSG_STATE_READ_DATALEN_LSB

            elif self.msg_state == MSG_STATE_READ_DATALEN_LSB:
                # data size LSB
                self.cur_data_size = b
                self.msg_state = MSG_STATE_READ_DATALEN_MSB

            elif self.msg_state == MSG_STATE_READ_DATALEN_MSB:
                # data size MSB
                self.cur_data_size = (b << 8) | self.cur_data_size
                self.msg_state = MSG_STATE_READ_CRC if self.cur_data_size == 0 else MSG_STATE_READ_DATA

            elif self.msg_state == MSG_STATE_READ_DATA:
                # data
                self.cur_msg_data.append(b)

                if len(self.cur_msg_data) == self.cur_data_size:
                    self.msg_state = MSG_STATE_READ_CRC

            elif self.msg_state == MSG_STATE_READ_CRC:
                # crc
                if crc8(self.cur_header + self.cur_msg_data) == b:
                    message = decode_message(self.cur_device_id, self.cur_command, self.cur_msg_data)
                    if message is not None:
                        messages.append(message)
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

        if len(messages) > 0:
            for message in messages:
                # Call the callback for each message if there is one
                if self.message_rcv_cb is not None and callable(self.message_rcv_cb):
                    self.message_rcv_cb(message)

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
        self.protocol = CCProtocol(message_rcv_cb=self.on_message_received)

    def clear_updates(self,):
        while len(self.updates_queue) > 0:
            self.updates_queue.popleft()

    def process(self,):
        pass
        # log(LOGLEVEL_DEBUG, "Process!")

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
        log(LOGLEVEL_DEBUG, "Sending message: %s", message.__dict__)
        # send sync byte plus message
        self.response_cb([CC_SYNC_BYTE] + message.get_tx_bytes())

    def raise_event(self, event):
        self.events_cb(event)

    def on_message_received(self, message):
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

            elif isinstance(message, (CCMsgAssignment, CCMsgUnassignment)):
                self.acknowledge_assignment(message)

    def acknowledge_assignment(self, message):
        if isinstance(message, CCMsgAssignment):
            cmd = CC_EV_ASSIGNMENT
            self.device.add_assignment(message.assignment)
        elif isinstance(message, CCMsgUnassignment):
            cmd = CC_EV_UNASSIGNMENT
            self.device.remove_assignment(message.assignment.id)

        self.raise_event(CCEvent(cmd, message.assignment))
        self.send_message(CCMsgAssignmentAcknowledge(self.protocol.address, cmd, None))
