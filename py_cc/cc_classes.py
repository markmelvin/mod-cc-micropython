import ustruct as struct
from .crc8 import crc8
from .cc_constants import *

SIZEOF_FLOAT = struct.calcsize('f')
SIZEOF_USHORT = struct.calcsize('H')
SIZEOF_UINT = struct.calcsize('I')

############################################
# Helper functions
############################################
def get_sized_string(data):
    size = data[0]
    return bytes(data[1:size+1]).decode()

def get_serialized_string(s, max_size=255):
    size = min(len(s), max_size)
    return [size] + [w for w in ("%s" % s[:size]).encode()]

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
        self.list_index = 0

    def update_from_actuator(self, actuator):
        if actuator.type == CC_ACTUATOR_MOMENTARY:
            if self.mode & CC_MODE_OPTIONS:
                self.list_index += 1
                if self.list_index >= len(self.list_items):
                    self.list_index = 0
                self.value = self.list_items[self.list_index].value

            else:
                if self.mode & CC_MODE_TRIGGER:
                    self.value = self.max
                elif self.mode & CC_MODE_TOGGLE:
                    self.value = 1.0 - self.value

        elif actuator.type == CC_ACTUATOR_CONTINUOUS:
            pass

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
    def __init__(self, _id, name, _type, _min, _max, supported_modes, max_assignments=None):
        self.id = _id
        self.name = name
        self.min = _min
        self.max = _max
        self.type = _type
        self.supported_modes = supported_modes
        self.max_assignments = max_assignments
        self.value = 0.0
        self.last_value = 0.0
        self.lock = False
        self.assignment = None

    def check_for_updates(self,):
        updated = False
        if self.assignment is not None:
            if self.type == CC_ACTUATOR_MOMENTARY:
                if self.value > 0.0:
                    if not self.lock:
                        self.lock = True
                        self.assignment.update_from_actuator(self)
                        updated = True
                else:
                    self.lock = False

            elif self.type == CC_ACTUATOR_CONTINUOUS:
                pass
                # # check if actuator value has changed the minimum required value
                # delta = (self.max + self.min) * 0.01
                # if abs(self.last_value - self.value) >= delta:
                #     self.last_value = self.value
                #     self.assignment.update_from_actuator(self)
                #     updated = True

        return updated

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
            # offset += SIZEOF_FLOAT
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
            actuators.append(CCActuator(i,
                                        actuator_name,
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