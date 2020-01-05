CC_SYNC_BYTE                = 0xA7
CC_BROADCAST_ADDRESS        = 0

MSG_STATE_IDLE              = 0
MSG_STATE_READ_ADDRESS      = 1
MSG_STATE_READ_COMMAND      = 2
MSG_STATE_READ_DATALEN_LSB  = 3
MSG_STATE_READ_DATALEN_MSB  = 4
MSG_STATE_READ_DATA         = 5
MSG_STATE_READ_CRC          = 6

CC_PROTOCOL_MAX_UNKNOWN_BYTES   = 3000

I_AM_ALIVE_PERIOD       = 50      # in sync cycles

# define firmware version
CC_FIRMWARE_MAJOR       = 0
CC_FIRMWARE_MINOR       = 1
CC_FIRMWARE_MICRO       = 0

# protocol version
CC_PROTOCOL_MAJOR       = 0
CC_PROTOCOL_MINOR       = 5

# define serial communication baud rate and frame period
CC_BAUD_RATE            = 500000      # in bps
CC_FRAME_PERIOD         = 1000        # in us

# define versions as strings
CC_PROTOCOL_VERSION     = "%d.%d" % (CC_PROTOCOL_MAJOR, CC_PROTOCOL_MINOR)
CC_FIRMWARE_VERSION     = "%d.%d.%d" % (CC_FIRMWARE_MAJOR, CC_FIRMWARE_MINOR, CC_FIRMWARE_MICRO)

CC_MSG_HEADER_SIZE      = 4

# fallback option for serial baud rate
CC_BAUD_RATE_FALLBACK   = 115200

# calculate how many bytes fit inside the frame
# FIXME: in the future change CC_BAUD_RATE_FALLBACK to CC_BAUD_RATE
BYTES_PER_FRAME         = ((CC_FRAME_PERIOD * CC_BAUD_RATE_FALLBACK) / (1000000 * 10))

# maximum number of updates which fit inside the frame
# the update command has 6 bytes of overhead and each update data need 5 bytes
MAX_UPDATES_PER_FRAME   = ((BYTES_PER_FRAME - 6) / 5)

# size of the handshake message in bytes
HANDSHAKE_SIZE_BYTES    = (CC_MSG_HEADER_SIZE + 2 + 7)

# size of the handshake message in microseconds
# FIXME: in the future change CC_BAUD_RATE_FALLBACK to CC_BAUD_RATE
HANDSHAKE_SIZE          = ((10 * 1000000 * HANDSHAKE_SIZE_BYTES) / CC_BAUD_RATE_FALLBACK)

# master will wait a period of 8 devices frames to receive handshakes
HANDSHAKES_PERIOD       = (((8 * CC_FRAME_PERIOD) / HANDSHAKE_SIZE) * HANDSHAKE_SIZE)

# Handshake statuses
CC_HANDSHAKE_OK         = 0
CC_UPDATE_AVAILABLE     = 1
CC_UPDATE_REQUIRED      = 2

CC_STATUSES = (CC_HANDSHAKE_OK,
               CC_UPDATE_AVAILABLE,
               CC_UPDATE_REQUIRED)

# Control chain commands
CC_CMD_CHAIN_SYNC       = 0
CC_CMD_HANDSHAKE        = 1
CC_CMD_DEV_CONTROL      = 2
CC_CMD_DEV_DESCRIPTOR   = 3
CC_CMD_ASSIGNMENT       = 4
CC_CMD_DATA_UPDATE      = 5
CC_CMD_UNASSIGNMENT     = 6

CC_COMMANDS = (CC_CMD_CHAIN_SYNC,
               CC_CMD_HANDSHAKE,
               CC_CMD_DEV_CONTROL,
               CC_CMD_DEV_DESCRIPTOR,
               CC_CMD_ASSIGNMENT,
               CC_CMD_DATA_UPDATE,
               CC_CMD_UNASSIGNMENT)

# Control chain events
CC_EV_HANDSHAKE_FAILED  = 0
CC_EV_ASSIGNMENT        = 1
CC_EV_UNASSIGNMENT      = 2
CC_EV_UPDATE            = 3
CC_EV_DEVICE_DISABLED   = 4
CC_EV_MASTER_RESETED    = 5

CC_EVENTS = (CC_EV_HANDSHAKE_FAILED,
             CC_EV_ASSIGNMENT,
             CC_EV_UNASSIGNMENT,
             CC_EV_UPDATE,
             CC_EV_DEVICE_DISABLED,
             CC_EV_MASTER_RESETED)

# Assignment modes
CC_MODE_TOGGLE          = 0x01
CC_MODE_TRIGGER         = 0x02
CC_MODE_OPTIONS         = 0x04
CC_MODE_REAL            = 0x10
CC_MODE_INTEGER         = 0x20
CC_MODE_FEEDBACK        = 0x40

CC_ASSIGNMENT_MODES = (CC_MODE_TOGGLE,
                       CC_MODE_TRIGGER,
                       CC_MODE_OPTIONS,
                       CC_MODE_REAL,
                       CC_MODE_INTEGER,
                       CC_MODE_FEEDBACK)

# Actuator types
CC_ACTUATOR_TYPE_UNKNOWN= -1
CC_ACTUATOR_CONTINUOUS  = 0
CC_ACTUATOR_DISCRETE    = 1
CC_ACTUATOR_SWITCH      = 2
CC_ACTUATOR_MOMENTARY   = 3

CC_ACTUATOR_TYPES = (CC_ACTUATOR_CONTINUOUS,
                     CC_ACTUATOR_DISCRETE,
                     CC_ACTUATOR_SWITCH,
                     CC_ACTUATOR_MOMENTARY)

# Device descriptor actions
CC_DEVICE_DESC_REQ      = 0
CC_DEVICE_DESC_ACK      = 1

CC_DEVICE_DESC_ACTIONS = (CC_DEVICE_DESC_REQ,
                          CC_DEVICE_DESC_ACK)

# communication states
WAITING_SYNCING         = 0
WAITING_HANDSHAKE       = 1
WAITING_DEV_DESCRIPTOR  = 2
LISTENING_REQUESTS      = 3

CC_COMMUNICATION_STATES= (WAITING_SYNCING,
                          WAITING_HANDSHAKE,
                          WAITING_DEV_DESCRIPTOR,
                          LISTENING_REQUESTS)

# sync message cycles definition
CC_SYNC_SETUP_CYCLE     = 0
CC_SYNC_REGULAR_CYCLE   = 1
CC_SYNC_HANDSHAKE_CYCLE = 2

CC_SYNC_CYCLES = (CC_SYNC_SETUP_CYCLE, CC_SYNC_REGULAR_CYCLE, CC_SYNC_HANDSHAKE_CYCLE)

