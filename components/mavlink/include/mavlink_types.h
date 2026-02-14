#ifndef MAVLINK_TYPES_H
#define MAVLINK_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// MAVLink 2 protocol constants
#define MAVLINK_STX_V2 0xFD
#define MAVLINK_STX_V1 0xFE
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_SIGNATURE_BLOCK_LEN 13
#define MAVLINK_CORE_HEADER_LEN 9
#define MAVLINK_CORE_HEADER_MAVLINK1_LEN 5
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1)
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)
#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN)

// MAVLink message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0

// MAVLink component IDs
#define MAV_COMP_ID_AUTOPILOT1 1
#define MAV_COMP_ID_ONBOARD_COMPUTER 191

// MAVLink system types
#define MAV_TYPE_QUADROTOR 2
#define MAV_TYPE_ONBOARD_CONTROLLER 18

// MAVLink autopilot types
#define MAV_AUTOPILOT_GENERIC 0
#define MAV_AUTOPILOT_ARDUPILOTMEGA 3
#define MAV_AUTOPILOT_PX4 12

// MAVLink modes
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define MAV_MODE_PREFLIGHT 0

// MAVLink states
#define MAV_STATE_UNINIT 0
#define MAV_STATE_BOOT 1
#define MAV_STATE_CALIBRATING 2
#define MAV_STATE_STANDBY 3
#define MAV_STATE_ACTIVE 4
#define MAV_STATE_CRITICAL 5
#define MAV_STATE_EMERGENCY 6
#define MAV_STATE_POWEROFF 7
#define MAV_STATE_FLIGHT_TERMINATION 8

// Parse states
typedef enum {
    MAVLINK_PARSE_STATE_UNINIT = 0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID1,
    MAVLINK_PARSE_STATE_GOT_MSGID2,
    MAVLINK_PARSE_STATE_GOT_MSGID3,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1,
    MAVLINK_PARSE_STATE_GOT_BAD_CRC,
    MAVLINK_PARSE_STATE_SIGNATURE_WAIT
} mavlink_parse_state_t;

typedef enum {
    MAVLINK_FRAMING_INCOMPLETE = 0,
    MAVLINK_FRAMING_OK = 1,
    MAVLINK_FRAMING_BAD_CRC = 2,
    MAVLINK_FRAMING_BAD_SIGNATURE = 3
} mavlink_framing_t;

// MAVLink message structure
typedef struct __mavlink_message {
    uint16_t checksum;      // Checksum
    uint8_t magic;          // Protocol magic marker
    uint8_t len;            // Length of payload
    uint8_t incompat_flags; // Incompatibility flags
    uint8_t compat_flags;   // Compatibility flags
    uint8_t seq;            // Sequence number
    uint8_t sysid;          // System ID
    uint8_t compid;         // Component ID
    uint32_t msgid:24;      // Message ID (24 bits)
    uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN + 7) / 8];
    uint8_t ck[2];          // CRC bytes
    uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
} mavlink_message_t;

// MAVLink status structure
typedef struct __mavlink_status {
    uint8_t msg_received;
    uint8_t buffer_overrun;
    uint8_t parse_error;
    mavlink_parse_state_t parse_state;
    uint8_t packet_idx;
    uint8_t current_rx_seq;
    uint8_t current_tx_seq;
    uint16_t packet_rx_success_count;
    uint16_t packet_rx_drop_count;
    uint8_t flags;
    uint8_t signature_wait;
    struct __mavlink_signing *signing;
    struct __mavlink_signing_streams *signing_streams;
} mavlink_status_t;

// Heartbeat message structure
typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;   // Custom mode
    uint8_t type;           // Vehicle type
    uint8_t autopilot;      // Autopilot type
    uint8_t base_mode;      // Base mode
    uint8_t system_status;  // System status
    uint8_t mavlink_version; // MAVLink version
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50

#endif // MAVLINK_TYPES_H
