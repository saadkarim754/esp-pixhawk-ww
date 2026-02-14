#ifndef MAVLINK_H
#define MAVLINK_H

#include "mavlink_types.h"
#include "common/mavlink_checksum.h"

// CRC extra bytes for message validation (from MAVLink spec)
// These are used to ensure message format compatibility
static const uint8_t MAVLINK_MESSAGE_CRCS[] = {
    50,  // HEARTBEAT (0)
    124, // SYS_STATUS (1)
    137, // SYSTEM_TIME (2)
    0,   // Reserved (3)
    237, // PING (4)
    217, // CHANGE_OPERATOR_CONTROL (5)
    104, // CHANGE_OPERATOR_CONTROL_ACK (6)
    119, // AUTH_KEY (7)
    0,   // Reserved (8-10)
    0, 0, 
    89,  // SET_MODE (11)
    // Add more as needed...
};

// Get CRC extra for a message ID
static inline uint8_t mavlink_get_crc_extra(uint32_t msgid) {
    if (msgid == 0) return 50;  // HEARTBEAT
    if (msgid < sizeof(MAVLINK_MESSAGE_CRCS)) {
        return MAVLINK_MESSAGE_CRCS[msgid];
    }
    return 0;
}

// Initialize a MAVLink status structure
static inline void mavlink_status_init(mavlink_status_t *status) {
    status->msg_received = 0;
    status->buffer_overrun = 0;
    status->parse_error = 0;
    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
    status->packet_idx = 0;
    status->current_rx_seq = 0;
    status->current_tx_seq = 0;
    status->packet_rx_success_count = 0;
    status->packet_rx_drop_count = 0;
    status->flags = 0;
    status->signature_wait = 0;
    status->signing = NULL;
    status->signing_streams = NULL;
}

// Parse a MAVLink byte stream - returns MAVLINK_FRAMING_OK when complete message received
static inline uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *msg, mavlink_status_t *status) {
    (void)chan;  // Channel not used in simple implementation
    
    uint8_t *payload = (uint8_t *)msg->payload64;
    
    switch (status->parse_state) {
    case MAVLINK_PARSE_STATE_UNINIT:
    case MAVLINK_PARSE_STATE_IDLE:
        if (c == MAVLINK_STX_V2) {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            msg->magic = c;
            msg->len = 0;
            msg->incompat_flags = 0;
            msg->compat_flags = 0;
            status->packet_idx = 0;
            crc_init(&msg->checksum);
        } else if (c == MAVLINK_STX_V1) {
            // MAVLink v1 - handle if needed
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_STX:
        msg->len = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_LENGTH:
        msg->incompat_flags = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
        msg->compat_flags = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
        msg->seq = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_SEQ:
        msg->sysid = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_SYSID:
        msg->compid = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_COMPID:
        msg->msgid = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_MSGID1:
        msg->msgid |= (uint32_t)c << 8;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_MSGID2:
        msg->msgid |= (uint32_t)c << 16;
        crc_accumulate(c, &msg->checksum);
        if (msg->len == 0) {
            // No payload
            crc_accumulate(mavlink_get_crc_extra(msg->msgid), &msg->checksum);
            status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        } else {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
            status->packet_idx = 0;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_MSGID3:
        payload[status->packet_idx++] = c;
        crc_accumulate(c, &msg->checksum);
        if (status->packet_idx >= msg->len) {
            // Add CRC extra byte
            crc_accumulate(mavlink_get_crc_extra(msg->msgid), &msg->checksum);
            status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
        msg->ck[0] = c;
        if ((msg->checksum & 0xFF) != c) {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC;
            status->parse_error++;
        } else {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_CRC1:
        msg->ck[1] = c;
        if ((msg->checksum >> 8) != c) {
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            status->packet_rx_drop_count++;
            return MAVLINK_FRAMING_BAD_CRC;
        }
        // Check for signature (if incompat_flags has bit 0 set)
        if (msg->incompat_flags & 0x01) {
            status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;
            status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;
        } else {
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            status->packet_rx_success_count++;
            return MAVLINK_FRAMING_OK;
        }
        break;
        
    case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
        status->signature_wait--;
        if (status->signature_wait == 0) {
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            status->packet_rx_success_count++;
            return MAVLINK_FRAMING_OK;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_BAD_CRC:
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        status->packet_rx_drop_count++;
        return MAVLINK_FRAMING_BAD_CRC;
        
    default:
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        break;
    }
    
    return MAVLINK_FRAMING_INCOMPLETE;
}

// Decode heartbeat message from a MAVLink message
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t *msg, mavlink_heartbeat_t *heartbeat) {
    const uint8_t *payload = (const uint8_t *)msg->payload64;
    heartbeat->custom_mode = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
    heartbeat->type = payload[4];
    heartbeat->autopilot = payload[5];
    heartbeat->base_mode = payload[6];
    heartbeat->system_status = payload[7];
    heartbeat->mavlink_version = payload[8];
}

// Pack a heartbeat message into buffer and return length
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id,
                                                   uint8_t *buf, uint8_t type, uint8_t autopilot,
                                                   uint8_t base_mode, uint32_t custom_mode,
                                                   uint8_t system_status) {
    uint16_t crc;
    
    buf[0] = MAVLINK_STX_V2;   // Magic
    buf[1] = 9;                // Payload length
    buf[2] = 0;                // Incompat flags
    buf[3] = 0;                // Compat flags
    buf[4] = 0;                // Seq (will be filled by caller if needed)
    buf[5] = system_id;        // System ID
    buf[6] = component_id;     // Component ID
    buf[7] = 0;                // Message ID low byte (HEARTBEAT = 0)
    buf[8] = 0;                // Message ID mid byte
    buf[9] = 0;                // Message ID high byte
    
    // Payload
    buf[10] = custom_mode & 0xFF;
    buf[11] = (custom_mode >> 8) & 0xFF;
    buf[12] = (custom_mode >> 16) & 0xFF;
    buf[13] = (custom_mode >> 24) & 0xFF;
    buf[14] = type;
    buf[15] = autopilot;
    buf[16] = base_mode;
    buf[17] = system_status;
    buf[18] = 2;  // MAVLink version 2
    
    // Calculate CRC
    crc_init(&crc);
    for (int i = 1; i <= 18; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(MAVLINK_MSG_ID_HEARTBEAT_CRC, &crc);  // CRC extra
    
    buf[19] = crc & 0xFF;
    buf[20] = (crc >> 8) & 0xFF;
    
    return 21;  // Total packet length
}

#endif // MAVLINK_H
