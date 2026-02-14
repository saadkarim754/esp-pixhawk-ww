#ifndef MAVLINK_H
#define MAVLINK_H

#include "mavlink_types.h"
#include "common/mavlink_checksum.h"

// CRC extra bytes for MAVLink message validation
// From: https://mavlink.io/en/guide/serialization.html#crc_extra
// Get CRC extra for a message ID
static inline uint8_t mavlink_get_crc_extra(uint32_t msgid) {
    switch (msgid) {
        case 0:   return 50;   // HEARTBEAT
        case 1:   return 124;  // SYS_STATUS
        case 2:   return 137;  // SYSTEM_TIME
        case 4:   return 237;  // PING
        case 5:   return 217;  // CHANGE_OPERATOR_CONTROL
        case 6:   return 104;  // CHANGE_OPERATOR_CONTROL_ACK
        case 7:   return 119;  // AUTH_KEY
        case 11:  return 89;   // SET_MODE
        case 20:  return 214;  // PARAM_REQUEST_READ
        case 21:  return 159;  // PARAM_REQUEST_LIST
        case 22:  return 220;  // PARAM_VALUE
        case 23:  return 168;  // PARAM_SET
        case 24:  return 24;   // GPS_RAW_INT
        case 25:  return 23;   // GPS_STATUS
        case 26:  return 170;  // SCALED_IMU
        case 27:  return 144;  // RAW_IMU
        case 28:  return 67;   // RAW_PRESSURE
        case 29:  return 115;  // SCALED_PRESSURE
        case 30:  return 39;   // ATTITUDE
        case 31:  return 246;  // ATTITUDE_QUATERNION
        case 32:  return 185;  // LOCAL_POSITION_NED
        case 33:  return 104;  // GLOBAL_POSITION_INT
        case 34:  return 237;  // RC_CHANNELS_SCALED
        case 35:  return 244;  // RC_CHANNELS_RAW
        case 36:  return 222;  // SERVO_OUTPUT_RAW
        case 37:  return 212;  // MISSION_REQUEST_PARTIAL_LIST
        case 38:  return 9;    // MISSION_WRITE_PARTIAL_LIST
        case 39:  return 254;  // MISSION_ITEM
        case 40:  return 230;  // MISSION_REQUEST
        case 41:  return 28;   // MISSION_SET_CURRENT
        case 42:  return 28;   // MISSION_CURRENT
        case 43:  return 132;  // MISSION_REQUEST_LIST
        case 44:  return 221;  // MISSION_COUNT
        case 45:  return 232;  // MISSION_CLEAR_ALL
        case 46:  return 11;   // MISSION_ITEM_REACHED
        case 47:  return 153;  // MISSION_ACK
        case 48:  return 41;   // SET_GPS_GLOBAL_ORIGIN
        case 49:  return 39;   // GPS_GLOBAL_ORIGIN
        case 50:  return 78;   // PARAM_MAP_RC
        case 51:  return 196;  // MISSION_REQUEST_INT
        case 54:  return 15;   // SAFETY_SET_ALLOWED_AREA
        case 55:  return 3;    // SAFETY_ALLOWED_AREA
        case 61:  return 167;  // ATTITUDE_QUATERNION_COV
        case 62:  return 183;  // NAV_CONTROLLER_OUTPUT
        case 63:  return 119;  // GLOBAL_POSITION_INT_COV
        case 64:  return 191;  // LOCAL_POSITION_NED_COV
        case 65:  return 118;  // RC_CHANNELS
        case 66:  return 148;  // REQUEST_DATA_STREAM
        case 67:  return 21;   // DATA_STREAM
        case 69:  return 243;  // MANUAL_CONTROL
        case 70:  return 124;  // RC_CHANNELS_OVERRIDE
        case 73:  return 38;   // MISSION_ITEM_INT
        case 74:  return 20;   // VFR_HUD
        case 75:  return 158;  // COMMAND_INT
        case 76:  return 152;  // COMMAND_LONG
        case 77:  return 143;  // COMMAND_ACK
        case 81:  return 106;  // MANUAL_SETPOINT
        case 82:  return 49;   // SET_ATTITUDE_TARGET
        case 83:  return 22;   // ATTITUDE_TARGET
        case 84:  return 143;  // SET_POSITION_TARGET_LOCAL_NED
        case 85:  return 140;  // POSITION_TARGET_LOCAL_NED
        case 86:  return 5;    // SET_POSITION_TARGET_GLOBAL_INT
        case 87:  return 150;  // POSITION_TARGET_GLOBAL_INT
        case 89:  return 231;  // LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
        case 90:  return 183;  // HIL_STATE
        case 91:  return 63;   // HIL_CONTROLS
        case 92:  return 54;   // HIL_RC_INPUTS_RAW
        case 93:  return 47;   // HIL_ACTUATOR_CONTROLS
        case 100: return 175;  // OPTICAL_FLOW
        case 101: return 102;  // GLOBAL_VISION_POSITION_ESTIMATE
        case 102: return 158;  // VISION_POSITION_ESTIMATE
        case 103: return 208;  // VISION_SPEED_ESTIMATE
        case 104: return 56;   // VICON_POSITION_ESTIMATE
        case 105: return 93;   // HIGHRES_IMU
        case 106: return 138;  // OPTICAL_FLOW_RAD
        case 107: return 108;  // HIL_SENSOR
        case 108: return 32;   // SIM_STATE
        case 109: return 185;  // RADIO_STATUS
        case 110: return 84;   // FILE_TRANSFER_PROTOCOL
        case 111: return 34;   // TIMESYNC
        case 112: return 174;  // CAMERA_TRIGGER
        case 113: return 124;  // HIL_GPS
        case 114: return 237;  // HIL_OPTICAL_FLOW
        case 115: return 4;    // HIL_STATE_QUATERNION
        case 116: return 76;   // SCALED_IMU2
        case 117: return 128;  // LOG_REQUEST_LIST
        case 118: return 56;   // LOG_ENTRY
        case 119: return 116;  // LOG_REQUEST_DATA
        case 120: return 134;  // LOG_DATA
        case 121: return 237;  // LOG_ERASE
        case 122: return 203;  // LOG_REQUEST_END
        case 123: return 250;  // GPS_INJECT_DATA
        case 124: return 87;   // GPS2_RAW
        case 125: return 203;  // POWER_STATUS
        case 126: return 220;  // SERIAL_CONTROL
        case 127: return 25;   // GPS_RTK
        case 128: return 226;  // GPS2_RTK
        case 129: return 46;   // SCALED_IMU3
        case 130: return 29;   // DATA_TRANSMISSION_HANDSHAKE
        case 131: return 223;  // ENCAPSULATED_DATA
        case 132: return 85;   // DISTANCE_SENSOR
        case 133: return 6;    // TERRAIN_REQUEST
        case 134: return 229;  // TERRAIN_DATA
        case 135: return 203;  // TERRAIN_CHECK
        case 136: return 1;    // TERRAIN_REPORT
        case 137: return 195;  // SCALED_PRESSURE2
        case 138: return 109;  // ATT_POS_MOCAP
        case 139: return 168;  // SET_ACTUATOR_CONTROL_TARGET
        case 140: return 181;  // ACTUATOR_CONTROL_TARGET
        case 141: return 47;   // ALTITUDE
        case 142: return 72;   // RESOURCE_REQUEST
        case 143: return 131;  // SCALED_PRESSURE3
        case 144: return 127;  // FOLLOW_TARGET
        case 146: return 103;  // CONTROL_SYSTEM_STATE
        case 147: return 154;  // BATTERY_STATUS
        case 148: return 178;  // AUTOPILOT_VERSION
        case 149: return 200;  // LANDING_TARGET
        case 152: return 208;  // MEMINFO (ArduPilot)
        case 162: return 189;  // FENCE_STATUS
        case 163: return 127;  // AHRS (ArduPilot)
        case 164: return 154;  // SIMSTATE
        case 165: return 21;   // HWSTATUS
        case 166: return 134;  // RADIO
        case 167: return 169;  // LIMITS_STATUS
        case 168: return 185;  // WIND
        case 169: return 139;  // DATA16
        case 170: return 108;  // DATA32
        case 171: return 49;   // DATA64
        case 172: return 182;  // DATA96
        case 173: return 229;  // RANGEFINDER
        case 174: return 85;   // AIRSPEED_AUTOCAL
        case 175: return 159;  // RALLY_POINT
        case 176: return 186;  // RALLY_FETCH_POINT
        case 177: return 65;   // COMPASSMOT_STATUS
        case 178: return 115;  // AHRS2 (ArduPilot)
        case 179: return 134;  // CAMERA_STATUS
        case 180: return 99;   // CAMERA_FEEDBACK
        case 181: return 90;   // BATTERY2
        case 182: return 149;  // AHRS3
        case 183: return 228;  // AUTOPILOT_VERSION_REQUEST
        case 184: return 15;   // REMOTE_LOG_DATA_BLOCK
        case 185: return 245;  // REMOTE_LOG_BLOCK_STATUS
        case 186: return 100;  // LED_CONTROL
        case 191: return 92;   // MAG_CAL_PROGRESS
        case 192: return 36;   // MAG_CAL_REPORT
        case 193: return 71;   // EKF_STATUS_REPORT (ArduPilot)
        case 194: return 134;  // PID_TUNING
        case 200: return 134;  // GIMBAL_REPORT
        case 201: return 205;  // GIMBAL_CONTROL
        case 214: return 69;   // GIMBAL_TORQUE_CMD_REPORT
        case 215: return 101;  // GOPRO_HEARTBEAT
        case 216: return 50;   // GOPRO_GET_REQUEST
        case 217: return 202;  // GOPRO_GET_RESPONSE
        case 218: return 17;   // GOPRO_SET_REQUEST
        case 219: return 162;  // GOPRO_SET_RESPONSE
        case 226: return 207;  // RPM
        case 230: return 163;  // ESTIMATOR_STATUS
        case 231: return 105;  // WIND_COV
        case 232: return 151;  // GPS_INPUT
        case 233: return 35;   // GPS_RTCM_DATA
        case 234: return 150;  // HIGH_LATENCY
        case 241: return 90;   // VIBRATION (ArduPilot)
        case 242: return 104;  // HOME_POSITION
        case 243: return 85;   // SET_HOME_POSITION
        case 244: return 95;   // MESSAGE_INTERVAL
        case 245: return 130;  // EXTENDED_SYS_STATE
        case 246: return 184;  // ADSB_VEHICLE
        case 247: return 81;   // COLLISION
        case 248: return 8;    // V2_EXTENSION
        case 249: return 204;  // MEMORY_VECT
        case 250: return 49;   // DEBUG_VECT
        case 251: return 170;  // NAMED_VALUE_FLOAT
        case 252: return 44;   // NAMED_VALUE_INT
        case 253: return 83;   // STATUSTEXT
        case 254: return 46;   // DEBUG
        case 256: return 71;   // SETUP_SIGNING
        case 257: return 131;  // BUTTON_CHANGE
        case 258: return 187;  // PLAY_TUNE
        case 259: return 92;   // CAMERA_INFORMATION
        case 260: return 146;  // CAMERA_SETTINGS
        case 261: return 179;  // STORAGE_INFORMATION
        case 262: return 12;   // CAMERA_CAPTURE_STATUS
        case 263: return 133;  // CAMERA_IMAGE_CAPTURED
        case 264: return 49;   // FLIGHT_INFORMATION
        case 265: return 26;   // MOUNT_ORIENTATION
        case 266: return 193;  // LOGGING_DATA
        case 267: return 35;   // LOGGING_DATA_ACKED
        case 268: return 14;   // LOGGING_ACK
        case 299: return 19;   // WIFI_CONFIG_AP
        case 300: return 217;  // PROTOCOL_VERSION
        case 310: return 28;   // UAVCAN_NODE_STATUS
        case 311: return 95;   // UAVCAN_NODE_INFO
        case 320: return 243;  // PARAM_EXT_REQUEST_READ
        case 321: return 88;   // PARAM_EXT_REQUEST_LIST
        case 322: return 243;  // PARAM_EXT_VALUE
        case 323: return 78;   // PARAM_EXT_SET
        case 324: return 132;  // PARAM_EXT_ACK
        case 330: return 23;   // OBSTACLE_DISTANCE
        case 331: return 91;   // ODOMETRY
        case 335: return 225;  // ISBD_LINK_STATUS
        case 339: return 199;  // RAW_RPM
        case 340: return 99;   // UTM_GLOBAL_POSITION
        case 350: return 232;  // DEBUG_FLOAT_ARRAY
        case 360: return 11;   // ORBIT_EXECUTION_STATUS
        case 370: return 98;   // SMART_BATTERY_INFO
        case 373: return 117;  // GENERATOR_STATUS
        case 375: return 251;  // ACTUATOR_OUTPUT_STATUS
        case 385: return 147;  // TUNNEL
        case 390: return 156;  // CAN_FRAME
        case 9000: return 113; // WHEEL_DISTANCE
        default:  return 0;    // Unknown message
    }
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
