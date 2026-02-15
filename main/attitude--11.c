/**
 * @file attitude--11.c
 * @brief ESP32 MAVLink Companion Computer - Attitude Data
 * 
 * Standalone module for receiving attitude data (roll, pitch, yaw) from Pixhawk.
 * 
 * Configuration:
 * - Baud Rate: 57600
 * - Protocol: MAVLink 2
 * - TX: GPIO 16
 * - RX: GPIO 17
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mavespstm.h"

// Configuration
#define PIXHAWK_UART_NUM     UART_NUM_2
#define PIXHAWK_TX_PIN       GPIO_NUM_16
#define PIXHAWK_RX_PIN       GPIO_NUM_17
#define PIXHAWK_BAUD_RATE    57600
#define UART_BUF_SIZE        1024

// MAVLink system configuration
#define COMPANION_SYSTEM_ID    1
#define COMPANION_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER

static const char *TAG = "Attitude11";

// MAVLink status and message buffers
static mavlink_status_t mav_status;
static mavlink_message_t mav_msg;
static uint8_t tx_seq = 0;

// Attitude data
static float current_roll = 0.0f;
static float current_pitch = 0.0f;
static float current_yaw = 0.0f;
static uint32_t attitude_count = 0;

// Heartbeat/Flight mode tracking
static uint32_t heartbeat_count = 0;
static uint8_t pixhawk_base_mode = 0;
static uint32_t pixhawk_custom_mode = 0;
static uint8_t pixhawk_system_status = 0;
static uint8_t pixhawk_type = 0;

// ArduPilot Copter flight modes
static const char* get_copter_mode_name(uint32_t custom_mode) {
    switch (custom_mode) {
        case 0:  return "STABILIZE";
        case 1:  return "ACRO";
        case 2:  return "ALT_HOLD";
        case 3:  return "AUTO";
        case 4:  return "GUIDED";
        case 5:  return "LOITER";
        case 6:  return "RTL";
        case 7:  return "CIRCLE";
        case 9:  return "LAND";
        case 11: return "DRIFT";
        case 13: return "SPORT";
        case 14: return "FLIP";
        case 15: return "AUTOTUNE";
        case 16: return "POSHOLD";
        case 17: return "BRAKE";
        case 18: return "THROW";
        case 19: return "AVOID_ADSB";
        case 20: return "GUIDED_NOGPS";
        case 21: return "SMART_RTL";
        case 22: return "FLOWHOLD";
        case 23: return "FOLLOW";
        case 24: return "ZIGZAG";
        case 25: return "SYSTEMID";
        case 26: return "AUTOROTATE";
        case 27: return "AUTO_RTL";
        default: return "UNKNOWN";
    }
}

// System status names
static const char* get_system_status_name(uint8_t status) {
    switch (status) {
        case 0: return "UNINIT";
        case 1: return "BOOT";
        case 2: return "CALIBRATING";
        case 3: return "STANDBY";
        case 4: return "ACTIVE";
        case 5: return "CRITICAL";
        case 6: return "EMERGENCY";
        case 7: return "POWEROFF";
        case 8: return "TERMINATION";
        default: return "UNKNOWN";
    }
}

/**
 * Initialize UART for Pixhawk communication
 */
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = PIXHAWK_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(PIXHAWK_UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PIXHAWK_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PIXHAWK_UART_NUM, PIXHAWK_TX_PIN, PIXHAWK_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized on TX:%d, RX:%d at %d baud", 
             PIXHAWK_TX_PIN, PIXHAWK_RX_PIN, PIXHAWK_BAUD_RATE);
}

/**
 * Send companion computer heartbeat to Pixhawk
 */
static void send_heartbeat(void)
{
    uint8_t buf[32];
    uint16_t len;
    
    len = mavlink_msg_heartbeat_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_GENERIC,
        0, 0,
        MAV_STATE_ACTIVE
    );
    
    buf[4] = tx_seq++;
    
    uint16_t crc;
    crc_init(&crc);
    for (int i = 1; i <= 18; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(MAVLINK_MSG_ID_HEARTBEAT_CRC, &crc);
    buf[19] = crc & 0xFF;
    buf[20] = (crc >> 8) & 0xFF;
    
    uart_write_bytes(PIXHAWK_UART_NUM, buf, len);
}

/**
 * Handle received heartbeat message
 */
static void handle_heartbeat(const mavlink_message_t *msg)
{
    if (msg->sysid == 255) return;  // Ignore GCS
    
    // Decode heartbeat
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);
    
    heartbeat_count++;
    pixhawk_type = hb.type;
    pixhawk_base_mode = hb.base_mode;
    pixhawk_custom_mode = hb.custom_mode;
    pixhawk_system_status = hb.system_status;
    
    // Check if armed
    bool armed = (hb.base_mode & 0x80) != 0;
    
    ESP_LOGI(TAG, "PIXHAWK: Mode=%s | Status=%s | %s",
             get_copter_mode_name(hb.custom_mode),
             get_system_status_name(hb.system_status),
             armed ? "ARMED" : "DISARMED");
}

/**
 * Handle received attitude message
 */
static void handle_attitude(const mavlink_message_t *msg)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(msg, &attitude);
    
    // Convert to degrees
    current_roll = mavlink_rad_to_deg(attitude.roll);
    current_pitch = mavlink_rad_to_deg(attitude.pitch);
    current_yaw = mavlink_rad_to_deg(attitude.yaw);
    
    attitude_count++;
    
    // Log every 10th message to avoid flooding (~4Hz input = every 2.5s)
    if (attitude_count % 10 == 1) {
        ESP_LOGI(TAG, "Roll: %7.2f° | Pitch: %7.2f° | Yaw: %7.2f°",
                 current_roll, current_pitch, current_yaw);
    }
}

/**
 * Process received MAVLink message
 */
static void process_mavlink_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handle_heartbeat(msg);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            handle_attitude(msg);
            break;
        default:
            break;
    }
}

/**
 * MAVLink receive task
 */
static void mavlink_rx_task(void *pvParameters)
{
    uint8_t data[128];
    static uint32_t bad_crc_count = 0;
    
    ESP_LOGI(TAG, "MAVLink RX task started");
    
    while (1) {
        int len = uart_read_bytes(PIXHAWK_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t result = mavlink_parse_char(0, data[i], &mav_msg, &mav_status);
                
                if (result == MAVLINK_FRAMING_OK) {
                    process_mavlink_message(&mav_msg);
                } else if (result == MAVLINK_FRAMING_BAD_CRC) {
                    bad_crc_count++;
                    if (bad_crc_count % 100 == 1) {
                        ESP_LOGW(TAG, "Bad CRC (total: %lu, msg_id: %lu)", bad_crc_count, mav_msg.msgid);
                    }
                }
            }
        }
    }
}

/**
 * Heartbeat send task
 */
static void mavlink_heartbeat_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MAVLink heartbeat TX task started");
    
    while (1) {
        send_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * Status monitoring task
 */
static void status_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Status monitoring task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        ESP_LOGI(TAG, "─────────────────────────────────────────");
        
        if (heartbeat_count > 0) {
            bool armed = (pixhawk_base_mode & 0x80) != 0;
            ESP_LOGI(TAG, "FLIGHT: %s | %s | %s",
                     get_copter_mode_name(pixhawk_custom_mode),
                     get_system_status_name(pixhawk_system_status),
                     armed ? "ARMED" : "DISARMED");
        }
        
        if (attitude_count > 0) {
            ESP_LOGI(TAG, "ATTITUDE: Roll=%6.1f° Pitch=%6.1f° Yaw=%6.1f°",
                     current_roll, current_pitch, current_yaw);
            ESP_LOGI(TAG, "STATS: Attitude msgs=%lu | Heartbeats=%lu",
                     attitude_count, heartbeat_count);
        } else {
            ESP_LOGW(TAG, "Waiting for attitude data from Pixhawk...");
        }
        
        ESP_LOGI(TAG, "─────────────────────────────────────────");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "   ESP32 MAVLink - Attitude Monitor");
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Pixhawk TELEM2 -> ESP32");
    ESP_LOGI(TAG, "TX: GPIO %d, RX: GPIO %d", PIXHAWK_TX_PIN, PIXHAWK_RX_PIN);
    ESP_LOGI(TAG, "Baud: %d, Protocol: MAVLink 2", PIXHAWK_BAUD_RATE);
    ESP_LOGI(TAG, "=========================================");
    
    // Initialize MAVLink status
    mavlink_status_init(&mav_status);
    
    // Initialize UART
    uart_init();
    
    // Create tasks
    xTaskCreate(mavlink_rx_task, "mavlink_rx", 4096, NULL, 10, NULL);
    xTaskCreate(mavlink_heartbeat_task, "mavlink_hb", 2048, NULL, 5, NULL);
    xTaskCreate(status_task, "status", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "System running. Waiting for Pixhawk attitude data...");
}
