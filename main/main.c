/**
 * ESP32 MAVLink Companion Computer
 * 
 * Connects to Pixhawk via TELEM2 (GPIO 16/17) and receives heartbeat messages.
 * Sends companion computer heartbeat back to establish bidirectional communication.
 * 
 * Configuration:
 * - Baud Rate: 57600
 * - Protocol: MAVLink 2
 * - TX: GPIO 17
 * - RX: GPIO 16
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mavlink.h"

// Configuration
#define PIXHAWK_UART_NUM     UART_NUM_2
#define PIXHAWK_TX_PIN       GPIO_NUM_16
#define PIXHAWK_RX_PIN       GPIO_NUM_17
#define PIXHAWK_BAUD_RATE    57600
#define UART_BUF_SIZE        1024

// MAVLink system configuration
#define COMPANION_SYSTEM_ID    1    // Same system ID as the autopilot
#define COMPANION_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER  // 191

static const char *TAG = "MAVLink";

// MAVLink status and message buffers
static mavlink_status_t mav_status;
static mavlink_message_t mav_msg;
static uint8_t tx_seq = 0;

// Statistics
static uint32_t heartbeat_count = 0;
static uint32_t last_heartbeat_time = 0;

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
 * Get system status string
 */
static const char* get_system_status_str(uint8_t status)
{
    switch (status) {
        case MAV_STATE_UNINIT:      return "UNINIT";
        case MAV_STATE_BOOT:        return "BOOT";
        case MAV_STATE_CALIBRATING: return "CALIBRATING";
        case MAV_STATE_STANDBY:     return "STANDBY";
        case MAV_STATE_ACTIVE:      return "ACTIVE";
        case MAV_STATE_CRITICAL:    return "CRITICAL";
        case MAV_STATE_EMERGENCY:   return "EMERGENCY";
        case MAV_STATE_POWEROFF:    return "POWEROFF";
        default:                    return "UNKNOWN";
    }
}

/**
 * Get autopilot type string
 */
static const char* get_autopilot_str(uint8_t autopilot)
{
    switch (autopilot) {
        case MAV_AUTOPILOT_GENERIC:         return "Generic";
        case MAV_AUTOPILOT_ARDUPILOTMEGA:   return "ArduPilot";
        case MAV_AUTOPILOT_PX4:             return "PX4";
        default:                            return "Unknown";
    }
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
        MAV_TYPE_ONBOARD_CONTROLLER,  // Type: Onboard companion controller
        MAV_AUTOPILOT_GENERIC,        // Autopilot: Generic
        0,                            // Base mode
        0,                            // Custom mode
        MAV_STATE_ACTIVE              // System status
    );
    
    // Update sequence number
    buf[4] = tx_seq++;
    
    // Recalculate CRC with new sequence
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
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);
    
    heartbeat_count++;
    last_heartbeat_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    ESP_LOGI(TAG, "HEARTBEAT #%lu from SYS:%d COMP:%d", 
             heartbeat_count, msg->sysid, msg->compid);
    ESP_LOGI(TAG, "  Type: %d, Autopilot: %s, State: %s",
             heartbeat.type, 
             get_autopilot_str(heartbeat.autopilot),
             get_system_status_str(heartbeat.system_status));
    ESP_LOGI(TAG, "  Base Mode: 0x%02X, Custom Mode: %lu",
             heartbeat.base_mode, heartbeat.custom_mode);
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
        default:
            // Other messages can be handled here
            ESP_LOGD(TAG, "MSG ID: %lu from SYS:%d COMP:%d", 
                     msg->msgid, msg->sysid, msg->compid);
            break;
    }
}

/**
 * MAVLink receive task - continuously reads from UART and parses MAVLink messages
 */
static void mavlink_rx_task(void *pvParameters)
{
    uint8_t data[128];
    
    ESP_LOGI(TAG, "MAVLink RX task started");
    
    while (1) {
        int len = uart_read_bytes(PIXHAWK_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t result = mavlink_parse_char(0, data[i], &mav_msg, &mav_status);
                
                if (result == MAVLINK_FRAMING_OK) {
                    process_mavlink_message(&mav_msg);
                } else if (result == MAVLINK_FRAMING_BAD_CRC) {
                    ESP_LOGW(TAG, "Bad CRC on message");
                }
            }
        }
    }
}

/**
 * Heartbeat send task - sends companion heartbeat every second
 */
static void mavlink_heartbeat_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MAVLink heartbeat task started");
    
    while (1) {
        send_heartbeat();
        ESP_LOGD(TAG, "Sent companion heartbeat (seq: %d)", tx_seq - 1);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Send heartbeat every 1 second
    }
}

/**
 * Status monitoring task - prints connection status periodically
 */
static void status_task(void *pvParameters)
{
    uint32_t prev_count = 0;
    
    ESP_LOGI(TAG, "Status monitoring task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Every 5 seconds
        
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t time_since_hb = current_time - last_heartbeat_time;
        
        if (heartbeat_count > prev_count) {
            ESP_LOGI(TAG, "STATUS: Connected | Heartbeats: %lu | Last: %lu ms ago | RX OK: %d | RX Drop: %d",
                     heartbeat_count, time_since_hb,
                     mav_status.packet_rx_success_count,
                     mav_status.packet_rx_drop_count);
        } else if (heartbeat_count == 0) {
            ESP_LOGW(TAG, "STATUS: Waiting for Pixhawk heartbeat...");
        } else {
            ESP_LOGW(TAG, "STATUS: No heartbeat for %lu ms (total: %lu)", 
                     time_since_hb, heartbeat_count);
        }
        
        prev_count = heartbeat_count;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "ESP32 MAVLink Companion Computer");
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "Pixhawk TELEM2 -> ESP32");
    ESP_LOGI(TAG, "ESP TX: GPIO %d, ESP RX: GPIO %d", PIXHAWK_TX_PIN, PIXHAWK_RX_PIN);
    ESP_LOGI(TAG, "Baud: %d, Protocol: MAVLink 2", PIXHAWK_BAUD_RATE);
    ESP_LOGI(TAG, "System ID: %d, Component ID: %d", COMPANION_SYSTEM_ID, COMPANION_COMPONENT_ID);
    ESP_LOGI(TAG, "=================================");
    
    // Initialize MAVLink status
    mavlink_status_init(&mav_status);
    
    // Initialize UART
    uart_init();
    
    // Create tasks
    xTaskCreate(mavlink_rx_task, "mavlink_rx", 4096, NULL, 10, NULL);
    xTaskCreate(mavlink_heartbeat_task, "mavlink_hb", 2048, NULL, 5, NULL);
    xTaskCreate(status_task, "status", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "All tasks started. Waiting for Pixhawk...");
}