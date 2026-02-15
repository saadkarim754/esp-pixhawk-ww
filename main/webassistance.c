/**
 * @file webassistance.c
 * @brief ESP32 MAVLink Companion Computer - Web Interface
 * 
 * Web-based control interface for Pixhawk:
 * - ESP32 in AP mode (creates WiFi network)
 * - Web interface showing attitude data (roll, pitch, yaw)
 * - Arm/Disarm controls
 * - Flight mode selection
 * - Status logs display
 * 
 * Configuration:
 * - WiFi AP SSID: Pixhawk-ESP32
 * - WiFi AP Password: pixhawk123
 * - Web interface: http://192.168.4.1
 * - MAVLink: TX GPIO 16, RX GPIO 17, 57600 baud
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "mavespstm.h"

// ============================================================================
// Configuration
// ============================================================================

// WiFi AP Configuration
#define WIFI_AP_SSID        "Pixhawk-ESP32"
#define WIFI_AP_PASS        "pixhawk123"
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CONN    4

// UART Configuration
#define PIXHAWK_UART_NUM     UART_NUM_2
#define PIXHAWK_TX_PIN       GPIO_NUM_16
#define PIXHAWK_RX_PIN       GPIO_NUM_17
#define PIXHAWK_BAUD_RATE    57600
#define UART_BUF_SIZE        1024

// MAVLink Configuration
#define COMPANION_SYSTEM_ID    1
#define COMPANION_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER
#define PIXHAWK_SYSTEM_ID      1
#define PIXHAWK_COMPONENT_ID   1

// Log buffer
#define LOG_BUFFER_SIZE        20
#define LOG_MSG_SIZE           100

static const char *TAG = "WebAssist";

// ============================================================================
// Global State
// ============================================================================

// MAVLink state
static mavlink_status_t mav_status;
static mavlink_message_t mav_msg;
static uint8_t tx_seq = 0;
static SemaphoreHandle_t uart_mutex;

// Attitude data
static float current_roll = 0.0f;
static float current_pitch = 0.0f;
static float current_yaw = 0.0f;
static uint32_t attitude_count = 0;

// Flight state
static uint32_t heartbeat_count = 0;
static uint8_t pixhawk_base_mode = 0;
static uint32_t pixhawk_custom_mode = 0;
static uint8_t pixhawk_system_status = 0;
static bool is_armed = false;
static bool is_connected = false;
static uint32_t last_heartbeat_time = 0;

// Log buffer (circular)
static char log_buffer[LOG_BUFFER_SIZE][LOG_MSG_SIZE];
static int log_head = 0;
static int log_count = 0;
static SemaphoreHandle_t log_mutex;

// ============================================================================
// ArduPilot Flight Modes
// ============================================================================

typedef struct {
    uint32_t mode_num;
    const char *name;
} flight_mode_t;

static const flight_mode_t copter_modes[] = {
    {0,  "STABILIZE"},
    {1,  "ACRO"},
    {2,  "ALT_HOLD"},
    {3,  "AUTO"},
    {4,  "GUIDED"},
    {5,  "LOITER"},
    {6,  "RTL"},
    {7,  "CIRCLE"},
    {9,  "LAND"},
    {16, "POSHOLD"},
    {17, "BRAKE"},
    {21, "SMART_RTL"},
};
#define NUM_COPTER_MODES (sizeof(copter_modes) / sizeof(copter_modes[0]))

static const char* get_mode_name(uint32_t mode) {
    for (int i = 0; i < NUM_COPTER_MODES; i++) {
        if (copter_modes[i].mode_num == mode) {
            return copter_modes[i].name;
        }
    }
    return "UNKNOWN";
}

static const char* get_status_name(uint8_t status) {
    switch (status) {
        case 0: return "UNINIT";
        case 1: return "BOOT";
        case 2: return "CALIBRATING";
        case 3: return "STANDBY";
        case 4: return "ACTIVE";
        case 5: return "CRITICAL";
        case 6: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// Logging
// ============================================================================

static void add_log(const char *fmt, ...) {
    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(log_buffer[log_head], LOG_MSG_SIZE, fmt, args);
        va_end(args);
        
        log_head = (log_head + 1) % LOG_BUFFER_SIZE;
        if (log_count < LOG_BUFFER_SIZE) log_count++;
        
        xSemaphoreGive(log_mutex);
    }
}

// ============================================================================
// UART / MAVLink
// ============================================================================

static void uart_init(void) {
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
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", PIXHAWK_TX_PIN, PIXHAWK_RX_PIN, PIXHAWK_BAUD_RATE);
}

static void send_mavlink_message(uint8_t *buf, uint16_t len) {
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buf[4] = tx_seq++;
        uart_write_bytes(PIXHAWK_UART_NUM, buf, len);
        xSemaphoreGive(uart_mutex);
    }
}

static void send_heartbeat(void) {
    uint8_t buf[32];
    uint16_t len = mavlink_msg_heartbeat_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_GENERIC,
        0, 0,
        MAV_STATE_ACTIVE
    );
    send_mavlink_message(buf, len);
}

static void send_arm_command(bool arm) {
    uint8_t buf[64];
    float arm_param = arm ? 1.0f : 0.0f;
    
    uint16_t len = mavlink_msg_command_long_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        PIXHAWK_SYSTEM_ID,
        PIXHAWK_COMPONENT_ID,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,          // confirmation
        arm_param,  // param1: 1=arm, 0=disarm
        0, 0, 0, 0, 0, 0  // param2-7 unused
    );
    
    send_mavlink_message(buf, len);
    add_log("Sent %s command", arm ? "ARM" : "DISARM");
    ESP_LOGI(TAG, "Sent %s command", arm ? "ARM" : "DISARM");
}

static void send_mode_command(uint32_t mode) {
    uint8_t buf[64];
    
    // Use COMMAND_LONG with MAV_CMD_DO_SET_MODE for proper ACK response
    // param1 = base_mode (1 = custom mode enabled)
    // param2 = custom_mode (ArduPilot flight mode number)
    float base_mode = 1.0f;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if (is_armed) {
        base_mode = 129.0f;  // 0x81 = custom mode + armed
    }
    
    uint16_t len = mavlink_msg_command_long_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        PIXHAWK_SYSTEM_ID,
        PIXHAWK_COMPONENT_ID,
        MAV_CMD_DO_SET_MODE,
        0,              // confirmation
        base_mode,      // param1: base mode
        (float)mode,    // param2: custom mode
        0, 0, 0, 0, 0   // param3-7 unused
    );
    
    send_mavlink_message(buf, len);
    add_log("Set mode: %s", get_mode_name(mode));
    ESP_LOGI(TAG, "Sent mode change to %s (%lu)", get_mode_name(mode), mode);
}

// ============================================================================
// MAVLink Message Handlers
// ============================================================================

static void handle_heartbeat(const mavlink_message_t *msg) {
    if (msg->sysid == 255) return;
    
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);
    
    // Detect mode change
    if (heartbeat_count > 0 && pixhawk_custom_mode != hb.custom_mode) {
        ESP_LOGI(TAG, "MODE CHANGED: %s -> %s", 
                 get_mode_name(pixhawk_custom_mode), get_mode_name(hb.custom_mode));
        add_log("Mode: %s", get_mode_name(hb.custom_mode));
    }
    
    // Detect arm state change
    bool new_armed = (hb.base_mode & 0x80) != 0;
    if (heartbeat_count > 0 && is_armed != new_armed) {
        ESP_LOGI(TAG, "ARM STATE: %s", new_armed ? "ARMED" : "DISARMED");
        add_log("%s", new_armed ? ">>> ARMED <<<" : ">>> DISARMED <<<");
    }
    
    heartbeat_count++;
    pixhawk_base_mode = hb.base_mode;
    pixhawk_custom_mode = hb.custom_mode;
    pixhawk_system_status = hb.system_status;
    is_armed = new_armed;
    is_connected = true;
    last_heartbeat_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Log heartbeat periodically
    if (heartbeat_count % 10 == 1) {
        ESP_LOGI(TAG, "PIXHAWK: Mode=%s Status=%s %s",
                 get_mode_name(pixhawk_custom_mode),
                 get_status_name(pixhawk_system_status),
                 is_armed ? "ARMED" : "DISARMED");
    }
}

static void handle_attitude(const mavlink_message_t *msg) {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(msg, &attitude);
    
    current_roll = mavlink_rad_to_deg(attitude.roll);
    current_pitch = mavlink_rad_to_deg(attitude.pitch);
    current_yaw = mavlink_rad_to_deg(attitude.yaw);
    attitude_count++;
    
    // Log every 20th message (~every 5 seconds at 4Hz)
    if (attitude_count % 20 == 1) {
        ESP_LOGI(TAG, "ATTITUDE: Roll=%6.1f° Pitch=%6.1f° Yaw=%6.1f°",
                 current_roll, current_pitch, current_yaw);
    }
}

static void handle_command_ack(const mavlink_message_t *msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(msg, &ack);
    
    const char *result_str = mavlink_result_to_string(ack.result);
    
    if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            add_log("ARM/DISARM: %s", result_str);
        } else {
            add_log("ARM/DISARM FAILED: %s", result_str);
        }
    } else if (ack.command == MAV_CMD_DO_SET_MODE) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            add_log("MODE CHANGE: %s", result_str);
        } else {
            add_log("MODE FAILED: %s (need GPS?)", result_str);
        }
    } else {
        add_log("CMD %d: %s", ack.command, result_str);
    }
    
    ESP_LOGI(TAG, "COMMAND_ACK: cmd=%d result=%s", ack.command, result_str);
}

static void handle_statustext(const mavlink_message_t *msg) {
    mavlink_statustext_t text;
    mavlink_msg_statustext_decode(msg, &text);
    
    // MAVLink text field is 50 bytes, may not be null-terminated and may contain garbage
    // Find actual length (first null or non-printable char, max 50)
    char safe_text[51];
    int len = 0;
    for (int i = 0; i < 50 && text.text[i] != '\0'; i++) {
        // Stop at first non-printable character (garbage data)
        if (text.text[i] < 32 || text.text[i] > 126) {
            break;
        }
        safe_text[len++] = text.text[i];
    }
    safe_text[len] = '\0';
    
    add_log("[%s] %s", mavlink_severity_to_string(text.severity), safe_text);
    ESP_LOGI(TAG, "STATUSTEXT: [%s] %s", mavlink_severity_to_string(text.severity), safe_text);
}

static void process_mavlink_message(const mavlink_message_t *msg) {
    // Debug: log unknown message IDs occasionally
    static uint32_t msg_counts[256] = {0};
    if (msg->msgid < 256) {
        msg_counts[msg->msgid]++;
        // Log first occurrence of each message type
        if (msg_counts[msg->msgid] == 1) {
            ESP_LOGI(TAG, "Received MSG_ID: %lu from SYS:%d", msg->msgid, msg->sysid);
        }
    }
    
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handle_heartbeat(msg);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            handle_attitude(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
            handle_command_ack(msg);
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:
            handle_statustext(msg);
            break;
    }
}

// ============================================================================
// FreeRTOS Tasks
// ============================================================================

static void mavlink_rx_task(void *pvParameters) {
    uint8_t data[128];
    
    ESP_LOGI(TAG, "MAVLink RX task started");
    
    while (1) {
        int len = uart_read_bytes(PIXHAWK_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t result = mavlink_parse_char(0, data[i], &mav_msg, &mav_status);
                if (result == MAVLINK_FRAMING_OK) {
                    process_mavlink_message(&mav_msg);
                }
            }
        }
        
        // Check connection timeout
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (is_connected && (now - last_heartbeat_time > 3000)) {
            is_connected = false;
            add_log("Connection lost!");
        }
    }
}

static void mavlink_heartbeat_task(void *pvParameters) {
    ESP_LOGI(TAG, "MAVLink heartbeat task started");
    
    while (1) {
        send_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// WiFi AP
// ============================================================================

static void wifi_init_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s, Password=%s", WIFI_AP_SSID, WIFI_AP_PASS);
    add_log("WiFi AP: %s", WIFI_AP_SSID);
}

// ============================================================================
// HTTP Server - HTML Page
// ============================================================================

static const char *HTML_PAGE = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"<meta charset='UTF-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>Pixhawk Control</title>\n"
"<style>\n"
"*{box-sizing:border-box;margin:0;padding:0}\n"
"body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:10px}\n"
".container{max-width:600px;margin:0 auto}\n"
"h1{text-align:center;color:#0f0;margin-bottom:20px;font-size:24px}\n"
".card{background:#16213e;border-radius:10px;padding:15px;margin-bottom:15px}\n"
".card h2{color:#0ff;margin-bottom:10px;font-size:16px}\n"
".status{display:flex;justify-content:space-between;margin-bottom:10px}\n"
".status-item{text-align:center;flex:1}\n"
".status-value{font-size:24px;font-weight:bold}\n"
".status-label{font-size:12px;color:#888}\n"
".armed{color:#f00}.disarmed{color:#0f0}\n"
".connected{color:#0f0}.disconnected{color:#f00}\n"
".attitude{display:flex;justify-content:space-around;text-align:center}\n"
".att-item{flex:1}\n"
".att-value{font-size:28px;font-weight:bold;color:#fff}\n"
".att-label{font-size:12px;color:#888}\n"
".controls{display:flex;gap:10px;margin-bottom:10px}\n"
".btn{flex:1;padding:15px;border:none;border-radius:8px;font-size:16px;font-weight:bold;cursor:pointer}\n"
".btn-arm{background:#f44;color:#fff}\n"
".btn-disarm{background:#4a4;color:#fff}\n"
".btn:disabled{opacity:0.5;cursor:not-allowed}\n"
"select{width:100%;padding:12px;font-size:16px;border-radius:8px;border:none;background:#0e4d92;color:#fff}\n"
".logs{background:#0a0a15;border-radius:8px;padding:10px;height:150px;overflow-y:auto;font-family:monospace;font-size:12px}\n"
".log-entry{padding:2px 0;border-bottom:1px solid #222}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class='container'>\n"
"<h1>PIXHAWK CONTROL</h1>\n"
"<div class='card'>\n"
"<h2>STATUS</h2>\n"
"<div class='status'>\n"
"<div class='status-item'><div id='conn' class='status-value disconnected'>---</div><div class='status-label'>Connection</div></div>\n"
"<div class='status-item'><div id='armed' class='status-value'>---</div><div class='status-label'>Armed</div></div>\n"
"<div class='status-item'><div id='mode' class='status-value'>---</div><div class='status-label'>Mode</div></div>\n"
"</div>\n"
"</div>\n"
"<div class='card'>\n"
"<h2>ATTITUDE</h2>\n"
"<div class='attitude'>\n"
"<div class='att-item'><div id='roll' class='att-value'>0.0</div><div class='att-label'>ROLL</div></div>\n"
"<div class='att-item'><div id='pitch' class='att-value'>0.0</div><div class='att-label'>PITCH</div></div>\n"
"<div class='att-item'><div id='yaw' class='att-value'>0.0</div><div class='att-label'>YAW</div></div>\n"
"</div>\n"
"</div>\n"
"<div class='card'>\n"
"<h2>CONTROLS</h2>\n"
"<div class='controls'>\n"
"<button id='armBtn' class='btn btn-arm' onclick='arm()'>ARM</button>\n"
"<button id='disarmBtn' class='btn btn-disarm' onclick='disarm()'>DISARM</button>\n"
"</div>\n"
"<select id='modeSelect' onchange='setMode()'>\n"
"<option value=''>-- Select Mode --</option>\n"
"<option value='0'>STABILIZE</option>\n"
"<option value='2'>ALT_HOLD</option>\n"
"<option value='5'>LOITER</option>\n"
"<option value='16'>POSHOLD</option>\n"
"<option value='4'>GUIDED</option>\n"
"<option value='3'>AUTO</option>\n"
"<option value='6'>RTL</option>\n"
"<option value='9'>LAND</option>\n"
"</select>\n"
"</div>\n"
"<div class='card'>\n"
"<h2>LOGS</h2>\n"
"<div id='logs' class='logs'></div>\n"
"</div>\n"
"</div>\n"
"<script>\n"
"function update(){fetch('/api/data').then(r=>r.json()).then(d=>{\n"
"document.getElementById('conn').textContent=d.connected?'OK':'LOST';\n"
"document.getElementById('conn').className='status-value '+(d.connected?'connected':'disconnected');\n"
"document.getElementById('armed').textContent=d.armed?'YES':'NO';\n"
"document.getElementById('armed').className='status-value '+(d.armed?'armed':'disarmed');\n"
"document.getElementById('mode').textContent=d.mode;\n"
"document.getElementById('roll').textContent=d.roll.toFixed(1)+'°';\n"
"document.getElementById('pitch').textContent=d.pitch.toFixed(1)+'°';\n"
"document.getElementById('yaw').textContent=d.yaw.toFixed(1)+'°';\n"
"var logsDiv=document.getElementById('logs');\n"
"logsDiv.innerHTML=d.logs.map(l=>'<div class=\"log-entry\">'+l+'</div>').join('');\n"
"logsDiv.scrollTop=logsDiv.scrollHeight;\n"
"}).catch(e=>console.error(e));}\n"
"function arm(){fetch('/api/arm',{method:'POST'});}\n"
"function disarm(){fetch('/api/disarm',{method:'POST'});}\n"
"function setMode(){var m=document.getElementById('modeSelect').value;if(m)fetch('/api/mode?m='+m,{method:'POST'});document.getElementById('modeSelect').value='';}\n"
"setInterval(update,500);update();\n"
"</script>\n"
"</body>\n"
"</html>";

// ============================================================================
// HTTP Handlers
// ============================================================================

static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req) {
    char json[1024];
    char logs_json[512] = "[";
    
    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < log_count; i++) {
            int idx = (log_head - log_count + i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
            if (i > 0) strcat(logs_json, ",");
            strcat(logs_json, "\"");
            // Escape quotes in log messages
            char *p = log_buffer[idx];
            char *d = logs_json + strlen(logs_json);
            while (*p && (d - logs_json) < 500) {
                if (*p == '"') { *d++ = '\\'; }
                *d++ = *p++;
            }
            *d = '\0';
            strcat(logs_json, "\"");
        }
        xSemaphoreGive(log_mutex);
    }
    strcat(logs_json, "]");
    
    snprintf(json, sizeof(json),
        "{\"connected\":%s,\"armed\":%s,\"mode\":\"%s\",\"status\":\"%s\","
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"logs\":%s}",
        is_connected ? "true" : "false",
        is_armed ? "true" : "false",
        get_mode_name(pixhawk_custom_mode),
        get_status_name(pixhawk_system_status),
        current_roll, current_pitch, current_yaw,
        logs_json
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

static esp_err_t arm_handler(httpd_req_t *req) {
    send_arm_command(true);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t disarm_handler(httpd_req_t *req) {
    send_arm_command(false);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t *req) {
    char buf[32];
    ESP_LOGI(TAG, "Mode handler called");
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char mode_str[8];
        if (httpd_query_key_value(buf, "m", mode_str, sizeof(mode_str)) == ESP_OK) {
            uint32_t mode = atoi(mode_str);
            ESP_LOGI(TAG, "Web request: Set mode to %lu (%s)", mode, get_mode_name(mode));
            send_mode_command(mode);
        }
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
        httpd_uri_t data = { .uri = "/api/data", .method = HTTP_GET, .handler = data_handler };
        httpd_uri_t arm = { .uri = "/api/arm", .method = HTTP_POST, .handler = arm_handler };
        httpd_uri_t disarm = { .uri = "/api/disarm", .method = HTTP_POST, .handler = disarm_handler };
        httpd_uri_t mode = { .uri = "/api/mode", .method = HTTP_POST, .handler = mode_handler };
        
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &data);
        httpd_register_uri_handler(server, &arm);
        httpd_register_uri_handler(server, &disarm);
        httpd_register_uri_handler(server, &mode);
        
        ESP_LOGI(TAG, "HTTP server started on port 80");
    }
    
    return server;
}

// ============================================================================
// Main
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "   ESP32 Pixhawk Web Assistant");
    ESP_LOGI(TAG, "=========================================");
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create mutexes
    uart_mutex = xSemaphoreCreateMutex();
    log_mutex = xSemaphoreCreateMutex();
    
    // Initialize MAVLink status
    mavlink_status_init(&mav_status);
    
    // Initialize UART
    uart_init();
    
    // Initialize WiFi AP
    wifi_init_ap();
    
    // Start HTTP server
    start_webserver();
    
    // Create tasks
    xTaskCreate(mavlink_rx_task, "mav_rx", 4096, NULL, 10, NULL);
    xTaskCreate(mavlink_heartbeat_task, "mav_hb", 2048, NULL, 5, NULL);
    
    add_log("System initialized");
    
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "WiFi: %s / %s", WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "Web: http://192.168.4.1");
    ESP_LOGI(TAG, "=========================================");
}
