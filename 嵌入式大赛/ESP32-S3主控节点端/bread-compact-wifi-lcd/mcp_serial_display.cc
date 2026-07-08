#include "mcp_serial_display.h"
#include "config.h"
#include "application.h"
#include "board.h"
#include "audio_codec.h"
#include "trackdock_mqtt_client.h"
#include "trackdock_scene.h"
#include "mcp_uwb_navigation.h"
#include "mcp_rfid_cargo_manager.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

static const char* TAG = "SerialDisplay";

#define UART_NUM            UART_NUM_2
#define UART_TX_PIN         MCP_ROOM_TX_PIN
#define UART_RX_PIN         MCP_ROOM_RX_PIN
#define UART_BAUD_RATE      9600
#define UART_BUF_SIZE       1024

#define CMD_PREFIX          "CMD|"
#define CMD_PREFIX_LEN      4

static const uint8_t TJC_TAIL[3] = {0xFF, 0xFF, 0xFF};
static uint8_t rx_buffer[UART_BUF_SIZE];
static int rx_index = 0;
static int ff_count = 0;

void SerialDisplay_ShowRfidInfo(const char* info);
static void StartSerialRfidScan();
static void StopSerialRfidScan();
static bool serial_rfid_scan_active = false;

static const char* RfidDisplayText(const std::string& cargo_json) {
    if (cargo_json.find("已识别") != std::string::npos ||
        cargo_json.find("扫描成功") != std::string::npos) {
        return "RFID scan success";
    }
    if (cargo_json.find("已存在") != std::string::npos) {
        return "RFID already scanned";
    }
    if (cargo_json.find("未检测到") != std::string::npos) {
        return "RFID no tag";
    }
    return "RFID scanning...";
}
static void SerialRfidScanTask(void* arg) {
    while (serial_rfid_scan_active) {
        std::string cargo = RfidCargoScanOnce();
        TrackDockMqtt_PublishCargoJson(cargo);
        SerialDisplay_ShowRfidInfo(RfidDisplayText(cargo));
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    vTaskDelete(NULL);
}

static void StartSerialRfidScan() {
    std::string cargo = RfidCargoStartCollecting();
    TrackDockMqtt_PublishCargoJson(cargo);
    SerialDisplay_ShowRfidInfo("RFID scan started");

    if (!serial_rfid_scan_active) {
        serial_rfid_scan_active = true;
        xTaskCreate(SerialRfidScanTask, "serial_rfid_scan", 4096, NULL, 4, NULL);
    }
}

static void StopSerialRfidScan() {
    serial_rfid_scan_active = false;
    std::string cargo = RfidCargoFinishCollecting();
    TrackDockMqtt_PublishCargoJson(cargo);
    SerialDisplay_ShowRfidInfo("RFID scan stopped");
}

static void SerialDisplay_SendCommand(const char* command) {
    if (!command) return;
    uart_write_bytes(UART_NUM, command, strlen(command));
    uart_write_bytes(UART_NUM, (const char*)TJC_TAIL, sizeof(TJC_TAIL));
}

static void MakeSafeText(const char* src, char* dst, size_t dst_size) {
    if (!src || !dst || dst_size == 0) return;

    size_t out = 0;
    for (size_t i = 0; src[i] != '\0' && out + 1 < dst_size; ++i) {
        char ch = src[i];
        if (ch == '"') {
            ch = '\'';
        }
        dst[out++] = ch;
    }
    dst[out] = '\0';
}

void SerialDisplay_UpdateText(const char* widget_name, const char* text) {
    if (!widget_name || !text) return;

    char safe_text[384];
    char cmd[512];
    MakeSafeText(text, safe_text, sizeof(safe_text));
    snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", widget_name, safe_text);

    SerialDisplay_SendCommand(cmd);
    ESP_LOGI(TAG, "Send to serial display: %s", cmd);
}

void SerialDisplay_ShowRfidInfo(const char* info) {
    SerialDisplay_UpdateText("t1", info);
}

void SerialDisplay_ShowLocationInfo(const char* info) {
    SerialDisplay_UpdateText("t1", info);
}

void SerialDisplay_ShowDialogText(const char* text) {
    SerialDisplay_UpdateText("g0", text);
}

static bool ParseVolumeProtocol(const uint8_t* data, int len, int* volume) {
    if (!data || !volume || len < 6) return false;
    if (data[0] != 0x55 || data[1] != 0xAA || data[2] != 0x01) return false;
    if (data[len - 2] != 0x0D || data[len - 1] != 0x0A) return false;

    int value = -1;
    int value_len = len - 5;
    if (value_len < 1 || value_len > 3) return false;

    if (value_len == 1 && data[3] <= 100) {
        value = data[3];
    } else {
        value = 0;
        for (int i = 3; i < len - 2; ++i) {
            if (data[i] < '0' || data[i] > '9') return false;
            value = value * 10 + (data[i] - '0');
        }
    }

    if (value < 0 || value > 100) return false;
    *volume = value;
    return true;
}

static bool ParseTextCommand(const char* frame, char* cmd_name, int max_len) {
    if (!frame || !cmd_name || max_len <= 0) return false;
    if (strncmp(frame, CMD_PREFIX, CMD_PREFIX_LEN) != 0) return false;

    const char* start = frame + CMD_PREFIX_LEN;
    const char* end = strstr(start, "\r\n");
    if (!end) return false;

    int len = end - start;
    if (len >= max_len) len = max_len - 1;
    strncpy(cmd_name, start, len);
    cmd_name[len] = '\0';
    return true;
}

static void SetVolume(int volume) {
    ESP_LOGI(TAG, "Set volume: %d", volume);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (codec) {
        codec->SetOutputVolume(volume);
    }
}

static void ExecuteCommand(const char* cmd_name) {
    ESP_LOGI(TAG, "Execute command: %s", cmd_name);

    Application& app = Application::GetInstance();

    if (strcmp(cmd_name, "start_scan") == 0) {
        StartSerialRfidScan();

    } else if (strcmp(cmd_name, "stop_scan") == 0) {
        StopSerialRfidScan();

    } else if (strcmp(cmd_name, "start_location") == 0) {
        bool ok = UwbNavigationStartPositioning();
        SerialDisplay_ShowLocationInfo(ok ? "UWB positioning started" : "UWB positioning failed");
        TrackDockMqtt_PublishTelemetry();

    } else if (strcmp(cmd_name, "stop_location") == 0) {
        UwbNavigationStop();
        SerialDisplay_ShowLocationInfo("UWB positioning stopped");
        TrackDockMqtt_PublishTelemetry();

    } else if (strcmp(cmd_name, "xiaozhi_start") == 0) {
        SerialDisplay_ShowDialogText("Xiaozhi: listening...");
        TrackDockMqtt_PublishDialog("Xiaozhi: listening...");
        app.StartListening();

    } else if (strcmp(cmd_name, "xiaozhi_stop") == 0) {
        SerialDisplay_ShowDialogText("Xiaozhi: stopped");
        TrackDockMqtt_PublishDialog("Xiaozhi: stopped");
        app.StopListening();

    } else if (strncmp(cmd_name, "scene_", 6) == 0) {
        const char* scene_id = cmd_name + 6;
        TrackDockScene_Set(scene_id);
        TrackDockMqtt_PublishState();
        char screen_message[64];
        char message[128];
        snprintf(screen_message, sizeof(screen_message), "Scene: %s", TrackDockScene_GetId());
        snprintf(message, sizeof(message), "已切换到%s场景", TrackDockScene_GetName());
        SerialDisplay_ShowDialogText(screen_message);
        TrackDockMqtt_PublishDialog(message);
        app.Alert("Scene mode", message, "neutral");
        ESP_LOGI(TAG, "Scene changed: %s (%s)", TrackDockScene_GetId(), TrackDockScene_GetName());

    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd_name);
    }
}

static bool IsDirectMotorFrame(const uint8_t* data, int len) {
    if (!data || len < 2) return false;
    bool line_frame = (len == 2) || (len == 4 && data[2] == 0x0D && data[3] == 0x0A);
    if (!line_frame) return false;
    return ((data[0] == 'K' && (data[1] == 'A' || data[1] == 'E' || data[1] == 'G' || data[1] == 'C')) ||
            (data[0] == 'Z' && data[1] == 'K'));
}

static bool IsTouchKeyFrame(const uint8_t* data, int len) {
    return data && len >= 4 && data[0] == 0x65;
}
static void ProcessFrame() {
    if (rx_index <= 0) return;
    rx_buffer[rx_index] = '\0';

    if (IsDirectMotorFrame(rx_buffer, rx_index) || IsTouchKeyFrame(rx_buffer, rx_index)) {
        return;
    }

    int volume = 0;
    if (ParseVolumeProtocol(rx_buffer, rx_index, &volume)) {
        SetVolume(volume);
        return;
    }

    char cmd_name[64];
    if (ParseTextCommand((const char*)rx_buffer, cmd_name, sizeof(cmd_name))) {
        ExecuteCommand(cmd_name);
        return;
    }

    // Movement keys may be wired directly to STM32, and TJC touch-key frames can also appear here.
    // They are intentionally ignored by ESP32 to keep this UART for business commands only.
    ESP_LOGD(TAG, "Ignore non-ESP32 serial frame, len=%d", rx_index);
}

static void FeedRxByte(uint8_t byte) {
    if (byte == 0xFF) {
        if (++ff_count >= 3) {
            rx_index = 0;
            ff_count = 0;
        }
        return;
    }
    ff_count = 0;

    if (rx_index == 0) {
        if (byte != 'C' && byte != 0x55 && byte != 'K' && byte != 'Z' && byte != 0x65) {
            return;
        }
    }

    if (rx_index >= UART_BUF_SIZE - 1) {
        ESP_LOGW(TAG, "RX buffer overflow, reset");
        rx_index = 0;
        return;
    }

    rx_buffer[rx_index++] = byte;

    if (rx_index >= 2 && rx_buffer[rx_index - 2] == 0x0D && rx_buffer[rx_index - 1] == 0x0A) {
        ProcessFrame();
        rx_index = 0;
    }
}

static void SerialDisplayRxTask(void* arg) {
    uint8_t data[128];

    while (true) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), 20 / portTICK_PERIOD_MS);
        for (int i = 0; i < len; ++i) {
            FeedRxByte(data[i]);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void InitSerialDisplay() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_flush(UART_NUM);
    SerialDisplay_SendCommand("bkcmd=0");

    xTaskCreate(SerialDisplayRxTask, "serial_display_rx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Serial display initialized (TX=%d, RX=%d, baud=%d)", UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

void RegisterMcpSerialDisplay() {
    ESP_LOGI(TAG, "Serial display uses local UART only; MCP tools are not registered");
}


