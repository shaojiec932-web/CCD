#ifdef MCP_STORAGE_LID_IMPLEMENTATION
#include "mcp_storage_lid.h"
#include "config.h"
#include "application.h"
#include "mcp_server.h"
#include "trackdock_mqtt_client.h"

#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <string>

#define TAG "STORAGE_LID"

static bool lid_initialized = false;
static bool lid_open = false;
static TimerHandle_t auto_close_timer = NULL;

static constexpr ledc_mode_t SERVO_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t SERVO_LEDC_TIMER = LEDC_TIMER_3;
static constexpr ledc_channel_t SERVO_LEDC_CHANNEL = LEDC_CHANNEL_7;
static constexpr int SERVO_FREQ_HZ = 50;
static constexpr int SERVO_DUTY_RES = 14;
static constexpr int SERVO_DUTY_MAX = (1 << SERVO_DUTY_RES) - 1;
static constexpr int SERVO_MIN_US = 500;
static constexpr int SERVO_MAX_US = 2500;
static constexpr int SERVO_PERIOD_US = 1000000 / SERVO_FREQ_HZ;

static uint32_t DegreeToDuty(int degree) {
    if (degree < 0) degree = 0;
    if (degree > 180) degree = 180;
    int pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * degree / 180;
    return (uint32_t)((pulse_us * SERVO_DUTY_MAX) / SERVO_PERIOD_US);
}

static void SetServoDegree(int degree) {
    StorageLidInit();
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, DegreeToDuty(degree));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

static void AutoCloseCallback(TimerHandle_t timer) {
    StorageLidClose(true);
}

static void StartAutoCloseTimer() {
    if (auto_close_timer == NULL) {
        auto_close_timer = xTimerCreate("storage_lid_close", pdMS_TO_TICKS(STORAGE_LID_AUTO_CLOSE_MS),
                                        pdFALSE, NULL, AutoCloseCallback);
    }
    xTimerStop(auto_close_timer, 0);
    xTimerStart(auto_close_timer, 0);
}

static void StopAutoCloseTimer() {
    if (auto_close_timer) {
        xTimerStop(auto_close_timer, 0);
    }
}

void StorageLidInit() {
    if (lid_initialized) return;

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = SERVO_LEDC_MODE;
    timer_conf.duty_resolution = (ledc_timer_bit_t)SERVO_DUTY_RES;
    timer_conf.timer_num = SERVO_LEDC_TIMER;
    timer_conf.freq_hz = SERVO_FREQ_HZ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {};
    channel_conf.gpio_num = STORAGE_LID_SERVO_GPIO;
    channel_conf.speed_mode = SERVO_LEDC_MODE;
    channel_conf.channel = SERVO_LEDC_CHANNEL;
    channel_conf.intr_type = LEDC_INTR_DISABLE;
    channel_conf.timer_sel = SERVO_LEDC_TIMER;
    channel_conf.duty = DegreeToDuty(STORAGE_LID_SERVO_CLOSE_DEG);
    channel_conf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    lid_initialized = true;
    lid_open = false;
    ESP_LOGI(TAG, "Storage lid servo initialized on GPIO%d", STORAGE_LID_SERVO_GPIO);
}

void StorageLidOpen(bool auto_close) {
    SetServoDegree(STORAGE_LID_SERVO_OPEN_DEG);
    lid_open = true;
    ESP_LOGI(TAG, "Storage lid opened");
    TrackDockMqtt_PublishState();

    if (auto_close) {
        StartAutoCloseTimer();
    } else {
        StopAutoCloseTimer();
    }
}

void StorageLidClose(bool notify) {
    SetServoDegree(STORAGE_LID_SERVO_CLOSE_DEG);
    lid_open = false;
    StopAutoCloseTimer();
    ESP_LOGI(TAG, "Storage lid closed");
    TrackDockMqtt_PublishState();

    if (notify) {
        const char* message = "10秒内未继续扫描，收纳盖已自动关闭。";
        Application::GetInstance().Alert("收纳盖", message, "neutral");
        TrackDockMqtt_PublishDialog(message);
    }
}

bool StorageLidIsOpen() {
    return lid_open;
}

const char* StorageLidStateText() {
    return lid_open ? "open" : "closed";
}

void RegisterMcpStorageLid() {
    StorageLidInit();

    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddTool("self.storage_lid.set_state",
        "Control the TarckDock storage lid. Use state=open to open the storage cover, or state=close to close it.",
        PropertyList({
            Property("state", kPropertyTypeString)
        }), [](const PropertyList& properties) -> ReturnValue {
            std::string state = properties["state"].value<std::string>();
            if (state == "open") {
                StorageLidOpen(false);
                return "storage lid opened";
            }
            if (state == "close" || state == "closed") {
                StorageLidClose(false);
                return "storage lid closed";
            }
            return "unknown storage lid state";
        });

    ESP_LOGI(TAG, "Storage lid MCP tool registered");
}
#endif // MCP_STORAGE_LID_IMPLEMENTATION
