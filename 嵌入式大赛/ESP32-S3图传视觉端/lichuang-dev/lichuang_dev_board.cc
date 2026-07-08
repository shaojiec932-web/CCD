#include "wifi_board.h"

#include "codecs/box_audio_codec.h"

#include "display/lcd_display.h"

#include "display/lvgl_display/lvgl_theme.h"

#include "display/emote_display.h"

#include "application.h"

#include "button.h"

#include "config.h"

#include "i2c_device.h"

#include "esp32_camera.h"

#include "led_control.h"

#include <mqtt.h>

#include <mbedtls/base64.h>

#include <img_converters.h>

#include <cstdlib>

#include <cstdio>

#include <memory>

#include <string>

// #include "mcp_uart_tools.h"  // 已注释：串口功能被房间控制替�?

#include <esp_log.h>

#include <esp_lcd_panel_vendor.h>

#include <esp_lcd_ili9341.h>

#include <driver/i2c_master.h>

#include <driver/spi_common.h>

#include <esp_lcd_touch_ft5x06.h>

#include <esp_lvgl_port.h>

#include <lvgl.h>

#include <freertos/FreeRTOS.h>

#include <freertos/task.h>

#define TAG "LichuangDevBoard"

#define TRACKDOCK_CAMERA_HOST "od90f9ff.ala.dedicated.aliyun.emqxcloud.cn"

#define TRACKDOCK_CAMERA_PORT 8883

#define TRACKDOCK_CAMERA_USERNAME "admin"

#define TRACKDOCK_CAMERA_PASSWORD "1234"

#define TRACKDOCK_CAMERA_PREFIX "trackdock/camera001"

#define TRACKDOCK_CAMERA_INTERVAL_MS 33

class TrackDockVisionDisplay : public SpiLcdDisplay {
private:
    struct SceneOption {
        const char* key;
        const char* name;
    };

    static constexpr int kSceneCount = 6;
    static const SceneOption* Scenes() {
        static const SceneOption scenes[kSceneCount] = {
            {"factory", "Factory"},
            {"hospital", "Hospital"},
            {"eldercare", "Elder"},
            {"library", "Library"},
            {"teaching", "School"},
            {"default", "Default"},
        };
        return scenes;
    }

    lv_obj_t* eye_page_ = nullptr;
    lv_obj_t* console_page_ = nullptr;
    lv_obj_t* scene_label_ = nullptr;
    lv_obj_t* scene_buttons_[kSceneCount] = {};
    lv_timer_t* console_timeout_timer_ = nullptr;
    bool console_visible_ = false;
    int active_scene_ = 5;

    lv_obj_t* CreateText(lv_obj_t* parent, const char* text, lv_color_t color, lv_text_align_t align) {
        lv_obj_t* label = lv_label_create(parent);
        lv_label_set_text(label, text);
        lv_obj_set_width(label, LV_PCT(100));
        lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_color(label, color, 0);
        lv_obj_set_style_text_align(label, align, 0);
        if (current_theme_) {
            auto theme = static_cast<LvglTheme*>(current_theme_);
            lv_obj_set_style_text_font(label, theme->text_font()->font(), 0);
        }
        return label;
    }

    void StylePage(lv_obj_t* page, lv_color_t color) {
        lv_obj_set_size(page, LV_HOR_RES, LV_VER_RES);
        lv_obj_set_style_radius(page, 0, 0);
        lv_obj_set_style_bg_color(page, color, 0);
        lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(page, 0, 0);
        lv_obj_set_style_pad_all(page, 0, 0);
        lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_OFF);
    }

    lv_obj_t* CreateEye(lv_obj_t* parent, int x) {
        lv_obj_t* eye = lv_obj_create(parent);
        lv_obj_set_size(eye, 92, 58);
        lv_obj_align(eye, LV_ALIGN_CENTER, x, -10);
        lv_obj_set_style_radius(eye, 29, 0);
        lv_obj_set_style_bg_color(eye, lv_color_hex(0xEAF7FF), 0);
        lv_obj_set_style_bg_opa(eye, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(eye, 3, 0);
        lv_obj_set_style_border_color(eye, lv_color_hex(0x5EC7FF), 0);
        lv_obj_set_scrollbar_mode(eye, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_flag(eye, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(eye, LV_OBJ_FLAG_EVENT_BUBBLE);

        lv_obj_t* pupil = lv_obj_create(eye);
        lv_obj_set_size(pupil, 30, 30);
        lv_obj_center(pupil);
        lv_obj_set_style_radius(pupil, 15, 0);
        lv_obj_set_style_bg_color(pupil, lv_color_hex(0x101721), 0);
        lv_obj_set_style_bg_opa(pupil, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(pupil, 0, 0);
        lv_obj_set_scrollbar_mode(pupil, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_flag(pupil, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(pupil, LV_OBJ_FLAG_EVENT_BUBBLE);

        lv_obj_t* shine = lv_obj_create(pupil);
        lv_obj_set_size(shine, 8, 8);
        lv_obj_align(shine, LV_ALIGN_TOP_LEFT, 6, 5);
        lv_obj_set_style_radius(shine, 4, 0);
        lv_obj_set_style_bg_color(shine, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(shine, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(shine, 0, 0);
        lv_obj_set_scrollbar_mode(shine, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_flag(shine, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(shine, LV_OBJ_FLAG_EVENT_BUBBLE);
        return eye;
    }

    void BuildEyePage(lv_obj_t* screen) {
        eye_page_ = lv_obj_create(screen);
        StylePage(eye_page_, lv_color_hex(0x071019));
        lv_obj_add_flag(eye_page_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(eye_page_, [](lv_event_t* e) {
            auto display = static_cast<TrackDockVisionDisplay*>(lv_event_get_user_data(e));
            if (display) {
                display->ShowConsole();
            }
        }, LV_EVENT_CLICKED, this);

        CreateEye(eye_page_, -58);
        CreateEye(eye_page_, 58);

        lv_obj_t* hint = CreateText(eye_page_, "TarckDock Vision", lv_color_hex(0xB8C7D6), LV_TEXT_ALIGN_CENTER);
        lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -28);
        lv_obj_t* tap = CreateText(eye_page_, "tap to select scene", lv_color_hex(0x5EC7FF), LV_TEXT_ALIGN_CENTER);
        lv_obj_align(tap, LV_ALIGN_BOTTOM_MID, 0, -8);
    }

    lv_obj_t* CreateSceneButton(lv_obj_t* parent, int index) {
        const auto* scenes = Scenes();
        lv_obj_t* btn = lv_btn_create(parent);
        lv_obj_set_size(btn, 96, 32);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x2D3440), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x151A22), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_set_scrollbar_mode(btn, LV_SCROLLBAR_MODE_OFF);

        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text(label, scenes[index].name);
        lv_obj_center(label);
        lv_obj_set_style_text_color(label, lv_color_hex(0xE8EEF5), 0);
        if (current_theme_) {
            auto theme = static_cast<LvglTheme*>(current_theme_);
            lv_obj_set_style_text_font(label, theme->text_font()->font(), 0);
        }

        lv_obj_add_event_cb(btn, [](lv_event_t* e) {
            auto display = static_cast<TrackDockVisionDisplay*>(lv_event_get_user_data(e));
            int selected = (int)(intptr_t)lv_obj_get_user_data((lv_obj_t*)lv_event_get_target(e));
            if (display) {
                display->SetScene(selected);
            }
        }, LV_EVENT_CLICKED, this);
        lv_obj_set_user_data(btn, (void*)(intptr_t)index);
        return btn;
    }

    void RefreshSceneButtons() {
        const auto* scenes = Scenes();
        for (int i = 0; i < kSceneCount; ++i) {
            if (!scene_buttons_[i]) continue;
            bool active = (i == active_scene_);
            lv_obj_set_style_bg_color(scene_buttons_[i], active ? lv_color_hex(0x1F7A4D) : lv_color_hex(0x151A22), 0);
            lv_obj_set_style_border_color(scene_buttons_[i], active ? lv_color_hex(0x57D68D) : lv_color_hex(0x2D3440), 0);
        }
        if (scene_label_) {
            char text[64];
            snprintf(text, sizeof(text), "Scene  %s", scenes[active_scene_].name);
            lv_label_set_text(scene_label_, text);
        }
    }

    void ResetConsoleTimeout() {
        if (console_timeout_timer_) {
            lv_timer_reset(console_timeout_timer_);
            lv_timer_resume(console_timeout_timer_);
        }
    }

    void HideConsole() {
        console_visible_ = false;
        if (console_timeout_timer_) {
            lv_timer_pause(console_timeout_timer_);
        }
        if (console_page_) {
            lv_obj_add_flag(console_page_, LV_OBJ_FLAG_HIDDEN);
        }
        if (eye_page_) {
            lv_obj_remove_flag(eye_page_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(eye_page_);
        }
    }

    void BuildConsolePage(lv_obj_t* screen) {
        console_page_ = lv_obj_create(screen);
        StylePage(console_page_, lv_color_hex(0x0B0F14));
        lv_obj_add_flag(console_page_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(console_page_, [](lv_event_t* e) {
            auto display = static_cast<TrackDockVisionDisplay*>(lv_event_get_user_data(e));
            if (display) {
                display->ResetConsoleTimeout();
            }
        }, LV_EVENT_CLICKED, this);
        lv_obj_set_style_pad_all(console_page_, 10, 0);
        lv_obj_set_flex_flow(console_page_, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(console_page_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_row(console_page_, 14, 0);

        lv_obj_t* top = lv_obj_create(console_page_);
        lv_obj_set_width(top, LV_PCT(100));
        lv_obj_set_height(top, 46);
        lv_obj_set_style_radius(top, 6, 0);
        lv_obj_set_style_bg_color(top, lv_color_hex(0x111820), 0);
        lv_obj_set_style_border_width(top, 1, 0);
        lv_obj_set_style_border_color(top, lv_color_hex(0x2D3440), 0);
        lv_obj_set_style_pad_all(top, 6, 0);
        lv_obj_set_scrollbar_mode(top, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_flex_flow(top, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(top, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        CreateText(top, "TarckDock", lv_color_hex(0xF3F6FA), LV_TEXT_ALIGN_CENTER);
        scene_label_ = CreateText(top, "Scene  Default", lv_color_hex(0x57D68D), LV_TEXT_ALIGN_CENTER);

        lv_obj_t* grid = lv_obj_create(console_page_);
        lv_obj_set_width(grid, LV_PCT(100));
        lv_obj_set_height(grid, 98);
        lv_obj_set_style_radius(grid, 6, 0);
        lv_obj_set_style_bg_color(grid, lv_color_hex(0x0F141B), 0);
        lv_obj_set_style_border_width(grid, 1, 0);
        lv_obj_set_style_border_color(grid, lv_color_hex(0x2D3440), 0);
        lv_obj_set_style_pad_all(grid, 7, 0);
        lv_obj_set_style_pad_row(grid, 8, 0);
        lv_obj_set_style_pad_column(grid, 5, 0);
        lv_obj_set_scrollbar_mode(grid, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
        lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        for (int i = 0; i < kSceneCount; ++i) {
            scene_buttons_[i] = CreateSceneButton(grid, i);
        }

        RefreshSceneButtons();
        lv_obj_add_flag(console_page_, LV_OBJ_FLAG_HIDDEN);

        console_timeout_timer_ = lv_timer_create([](lv_timer_t* timer) {
            auto display = static_cast<TrackDockVisionDisplay*>(lv_timer_get_user_data(timer));
            if (display) {
                display->HideConsole();
            }
        }, 10000, this);
        lv_timer_pause(console_timeout_timer_);
    }

public:
    TrackDockVisionDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                           int width, int height, int offset_x, int offset_y,
                           bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {}

    virtual void SetupUI() override {
        LcdDisplay::SetupUI();
        DisplayLockGuard lock(this);
        auto screen = lv_screen_active();
        BuildEyePage(screen);
        BuildConsolePage(screen);
        lv_obj_move_foreground(eye_page_);
    }

    virtual void SetTheme(Theme* theme) override {
        LcdDisplay::SetTheme(theme);
        DisplayLockGuard lock(this);
        RefreshSceneButtons();
        if (console_visible_ && console_page_) {
            lv_obj_move_foreground(console_page_);
        } else if (eye_page_) {
            lv_obj_move_foreground(eye_page_);
        }
    }

    virtual void SetEmotion(const char* emotion) override {
        LcdDisplay::SetEmotion(emotion);
        DisplayLockGuard lock(this);
        if (console_visible_ && console_page_) {
            lv_obj_move_foreground(console_page_);
        } else if (eye_page_) {
            lv_obj_move_foreground(eye_page_);
        }
    }

    void ShowConsole() {
        console_visible_ = true;
        ResetConsoleTimeout();
        if (eye_page_) {
            lv_obj_add_flag(eye_page_, LV_OBJ_FLAG_HIDDEN);
        }
        if (console_page_) {
            lv_obj_remove_flag(console_page_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(console_page_);
        }
    }


    void SetScene(int index) {
        if (index < 0 || index >= kSceneCount) return;
        DisplayLockGuard lock(this);
        active_scene_ = index;
        RefreshSceneButtons();
        ResetConsoleTimeout();
        ShowConsole();
        ESP_LOGI(TAG, "Vision scene selected: %s", ActiveSceneKey());
    }

    const char* ActiveSceneKey() const { return Scenes()[active_scene_].key; }
    const char* ActiveSceneName() const { return Scenes()[active_scene_].name; }
};

static TrackDockVisionDisplay* g_vision_display = nullptr;

static std::string Base64Encode(const uint8_t* data, size_t len) {

    if (!data || len == 0) return "";

    size_t out_len = 0;

    mbedtls_base64_encode(nullptr, 0, &out_len, data, len);

    std::string out(out_len, '\0');

    if (mbedtls_base64_encode(reinterpret_cast<unsigned char*>(&out[0]), out.size(), &out_len, data, len) != 0) {

        return "";

    }

    out.resize(out_len);

    return out;

}

static bool PublishCameraState(Mqtt* mqtt, const char* state) {
    if (!mqtt || !mqtt->IsConnected()) return false;
    std::string payload = std::string("{\"device\":\"camera001\",\"state\":\"") + state + "\"";
    if (g_vision_display) {
        payload += ",\"scene\":\"";
        payload += g_vision_display->ActiveSceneKey();
        payload += "\",\"scene_name\":\"";
        payload += g_vision_display->ActiveSceneName();
        payload += "\"";
    }
    payload += "}";
    return mqtt->Publish(std::string(TRACKDOCK_CAMERA_PREFIX) + "/state", payload, 0);
}

static void TrackDockCameraMqttTask(void* arg) {

    uint32_t frame_id = 0;

    std::unique_ptr<Mqtt> mqtt;

    bool announced = false;

    vTaskDelay(pdMS_TO_TICKS(8000));

    while (true) {

        if (!mqtt || !mqtt->IsConnected()) {

            announced = false;

            auto network = Board::GetInstance().GetNetwork();

            if (!network) {

                vTaskDelay(pdMS_TO_TICKS(3000));

                continue;

            }

            mqtt = network->CreateMqtt(1);

            mqtt->SetKeepAlive(60);

            mqtt->OnConnected([&announced]() {

                announced = false;

                ESP_LOGI(TAG, "TrackDock camera MQTT connected");

            });

            mqtt->OnDisconnected([]() {

                ESP_LOGW(TAG, "TrackDock camera MQTT disconnected");

            });

            mqtt->OnError([](const std::string& error) {

                ESP_LOGW(TAG, "TrackDock camera MQTT error: %s", error.c_str());

            });

            std::string client_id = std::string("trackdock-camera-") + Board::GetInstance().GetUuid();

            ESP_LOGI(TAG, "Connecting TrackDock camera MQTT: %s:%d", TRACKDOCK_CAMERA_HOST, TRACKDOCK_CAMERA_PORT);

            if (!mqtt->Connect(TRACKDOCK_CAMERA_HOST, TRACKDOCK_CAMERA_PORT, client_id,

                               TRACKDOCK_CAMERA_USERNAME, TRACKDOCK_CAMERA_PASSWORD)) {

                ESP_LOGW(TAG, "TrackDock camera MQTT connect failed, retry later");

                mqtt.reset();

                vTaskDelay(pdMS_TO_TICKS(5000));

                continue;

            }

        }

        if (!announced) {

            PublishCameraState(mqtt.get(), "online");

            announced = true;

        }

        camera_fb_t* fb = esp_camera_fb_get();

        if (!fb) {

            ESP_LOGW(TAG, "Camera frame capture failed");

            PublishCameraState(mqtt.get(), "capture_failed");

            vTaskDelay(pdMS_TO_TICKS(1000));

            continue;

        }

        const uint8_t* jpg_data = fb->buf;

        size_t jpg_len = fb->len;

        uint8_t* converted_jpg = nullptr;

        if (fb->format != PIXFORMAT_JPEG) {

            if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format, 55, &converted_jpg, &jpg_len)) {

                ESP_LOGW(TAG, "Camera frame convert to JPEG failed, format=%d", fb->format);

                esp_camera_fb_return(fb);

                vTaskDelay(pdMS_TO_TICKS(1000));

                continue;

            }

            jpg_data = converted_jpg;

        }

        std::string image = Base64Encode(jpg_data, jpg_len);

        if (!image.empty()) {

            std::string payload;

            payload.reserve(image.size() + 180);

            payload = "{\"device\":\"camera001\",\"format\":\"jpeg\"";
            if (g_vision_display) {
                payload += ",\"scene\":\"";
                payload += g_vision_display->ActiveSceneKey();
                payload += "\",\"scene_name\":\"";
                payload += g_vision_display->ActiveSceneName();
                payload += "\"";
            }
            payload += ",\"width\":";

            payload += std::to_string(fb->width);

            payload += ",\"height\":";

            payload += std::to_string(fb->height);

            payload += ",\"seq\":";

            payload += std::to_string(++frame_id);

            payload += ",\"image\":\"data:image/jpeg;base64,";

            payload += image;

            payload += "\"}";

            mqtt->Publish(std::string(TRACKDOCK_CAMERA_PREFIX) + "/frame", payload, 0);

        }

        if (frame_id % 30 == 0) {

            ESP_LOGI(TAG, "Camera frame published: %ux%u, jpeg=%u, seq=%lu",

                     (unsigned)fb->width, (unsigned)fb->height, (unsigned)jpg_len, (unsigned long)frame_id);

        }

        if (converted_jpg) {

            free(converted_jpg);

        }

        esp_camera_fb_return(fb);

        vTaskDelay(pdMS_TO_TICKS(TRACKDOCK_CAMERA_INTERVAL_MS));

    }

}

class Pca9557 : public I2cDevice {

public:

    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {

        WriteReg(0x01, 0x03);

        WriteReg(0x03, 0xf8);

    }

    void SetOutputState(uint8_t bit, uint8_t level) {

        uint8_t data = ReadReg(0x01);

        data = (data & ~(1 << bit)) | (level << bit);

        WriteReg(0x01, data);

    }

};

class CustomAudioCodec : public BoxAudioCodec {

private:

    Pca9557* pca9557_;

public:

    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557) 

        : BoxAudioCodec(i2c_bus, 

                       AUDIO_INPUT_SAMPLE_RATE, 

                       AUDIO_OUTPUT_SAMPLE_RATE,

                       AUDIO_I2S_GPIO_MCLK, 

                       AUDIO_I2S_GPIO_BCLK, 

                       AUDIO_I2S_GPIO_WS, 

                       AUDIO_I2S_GPIO_DOUT, 

                       AUDIO_I2S_GPIO_DIN,

                       GPIO_NUM_NC, 

                       AUDIO_CODEC_ES8311_ADDR, 

                       AUDIO_CODEC_ES7210_ADDR, 

                       AUDIO_INPUT_REFERENCE),

          pca9557_(pca9557) {

    }

    virtual void EnableOutput(bool enable) override {

        BoxAudioCodec::EnableOutput(enable);

        if (enable) {

            pca9557_->SetOutputState(1, 1);

        } else {

            pca9557_->SetOutputState(1, 0);

        }

    }

};

class LichuangDevBoard : public WifiBoard {

private:

    i2c_master_bus_handle_t i2c_bus_;

    i2c_master_dev_handle_t pca9557_handle_;

    Button boot_button_;

    Display* display_;

    Pca9557* pca9557_;

    Esp32Camera* camera_;

    LedControl* led_control_;

    void InitializeI2c() {

        // Initialize I2C peripheral

        i2c_master_bus_config_t i2c_bus_cfg = {

            .i2c_port = (i2c_port_t)1,

            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,

            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,

            .clk_source = I2C_CLK_SRC_DEFAULT,

            .glitch_ignore_cnt = 7,

            .intr_priority = 0,

            .trans_queue_depth = 0,

            .flags = {

                .enable_internal_pullup = 1,

            },

        };

        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557

        pca9557_ = new Pca9557(i2c_bus_, 0x19);

    }

    void InitializeSpi() {

        spi_bus_config_t buscfg = {};

        buscfg.mosi_io_num = GPIO_NUM_40;

        buscfg.miso_io_num = GPIO_NUM_NC;

        buscfg.sclk_io_num = GPIO_NUM_41;

        buscfg.quadwp_io_num = GPIO_NUM_NC;

        buscfg.quadhd_io_num = GPIO_NUM_NC;

        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);

        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    }

    void InitializeButtons() {

        boot_button_.OnClick([this]() {

            auto& app = Application::GetInstance();

            // During startup (before connected), pressing BOOT button enters Wi-Fi config mode without reboot

            if (app.GetDeviceState() == kDeviceStateStarting) {

                EnterWifiConfigMode();

                return;

            }

            app.ToggleChatState();

        });

#if CONFIG_USE_DEVICE_AEC

        boot_button_.OnDoubleClick([this]() {

            auto& app = Application::GetInstance();

            if (app.GetDeviceState() == kDeviceStateIdle) {

                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);

            }

        });

#endif

    }

    void InitializeSt7789Display() {

        esp_lcd_panel_io_handle_t panel_io = nullptr;

        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始�?        ESP_LOGD(TAG, "Install panel IO");

        esp_lcd_panel_io_spi_config_t io_config = {};

        io_config.cs_gpio_num = GPIO_NUM_NC;

        io_config.dc_gpio_num = GPIO_NUM_39;

        io_config.spi_mode = 2;

        io_config.pclk_hz = 80 * 1000 * 1000;

        io_config.trans_queue_depth = 10;

        io_config.lcd_cmd_bits = 8;

        io_config.lcd_param_bits = 8;

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789

        ESP_LOGD(TAG, "Install LCD driver");

        esp_lcd_panel_dev_config_t panel_config = {};

        panel_config.reset_gpio_num = GPIO_NUM_NC;

        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;

        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        

        esp_lcd_panel_reset(panel);

        pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);

        esp_lcd_panel_invert_color(panel, true);

        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);

        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        esp_lcd_panel_disp_on_off(panel, true);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE

        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);

#else

        g_vision_display = new TrackDockVisionDisplay(panel_io, panel,

            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);

        display_ = g_vision_display;

#endif

    }

    void InitializeTouch()

    {

        esp_lcd_touch_handle_t tp;

        esp_lcd_touch_config_t tp_cfg = {

            .x_max = DISPLAY_HEIGHT,

            .y_max = DISPLAY_WIDTH,

            .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset

            .int_gpio_num = GPIO_NUM_NC, 

            .levels = {

                .reset = 0,

                .interrupt = 0,

            },

            .flags = {

                .swap_xy = 1,

                .mirror_x = 1,

                .mirror_y = 0,

            },

        };

        esp_lcd_panel_io_handle_t tp_io_handle = NULL;

        esp_lcd_panel_io_i2c_config_t tp_io_config = {

            .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS,

            .control_phase_bytes = 1,

            .dc_bit_offset = 0,

            .lcd_cmd_bits = 8,

            .flags =

            {

                .disable_control_phase = 1,

            }

        };

        tp_io_config.scl_speed_hz = 400000;

        esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);

        esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp);

        assert(tp);

        /* Add touch input (for selected screen) */

        const lvgl_port_touch_cfg_t touch_cfg = {

            .disp = lv_display_get_default(), 

            .handle = tp,

        };

        if(touch_cfg.disp) {

            lvgl_port_add_touch(&touch_cfg);

        } else {

            ESP_LOGE(TAG, "Touch display is not initialized");

        }

    }

    void InitializeCamera() {

        // Open camera power

        pca9557_->SetOutputState(2, 0);

        camera_config_t config = {};

        config.ledc_channel = LEDC_CHANNEL_2;

        config.ledc_timer = LEDC_TIMER_2;

        config.pin_d0 = CAMERA_PIN_D0;

        config.pin_d1 = CAMERA_PIN_D1;

        config.pin_d2 = CAMERA_PIN_D2;

        config.pin_d3 = CAMERA_PIN_D3;

        config.pin_d4 = CAMERA_PIN_D4;

        config.pin_d5 = CAMERA_PIN_D5;

        config.pin_d6 = CAMERA_PIN_D6;

        config.pin_d7 = CAMERA_PIN_D7;

        config.pin_xclk = CAMERA_PIN_XCLK;

        config.pin_pclk = CAMERA_PIN_PCLK;

        config.pin_vsync = CAMERA_PIN_VSYNC;

        config.pin_href = CAMERA_PIN_HREF;

        config.pin_sccb_sda = -1;

        config.pin_sccb_scl = -1;

        config.sccb_i2c_port = 1;

        config.pin_pwdn = CAMERA_PIN_PWDN;

        config.pin_reset = CAMERA_PIN_RESET;

        config.xclk_freq_hz = XCLK_FREQ_HZ;

        config.pixel_format = PIXFORMAT_RGB565;

        config.frame_size = FRAMESIZE_QVGA;

        config.jpeg_quality = 12;

        config.fb_count = 2;

        config.fb_location = CAMERA_FB_IN_PSRAM;

        config.grab_mode = CAMERA_GRAB_LATEST;

        camera_ = new Esp32Camera(config);

    }

    void InitializeLed() {

        // 注意：IO10和IO11用作房间控制，不再用作LED控制

        led_control_ = nullptr;

        ESP_LOGI(TAG, "LED control disabled (IO10/IO11 used for room control)");

    }

    void InitializeTools() {

        ESP_LOGI(TAG, "Vision node mode: board MCP tools are disabled");

    }

    void InitializeCameraMqtt() {

        xTaskCreate(TrackDockCameraMqttTask, "trackdock_camera", 8192, NULL, 4, NULL);

        ESP_LOGI(TAG, "TrackDock camera MQTT task started");
    }

public:

    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO), led_control_(nullptr) {

        InitializeI2c();

        InitializeSpi();

        InitializeSt7789Display();

        InitializeTouch();

        InitializeButtons();

        InitializeCamera();

        InitializeLed();

        InitializeTools();

        InitializeCameraMqtt();

        GetBacklight()->RestoreBrightness();

    }

    ~LichuangDevBoard() {

        if (led_control_) {

            delete led_control_;

        }

    }

    virtual AudioCodec* GetAudioCodec() override {

        static CustomAudioCodec audio_codec(

            i2c_bus_, 

            pca9557_);

        return &audio_codec;

    }

    virtual Display* GetDisplay() override {

        return display_;

    }

    

    virtual Backlight* GetBacklight() override {

        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);

        return &backlight;

    }

    virtual Camera* GetCamera() override {

        return camera_;

    }

};

DECLARE_BOARD(LichuangDevBoard);


