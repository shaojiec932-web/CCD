#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "mcp_stm32_motor_control.h"  // STM32电机控制MCP工具
#include "mcp_rfid_cargo_manager.h"   // RFID货物管理MCP工具
#include "mcp_uwb_navigation.h"       // UWB定位导航MCP工具
#include "mcp_serial_display.h"       // 陶晶驰串口屏通信MCP工具
#include "mcp_storage_lid.h"
#include "trackdock_mqtt_client.h"   // TarckDock云端APP本地控制通道

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardLCD"

static void HumanSensorTask(void* arg) {
    bool last_active = false;
    TickType_t last_trigger_tick = 0;

    while (true) {
        bool active = gpio_get_level(HUMAN_SENSOR_GPIO) == HUMAN_SENSOR_ACTIVE_LEVEL;
        TickType_t now = xTaskGetTickCount();

        if (active && !last_active && (now - last_trigger_tick) > pdMS_TO_TICKS(5000)) {
            last_trigger_tick = now;
            auto& app = Application::GetInstance();

            if (Stm32MotorIsRunning()) {
                Stm32MotorEmergencyStop();
                SerialDisplay_ShowDialogText("Person detected, car stopped");
                ESP_LOGW(TAG, "Human detected while car running, emergency stop sent");
            } else {
                SerialDisplay_ShowDialogText("Hello, I am here");
                app.Alert("Hi", "Hello, I am here", "happy");
                app.StartListening();
                ESP_LOGI(TAG, "Human detected, greeting and start listening");
            }
        }

        last_active = active;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void SetBuzzer(bool active) {
    gpio_set_level(BUZZER_GPIO, active ? BUZZER_ACTIVE_LEVEL : !BUZZER_ACTIVE_LEVEL);
}

static void SmokeAlarmTask(void* arg) {
    bool last_alarm = false;
    TickType_t last_alarm_tick = 0;

    SetBuzzer(false);

    while (true) {
        bool smoke1 = gpio_get_level(SMOKE_SENSOR_1_GPIO) == SMOKE_SENSOR_ACTIVE_LEVEL;
        bool smoke2 = gpio_get_level(SMOKE_SENSOR_2_GPIO) == SMOKE_SENSOR_ACTIVE_LEVEL;
        bool alarm = smoke1 || smoke2;
        TickType_t now = xTaskGetTickCount();

        if (alarm) {
            SetBuzzer(true);
        } else {
            SetBuzzer(false);
        }

        if (alarm && (!last_alarm || (now - last_alarm_tick) > pdMS_TO_TICKS(10000))) {
            last_alarm_tick = now;

            if (Stm32MotorIsRunning()) {
                Stm32MotorEmergencyStop();
            }

            const char* message = smoke1 && smoke2
                ? "工厂烟雾报警：两个传感器均触发，请立即检查现场。"
                : (smoke1 ? "工厂烟雾报警：1号传感器触发，请立即检查现场。"
                          : "工厂烟雾报警：2号传感器触发，请立即检查现场。");

            Application::GetInstance().Alert("烟雾报警", message, "warning");
            TrackDockMqtt_PublishEvent(message);
            TrackDockMqtt_PublishState();
            ESP_LOGW(TAG, "Smoke alarm triggered, smoke1=%d, smoke2=%d", smoke1, smoke2);
        }

        last_alarm = alarm;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


class CompactWifiBoardLCD : public WifiBoard {
private:
 
    Button boot_button_;
    Button dialog_start_button_;
    Button dialog_stop_button_;
    i2c_master_bus_handle_t display_i2c_bus_ = nullptr;
    esp_lcd_panel_io_handle_t oled_panel_io_ = nullptr;
    esp_lcd_panel_handle_t oled_panel_ = nullptr;
    Display* display_ = nullptr;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = OLED_DISPLAY_SDA_PIN,
            .scl_io_num = OLED_DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeOledDisplay() {
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = 400 * 1000,
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &oled_panel_io_));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(OLED_DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(oled_panel_io_, &panel_config, &oled_panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(oled_panel_));

        if (esp_lcd_panel_init(oled_panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SSD1306 display");
            display_ = new NoDisplay();
            return;
        }

        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(oled_panel_, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(oled_panel_, true));

        display_ = new OledDisplay(oled_panel_io_, oled_panel_, OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT,
                                   OLED_DISPLAY_MIRROR_X, OLED_DISPLAY_MIRROR_Y);
        ESP_LOGI(TAG, "SSD1306 OLED initialized (SDA=%d, SCL=%d, %dx%d)",
                 OLED_DISPLAY_SDA_PIN, OLED_DISPLAY_SCL_PIN, OLED_DISPLAY_WIDTH, OLED_DISPLAY_HEIGHT);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
#elif defined(LCD_TYPE_ST7735_SERIAL)
        // ST7735 使用 ST7789 驱动（协议兼容）
        ESP_LOGI(TAG, "Initializing ST7735 LCD (using ST7789 driver)");
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeHumanSensor() {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << HUMAN_SENSOR_GPIO);
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        xTaskCreate(HumanSensorTask, "human_sensor", 3072, NULL, 4, NULL);
        ESP_LOGI(TAG, "Human sensor initialized on GPIO%d, active level=%d", HUMAN_SENSOR_GPIO, HUMAN_SENSOR_ACTIVE_LEVEL);
    }

    void InitializeSmokeAlarm() {
        gpio_config_t sensor_conf = {};
        sensor_conf.intr_type = GPIO_INTR_DISABLE;
        sensor_conf.mode = GPIO_MODE_INPUT;
        sensor_conf.pin_bit_mask = (1ULL << SMOKE_SENSOR_1_GPIO) | (1ULL << SMOKE_SENSOR_2_GPIO);
        sensor_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        sensor_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&sensor_conf);

        gpio_config_t buzzer_conf = {};
        buzzer_conf.intr_type = GPIO_INTR_DISABLE;
        buzzer_conf.mode = GPIO_MODE_OUTPUT;
        buzzer_conf.pin_bit_mask = (1ULL << BUZZER_GPIO);
        buzzer_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        buzzer_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&buzzer_conf);
        SetBuzzer(false);

        xTaskCreate(SmokeAlarmTask, "smoke_alarm", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "Smoke alarm initialized (S1=%d, S2=%d, buzzer=%d active-low)",
                 SMOKE_SENSOR_1_GPIO, SMOKE_SENSOR_2_GPIO, BUZZER_GPIO);
    }
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        dialog_start_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                return;
            }
            app.StartListening();
            ESP_LOGI(TAG, "GPIO1 dialog start button clicked");
        });

        dialog_stop_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                return;
            }
            app.StopListening();
            ESP_LOGI(TAG, "GPIO46 dialog stop button clicked");
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeTools() {

        // RFID货物管理：扫描RFID标签，管理货物收纳和运输
        RegisterMcpRfidCargoManager();

        // STM32电机控制：手动控制和自动驾驶
        RegisterMcpStm32MotorControl();

        // UWB定位导航：室内定位和路径规划
        RegisterMcpUwbNavigation();

        // 收纳盖舵机：RFID扫描成功自动开盖，语音/客户端可关闭
        RegisterMcpStorageLid();

        // 陶晶驰串口屏：显示状态和接收触控指令
        InitSerialDisplay();

        // TarckDock APP：网页/手机端本地控制通道，不占用MCP工具数量
        InitTrackDockMqttClient();
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO),
        dialog_start_button_(TOUCH_BUTTON_GPIO),
        dialog_stop_button_(USER_BUTTON_1_GPIO) {
        InitializeDisplayI2c();
        InitializeOledDisplay();
        InitializeButtons();
        InitializeTools();
        InitializeHumanSensor();
        InitializeSmokeAlarm();
        
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);

#define MCP_STORAGE_LID_IMPLEMENTATION
#include "mcp_storage_lid.cc"

#define TRACKDOCK_SCENE_IMPLEMENTATION
#include "trackdock_scene.cc"

#define TRACKDOCK_MQTT_CLIENT_IMPLEMENTATION
#include "trackdock_mqtt_client.cc"

