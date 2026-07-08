#include "mcp_rfid_cargo_manager.h"
#include "mcp_server.h"
#include "mcp_storage_lid.h"
#include "config.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
#include <vector>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>

#define TAG "MCP_RFID_CARGO"

// RFID扫描状态
enum RfidScanState {
    SCAN_IDLE,           // 空闲状态
    SCAN_COLLECTING,     // 正在收纳货物
    SCAN_READY           // 已收纳完成，准备运输
};

// 货物信息结构
struct CargoInfo {
    std::string uid;      // RFID卡片UID
    std::string name;     // 货物名称
    uint32_t timestamp;   // 添加时间戳
};

// 全局状态
static RfidScanState scan_state = SCAN_IDLE;
static std::vector<CargoInfo> cargo_list;
static spi_device_handle_t rfid_spi = NULL;
static bool rfid_initialized = false;
static led_strip_handle_t led_strip = NULL;
static std::string pending_cargo_name;

// RC522寄存器定义
#define RC522_REG_COMMAND       0x01
#define RC522_REG_COMM_IRQ      0x04
#define RC522_REG_ERROR         0x06
#define RC522_REG_STATUS2       0x08
#define RC522_REG_FIFO_DATA     0x09
#define RC522_REG_FIFO_LEVEL    0x0A
#define RC522_REG_CONTROL       0x0C
#define RC522_REG_BIT_FRAMING   0x0D
#define RC522_REG_MODE          0x11
#define RC522_REG_TX_CONTROL    0x14
#define RC522_REG_TX_ASK        0x15
#define RC522_REG_CRC_RESULT_L  0x21
#define RC522_REG_CRC_RESULT_H  0x22
#define RC522_REG_VERSION       0x37

// RC522命令
#define RC522_CMD_IDLE          0x00
#define RC522_CMD_TRANSCEIVE    0x0C
#define RC522_CMD_CALC_CRC      0x03

// PICC命令
#define PICC_CMD_REQA           0x26
#define PICC_CMD_SELECT_1       0x93
#define PICC_CMD_SELECT_2       0x20

static bool rc522_configured = false;

// 初始化RFID SPI
static void InitializeRfidSpi() {
    if (rfid_initialized) return;

    // 配置SPI总线（如果还没有初始化）
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = RFID_SPI_MOSI_PIN;
    buscfg.miso_io_num = RFID_SPI_MISO_PIN;
    buscfg.sclk_io_num = RFID_SPI_SCK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    // 初始化SPI总线（使用SPI2_HOST，因为SPI3_HOST被LCD占用）
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI总线初始化失败: %d", ret);
        return;
    }

    // 配置SPI设备（RC522使用全双工模式）
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1000000;  // 1MHz
    devcfg.mode = 0;
    devcfg.spics_io_num = RFID_SPI_CS_PIN;
    devcfg.queue_size = 7;
    devcfg.flags = 0;  // 全双工模式

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &rfid_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI设备添加失败: %d", ret);
        return;
    }

    rfid_initialized = true;
    ESP_LOGI(TAG, "RFID RC522 SPI初始化完成 (MOSI:%d, MISO:%d, SCK:%d, CS:%d)",
             RFID_SPI_MOSI_PIN, RFID_SPI_MISO_PIN, RFID_SPI_SCK_PIN, RFID_SPI_CS_PIN);
}

// 写RC522寄存器
static void RC522_WriteRegister(uint8_t reg, uint8_t value) {
    if (!rfid_initialized) return;

    uint8_t tx_data[2] = {(uint8_t)((reg << 1) & 0x7E), value};
    spi_transaction_t trans = {};
    trans.length = 16;
    trans.tx_buffer = tx_data;
    spi_device_transmit(rfid_spi, &trans);
}

// 读RC522寄存器
static uint8_t RC522_ReadRegister(uint8_t reg) {
    if (!rfid_initialized) return 0;

    uint8_t tx_data[2] = {(uint8_t)(((reg << 1) & 0x7E) | 0x80), 0x00};
    uint8_t rx_data[2] = {0};
    spi_transaction_t trans = {};
    trans.length = 16;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;
    spi_device_transmit(rfid_spi, &trans);
    return rx_data[1];
}

// 初始化RC522
static bool RC522_Init() {
    if (rc522_configured) return true;

    InitializeRfidSpi();
    if (!rfid_initialized) return false;

    // 配置WS2812 RGB LED（GPIO 48）
    if (led_strip == NULL) {
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = BUILTIN_LED_GPIO;
        strip_config.max_leds = 1;  // 只有一颗LED
        strip_config.led_model = LED_MODEL_WS2812;
        strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
        strip_config.flags.invert_out = false;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
        rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10MHz
        rmt_config.flags.with_dma = false;

        esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
        if (ret == ESP_OK) {
            led_strip_clear(led_strip);  // 初始熄灭
            ESP_LOGI(TAG, "WS2812 RGB LED初始化成功");
        } else {
            ESP_LOGE(TAG, "WS2812 RGB LED初始化失败: %d", ret);
        }
    }

    // 软复位
    RC522_WriteRegister(RC522_REG_COMMAND, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 配置定时器
    RC522_WriteRegister(0x2A, 0x8D);
    RC522_WriteRegister(0x2B, 0x3E);
    RC522_WriteRegister(0x2D, 30);
    RC522_WriteRegister(0x2C, 0);

    // 配置天线
    RC522_WriteRegister(RC522_REG_TX_ASK, 0x40);
    RC522_WriteRegister(RC522_REG_MODE, 0x3D);

    // 打开天线
    uint8_t temp = RC522_ReadRegister(RC522_REG_TX_CONTROL);
    if (!(temp & 0x03)) {
        RC522_WriteRegister(RC522_REG_TX_CONTROL, temp | 0x03);
    }

    uint8_t version = RC522_ReadRegister(RC522_REG_VERSION);
    if (version == 0x00 || version == 0xFF) {
        ESP_LOGW(TAG, "RC522版本寄存器异常: 0x%02X，请检查供电、接线和SPI引脚", version);
    }

    rc522_configured = true;
    ESP_LOGI(TAG, "RC522初始化完成, Version=0x%02X", version);
    return true;
}

static bool RC522_Transceive(const uint8_t* send_data, uint8_t send_len,
                             uint8_t* back_data, uint8_t* back_len, uint8_t* valid_bits) {
    if (!send_data || send_len == 0 || !back_len) return false;

    RC522_WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
    RC522_WriteRegister(RC522_REG_COMM_IRQ, 0x7F);
    RC522_WriteRegister(RC522_REG_FIFO_LEVEL, 0x80);

    for (uint8_t i = 0; i < send_len; ++i) {
        RC522_WriteRegister(RC522_REG_FIFO_DATA, send_data[i]);
    }

    RC522_WriteRegister(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    RC522_WriteRegister(RC522_REG_BIT_FRAMING, RC522_ReadRegister(RC522_REG_BIT_FRAMING) | 0x80);

    bool completed = false;
    for (int i = 0; i < 200; ++i) {
        uint8_t irq = RC522_ReadRegister(RC522_REG_COMM_IRQ);
        if (irq & 0x30) {
            completed = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    RC522_WriteRegister(RC522_REG_BIT_FRAMING, RC522_ReadRegister(RC522_REG_BIT_FRAMING) & 0x7F);

    if (!completed) return false;

    uint8_t error = RC522_ReadRegister(RC522_REG_ERROR);
    if (error & 0x13) return false;

    uint8_t fifo_len = RC522_ReadRegister(RC522_REG_FIFO_LEVEL);
    if (fifo_len > *back_len) fifo_len = *back_len;

    for (uint8_t i = 0; i < fifo_len; ++i) {
        if (back_data) {
            back_data[i] = RC522_ReadRegister(RC522_REG_FIFO_DATA);
        } else {
            RC522_ReadRegister(RC522_REG_FIFO_DATA);
        }
    }

    *back_len = fifo_len;
    if (valid_bits) {
        *valid_bits = RC522_ReadRegister(RC522_REG_CONTROL) & 0x07;
    }

    return true;
}

// 请求卡片
static bool RC522_Request(uint8_t* atqa) {
    RC522_WriteRegister(RC522_REG_BIT_FRAMING, 0x07);
    uint8_t req_cmd = PICC_CMD_REQA;
    uint8_t back_len = 2;
    uint8_t valid_bits = 0;
    bool ok = RC522_Transceive(&req_cmd, 1, atqa, &back_len, &valid_bits);
    return ok && back_len == 2;
}

// 读取卡片UID（ISO14443A 4字节UID）
static bool RC522_ReadUID(uint8_t* uid, uint8_t* uid_len) {
    uint8_t atqa[2];
    if (!RC522_Request(atqa)) {
        return false;
    }

    RC522_WriteRegister(RC522_REG_BIT_FRAMING, 0x00);

    uint8_t anticoll_cmd[2] = {PICC_CMD_SELECT_1, PICC_CMD_SELECT_2};
    uint8_t response[5] = {0};
    uint8_t response_len = sizeof(response);
    uint8_t valid_bits = 0;
    if (!RC522_Transceive(anticoll_cmd, sizeof(anticoll_cmd), response, &response_len, &valid_bits)) {
        return false;
    }

    if (response_len != 5) return false;

    uint8_t bcc = response[0] ^ response[1] ^ response[2] ^ response[3];
    if (bcc != response[4]) {
        ESP_LOGW(TAG, "RFID UID校验失败");
        return false;
    }

    memcpy(uid, response, 4);
    *uid_len = 4;

    return true;
}

// 扫描RFID卡片
static std::string ScanRfidCard() {
    if (!RC522_Init()) return "";

    uint8_t uid[10];
    uint8_t uid_len = 0;

    // 尝试读取卡片
    if (RC522_ReadUID(uid, &uid_len)) {
        char uid_str[32];
        snprintf(uid_str, sizeof(uid_str), "%02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3]);
        ESP_LOGI(TAG, "检测到RFID卡片: %s", uid_str);

        // 点亮WS2812 RGB LED指示刷卡成功（绿色）
        if (led_strip != NULL) {
            led_strip_set_pixel(led_strip, 0, 0, 255, 0);  // 绿色 (GRB)
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));  // LED亮500ms
            led_strip_clear(led_strip);  // 熄灭
        }

        return std::string(uid_str);
    }

    return "";
}

// 根据UID获取货物名称（可扩展为数据库查询）
static std::string GetCargoNameByUID(const std::string& uid) {
    // 简化版本：根据UID生成货物名称
    // 实际项目中应该查询数据库或配置文件
    static const char* cargo_types[] = {"电子元件", "医疗用品", "食品包裹", "文件资料", "工具设备"};
    uint32_t hash = 0;
    for (char c : uid) {
        hash = hash * 31 + c;
    }
    return cargo_types[hash % 5];
}

// 生成货物列表JSON字符串
static std::string GenerateCargoListJson() {
    std::string json = "{\"cargo_list\":[";
    for (size_t i = 0; i < cargo_list.size(); i++) {
        if (i > 0) json += ",";
        json += "{\"uid\":\"" + cargo_list[i].uid + "\",";
        json += "\"name\":\"" + cargo_list[i].name + "\",";
        json += "\"timestamp\":" + std::to_string(cargo_list[i].timestamp) + "}";
    }
    json += "],\"total\":" + std::to_string(cargo_list.size()) + "}";
    return json;
}

static const char* RfidStateText() {
    switch (scan_state) {
        case SCAN_IDLE: return "idle";
        case SCAN_COLLECTING: return "scanning";
        case SCAN_READY: return "ready";
        default: return "unknown";
    }
}

static std::string CargoJson(const std::string& uid, const std::string& name, const std::string& status, const std::string& message) {
    std::string json = "{\"cargo\":{\"uid\":\"" + uid + "\",\"name\":\"" + name + "\",\"status\":\"" + status + "\",\"message\":\"" + message + "\"";
    json += ",\"total\":" + std::to_string(cargo_list.size());
    json += ",\"list\":[";
    for (size_t i = 0; i < cargo_list.size(); ++i) {
        if (i > 0) json += ",";
        json += "{\"uid\":\"" + cargo_list[i].uid + "\",\"name\":\"" + cargo_list[i].name + "\",\"timestamp\":" + std::to_string(cargo_list[i].timestamp) + "}";
    }
    json += "]}}";
    return json;
}

std::string RfidCargoStartCollecting() {
    return RfidCargoStartCollecting("");
}

std::string RfidCargoStartCollecting(const std::string& target_name) {
    InitializeRfidSpi();
    scan_state = SCAN_COLLECTING;
    pending_cargo_name = target_name;
    ESP_LOGI(TAG, "Local RFID scan started, target=%s", pending_cargo_name.c_str());

    std::string name = pending_cargo_name.empty() ? "等待扫描" : pending_cargo_name;
    std::string message = pending_cargo_name.empty()
        ? "RFID扫描已开始，请将标签靠近扫描区"
        : ("准备扫描货物：" + pending_cargo_name);
    return CargoJson("--", name, "扫描中", message);
}

std::string RfidCargoScanOnce() {
    if (scan_state != SCAN_COLLECTING) {
        scan_state = SCAN_COLLECTING;
    }

    std::string uid = ScanRfidCard();
    if (uid.empty()) {
        return CargoJson("--", "等待扫描", "扫描中", "未检测到RFID标签");
    }

    for (const auto& cargo : cargo_list) {
        if (cargo.uid == uid) {
            return CargoJson(cargo.uid, cargo.name, "已存在", "该货物已扫描过");
        }
    }

    CargoInfo new_cargo;
    new_cargo.uid = uid;
    new_cargo.name = pending_cargo_name.empty() ? GetCargoNameByUID(uid) : pending_cargo_name;
    new_cargo.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    cargo_list.push_back(new_cargo);

    pending_cargo_name.clear();
    StorageLidOpen(true);

    std::string message = "货物扫描成功，当前共 " + std::to_string(cargo_list.size()) + " 件";
    ESP_LOGI(TAG, "Local RFID cargo added: %s (%s)", new_cargo.name.c_str(), uid.c_str());
    return CargoJson(new_cargo.uid, new_cargo.name, "已识别", message);
}

std::string RfidCargoFinishCollecting() {
    scan_state = cargo_list.empty() ? SCAN_IDLE : SCAN_READY;
    std::string status = cargo_list.empty() ? "未识别" : "已完成";
    std::string message = cargo_list.empty() ? "未扫描到货物" : ("货物扫描完成，共 " + std::to_string(cargo_list.size()) + " 件");
    ESP_LOGI(TAG, "Local RFID scan finished, total=%d", (int)cargo_list.size());
    return CargoJson("--", cargo_list.empty() ? "等待扫描" : "货物清单", status, message);
}

std::string RfidCargoClear() {
    size_t count = cargo_list.size();
    cargo_list.clear();
    scan_state = SCAN_IDLE;
    return CargoJson("--", "等待扫描", "已清空", "已清空货物列表，共移除 " + std::to_string(count) + " 件");
}

std::string RfidCargoGetStatusJson() {
    return "{\"rfid\":{\"state\":\"" + std::string(RfidStateText()) + "\",\"count\":" + std::to_string(cargo_list.size()) + "}}";
}
// MCP工具注册 - RFID货物管理
void RegisterMcpRfidCargoManager() {
    // 初始化RFID硬件
    InitializeRfidSpi();

    auto& mcp_server = McpServer::GetInstance();

    // 开始收纳货物
    mcp_server.AddTool("self.rfid.cargo.start_collecting",
        "开始收纳货物模式。当用户要求扫描RFID、收纳货物、添加货物时，必须先调用此工具进入收纳模式",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            scan_state = SCAN_COLLECTING;
            cargo_list.clear();
            ESP_LOGI(TAG, "开始收纳货物模式");
            return "已进入货物收纳模式，请将货物的RFID标签靠近扫描器";
        });

    // 扫描单个货物
    mcp_server.AddTool("self.rfid.cargo.scan_item",
        "扫描RFID标签并添加货物。用户要求'扫描RFID'、'扫描标签'、'添加货物'时调用此工具。必须先调用start_collecting",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            if (scan_state != SCAN_COLLECTING) {
                return "请先调用start_collecting开始收纳模式";
            }

            std::string uid = ScanRfidCard();
            if (uid.empty()) {
                return "未检测到RFID卡片，请将标签靠近扫描器";
            }

            // 检查是否已存在
            for (const auto& cargo : cargo_list) {
                if (cargo.uid == uid) {
                    return "该货物已经添加过了：" + cargo.name;
                }
            }

            // 添加新货物
            CargoInfo new_cargo;
            new_cargo.uid = uid;
            new_cargo.name = GetCargoNameByUID(uid);
            new_cargo.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
            cargo_list.push_back(new_cargo);

            ESP_LOGI(TAG, "添加货物: %s (UID: %s)", new_cargo.name.c_str(), uid.c_str());

            std::string response = "已添加货物：" + new_cargo.name + "。";
            response += "当前已收纳 " + std::to_string(cargo_list.size()) + " 件货物";
            return response;
        });

    // 完成收纳
    mcp_server.AddTool("self.rfid.cargo.finish_collecting",
        "完成货物收纳，准备运输",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            if (scan_state != SCAN_COLLECTING) {
                return "当前不在收纳模式";
            }

            if (cargo_list.empty()) {
                return "还没有添加任何货物";
            }

            scan_state = SCAN_READY;
            ESP_LOGI(TAG, "完成货物收纳，共 %d 件", cargo_list.size());

            std::string response = "货物收纳完成！共收纳 " + std::to_string(cargo_list.size()) + " 件货物：\n";
            for (size_t i = 0; i < cargo_list.size(); i++) {
                response += std::to_string(i + 1) + ". " + cargo_list[i].name + "\n";
            }
            response += "准备就绪，可以开始运输";
            return response;
        });

    // 查询货物列表
    mcp_server.AddTool("self.rfid.cargo.get_list",
        "获取当前货物列表（JSON格式，用于网页显示）",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            return GenerateCargoListJson();
        });

    // 清空货物列表
    mcp_server.AddTool("self.rfid.cargo.clear",
        "清空货物列表，完成运输后调用",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            size_t count = cargo_list.size();
            cargo_list.clear();
            scan_state = SCAN_IDLE;
            ESP_LOGI(TAG, "清空货物列表");
            return "已清空货物列表，共移除 " + std::to_string(count) + " 件货物";
        });

    // 获取当前状态
    mcp_server.AddTool("self.rfid.cargo.get_status",
        "获取RFID货物管理系统当前状态",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            std::string state_str;
            switch (scan_state) {
                case SCAN_IDLE: state_str = "空闲"; break;
                case SCAN_COLLECTING: state_str = "正在收纳"; break;
                case SCAN_READY: state_str = "准备运输"; break;
            }

            std::string response = "状态：" + state_str + "\n";
            response += "货物数量：" + std::to_string(cargo_list.size()) + " 件";
            return response;
        });

    ESP_LOGI(TAG, "RFID货物管理MCP工具注册完成");
}
