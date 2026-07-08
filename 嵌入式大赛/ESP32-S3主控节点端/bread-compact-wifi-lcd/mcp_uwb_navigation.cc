#include "mcp_uwb_navigation.h"
#include "mcp_server.h"
#include "config.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
#include <map>
#include <string>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "MCP_UWB_NAV"

// UART配置（与STM32通信）
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     STM32_UART_BAUD

// UWB导航状态
enum UwbNavState {
    NAV_IDLE,           // 空闲状态
    NAV_POSITIONING,    // 正在定位
    NAV_NAVIGATING,     // 正在导航（手动模式）
    NAV_AUTO_DRIVING,   // 自动驾驶中
    NAV_ARRIVED         // 已到达目标
};

// 位置坐标结构
struct Position {
    float x;            // X坐标 (米)
    float y;            // Y坐标 (米)
    float z;            // Z坐标 (米，高度)
    uint32_t timestamp; // 时间戳
};

// 目标点信息
struct Waypoint {
    std::string name;   // 目标点名称
    Position pos;       // 目标坐标
};

// 全局状态
static UwbNavState nav_state = NAV_IDLE;
static Position current_position = {0.0f, 0.0f, 0.0f, 0};
static bool current_position_valid = false;
static Waypoint target_waypoint;
static spi_device_handle_t uwb_spi = NULL;
static bool uwb_initialized = false;

// 预定义的目标点（可扩展为配置文件）
static std::map<std::string, Position> waypoint_map = {
    {"1号房", {1.0f, 1.0f, 0.0f, 0}},
    {"2号房", {3.0f, 1.0f, 0.0f, 0}},
    {"3号房", {5.0f, 1.0f, 0.0f, 0}},
    {"4号房", {1.0f, 3.0f, 0.0f, 0}},
    {"5号房", {3.0f, 3.0f, 0.0f, 0}},
    {"6号房", {5.0f, 3.0f, 0.0f, 0}},
    {"仓库", {0.0f, 0.0f, 0.0f, 0}},
    {"充电站", {6.0f, 0.0f, 0.0f, 0}},
};

// DW1000寄存器定义
#define DW1000_REG_DEV_ID       0x00
#define DW1000_EXPECTED_DEV_ID  0xDECA0130
#define DW1000_REG_SYS_CFG      0x04
#define DW1000_REG_TX_FCTRL     0x08
#define DW1000_REG_SYS_CTRL     0x0D
#define DW1000_REG_SYS_STATUS   0x0F
#define DW1000_REG_RX_FINFO     0x10
#define DW1000_REG_RX_BUFFER    0x11
#define DW1000_REG_TX_BUFFER    0x09

// Initialize UWB SPI on its own bus. RFID uses SPI2_HOST, UWB uses SPI3_HOST.
static void InitializeUwbSpi() {
    if (uwb_initialized) return;

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = UWB_SPI_MOSI_PIN;
    buscfg.miso_io_num = UWB_SPI_MISO_PIN;
    buscfg.sclk_io_num = UWB_SPI_SCK_PIN;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = 1024;

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "UWB SPI3 bus init failed: %d", ret);
        return;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 2000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = UWB_SPI_CS_PIN;
    devcfg.queue_size = 7;
    devcfg.flags = 0;

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &uwb_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UWB SPI3 device add failed: %d", ret);
        return;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << UWB_IRQ_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_set_direction(UWB_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(UWB_RST_PIN, 1);

    uwb_initialized = true;
    ESP_LOGI(TAG, "UWB DW1000 SPI initialized on SPI3_HOST (MOSI:%d, MISO:%d, SCK:%d, CS:%d, IRQ:%d, RST:%d)",
             UWB_SPI_MOSI_PIN, UWB_SPI_MISO_PIN, UWB_SPI_SCK_PIN, UWB_SPI_CS_PIN, UWB_IRQ_PIN, UWB_RST_PIN);
}
static void DW1000_WriteRegister(uint8_t reg, uint8_t* data, uint16_t len) {
    if (!uwb_initialized || (!data && len > 0)) return;

    uint8_t tx_buf[1 + 16] = {0};
    if (len > 16) {
        ESP_LOGE(TAG, "DW1000 write too long: %u", len);
        return;
    }

    tx_buf[0] = 0x80 | reg;  // 写操作
    if (len > 0) {
        memcpy(&tx_buf[1], data, len);
    }

    spi_transaction_t trans = {};
    trans.length = (1 + len) * 8;
    trans.tx_buffer = tx_buf;

    esp_err_t ret = spi_device_transmit(uwb_spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DW1000 write reg 0x%02X failed: %d", reg, ret);
    }
}

// 读DW1000寄存器
static void DW1000_ReadRegister(uint8_t reg, uint8_t* data, uint16_t len) {
    if (!uwb_initialized || !data || len == 0) return;

    uint8_t tx_buf[1 + 16] = {0};
    uint8_t rx_buf[1 + 16] = {0};
    if (len > 16) {
        ESP_LOGE(TAG, "DW1000 read too long: %u", len);
        memset(data, 0, len);
        return;
    }

    tx_buf[0] = reg & 0x3F;  // 读操作

    spi_transaction_t trans = {};
    trans.length = (1 + len) * 8;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    esp_err_t ret = spi_device_transmit(uwb_spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DW1000 read reg 0x%02X failed: %d", reg, ret);
        memset(data, 0, len);
        return;
    }

    memcpy(data, &rx_buf[1], len);
}

// 硬件复位DW1000
static void DW1000_HardReset() {
    gpio_set_level(UWB_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(UWB_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "DW1000硬件复位完成");
}

// 初始化DW1000
static bool DW1000_Init() {
    InitializeUwbSpi();
    if (!uwb_initialized) return false;

    // 硬件复位
    DW1000_HardReset();

    // 读取设备ID验证通信
    uint8_t dev_id[4];
    DW1000_ReadRegister(DW1000_REG_DEV_ID, dev_id, 4);
    uint32_t device_id = (dev_id[3] << 24) | (dev_id[2] << 16) | (dev_id[1] << 8) | dev_id[0];

    if (device_id != DW1000_EXPECTED_DEV_ID) {
        ESP_LOGE(TAG, "DW1000设备ID错误: 0x%08X (期望: 0x%08X)", device_id, DW1000_EXPECTED_DEV_ID);
        return false;
    }

    ESP_LOGI(TAG, "DW1000初始化成功，设备ID: 0x%08X", device_id);
    return true;
}

// 从基站获取当前位置（真实测距解算接口）
static bool GetCurrentPosition(Position* pos) {
    (void)pos;
    static bool warned = false;
    if (!warned) {
        ESP_LOGW(TAG, "UWB暂无有效定位数据：未接入真实TWR测距解算");
        warned = true;
    }
    current_position_valid = false;
    return false;
}

// 计算两点之间的距离
static float CalculateDistance(const Position& p1, const Position& p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

// 计算导航方向角度
static float CalculateDirection(const Position& from, const Position& to) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    return atan2f(dy, dx) * 180.0f / M_PI;
}

// 生成导航指令
static std::string GenerateNavigationCommand(const Position& current, const Position& target) {
    float distance = CalculateDistance(current, target);
    float direction = CalculateDirection(current, target);

    std::string cmd;
    if (distance < 0.3f) {
        cmd = "已到达目标";
    } else if (direction >= -45 && direction < 45) {
        cmd = "前进";
    } else if (direction >= 45 && direction < 135) {
        cmd = "左转";
    } else if (direction >= -135 && direction < -45) {
        cmd = "右转";
    } else {
        cmd = "后退";
    }

    return cmd;
}

// 发送STM32运动指令
static void SendMotorCommand(const char* cmd) {
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    ESP_LOGI(TAG, "发送STM32运动指令: %s", cmd);
}

// 将导航指令转换为STM32指令
static const char* ConvertToMotorCommand(const std::string& nav_cmd) {
    if (nav_cmd == "前进") return "KA";
    if (nav_cmd == "后退") return "KE";
    if (nav_cmd == "左转") return "KG";
    if (nav_cmd == "右转") return "KC";
    if (nav_cmd == "已到达目标") return "ZK";  // 停止
    return "ZK";  // 默认停止
}

// 自动驾驶任务
static void AutoDrivingTask(void* param) {
    ESP_LOGI(TAG, "自动驾驶任务启动");

    while (nav_state == NAV_AUTO_DRIVING) {
        // 获取当前位置
        current_position_valid = GetCurrentPosition(&current_position);
        if (!current_position_valid) {
            SendMotorCommand("ZK");
            ESP_LOGW(TAG, "UWB暂无有效定位数据，自动驾驶暂停");
            nav_state = NAV_POSITIONING;
            break;
        }

        // 生成导航指令
        std::string nav_cmd = GenerateNavigationCommand(current_position, target_waypoint.pos);

        // 转换为电机指令并发送
        const char* motor_cmd = ConvertToMotorCommand(nav_cmd);
        SendMotorCommand(motor_cmd);

        // 检查是否到达
        if (nav_cmd == "已到达目标") {
            nav_state = NAV_ARRIVED;
            ESP_LOGI(TAG, "已到达目标: %s", target_waypoint.name.c_str());
            break;
        }

        // 等待一段时间后更新
        vTaskDelay(pdMS_TO_TICKS(500));  // 每500ms更新一次
    }

    // 停止电机
    SendMotorCommand("ZK");
    ESP_LOGI(TAG, "自动驾驶任务结束");
    vTaskDelete(NULL);
}

bool UwbNavigationStartPositioning() {
    bool ok = DW1000_Init();
    if (ok) {
        nav_state = NAV_POSITIONING;
        current_position_valid = GetCurrentPosition(&current_position);
    }
    ESP_LOGI(TAG, "Local UWB positioning start: %s", ok ? "ok" : "failed");
    return ok;
}

void UwbNavigationStop() {
    bool was_auto_driving = (nav_state == NAV_AUTO_DRIVING);
    nav_state = NAV_IDLE;
    if (was_auto_driving) {
        SendMotorCommand("ZK");
    }
    ESP_LOGI(TAG, "Local UWB positioning stopped");
}

std::string UwbNavigationGetTelemetryJson() {
    if (nav_state != NAV_IDLE) {
        current_position_valid = GetCurrentPosition(&current_position);
    }

    const char* state = "idle";
    switch (nav_state) {
        case NAV_IDLE: state = "idle"; break;
        case NAV_POSITIONING: state = "positioning"; break;
        case NAV_NAVIGATING: state = "navigating"; break;
        case NAV_AUTO_DRIVING: state = "auto_driving"; break;
        case NAV_ARRIVED: state = "arrived"; break;
        default: break;
    }

    char json[192];
    if (current_position_valid) {
        snprintf(json, sizeof(json), "{\"uwb\":{\"state\":\"%s\",\"position_valid\":true,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}}",
                 state, current_position.x, current_position.y, current_position.z);
    } else {
        snprintf(json, sizeof(json), "{\"uwb\":{\"state\":\"%s\",\"position_valid\":false}}", state);
    }
    return std::string(json);
}
// MCP工具注册 - UWB定位导航
void RegisterMcpUwbNavigation() {
    // 自动初始化UWB硬件
    DW1000_Init();

    auto& mcp_server = McpServer::GetInstance();

    // 初始化UWB模块
    mcp_server.AddTool("self.uwb.nav.initialize",
        "初始化UWB定位模块",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            if (DW1000_Init()) {
                return "UWB定位模块初始化成功";
            } else {
                return "UWB定位模块初始化失败";
            }
        });

    // 获取当前位置
    mcp_server.AddTool("self.uwb.nav.get_position",
        "获取小车当前位置坐标",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            current_position_valid = GetCurrentPosition(&current_position);
            if (current_position_valid) {
                char pos_str[128];
                snprintf(pos_str, sizeof(pos_str),
                         "当前位置: X=%.2fm, Y=%.2fm, Z=%.2fm",
                         current_position.x, current_position.y, current_position.z);
                return std::string(pos_str);
            }
            return "UWB暂无有效定位数据";
        });

    // 前往目标点（手动模式）
    mcp_server.AddTool("self.uwb.nav.goto",
        "导航到指定目标点（手动模式，需要手动调用get_command获取指令），用户说'前往X号房'或'去X号房'时调用",
        PropertyList({
            Property("destination", kPropertyTypeString)
        }), [](const PropertyList& args) -> ReturnValue {
            std::string dest = args["destination"].value<std::string>();

            // 查找目标点
            auto it = waypoint_map.find(dest);
            if (it == waypoint_map.end()) {
                return "未找到目标点：" + dest + "。可用目标点：1-6号房、仓库、充电站";
            }

            target_waypoint.name = dest;
            target_waypoint.pos = it->second;
            nav_state = NAV_NAVIGATING;

            ESP_LOGI(TAG, "开始导航到: %s (%.2f, %.2f)",
                     dest.c_str(), target_waypoint.pos.x, target_waypoint.pos.y);

            return "已设置导航目标：" + dest + "，等待UWB有效定位数据";
        });

    // 自动驾驶到目标点
    mcp_server.AddTool("self.uwb.nav.auto_drive_to",
        "自动驾驶到指定目标点（自动控制STM32电机），用户说'自动前往X号房'时调用",
        PropertyList({
            Property("destination", kPropertyTypeString)
        }), [](const PropertyList& args) -> ReturnValue {
            std::string dest = args["destination"].value<std::string>();

            // 查找目标点
            auto it = waypoint_map.find(dest);
            if (it == waypoint_map.end()) {
                return "未找到目标点：" + dest + "。可用目标点：1-6号房、仓库、充电站";
            }

            // 检查是否已在自动驾驶中
            if (nav_state == NAV_AUTO_DRIVING) {
                return "已在自动驾驶中，请先停止当前任务";
            }

            target_waypoint.name = dest;
            target_waypoint.pos = it->second;
            current_position_valid = GetCurrentPosition(&current_position);
            if (!current_position_valid) {
                return std::string("UWB暂无有效定位数据，无法自动驾驶");
            }
            nav_state = NAV_AUTO_DRIVING;

            // 创建自动驾驶任务
            xTaskCreate(AutoDrivingTask, "auto_driving", 4096, NULL, 5, NULL);

            ESP_LOGI(TAG, "开始自动驾驶到: %s (%.2f, %.2f)",
                     dest.c_str(), target_waypoint.pos.x, target_waypoint.pos.y);

            return "开始自动驾驶到" + dest + "，小车将自动行驶";
        });

    // 获取导航状态
    mcp_server.AddTool("self.uwb.nav.get_status",
        "获取当前导航状态和进度",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            if (nav_state == NAV_IDLE) {
                return "当前空闲，未在导航中";
            }

            if (nav_state == NAV_ARRIVED) {
                return "已到达目标：" + target_waypoint.name;
            }

            current_position_valid = GetCurrentPosition(&current_position);
            if (!current_position_valid) {
                return std::string("UWB暂无有效定位数据，无法计算当前位置和目标距离");
            }
            float distance = CalculateDistance(current_position, target_waypoint.pos);
            float direction = CalculateDirection(current_position, target_waypoint.pos);

            std::string mode_str = (nav_state == NAV_AUTO_DRIVING) ? "自动驾驶" : "手动导航";

            char status[256];
            snprintf(status, sizeof(status),
                     "模式：%s\n正在前往：%s\n当前位置：(%.2f, %.2f)\n目标位置：(%.2f, %.2f)\n剩余距离：%.2fm\n方向角度：%.1f°",
                     mode_str.c_str(),
                     target_waypoint.name.c_str(),
                     current_position.x, current_position.y,
                     target_waypoint.pos.x, target_waypoint.pos.y,
                     distance, direction);

            return std::string(status);
        });

    // 获取导航指令（供STM32电机控制使用）
    mcp_server.AddTool("self.uwb.nav.get_command",
        "获取下一步导航指令（前进/左转/右转/后退/已到达）",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            if (nav_state != NAV_NAVIGATING) {
                return "未在导航中";
            }

            current_position_valid = GetCurrentPosition(&current_position);
            if (!current_position_valid) {
                return std::string("UWB暂无有效定位数据，无法生成导航指令");
            }
            std::string cmd = GenerateNavigationCommand(current_position, target_waypoint.pos);

            if (cmd == "已到达目标") {
                nav_state = NAV_ARRIVED;
            }

            return cmd;
        });

    // 停止导航/自动驾驶
    mcp_server.AddTool("self.uwb.nav.stop",
        "停止当前导航任务或自动驾驶",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            bool was_auto_driving = (nav_state == NAV_AUTO_DRIVING);
            nav_state = NAV_IDLE;

            // 如果是自动驾驶模式，发送停止指令
            if (was_auto_driving) {
                SendMotorCommand("ZK");
            }

            ESP_LOGI(TAG, "导航已停止");
            return "导航已停止";
        });

    // 添加自定义目标点
    mcp_server.AddTool("self.uwb.nav.add_waypoint",
        "添加自定义目标点",
        PropertyList({
            Property("name", kPropertyTypeString),
            Property("x", kPropertyTypeInteger),
            Property("y", kPropertyTypeInteger)
        }), [](const PropertyList& args) -> ReturnValue {
            std::string name = args["name"].value<std::string>();
            float x = (float)args["x"].value<int>();
            float y = (float)args["y"].value<int>();

            Position pos = {x, y, 0.0f, 0};
            waypoint_map[name] = pos;

            ESP_LOGI(TAG, "添加目标点: %s (%.2f, %.2f)", name.c_str(), x, y);
            return "已添加目标点：" + name;
        });

    // 列出所有目标点
    mcp_server.AddTool("self.uwb.nav.list_waypoints",
        "列出所有可用的目标点",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            std::string list = "可用目标点：\n";
            for (const auto& wp : waypoint_map) {
                char line[128];
                snprintf(line, sizeof(line), "- %s: (%.2f, %.2f)\n",
                         wp.first.c_str(), wp.second.x, wp.second.y);
                list += line;
            }
            return list;
        });

    // Web/JSON helper tools are intentionally not registered on this board to stay under the MCP tool limit.
    ESP_LOGI(TAG, "UWB定位导航MCP工具注册完成");
}


