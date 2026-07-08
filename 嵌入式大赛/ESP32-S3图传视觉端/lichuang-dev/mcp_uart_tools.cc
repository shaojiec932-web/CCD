#include "mcp_uart_tools.h"
#include "mcp_server.h"
#include "config.h"
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <driver/gpio.h>

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024

#define TAG "MCP_UART"

// 停止指令
static const char* STOP_CMD = "ZK";
// 2秒自动停止定时器
static TimerHandle_t auto_stop_timer = NULL;

// 串口初始化
static void InitializeUart() {
    static bool initialized = false;

    if (initialized) return;
    initialized = true;
    
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装UART驱动程序
    esp_err_t err = uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART驱动安装失败: %d", err);
        return;
    } 
    err = uart_param_config(UART_PORT_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART参数配置失败: %d", err);
        return;
    }
    err = uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART引脚配置失败: %d", err);
        return;
    }
}

// 串口发送函数
static void SendUartCommand(const char* cmd) {
    InitializeUart();
    vTaskDelay(pdMS_TO_TICKS(1500));   
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    ESP_LOGI(TAG, "发送串口指令: %s", cmd);
}

// 定时器回调：自动发送停止指令
static void AutoStopCallback(TimerHandle_t xTimer) {
    SendUartCommand(STOP_CMD);
    ESP_LOGI(TAG, "自动发送停止指令: %s", STOP_CMD);
}

// 启动或重置2秒自动停止定时器
static void StartOrResetStopTimer() {
    if (auto_stop_timer == NULL) {
        auto_stop_timer = xTimerCreate("AutoStopTimer", pdMS_TO_TICKS(2000), pdFALSE, NULL, AutoStopCallback);
    }
    xTimerStop(auto_stop_timer, 0);
    xTimerStart(auto_stop_timer, 0);
}

// 发送动作指令并自动2秒后停止
static void SendActionCommand(const char* cmd) {
    SendUartCommand(cmd);
    StartOrResetStopTimer();
}

// 立即停止（发送ZK并停止定时器）
static void SendStopCommand() {
    SendUartCommand(STOP_CMD);
    if (auto_stop_timer) {
        xTimerStop(auto_stop_timer, 0);
    }
}

// MCP工具注册
void RegisterMcpUartTools() {
    auto& mcp_server = McpServer::GetInstance();
    
    // 前进
    mcp_server.AddTool("self.uart.go_forward", 
        "去6号房，发送KA", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("KA");
            return true;
        });
    
    // 后退
    mcp_server.AddTool("self.uart.back_up", 
        "往后退移动，发送KE", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("KE");
            return true;
        });
    
    // 左转
    mcp_server.AddTool("self.uart.turn_left", 
        "往左转移动，发送KG", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("KG");
            return true;
        });
    
    // 右转
    mcp_server.AddTool("self.uart.turn_right", 
        "往右转移动，发送KC", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("KC");
            return true;
        });

    // 右移
    mcp_server.AddTool("self.uart.right", 
        "往右方向移动，发送JC", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("JC");
            return true;
        });
    
    // 左移
    mcp_server.AddTool("self.uart.left", 
        "往左方向移动，发送JG", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("JG");
            return true;
        });

    // 右上
    mcp_server.AddTool("self.uart.upper_right", 
        "往右上移动，发送JB", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("JB");
            return true;
        });
    
    // 右下
    mcp_server.AddTool("self.uart.low_right", 
        "往右下移动，发送JD", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("JD");
            return true;
        });
    
    // 左上
    mcp_server.AddTool("self.uart.up_left", 
        "往左上移动，发送JH", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("JH");
            return true;
        });
    
    // 左下
    mcp_server.AddTool("self.uart.low_left", 
        "往左下移动，发送JF", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("JF");
            return true;
        });
    
    // 加速
    mcp_server.AddTool("self.uart.speed_up", 
        "加速，发送KX", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("KX");
            return true;
        });
    
    // 减速
    mcp_server.AddTool("self.uart.speed_cut", 
        "减速，发送KY", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendActionCommand("KY");
            return true;
        });
    
    // 停止
    mcp_server.AddTool("self.uart.stop", 
        "停止停下，发送ZK", 
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            SendStopCommand();
            return true;
        });
}
