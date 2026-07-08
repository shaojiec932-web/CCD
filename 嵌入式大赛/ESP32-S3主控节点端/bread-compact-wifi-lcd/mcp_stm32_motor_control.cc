#include "mcp_stm32_motor_control.h"
#include "mcp_server.h"
#include "config.h"
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <driver/gpio.h>

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     STM32_UART_BAUD
#define UART_BUF_SIZE      1024

#define TAG "MCP_STM32_MOTOR"

// 停止指令
static const char* STOP_CMD = "ZK";
// 2秒自动停止定时器
static TimerHandle_t auto_stop_timer = NULL;
static bool motor_running = false;
static int motor_speed = 50;

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
    err = uart_set_pin(UART_PORT_NUM, STM32_UART_TX_PIN, STM32_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART引脚配置失败: %d", err);
        return;
    }

    ESP_LOGI(TAG, "STM32 UART初始化完成 (TX:%d, RX:%d, 波特率:%d)", STM32_UART_TX_PIN, STM32_UART_RX_PIN, UART_BAUD_RATE);
}

// 串口发送函数
static void SendUartCommand(const char* cmd) {
    InitializeUart();
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    ESP_LOGI(TAG, "发送STM32指令: %s", cmd);
}

// 定时器回调：自动发送停止指令
static void AutoStopCallback(TimerHandle_t xTimer) {
    motor_running = false;
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
    motor_running = true;
    SendUartCommand(cmd);
    StartOrResetStopTimer();
}

// 立即停止（发送ZK并停止定时器）
static void SendStopCommand() {
    motor_running = false;
    SendUartCommand(STOP_CMD);
    if (auto_stop_timer) {
        xTimerStop(auto_stop_timer, 0);
    }
}


void Stm32MotorEmergencyStop() {
    SendStopCommand();
}

bool Stm32MotorIsRunning() {
    return motor_running;
}

void Stm32MotorGoForward() {
    SendActionCommand("KA");
}

void Stm32MotorBackUp() {
    SendActionCommand("KE");
}

void Stm32MotorTurnLeft() {
    SendActionCommand("KG");
}

void Stm32MotorTurnRight() {
    SendActionCommand("KC");
}

void Stm32MotorStop() {
    SendStopCommand();
}

void Stm32MotorSetSpeed(int speed) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    motor_speed = speed;
    ESP_LOGI(TAG, "Set motor speed value locally: %d", motor_speed);
}

int Stm32MotorGetSpeed() {
    return motor_speed;
}
// MCP工具注册 - STM32电机控制
void RegisterMcpStm32MotorControl() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool("self.stm32.motor.go_forward",
        "Move the car forward for about 2 seconds. Use when the user says forward or go ahead.",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            Stm32MotorGoForward();
            return "car moving forward";
        });

    mcp_server.AddTool("self.stm32.motor.back_up",
        "Move the car backward for about 2 seconds. Use when the user says back up or reverse.",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            Stm32MotorBackUp();
            return "car moving backward";
        });

    mcp_server.AddTool("self.stm32.motor.turn_left",
        "Turn the car left for about 2 seconds. Use when the user says turn left.",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            Stm32MotorTurnLeft();
            return "car turning left";
        });

    mcp_server.AddTool("self.stm32.motor.turn_right",
        "Turn the car right for about 2 seconds. Use when the user says turn right.",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            Stm32MotorTurnRight();
            return "car turning right";
        });

    mcp_server.AddTool("self.stm32.motor.stop",
        "Stop the car immediately. Use when the user says stop.",
        PropertyList(), [](const PropertyList&) -> ReturnValue {
            Stm32MotorStop();
            return "car stopped";
        });

    ESP_LOGI(TAG, "STM32 motor MCP tools registered: forward/back/left/right/stop");
}

