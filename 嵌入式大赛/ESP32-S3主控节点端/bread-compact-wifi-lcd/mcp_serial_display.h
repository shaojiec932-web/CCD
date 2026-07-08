#pragma once

#include <cstdint>

// 初始化串口屏通信
void InitSerialDisplay();

// 注册串口屏MCP工具
void RegisterMcpSerialDisplay();

// 发送文本到串口屏控件
void SerialDisplay_UpdateText(const char* widget_name, const char* text);

// 发送RFID信息到page0的t0控件
void SerialDisplay_ShowRfidInfo(const char* info);

// 发送定位信息到page0的t0控件
void SerialDisplay_ShowLocationInfo(const char* info);

// 发送对话内容到page1的g1控件
void SerialDisplay_ShowDialogText(const char* text);
