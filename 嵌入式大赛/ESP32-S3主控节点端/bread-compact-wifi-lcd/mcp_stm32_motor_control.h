#ifndef _MCP_STM32_MOTOR_CONTROL_H_
#define _MCP_STM32_MOTOR_CONTROL_H_

void RegisterMcpStm32MotorControl();
void Stm32MotorEmergencyStop();
bool Stm32MotorIsRunning();
void Stm32MotorGoForward();
void Stm32MotorBackUp();
void Stm32MotorTurnLeft();
void Stm32MotorTurnRight();
void Stm32MotorStop();
void Stm32MotorSetSpeed(int speed);
int Stm32MotorGetSpeed();

#endif // _MCP_STM32_MOTOR_CONTROL_H_