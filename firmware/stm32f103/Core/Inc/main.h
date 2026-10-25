/** 自编入口头：只声明启动错误停留函数，外设状态由 board 拥有。 */
#ifndef MAIN_H
#define MAIN_H
#include "stm32f1xx_hal.h"
/** 初始化失败时禁止 PWM 并永久停留；不复位、不重试使能、不操作串口。 */
void Error_Handler(void);
#endif
