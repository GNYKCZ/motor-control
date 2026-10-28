/** 仅主机C++寄存器模型：保持生产C接口的链接名称，不改生产源码。 */
#ifndef TEST_UART_LINKAGE_H
#define TEST_UART_LINKAGE_H
extern "C" {
#include "board_io.h"
#include "command.h"
#include "motor_runtime.h"
}
#endif
