/** P1a-1无输出命令：IRQ逐字节校验，STOP先禁用再排回执；不提供使能入口。 */
#ifndef COMMAND_H
#define COMMAND_H
#include <stdbool.h>
#include <stdint.h>

/** 采样/串口IRQ启用前调用一次；重启清序号和待回执，不重放动作。 */
void command_init(void);
/** UART RX IRQ调用，单字节、最多8轮CRC；now_us为设备时间。 */
void command_receive_byte(uint8_t byte, uint32_t now_us);
/** UART错误IRQ调用：丢到真实LF边界，不将尾片段当新命令。 */
void command_receive_error(void);
/** 前台每次最多生成一个回执字符；true表示回执占用发送优先级，日志本轮让出。 */
bool command_poll(uint32_t now_us);
#endif
