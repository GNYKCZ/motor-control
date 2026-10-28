/** 前台状态导出。输入为 runtime 一致快照，输出为 UART 文本。
 * 本模块拥有静态行缓冲及聚合耗时；UART完成前不重写缓冲；中断不得调用格式化。 */
#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>
/** 前台以HAL毫秒计数调用，每100ms尝试开始一行，之后每次只转换一个uint32。
 * 0准备、1..16字段；调用间必须返回主循环服务feedback，同行只读紧凑冻结快照。
 * 无返回；串口忙或提交失败就放弃该行，绝不等待，不修改runtime/采样状态。
 * 毫秒计数允许一次回绕，调用间隔须小于 2^31 ms。 */
void telemetry_poll_status(uint32_t now_ms);
#endif
