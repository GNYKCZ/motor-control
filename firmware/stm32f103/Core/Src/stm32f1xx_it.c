/** Cortex 中断路由：SysTick 维护 HAL 毫秒计数；不可恢复异常禁止 PWM 后停留。
 * 外设 IRQ 实现在 board_io.c，以保持设备状态只有一个所有者。 */
#include "main.h"
#include "board_io.h"

/** SysTick：只增加毫秒时间，不运行控制/导出；由 HAL_Init 配置。 */
void SysTick_Handler(void) { HAL_IncTick(); }
/** NMI：时钟等不可恢复异常，无自动恢复/使能。 */
void NMI_Handler(void) { board_disable_output(); while (1) {} }
/** 硬故障：禁止调制后停留；不假称可应对任意芯片/供电故障。 */
void HardFault_Handler(void) { board_disable_output(); while (1) {} }
/** 内存管理故障：禁止调制后停留。 */
void MemManage_Handler(void) { board_disable_output(); while (1) {} }
/** 总线故障：禁止调制后停留。 */
void BusFault_Handler(void) { board_disable_output(); while (1) {} }
/** 非法指令等使用故障：禁止调制后停留。 */
void UsageFault_Handler(void) { board_disable_output(); while (1) {} }
