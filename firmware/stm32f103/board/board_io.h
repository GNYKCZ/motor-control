/** 板级入口：使用生成的外设配置，拥有 DMA 私有缓冲和串口发送状态，runtime 不直接操作设备。
 * 主函数负责初始化/前台串口；中断负责完成帧发布。生产profile为空，外部命令不能使能PWM。 */
#ifndef BOARD_IO_H
#define BOARD_IO_H
#include <stdbool.h>
#include <stdint.h>

/** 保存当前 PRIMASK 后屏蔽 IRQ，返回旧值；仅允许很短的值对象复制，不等待设备。
 * 调用者必须配对 board_leave_critical，嵌套时不能直接无条件打开中断。 */
uint32_t board_enter_critical(void);
/** 恢复传入的旧 PRIMASK，不修改其他状态；参数必须来自配对 enter。 */
void board_leave_critical(uint32_t previous_mask);
/** 禁止 TIM1 MOE/CCER，PA8/9/10 接管为 GPIO 低电平；启动/ISR/故障均可调用。
 * 无参数/返回；不停止采样计数器。不能关闭板卡固定接高的 EN/nSLEEP，非高阻断能。 */
void board_disable_output(void);
/** PWM预装载所需的值对象。margin_ticks为经批准的共同低窗口余量，不是电流限值。 */
typedef struct { uint16_t u, v, w, margin_ticks; } board_pwm_ticks_t;
/** 生成TIM1初始化末尾、切AF前调用；仅准备MOE0/idle低，不启动计数或输出。 */
void board_prepare_output_idle(void);
/** 初始化TIM3自由计时和Break中断；仅在全部生成初始化后调用一次。 */
void board_initialize_output(void);
/** 无锁读取驱动输入/固定PWM配置；调用者须持短临界区，true不是实物保护证明。 */
bool board_output_is_ready(void);
/** 已持runtime锁：装载三路预置并等自然UEV，不开MOE；generation用于拒绝撤销后的IRQ。 */
bool board_prepare_pulse(board_pwm_ticks_t ticks, uint32_t generation);
/** 已持锁：设最近绝对设备us期限，比较值越过则返回false，禁止等16bit回绕。 */
bool board_schedule_deadline(uint32_t deadline_us);
/** 已持锁取消TIM3比较中断，不停止独立时基。 */
void board_cancel_deadline(void);
/** 生成初始化之后，前台校准 ADC、启用 SPI 和设备计时；不启动采样事件。
 * 返回成功与否；失败保持 PWM 禁止。仅在 main 完成全部 MX_*_Init 后调用一次。 */
bool board_initialize_devices(void);
/** 前台启动 ADC/DMA 定时调度；仅在设备初始化成功、runtime 已 DISABLED 后调用一次。
 * 返回成功与否，失败不启动正常采样；无任何 PWM 通道启动操作。 */
bool board_start_sampling(void);
/** 返回 1 MHz、32 bit 设备微秒计数，模 2^32（约 71.6 min）回绕。
 * 仅设备初始化成功后有效；在 IRQ 和前台均可调用，短临界区保护溢出扩展。 */
uint32_t board_get_time_us(void);
/** 初始化后启用USART2接收/错误IRQ；不发送命令、不使能输出。 */
void board_start_serial_receive(void);
/** 前台查询串口是否仍持有上次缓冲，true 时调用者不能修改该缓冲。 */
bool board_serial_is_busy(void);
/** 前台提交 bytes[0..length)，长度 1..BOARD_SERIAL_BUFFER_BYTES。缓冲须持续有效且不修改直到 busy=false。
 * 忙/空指针/越界返回 false；成功只启用 TXE IRQ，无轮询等待、无动态分配。 */
bool board_start_serial_write(const char *bytes, uint32_t length);
#endif
