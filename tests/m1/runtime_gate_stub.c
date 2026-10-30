/** 旧采样/串口回归的板级替身：没有可用PWM设备，不替代新增真实board_pwm寄存器测试。 */
#include "board_io.h"
bool board_output_is_ready(void) { return false; }
bool board_prepare_pulse(board_pwm_ticks_t ticks, uint32_t generation) { (void)ticks; (void)generation; return false; }
bool board_update_scan_point(board_pwm_ticks_t ticks, uint32_t generation, unsigned step)
{ (void)ticks; (void)generation; (void)step; return false; }
bool board_schedule_deadline(uint32_t deadline_us) { (void)deadline_us; return true; }
void board_cancel_deadline(void) { }
