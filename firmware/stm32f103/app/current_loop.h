/** 1kHz受限电流环适配：前台独占PI，IRQ只交接完整采样和硬件提交确认。 */
#ifndef APP_CURRENT_LOOP_H
#define APP_CURRENT_LOOP_H
#include "sample_frame.h"
#include "sensor_input.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void current_loop_init(void);
/** 启动前无输出的真实数学路径计时通过；不是硬件峰值或WCET认证。 */
bool current_loop_is_ready(void);
/** 只读准备计时失败，用于提示准确原因，不解除任何输出资格。 */
bool current_loop_timing_failed(void);
void current_loop_begin(uint32_t generation, sensor_offset_t offset);
void current_loop_abort(void);
void current_loop_queue(const motor_sample_t *sample);
void current_loop_poll(void);
/** board自然UEV持锁确认后才能采纳临时PI；序号/代次不匹配拒绝。 */
bool current_loop_applied(uint32_t generation,uint32_t sequence,uint32_t event_us);
/** 停机后有限RAM块导出，不占输出IRQ或改变参考。 */
bool current_loop_report_poll(void);
#ifdef __cplusplus
}
#endif
#endif
