/** 格式化与发送均在低频前台，ADC IRQ 不调用本文件。
 * 行缓冲的所有权：格式化时属于前台，成功提交后借给 UART，busy=false 后收回。 */
#include "telemetry.h"
#include "motor_runtime.h"
#include "board_io.h"
#include "board_config.h"
#include <stdio.h>

static char serial_line[BOARD_SERIAL_BUFFER_BYTES]; /**< 唯一静态发送行，不使用堆。 */
static uint32_t last_export_ms;                    /**< 上次尝试导出的前台时间。 */
static uint32_t skipped_exports;                   /**< 忙/格式超长/提交失败的次数。 */

void telemetry_poll_status(uint32_t now_ms)
{
    if (now_ms - last_export_ms < BOARD_TELEMETRY_PERIOD_MS) {
        return;
    }
    last_export_ms = now_ms;
    if (board_serial_is_busy()) {
        skipped_exports++;
        return;
    }
    /* 只有前台串行调用；一致快照放静态暂存，给1KiB主栈保留IRQ嵌套空间。 */
    static motor_status_t status;
    if (!motor_runtime_read_status(&status)) {
        skipped_exports++;
        return;
    }
    /* 浮点均值/换算只在前台、临界区外进行；IRQ仅做O(1)整数统计。
     * 未完成或中止/FAULT时数值发布为0且mv=0；零值不能冒充真实零电流。 */
    sensor_offset_t offset = status.estimated_offset;
    uint32_t mean_u_mc = offset.accepted_count == 0u ? 0u : offset.sum_u_count * 1000u / offset.accepted_count;
    uint32_t mean_v_mc = offset.accepted_count == 0u ? 0u : offset.sum_v_count * 1000u / offset.accepted_count;
    bool complete = offset.accepted_count == SENSOR_OFFSET_SAMPLE_COUNT;
    sensor_input_t measured = {0};
    if (complete && !status.offset_is_aborted && status.state == MOTOR_RUNTIME_DISABLED) {
        sensor_input_config_t config = {(float)offset.sum_u_count / offset.accepted_count,
            (float)offset.sum_v_count / offset.accepted_count, BOARD_NOMINAL_A_PER_COUNT};
        (void)sensor_input_convert(status.sample.raw_current_u_count, status.sample.raw_current_v_count,
            status.sample.encoder.raw_angle_count, status.sample.is_valid, config, &measured);
    }
    /* 文本字段显式序列化，不发送 C 结构体内存布局，也不伪造物理电流/速度。 */
    int length = snprintf(serial_line, sizeof(serial_line),
        "M3B1,s=%lu,f=%lu,n=%lu,t=%lu,e=%lu,es=%lu,ec=%lu,u=%u,v=%u,a=%u,mg=%u,ok=%u,bad=%lu,skip=%lu,"
        "cn=%lu,cb=%lu,ca=%u,cd=%u,cv=%u,ou_mc=%lu,ov_mc=%lu,umin=%u,umax=%u,vmin=%u,vmax=%u,"
        "mv=%u,iu_mA=%ld,iv_mA=%ld,ma_mrad=%ld,scale_nA=%ld,"
        "fp=%u,fd=%u,fw=%u,ff=%u,fe=%lu,ft=%lu,fr=%u,fm=%lu,fdt=%lu,fn=%u,"
        "fc=%lld,fa_mrad=%lld,vd_mrad_s=%ld,vw_mrad_s=%ld,wd_count=%lld,wdt_us=%llu,"
        "fo=%lu,fs=%u,dir=1,dir_ok=0,pwm=blocked,power=unverified\r\n",
        (unsigned long)status.state, (unsigned long)status.fault_flags,
        (unsigned long)status.sample.sequence, (unsigned long)status.sample.current_completed_us,
        (unsigned long)status.sample.encoder.sequence,
        (unsigned long)status.sample.encoder.read_started_us,
        (unsigned long)status.sample.encoder.read_completed_us,
        (unsigned int)status.sample.raw_current_u_count,
        (unsigned int)status.sample.raw_current_v_count,
        (unsigned int)status.sample.encoder.raw_angle_count,
        (unsigned int)status.sample.encoder.magnetic_status,
        status.sample.is_valid ? 1u : 0u,
        (unsigned long)status.rejected_count, (unsigned long)skipped_exports,
        (unsigned long)offset.accepted_count, (unsigned long)status.offset_rejected_count,
        status.offset_is_aborted ? 1u : 0u, complete ? 1u : 0u, status.calibration_verified ? 1u : 0u,
        (unsigned long)mean_u_mc, (unsigned long)mean_v_mc,
        (unsigned int)offset.minimum_u_count, (unsigned int)offset.maximum_u_count,
        (unsigned int)offset.minimum_v_count, (unsigned int)offset.maximum_v_count,
        measured.is_valid ? 1u : 0u, (long)(measured.nominal_current_u_A * 1000.0f),
        (long)(measured.nominal_current_v_A * 1000.0f), (long)(measured.mechanical_angle_rad * 1000.0f),
        (long)(BOARD_NOMINAL_A_PER_COUNT * 1000000000.0f),
        status.feedback.is_valid ? 1u : 0u, status.feedback.direct_is_valid ? 1u : 0u,
        status.feedback.window_is_valid ? 1u : 0u, status.feedback.is_first_sample ? 1u : 0u,
        (unsigned long)status.feedback_sequence, (unsigned long)status.feedback_completed_us,
        (unsigned int)status.feedback_raw_count, (unsigned long)status.feedback.midpoint_us,
        (unsigned long)status.feedback.dt_us, (unsigned int)status.feedback.window_sample_count,
        (long long)status.feedback.continuous_count, (long long)(status.feedback.continuous_angle_rad * 1000.0),
        (long)(status.feedback.direct_speed_rad_s * 1000.0), (long)(status.feedback.window_speed_rad_s * 1000.0),
        (long long)status.feedback.window_delta_count, (unsigned long long)status.feedback.window_dt_us,
        (unsigned long)status.feedback_overruns, (unsigned int)BOARD_FEEDBACK_DIVIDER);
    if (length <= 0 || (uint32_t)length >= sizeof(serial_line)
        || !board_start_serial_write(serial_line, (uint32_t)length)) {
        skipped_exports++;
    }
}
