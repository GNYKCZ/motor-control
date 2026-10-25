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
    motor_status_t status;
    if (!motor_runtime_read_status(&status)) {
        skipped_exports++;
        return;
    }
    /* 文本字段显式序列化，不发送 C 结构体内存布局，也不伪造物理电流/速度。 */
    int length = snprintf(serial_line, sizeof(serial_line),
        "M1,s=%lu,f=%lu,n=%lu,t=%lu,e=%lu,es=%lu,ec=%lu,u=%u,v=%u,a=%u,mg=%u,ok=%u,bad=%lu,skip=%lu,pwm=blocked,power=unverified\r\n",
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
        (unsigned long)status.rejected_count, (unsigned long)skipped_exports);
    if (length <= 0 || (uint32_t)length >= sizeof(serial_line)
        || !board_start_serial_write(serial_line, (uint32_t)length)) {
        skipped_exports++;
    }
}
