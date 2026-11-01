/** M3B3小型诊断帧：准备只读紧凑快照；每次poll最多转换一个uint32。
 * 不调用printf/名义换算，不打包未发送字段；UART TC前不重写借用缓冲。 */
#include "telemetry.h"
#include "motor_runtime.h"
#include "board_io.h"
#include "board_config.h"
#include <stddef.h>

#define TELEMETRY_FIELDS 16u
static const char *const field_names[TELEMETRY_FIELDS] = {
    "s", "f", "n", "t", "u", "v", "a", "ok", "fp", "fd", "fw", "fo",
    "fb_us", "mb_us", "log_us", "log_id"
};
static char serial_line[BOARD_SERIAL_BUFFER_BYTES];
static size_t line_length;
static unsigned next_field; /* 0空闲/准备；1..16表示下一次要输出的字段。 */
static uint32_t last_export_ms;
static motor_log_snapshot_t status; /* 实际需要的约40B，非完整运行状态。 */
static uint32_t log_max_us, log_max_id;
static uint32_t snapshot_log_us, snapshot_log_id;

/** 固定字段值，均为uint32范围；不读取实时状态，不执行float/double/64位展示换算。 */
static uint32_t field_value(unsigned field)
{
    switch (field) {
    case 0u: return (uint32_t)status.state;
    case 1u: return status.fault_flags;
    case 2u: return status.sequence;
    case 3u: return status.completed_us;
    case 4u: return status.raw_u;
    case 5u: return status.raw_v;
    case 6u: return status.raw_angle;
    case 7u: return status.sample_is_valid ? 1u : 0u;
    case 8u: return status.position_is_valid ? 1u : 0u;
    case 9u: return status.direct_is_valid ? 1u : 0u;
    case 10u: return status.window_is_valid ? 1u : 0u;
    case 11u: return status.feedback_overruns;
    case 12u: return status.feedback_max_us;
    case 13u: return status.mailbox_max_us;
    case 14u: return snapshot_log_us;
    case 15u: return snapshot_log_id;
    default: return 0u; /* 调用者只传0..15，无运行期注册或可变格式串。 */
    }
}

/** 一次最多10轮十进制除法，支持0和UINT32_MAX，无通用格式解释/分配。
 * 先检查整字段剩余容量；失败不提交半行，也不让UART借用不完整缓冲。 */
static bool append_field(const char *name, uint32_t value, bool last)
{
    char digits[10];
    size_t count = 0u, name_length = 0u;
    do {
        digits[count++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value != 0u); /* uint32十进制最多10位，静态上界不依赖收到的字符串。 */
    while (name[name_length] != '\0') { name_length++; } /* 固定字段名，最长6字节。 */
    size_t needed = name_length + 1u + count + (last ? 2u : 1u);
    if (needed >= sizeof(serial_line) - line_length) { return false; }
    for (size_t i = 0u; i < name_length; ++i) { serial_line[line_length++] = name[i]; }
    serial_line[line_length++] = '=';
    while (count != 0u) { serial_line[line_length++] = digits[--count]; }
    if (last) {
        serial_line[line_length++] = '\r';
        serial_line[line_length++] = '\n';
    } else { serial_line[line_length++] = ','; }
    serial_line[line_length] = '\0';
    return true;
}

void telemetry_poll_status(uint32_t now_ms)
{
    if (motor_runtime_output_is_busy()) { next_field=0u; return; }
    if (next_field == 0u) {
        if (now_ms - last_export_ms < BOARD_TELEMETRY_PERIOD_MS) { return; }
        last_export_ms = now_ms;
    }
    if (board_serial_is_busy()) {
        next_field = 0u;
        return; /* 本轮不再维护未输出的skip统计，也不等待UART。 */
    }
    uint32_t started_us = board_get_time_us();
    unsigned step = next_field;
    if (step == 0u) {
        static const char prefix[] = "M3B3,";
        line_length = 0u;
        if (sizeof(serial_line) >= sizeof(prefix) && motor_runtime_read_log(&status)) {
            snapshot_log_us = log_max_us;
            snapshot_log_id = log_max_id;
            for (size_t i = 0u; i < sizeof(prefix) - 1u; ++i) { serial_line[line_length++] = prefix[i]; }
            serial_line[line_length] = '\0';
            next_field = 1u;
        }
    } else {
        bool last = step == TELEMETRY_FIELDS;
        if (!append_field(field_names[step - 1u], field_value(step - 1u), last)) {
            next_field = 0u;
        } else if (last) {
            next_field = 0u;
            (void)board_start_serial_write(serial_line, (uint32_t)line_length);
        } else { next_field++; }
    }
    uint32_t elapsed_us = board_get_time_us() - started_us;
    if (elapsed_us > log_max_us) {
        log_max_us = elapsed_us;
        log_max_id = step; /* 0准备，1..16与field_names逐项对应；同行快照下一行才看到本次更新。 */
    }
}
