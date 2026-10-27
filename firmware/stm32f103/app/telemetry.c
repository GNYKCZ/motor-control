/** 前台分步导出：一次poll只准备或格式化一个小段，随后返回主循环服务反馈。
 * 同行使用冻结快照；UART TC前不重写行缓冲；中断不调用本模块。 */
#include "telemetry.h"
#include "motor_runtime.h"
#include "board_io.h"
#include "board_config.h"
#include <stdio.h>

#define TELEMETRY_PARTS 14u /* 0准备，1..12数字段，13标签及异步提交。 */
static char serial_line[BOARD_SERIAL_BUFFER_BYTES];
static size_t line_length;
static uint8_t next_part; /* 0空闲；非零表示前台独占组装中的行。 */
static uint32_t last_export_ms, skipped_exports;
static motor_status_t status; /* 前台固定存储，避免大快照占主栈。 */
static sensor_input_t measured;
static uint32_t mean_u_mc, mean_v_mc, snapshot_skips;
static bool complete;
static uint32_t part_max_us[TELEMETRY_PARTS]; /* 有限墙钟观测，含抢占，非WCET。 */
static uint32_t snapshot_part_max_us[TELEMETRY_PARTS]; /* 整行冻结已完成步骤的计时。 */

/** 只在准备步骤换算名义量，不在各格式段重复计算。 */
static bool prepare_snapshot(void)
{
    if (!motor_runtime_read_status(&status)) { return false; }
    sensor_offset_t offset = status.estimated_offset;
    mean_u_mc = offset.accepted_count == 0u ? 0u : offset.sum_u_count * 1000u / offset.accepted_count;
    mean_v_mc = offset.accepted_count == 0u ? 0u : offset.sum_v_count * 1000u / offset.accepted_count;
    complete = offset.accepted_count == SENSOR_OFFSET_SAMPLE_COUNT;
    sensor_input_t empty = {0};
    measured = empty;
    if (complete && !status.offset_is_aborted && status.state == MOTOR_RUNTIME_DISABLED) {
        sensor_input_config_t config = {(float)offset.sum_u_count / offset.accepted_count,
            (float)offset.sum_v_count / offset.accepted_count, BOARD_NOMINAL_A_PER_COUNT};
        (void)sensor_input_convert(status.sample.raw_current_u_count, status.sample.raw_current_v_count,
            status.sample.encoder.raw_angle_count, status.sample.is_valid, config, &measured);
    }
    snapshot_skips = skipped_exports;
    for (unsigned i = 0u; i < TELEMETRY_PARTS; ++i) { snapshot_part_max_us[i] = part_max_us[i]; }
    return true;
}

/* 每段最多7个整数转换；每次只写剩余容量，截断统一中止整行。 */
#define APPEND(...) snprintf(serial_line + line_length, sizeof(serial_line) - line_length, __VA_ARGS__)
static int format_part(unsigned part)
{
    switch (part) {
    case 1u:
        return APPEND("M3B2,s=%lu,f=%lu,n=%lu,t=%lu,e=%lu,es=%lu,ec=%lu,",
            (unsigned long)status.state,
            (unsigned long)status.fault_flags,
            (unsigned long)status.sample.sequence,
            (unsigned long)status.sample.current_completed_us,
            (unsigned long)status.sample.encoder.sequence,
            (unsigned long)status.sample.encoder.read_started_us,
            (unsigned long)status.sample.encoder.read_completed_us);
    case 2u:
        return APPEND("u=%u,v=%u,a=%u,mg=%u,ok=%u,bad=%lu,skip=%lu,",
            (unsigned int)status.sample.raw_current_u_count,
            (unsigned int)status.sample.raw_current_v_count,
            (unsigned int)status.sample.encoder.raw_angle_count,
            (unsigned int)status.sample.encoder.magnetic_status,
            status.sample.is_valid ? 1u : 0u,
            (unsigned long)status.rejected_count,
            (unsigned long)snapshot_skips);
    case 3u:
        return APPEND("cn=%lu,cb=%lu,ca=%u,cd=%u,cv=%u,ou_mc=%lu,ov_mc=%lu,",
            (unsigned long)status.estimated_offset.accepted_count,
            (unsigned long)status.offset_rejected_count,
            status.offset_is_aborted ? 1u : 0u,
            complete ? 1u : 0u,
            status.calibration_verified ? 1u : 0u,
            (unsigned long)mean_u_mc,
            (unsigned long)mean_v_mc);
    case 4u:
        return APPEND("umin=%u,umax=%u,vmin=%u,vmax=%u,",
            (unsigned int)status.estimated_offset.minimum_u_count,
            (unsigned int)status.estimated_offset.maximum_u_count,
            (unsigned int)status.estimated_offset.minimum_v_count,
            (unsigned int)status.estimated_offset.maximum_v_count);
    case 5u:
        return APPEND("mv=%u,iu_mA=%ld,iv_mA=%ld,ma_mrad=%ld,scale_nA=%ld,",
            measured.is_valid ? 1u : 0u,
            (long)(measured.nominal_current_u_A * 1000.0f),
            (long)(measured.nominal_current_v_A * 1000.0f),
            (long)(measured.mechanical_angle_rad * 1000.0f),
            (long)(BOARD_NOMINAL_A_PER_COUNT * 1000000000.0f));
    case 6u:
        return APPEND("fp=%u,fd=%u,fw=%u,ff=%u,fe=%lu,ft=%lu,fr=%u,",
            status.feedback.is_valid ? 1u : 0u,
            status.feedback.direct_is_valid ? 1u : 0u,
            status.feedback.window_is_valid ? 1u : 0u,
            status.feedback.is_first_sample ? 1u : 0u,
            (unsigned long)status.feedback_sequence,
            (unsigned long)status.feedback_completed_us,
            (unsigned int)status.feedback_raw_count);
    case 7u:
        return APPEND("fm=%lu,fdt=%lu,fn=%u,fc=%lld,fa_mrad=%lld,",
            (unsigned long)status.feedback.midpoint_us,
            (unsigned long)status.feedback.dt_us,
            (unsigned int)status.feedback.window_sample_count,
            (long long)status.feedback.continuous_count,
            (long long)(status.feedback.continuous_angle_rad * 1000.0));
    case 8u:
        return APPEND("vd_mrad_s=%ld,vw_mrad_s=%ld,wd_count=%lld,wdt_us=%llu,",
            (long)(status.feedback.direct_speed_rad_s * 1000.0),
            (long)(status.feedback.window_speed_rad_s * 1000.0),
            (long long)status.feedback.window_delta_count,
            (unsigned long long)status.feedback.window_dt_us);
    case 9u:
        return APPEND("fo=%lu,fs=%u,dir=1,dir_ok=0,",
            (unsigned long)status.feedback_overruns,
            (unsigned int)BOARD_FEEDBACK_DIVIDER);
    case 10u:
        return APPEND("fb_poll_us=%lu,fb_step_us=%lu,mb_wait_us=%lu,mb_max_us=%lu,mb_overrun_us=%lu,",
            (unsigned long)status.feedback_poll_max_us,
            (unsigned long)status.feedback_step_max_us,
            (unsigned long)status.mailbox_wait_us,
            (unsigned long)status.mailbox_wait_max_us,
            (unsigned long)status.mailbox_overrun_us);
    case 11u:
        return APPEND("tp0_us=%lu,tp1_us=%lu,tp2_us=%lu,tp3_us=%lu,tp4_us=%lu,tp5_us=%lu,tp6_us=%lu,",
            (unsigned long)snapshot_part_max_us[0],
            (unsigned long)snapshot_part_max_us[1],
            (unsigned long)snapshot_part_max_us[2],
            (unsigned long)snapshot_part_max_us[3],
            (unsigned long)snapshot_part_max_us[4],
            (unsigned long)snapshot_part_max_us[5],
            (unsigned long)snapshot_part_max_us[6]);
    case 12u:
        return APPEND("tp7_us=%lu,tp8_us=%lu,tp9_us=%lu,tp10_us=%lu,tp11_us=%lu,tp12_us=%lu,tp13_us=%lu,",
            (unsigned long)snapshot_part_max_us[7],
            (unsigned long)snapshot_part_max_us[8],
            (unsigned long)snapshot_part_max_us[9],
            (unsigned long)snapshot_part_max_us[10],
            (unsigned long)snapshot_part_max_us[11],
            (unsigned long)snapshot_part_max_us[12],
            (unsigned long)snapshot_part_max_us[13]);
    case 13u:
        return APPEND("pwm=blocked,power=unverified\r\n");
    default:
        return -1;
    }
}
#undef APPEND

void telemetry_poll_status(uint32_t now_ms)
{
    if (next_part == 0u) {
        if (now_ms - last_export_ms < BOARD_TELEMETRY_PERIOD_MS) { return; }
        last_export_ms = now_ms;
    }
    if (board_serial_is_busy()) {
        skipped_exports++;
        next_part = 0u;
        return;
    }
    uint32_t started_us = board_get_time_us();
    unsigned part = next_part;
    if (part == 0u) {
        line_length = 0u;
        if (prepare_snapshot()) { next_part = 1u; }
        else { skipped_exports++; }
    } else {
        int length = format_part(part);
        if (length <= 0 || (size_t)length >= sizeof(serial_line) - line_length) {
            skipped_exports++;
            next_part = 0u; /* 半行从不提交；UART空闲后才可覆盖旧行。 */
        } else {
            line_length += (size_t)length;
            if (part == TELEMETRY_PARTS - 1u) {
                next_part = 0u;
                if (!board_start_serial_write(serial_line, (uint32_t)line_length)) { skipped_exports++; }
            } else { next_part++; }
        }
    }
    uint32_t elapsed_us = board_get_time_us() - started_us;
    if (elapsed_us > part_max_us[part]) { part_max_us[part] = elapsed_us; }
}
