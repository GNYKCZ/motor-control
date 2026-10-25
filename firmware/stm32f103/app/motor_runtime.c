/** 运行状态所有者。前台与 ADC/故障 IRQ 共享下列静态状态，统一以短临界区互斥。
 * 本批永无正常输出分支；即使任意输入错误，board_disable_output 也先于状态更新。 */
#include "motor_runtime.h"
#include "board_config.h"
#include "board_io.h"
#include <stddef.h>

static motor_status_t runtime_status;  /**< 全部状态只在本文件写入。 */
static uint32_t first_sample_time_us; /**< 首包时刻，用于启动等待界限。 */
static uint32_t boot_completed_us; /**< 启动握手时间，检测完全没有 ADC 事件的情况。 */
static bool has_received_sample; /**< 是否收到过完整包；不能用会回绕的统计计数判断首次。 */

void motor_runtime_init(void)
{
    board_disable_output();
    uint32_t previous_mask = board_enter_critical();
    motor_status_t initial_status = {0};
    runtime_status = initial_status;
    runtime_status.state = MOTOR_RUNTIME_BOOT;
    runtime_status.pwm_is_permanently_blocked = true;
    first_sample_time_us = 0u;
    boot_completed_us = 0u;
    has_received_sample = false;
    board_leave_critical(previous_mask);
}

void motor_runtime_finish_boot(bool devices_are_ready)
{
    board_disable_output();
    uint32_t previous_mask = board_enter_critical();
    if (devices_are_ready && runtime_status.state == MOTOR_RUNTIME_BOOT) {
        runtime_status.state = MOTOR_RUNTIME_DISABLED;
        boot_completed_us = board_get_time_us();
    } else {
        runtime_status.state = MOTOR_RUNTIME_FAULT;
        runtime_status.fault_flags |= MOTOR_FAULT_INITIALIZATION;
    }
    board_leave_critical(previous_mask);
}

void motor_runtime_latch_fault(motor_fault_t fault)
{
    board_disable_output();
    uint32_t previous_mask = board_enter_critical();
    runtime_status.fault_flags |= (uint32_t)fault;
    runtime_status.state = MOTOR_RUNTIME_FAULT;
    board_leave_critical(previous_mask);
}

void motor_runtime_handle_sample(const motor_sample_t *sample)
{
    board_disable_output();
    uint32_t previous_mask = board_enter_critical();
    if (sample == NULL || runtime_status.state == MOTOR_RUNTIME_BOOT) {
        runtime_status.state = MOTOR_RUNTIME_FAULT;
        runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
        runtime_status.rejected_count++;
        board_leave_critical(previous_mask);
        return;
    }
    motor_sample_t checked_sample = board_make_sample(sample->sequence,
        sample->current_completed_us, sample->raw_current_u_count,
        sample->raw_current_v_count, &sample->encoder);
    bool is_valid = checked_sample.is_valid && sample->is_valid;
    if (!runtime_status.has_valid_sample
        && sample->current_completed_us - boot_completed_us > BOARD_BOOT_SAMPLE_LIMIT_US) {
        runtime_status.state = MOTOR_RUNTIME_FAULT;
        runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
        is_valid = false;
    }
    if (!has_received_sample) {
        first_sample_time_us = sample->current_completed_us;
        has_received_sample = true;
    } else {
        uint32_t gap_us = sample->current_completed_us - runtime_status.sample.current_completed_us;
        is_valid = is_valid && board_sequence_is_newer(sample->sequence, runtime_status.sample.sequence)
            && gap_us > 0u && gap_us <= BOARD_SAMPLE_GAP_LIMIT_US;
        if (runtime_status.has_valid_sample) {
            is_valid = is_valid && board_sequence_is_newer(sample->encoder.sequence,
                runtime_status.sample.encoder.sequence);
        }
    }
    runtime_status.sample = *sample;
    runtime_status.sample.is_valid = is_valid;
    runtime_status.sample_count++;
    if (is_valid) {
        runtime_status.has_valid_sample = true;
    } else {
        runtime_status.rejected_count++;
        if (runtime_status.has_valid_sample
            || sample->current_completed_us - first_sample_time_us >= BOARD_BOOT_SAMPLE_LIMIT_US) {
            runtime_status.state = MOTOR_RUNTIME_FAULT;
            runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
        }
    }
    board_leave_critical(previous_mask);
}

bool motor_runtime_read_status(motor_status_t *status)
{
    if (status == NULL) {
        return false;
    }
    uint32_t previous_mask = board_enter_critical();
    *status = runtime_status;
    board_leave_critical(previous_mask);
    return true;
}

void motor_runtime_check_deadline(uint32_t now_us)
{
    uint32_t previous_mask = board_enter_critical();
    uint32_t reference_us = runtime_status.has_valid_sample
        ? runtime_status.sample.current_completed_us : boot_completed_us;
    uint32_t limit_us = runtime_status.has_valid_sample
        ? BOARD_SAMPLE_GAP_LIMIT_US : BOARD_BOOT_SAMPLE_LIMIT_US;
    uint32_t elapsed_us = now_us - reference_us;
    /* now_us 在前台调用前取得，ADC 可能随后发布更晚时间；这种交错不是超时。
     * 以半周期规则拒绝倒序观察，下次前台调用会使用新时间再检查。 */
    if (runtime_status.state == MOTOR_RUNTIME_DISABLED
        && elapsed_us < 0x80000000u && elapsed_us > limit_us) {
        /* 持锁直接更新，避免退出临界区后新包到达而误把旧超时锁为故障。 */
        board_disable_output();
        runtime_status.state = MOTOR_RUNTIME_FAULT;
        runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
    }
    board_leave_critical(previous_mask);
}
