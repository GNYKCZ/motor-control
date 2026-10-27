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
static feedback_state_t feedback_state; /**< 只由前台init/poll写；IRQ从不复制或计算该大状态。 */
static feedback_sample_t feedback_mailbox; /**< 单槽值交接；所有访问持runtime短临界区。 */
static bool feedback_pending;
static bool feedback_has_raw; /**< 首个有效原始帧建立基线，不要求ADC与encoder序号相等。 */
static uint32_t feedback_raw_sequence;
static uint32_t feedback_adc_sequence;
static uint32_t feedback_divider;

/** 已持锁：失效已发布结果及待处理输入，不重置前台独占的算法状态。 */
static void motor_runtime_clear_feedback(void)
{
    feedback_result_t empty = {0};
    runtime_status.feedback = empty;
    runtime_status.feedback_sequence = 0u;
    runtime_status.feedback_completed_us = 0u;
    runtime_status.feedback_raw_count = 0u;
    feedback_pending = false;
}

/** 已持锁：每个原始帧都核验，只把固定分频后的真实值交给前台。无浮点/循环。 */
static void motor_runtime_queue_feedback(const motor_sample_t *sample, bool valid)
{
    if (runtime_status.state != MOTOR_RUNTIME_DISABLED) { return; }
    if (!valid && !feedback_has_raw) { return; } /* 保留M1首帧启动宽限。 */
    if (!valid || (feedback_has_raw &&
        (sample->sequence - feedback_adc_sequence != 1u ||
         sample->encoder.sequence - feedback_raw_sequence != 1u))) {
        runtime_status.state = MOTOR_RUNTIME_FAULT;
        runtime_status.fault_flags |= MOTOR_FAULT_FEEDBACK;
        return;
    }
    feedback_has_raw = true;
    feedback_adc_sequence = sample->sequence;
    feedback_raw_sequence = sample->encoder.sequence;
    if (feedback_divider == 0u) {
        if (feedback_pending) {
            runtime_status.feedback_overruns++;
            runtime_status.state = MOTOR_RUNTIME_FAULT;
            runtime_status.fault_flags |= MOTOR_FAULT_FEEDBACK;
            return;
        }
        feedback_sample_t input = {sample->encoder.raw_angle_count, sample->encoder.sequence,
            sample->encoder.read_started_us, sample->encoder.read_completed_us, true};
        feedback_mailbox = input;
        feedback_pending = true;
        feedback_divider = BOARD_FEEDBACK_DIVIDER;
    }
    feedback_divider--;
}

/** 持有runtime临界区时中止未完成候选；不清已有证据，不因后续好帧自动复位。 */
static void motor_runtime_stop_offset(bool rejected_frame)
{
    if (!runtime_status.offset_is_aborted
        && runtime_status.estimated_offset.accepted_count < SENSOR_OFFSET_SAMPLE_COUNT) {
        runtime_status.offset_is_aborted = true;
        if (rejected_frame) { runtime_status.offset_rejected_count++; }
    }
}

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
    feedback_pending = false;
    feedback_has_raw = false;
    feedback_divider = 0u;
    /* 在采样起动前初始化，direction=+1仅沿用raw正向，未做物理方向标定。 */
    feedback_config_t config = {BOARD_FEEDBACK_MAX_SPEED_RAD_S, BOARD_FEEDBACK_GAP_LIMIT_US,
        BOARD_ENCODER_TRANSFER_LIMIT_US, BOARD_FEEDBACK_WINDOW, 1};
    if (!feedback_init_spaced(&feedback_state, config, BOARD_FEEDBACK_DIVIDER)) {
        runtime_status.state = MOTOR_RUNTIME_FAULT;
        runtime_status.fault_flags |= MOTOR_FAULT_INITIALIZATION;
    }
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
        motor_runtime_stop_offset(false);
        motor_runtime_clear_feedback();
    }
    board_leave_critical(previous_mask);
}

void motor_runtime_latch_fault(motor_fault_t fault)
{
    board_disable_output();
    uint32_t previous_mask = board_enter_critical();
    runtime_status.fault_flags |= (uint32_t)fault;
    runtime_status.state = MOTOR_RUNTIME_FAULT;
    motor_runtime_stop_offset(false);
    motor_runtime_clear_feedback();
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
        motor_runtime_stop_offset(true);
        motor_runtime_clear_feedback();
        board_leave_critical(previous_mask);
        return;
    }
    motor_sample_t checked_sample = board_make_sample(sample->sequence,
        sample->current_completed_us, sample->raw_current_u_count,
        sample->raw_current_v_count, &sample->encoder);
    bool is_valid = checked_sample.is_valid && sample->is_valid;
    /* 候选批比M1“序号更新”更严格：两种完成序号必须逐一连续，模回绕允许。
     * 跳号包仍按原M1契约判断raw有效，但不得用于补齐零偏候选。 */
    bool offset_is_continuous = !has_received_sample
        || ((uint32_t)(sample->sequence - runtime_status.sample.sequence) == 1u
            && (uint32_t)(sample->encoder.sequence - runtime_status.sample.encoder.sequence) == 1u);
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
        if (!offset_is_continuous) { motor_runtime_stop_offset(true); }
        if (runtime_status.state == MOTOR_RUNTIME_DISABLED && !runtime_status.offset_is_aborted
            && runtime_status.estimated_offset.accepted_count < SENSOR_OFFSET_SAMPLE_COUNT
            && !sensor_offset_add(&runtime_status.estimated_offset, sample->raw_current_u_count,
                sample->raw_current_v_count, true)) {
            motor_runtime_stop_offset(true);
            runtime_status.state = MOTOR_RUNTIME_FAULT;
            runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
        }
    } else {
        motor_runtime_stop_offset(true);
        runtime_status.rejected_count++;
        if (runtime_status.has_valid_sample
            || sample->current_completed_us - first_sample_time_us >= BOARD_BOOT_SAMPLE_LIMIT_US) {
            runtime_status.state = MOTOR_RUNTIME_FAULT;
            runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
        }
    }
    motor_runtime_queue_feedback(sample, is_valid);
    if (runtime_status.state == MOTOR_RUNTIME_FAULT) {
        motor_runtime_stop_offset(false);
        motor_runtime_clear_feedback();
    }
    board_leave_critical(previous_mask);
}

void motor_runtime_poll_feedback(void)
{
    feedback_sample_t input;
    uint32_t previous_mask = board_enter_critical();
    if (runtime_status.state != MOTOR_RUNTIME_DISABLED || !feedback_pending) {
        board_leave_critical(previous_mask);
        return;
    }
    input = feedback_mailbox;
    feedback_pending = false;
    board_leave_critical(previous_mask);
    /* 中断保持开放；有界历史更新及double运算全部在前台，IRQ不访问算法状态。 */
    feedback_result_t result;
    bool valid = feedback_step(&feedback_state, input, &result);
    previous_mask = board_enter_critical();
    if (runtime_status.state == MOTOR_RUNTIME_DISABLED) {
        /* RV-001：必须以发布时的设备当前时间检查。ADC停更时最后一帧时间也停住，
         * 不能用它代替now，否则前台延迟后会先发布过期有效值，再到下轮才报超时。
         * 持锁取时与最近ADC一致，避免取时后新ADC发布造成无符号倒序误报。 */
        uint32_t now_us = board_get_time_us();
        uint32_t age = now_us - input.read_completed_us;
        bool sample_is_late = now_us - runtime_status.sample.current_completed_us > BOARD_SAMPLE_GAP_LIMIT_US;
        if (!valid || age > BOARD_FEEDBACK_GAP_LIMIT_US || sample_is_late) {
            board_disable_output();
            runtime_status.state = MOTOR_RUNTIME_FAULT;
            if (!valid || age > BOARD_FEEDBACK_GAP_LIMIT_US) {
                runtime_status.fault_flags |= MOTOR_FAULT_FEEDBACK;
            }
            if (sample_is_late) { runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE; }
            motor_runtime_stop_offset(false);
            motor_runtime_clear_feedback();
        } else {
            runtime_status.feedback = result;
            runtime_status.feedback_sequence = input.sequence;
            runtime_status.feedback_completed_us = input.read_completed_us;
            runtime_status.feedback_raw_count = input.raw_angle_count;
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
        motor_runtime_stop_offset(false);
        motor_runtime_clear_feedback();
    }
    board_leave_critical(previous_mask);
}
