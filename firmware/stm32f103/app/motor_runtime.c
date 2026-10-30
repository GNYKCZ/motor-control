/** 运行状态所有者。前台与 ADC/故障 IRQ 共享下列静态状态，统一以短临界区互斥。
 * 生产仅固定本地按键profile；串口无使能入口。合成profile仅在host构建，不进入目标映像。 */
#include "motor_runtime.h"
#include "board_config.h"
#include "board_io.h"
#include <stddef.h>

#if defined(MOTOR_GATE_TEST_PROFILES) && (defined(__ARM_ARCH) || defined(__arm__))
#error "Synthetic output profiles are forbidden in target firmware"
#endif

typedef struct { board_pwm_ticks_t ticks; uint32_t arm_us, duration_us; } pwm_profile_t;
#ifdef MOTOR_GATE_TEST_PROFILES
/* 只供host时间轨迹，不是实际电机的安全限值，不得进入目标构建。 */
static const pwm_profile_t test_profile = {{100u,200u,300u,40u},10000u,1000u};
#endif
#ifndef MOTOR_GATE_TEST_PROFILES
/* 商家标称R/L推导的12V低能量候选，不代表实测电流上界；只能由本地新手势消耗一次。 */
static const pwm_profile_t local_profile = {{60u,0u,0u,434u},5000u,20000u};
static bool motor_runtime_arm(unsigned profile_id);
static bool motor_runtime_pulse(void);
#endif
static motor_action_t last_action;
static uint32_t zero_began_us, zero_ended_us;
static bool zero_explicit;
static motor_current_snapshot_t current_capture;
static bool button_spent, button_raw, button_seen, still_seen, fault_seen;
static unsigned button_phase; /**< 0等稳定释放，1READY，2PRESSED；spent独立锁住本启动机会。 */
static uint32_t button_last_us, button_changed_us, still_since_us;
static uint16_t position_previous;
static int32_t position_count, still_min, still_max, action_origin;
static const pwm_profile_t *output_profile;
static uint32_t output_generation, arm_deadline_us, pulse_deadline_us;

unsigned motor_runtime_output_profile_count(void)
{
#ifdef MOTOR_GATE_TEST_PROFILES
    return 1u;
#else
    return 1u; /* 唯一本地固定实验profile，不接受串口参数或动态注册。 */
#endif
}

static motor_status_t runtime_status; /**< 唯一生命周期所有者，所有共享访问持短锁。 */

static bool output_is_protected(void)
{
    return runtime_status.state==MOTOR_RUNTIME_ARMED || runtime_status.state==MOTOR_RUNTIME_STARTING
        || runtime_status.state==MOTOR_RUNTIME_ACTIVE;
}
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
static uint32_t feedback_enqueued_us; /**< 待消费输入所属ADC完成时刻，和原始序号一起保留。 */

/** 当前DISABLED或受保护状态都继续处理反馈；FAULT/BOOT不得重新发布有效。 */
static bool input_is_monitored(void)
{
    return runtime_status.state==MOTOR_RUNTIME_DISABLED || output_is_protected();
}

/** 已持锁：真实现在时刻检查；失联、资格或脉冲期限均不依赖前台循环。 */
static void expire_output(uint32_t now)
{
    if (!output_is_protected()) { return; }
    if (now-runtime_status.sample.current_completed_us>BOARD_SAMPLE_GAP_LIMIT_US) {
        motor_runtime_latch_fault(MOTOR_FAULT_SAMPLE);
    } else if (now-runtime_status.feedback_completed_us>BOARD_FEEDBACK_GAP_LIMIT_US) {
        motor_runtime_latch_fault(MOTOR_FAULT_FEEDBACK);
    } else if ((int32_t)(now-(runtime_status.state==MOTOR_RUNTIME_ARMED ? arm_deadline_us : pulse_deadline_us))>=0) {
        if (last_action.reason==MOTOR_ACTION_WAIT_START || last_action.reason==MOTOR_ACTION_ACTIVE) {
            last_action.reason=MOTOR_ACTION_DURATION;
        }
        motor_runtime_stop();
    }
}

/** 已持锁：绝对期限不随新命令续期；失败立即撤销，禁止递归重试或等16bit下一圈。 */
static void schedule_output_deadline(void)
{
    if (!output_is_protected()) { board_cancel_deadline(); return; }
    uint32_t now=board_get_time_us();
    uint32_t deadline=runtime_status.sample.current_completed_us+BOARD_SAMPLE_GAP_LIMIT_US+1u;
    uint32_t feedback_deadline=runtime_status.feedback_completed_us+BOARD_FEEDBACK_GAP_LIMIT_US+1u;
    if ((int32_t)(feedback_deadline-deadline)<0) { deadline=feedback_deadline; }
    uint32_t output_deadline=runtime_status.state==MOTOR_RUNTIME_ARMED ? arm_deadline_us : pulse_deadline_us;
    if ((int32_t)(output_deadline-deadline)<0) { deadline=output_deadline; }
    if ((int32_t)(deadline-now)<=0 || !board_schedule_deadline(deadline)) {
        expire_output(board_get_time_us());
        if (output_is_protected()) { motor_runtime_latch_fault(MOTOR_FAULT_DEVICE); }
    }
}

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
    if (!input_is_monitored()) { return; }
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
            uint32_t age_us = sample->current_completed_us - feedback_enqueued_us;
            if (age_us > runtime_status.mailbox_wait_max_us) {
                runtime_status.mailbox_wait_max_us = age_us;
            }
            runtime_status.state = MOTOR_RUNTIME_FAULT;
            runtime_status.fault_flags |= MOTOR_FAULT_FEEDBACK;
            return;
        }
        feedback_sample_t input = {sample->encoder.raw_angle_count, sample->encoder.sequence,
            sample->encoder.read_started_us, sample->encoder.read_completed_us, true};
        feedback_mailbox = input;
        feedback_enqueued_us = sample->current_completed_us;
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
    runtime_status.pwm_is_permanently_blocked = motor_runtime_output_profile_count()==0u;
    last_action=(motor_action_t){0}; button_spent=false; button_phase=0u; button_seen=false;
    zero_began_us=0u; zero_ended_us=0u;
    zero_explicit=false;
    current_capture=(motor_current_snapshot_t){0};
    still_seen=false; fault_seen=false; position_count=0; still_min=0; still_max=0;
    output_profile=NULL; output_generation++; arm_deadline_us=0u; pulse_deadline_us=0u;
    board_cancel_deadline();
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

void motor_runtime_stop(void)
{
    uint32_t mask=board_enter_critical();
    if (last_action.reason==MOTOR_ACTION_WAIT_START || last_action.reason==MOTOR_ACTION_ACTIVE) {
        last_action.reason=MOTOR_ACTION_STOP;
    }
    board_disable_output();
    if (output_is_protected() && last_action.id!=0u && !(last_action.timing_flags & 0x40u)) {
        last_action.ended_us=board_get_time_us(); last_action.timing_flags |= 0x40u;
    }
    button_phase=0u; button_seen=false;
    output_generation++; output_profile=NULL;
    board_cancel_deadline();
    if (output_is_protected()) { runtime_status.state=MOTOR_RUNTIME_DISABLED; }
    board_leave_critical(mask);
}

void motor_runtime_latch_fault(motor_fault_t fault)
{
    uint32_t mask=board_enter_critical();
    if (last_action.reason==MOTOR_ACTION_WAIT_START || last_action.reason==MOTOR_ACTION_ACTIVE) {
        last_action.reason=MOTOR_ACTION_FAULT;
    }
    bool repeated=fault_seen; fault_seen=true;
    unsigned old_button_phase=button_phase; bool old_button_seen=button_seen;
    motor_runtime_stop(); /* 每条故障都撤销输出代次/期限；重复已锁存故障不阻止只读拒绝试验手势。 */
    if (repeated) { button_phase=old_button_phase; button_seen=old_button_seen; }
    runtime_status.fault_flags |= (uint32_t)fault;
    runtime_status.state=MOTOR_RUNTIME_FAULT;
    motor_runtime_stop_offset(false);
    motor_runtime_clear_feedback();
    board_leave_critical(mask);
}

void motor_runtime_communication_error(void)
{
    uint32_t mask=board_enter_critical();
    if (output_is_protected()) { motor_runtime_latch_fault(MOTOR_FAULT_COMMUNICATION); }
    board_leave_critical(mask);
}

/** 已持锁，使用现在的时间；曾经有效不能替代当前新鲜输入。 */
static bool input_data_are_valid(void)
{
    uint32_t now=board_get_time_us();
    return runtime_status.fault_flags==0u && runtime_status.sample.is_valid
        && runtime_status.feedback.is_valid && runtime_status.feedback.direct_is_valid
        && runtime_status.feedback.window_is_valid
        && now-runtime_status.sample.current_completed_us<=BOARD_SAMPLE_GAP_LIMIT_US
        && now-runtime_status.feedback_completed_us<=BOARD_FEEDBACK_GAP_LIMIT_US;
}

static bool output_inputs_are_valid(void)
{
    return input_data_are_valid() && board_output_is_ready();
}

#ifndef MOTOR_GATE_TEST_PROFILES
static
#endif
bool motor_runtime_arm(unsigned profile_id)
{
    uint32_t mask=board_enter_critical();
    const pwm_profile_t *candidate=NULL;
#ifdef MOTOR_GATE_TEST_PROFILES
    if (profile_id==1u) { candidate=&test_profile; }
#else
    if (profile_id==1u) { candidate=&local_profile; }
#endif
    bool accepted=candidate!=NULL && runtime_status.state==MOTOR_RUNTIME_DISABLED && output_inputs_are_valid();
    if (accepted) {
        output_profile=candidate; output_generation++;
        arm_deadline_us=board_get_time_us()+candidate->arm_us;
        runtime_status.state=MOTOR_RUNTIME_ARMED;
        schedule_output_deadline();
        accepted=runtime_status.state==MOTOR_RUNTIME_ARMED;
    } else if (output_is_protected()) { motor_runtime_stop(); }
    board_leave_critical(mask);
    return accepted;
}

#ifndef MOTOR_GATE_TEST_PROFILES
static
#endif
bool motor_runtime_pulse(void)
{
    uint32_t mask=board_enter_critical();
    bool accepted=runtime_status.state==MOTOR_RUNTIME_ARMED && output_profile!=NULL
        && (int32_t)(arm_deadline_us-board_get_time_us())>0 && output_inputs_are_valid();
    if (accepted) {
#ifdef MOTOR_GATE_TEST_PROFILES
        pulse_deadline_us=board_get_time_us()+output_profile->duration_us;
#else
        /* 本地释放的绝对期限包含资格检查和STARTING，不能因内部事务耗时后移。 */
        pulse_deadline_us=last_action.requested_us+output_profile->duration_us;
#endif
        runtime_status.state=MOTOR_RUNTIME_STARTING;
        motor_runtime_stop_offset(false); /* 开始后永不再收集带流零偏候选。 */
        schedule_output_deadline();
        accepted=runtime_status.state==MOTOR_RUNTIME_STARTING
            && board_prepare_pulse(output_profile->ticks,output_generation);
    }
    if (!accepted && output_is_protected()) { motor_runtime_stop(); }
    board_leave_critical(mask);
    return accepted;
}

bool motor_runtime_commit_output(uint32_t generation)
{
    /* board已持同一短锁；成功只授予本次MOE写，其他路径从不自动使能。 */
    if (runtime_status.state!=MOTOR_RUNTIME_STARTING || generation!=output_generation
        || (int32_t)(pulse_deadline_us-board_get_time_us())<=0 || !output_inputs_are_valid()) { return false; }
    runtime_status.state=MOTOR_RUNTIME_ACTIVE;
    return true;
}

bool motor_runtime_output_deadline_reached(uint32_t now_us)
{
    return (runtime_status.state==MOTOR_RUNTIME_STARTING || runtime_status.state==MOTOR_RUNTIME_ACTIVE)
        && (int32_t)(now_us-pulse_deadline_us)>=0;
}

void motor_runtime_handle_sample(const motor_sample_t *sample)
{
    uint32_t previous_mask = board_enter_critical();
    if (!output_is_protected()) { board_disable_output(); }
    if (sample == NULL || runtime_status.state == MOTOR_RUNTIME_BOOT) {
        runtime_status.state = MOTOR_RUNTIME_FAULT;
        runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
        runtime_status.rejected_count++;
        motor_runtime_stop_offset(true);
        motor_runtime_clear_feedback();
        motor_runtime_latch_fault(MOTOR_FAULT_SAMPLE);
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
    /* 先保留触发位移保护的最后一个有效样本；尚未开门/截止后的数据不得冒充带流。
     * 最多32帧，仅整数和/极值，无IRQ格式化；超容量冻结并标记，不覆盖早期证据。 */
    if (is_valid && runtime_status.state==MOTOR_RUNTIME_ACTIVE && (last_action.timing_flags & 0x80u)
        && sample->current_completed_us-last_action.started_us<0x80000000u
        && (int32_t)(sample->current_completed_us-pulse_deadline_us)<0) {
        sensor_offset_t *stats=&current_capture.samples;
        if (stats->accepted_count>=32u) { current_capture.overflow=1u; }
        else {
            if (stats->accepted_count==0u) {
                current_capture.began_us=sample->current_completed_us;
                current_capture.action_id=last_action.id;
                current_capture.first_angle=sample->encoder.raw_angle_count;
            }
            (void)sensor_offset_add(stats,sample->raw_current_u_count,sample->raw_current_v_count,true);
            current_capture.ended_us=sample->current_completed_us;
            current_capture.last_angle=sample->encoder.raw_angle_count;
        }
    }
    if (is_valid) {
        uint16_t raw=sample->encoder.raw_angle_count;
        if (!still_seen) {
            still_seen=true; position_previous=raw; position_count=0;
            still_min=0; still_max=0; still_since_us=sample->current_completed_us;
        } else {
            int32_t step=(int32_t)((raw-position_previous)&16383u);
            if (step>=8192) { step-=16384; }
            position_previous=raw;
            /* 无动作时只需维护有界静止窗口；活动期间4ms，计数和不会接近int32边界。 */
            position_count+=step;
            if (position_count<still_min) { still_min=position_count; }
            if (position_count>still_max) { still_max=position_count; }
            if (still_max-still_min>=16) {
                if (zero_explicit) { motor_runtime_stop_offset(false); }
                still_since_us=sample->current_completed_us;
                if (runtime_status.state!=MOTOR_RUNTIME_STARTING && runtime_status.state!=MOTOR_RUNTIME_ACTIVE) {
                    position_count=0;
                }
                still_min=position_count; still_max=position_count;
            }
            if ((runtime_status.state==MOTOR_RUNTIME_STARTING || runtime_status.state==MOTOR_RUNTIME_ACTIVE)
                && (position_count-action_origin>=46 || position_count-action_origin<=-46)) {
                motor_runtime_latch_fault(MOTOR_FAULT_EXPERIMENT);
            }
        }
    } else { still_seen=false; }
    runtime_status.sample = *sample;
    runtime_status.sample.is_valid = is_valid;
    runtime_status.sample_count++;
    if (is_valid) {
        runtime_status.has_valid_sample = true;
        if (!offset_is_continuous) { motor_runtime_stop_offset(true); }
        if (runtime_status.state == MOTOR_RUNTIME_DISABLED && !runtime_status.offset_is_aborted
            && runtime_status.estimated_offset.accepted_count < SENSOR_OFFSET_SAMPLE_COUNT) {
            if (!sensor_offset_add(&runtime_status.estimated_offset, sample->raw_current_u_count,
                    sample->raw_current_v_count, true)) {
                motor_runtime_stop_offset(true);
                runtime_status.state = MOTOR_RUNTIME_FAULT;
                runtime_status.fault_flags |= MOTOR_FAULT_SAMPLE;
            } else {
                if (runtime_status.estimated_offset.accepted_count==1u) { zero_began_us=sample->current_completed_us; }
                zero_ended_us=sample->current_completed_us;
            }
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
        motor_runtime_latch_fault((motor_fault_t)runtime_status.fault_flags);
    } else { schedule_output_deadline(); }
    board_leave_critical(previous_mask);
}

void motor_runtime_poll_feedback(void)
{
    feedback_sample_t input;
    uint32_t previous_mask = board_enter_critical();
    if (!input_is_monitored() || !feedback_pending) {
        board_leave_critical(previous_mask);
        return;
    }
    input = feedback_mailbox;
    uint32_t poll_started_us = board_get_time_us();
    uint32_t wait_us = poll_started_us - feedback_enqueued_us;
    if (wait_us > runtime_status.mailbox_wait_max_us) {
        runtime_status.mailbox_wait_max_us = wait_us;
    }
    feedback_pending = false;
    board_leave_critical(previous_mask);
    /* 中断保持开放；有界历史更新及double运算全部在前台，IRQ不访问算法状态。 */
    feedback_result_t result;
    bool valid = feedback_step(&feedback_state, input, &result);
    previous_mask = board_enter_critical();
    if (input_is_monitored()) {
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
            motor_runtime_latch_fault((motor_fault_t)runtime_status.fault_flags);
        } else {
            runtime_status.feedback = result;
            runtime_status.feedback_sequence = input.sequence;
            runtime_status.feedback_completed_us = input.read_completed_us;
            runtime_status.feedback_raw_count = input.raw_angle_count;
        }
    }
    schedule_output_deadline();
    uint32_t poll_elapsed_us = board_get_time_us() - poll_started_us;
    if (poll_elapsed_us > runtime_status.feedback_poll_max_us) {
        runtime_status.feedback_poll_max_us = poll_elapsed_us;
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

bool motor_runtime_read_log(motor_log_snapshot_t *snapshot)
{
    if (snapshot == NULL) { return false; }
    uint32_t previous_mask = board_enter_critical();
    snapshot->state = runtime_status.state;
    snapshot->fault_flags = runtime_status.fault_flags;
    snapshot->sequence = runtime_status.sample.sequence;
    snapshot->completed_us = runtime_status.sample.current_completed_us;
    snapshot->raw_u = runtime_status.sample.raw_current_u_count;
    snapshot->raw_v = runtime_status.sample.raw_current_v_count;
    snapshot->raw_angle = runtime_status.sample.encoder.raw_angle_count;
    snapshot->sample_is_valid = runtime_status.sample.is_valid;
    snapshot->position_is_valid = runtime_status.feedback.is_valid;
    snapshot->direct_is_valid = runtime_status.feedback.direct_is_valid;
    snapshot->window_is_valid = runtime_status.feedback.window_is_valid;
    snapshot->feedback_overruns = runtime_status.feedback_overruns;
    snapshot->feedback_max_us = runtime_status.feedback_poll_max_us;
    snapshot->mailbox_max_us = runtime_status.mailbox_wait_max_us;
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
    if (input_is_monitored()
        && elapsed_us < 0x80000000u && elapsed_us > limit_us) {
        motor_runtime_latch_fault(MOTOR_FAULT_SAMPLE);
    }
    expire_output(board_get_time_us());
    if (output_is_protected()) {
        /* 所有期限已处理；前台的旧now不用于重新安排过去的比较点。 */
        schedule_output_deadline();
    }
    board_leave_critical(previous_mask);
}

/** board中断已持锁，只记录MOE写入近邻的MCU时间；0可以是合法回绕时刻。 */
void motor_runtime_record_gate(bool opened, uint32_t now_us)
{
    if (last_action.id==0u) { return; }
    if (opened) {
        last_action.started_us=now_us; last_action.timing_flags |= 0x80u;
        last_action.reason=MOTOR_ACTION_ACTIVE;
    } else { last_action.ended_us=now_us; last_action.timing_flags |= 0x40u; }
}

bool motor_runtime_read_reply(motor_log_snapshot_t *status, motor_action_t *action)
{
    if (status==NULL || action==NULL) { return false; }
    uint32_t mask=board_enter_critical();
    bool valid=motor_runtime_read_log(status);
    *action=last_action;
    board_leave_critical(mask);
    return valid;
}

void motor_runtime_poll_button(bool pressed, uint32_t now_us)
{
    uint32_t mask=board_enter_critical();
    if (button_spent) { board_leave_critical(mask); return; }
    if (!button_seen || now_us-button_last_us>5000u) {
        button_seen=true; button_phase=0u; button_raw=pressed;
        button_changed_us=now_us; button_last_us=now_us;
        board_leave_critical(mask); return;
    }
    if (now_us-button_last_us<1000u) { board_leave_critical(mask); return; }
    button_last_us=now_us;
    if (pressed!=button_raw) { button_raw=pressed; button_changed_us=now_us; }
    if (now_us-button_changed_us>=20000u) {
        if (button_phase==0u && !pressed) { button_phase=1u; }
        else if (button_phase==1u && pressed) { button_phase=2u; }
        else if (button_phase==2u && !pressed) {
            button_spent=true; /* 先消耗，资格失败/故障/STOP均不返还；只有Reset可建新机会。 */
            last_action=(motor_action_t){1u,board_get_time_us(),0u,0u,MOTOR_ACTION_REJECTED,0u};
            action_origin=position_count;
            bool still=still_seen && board_get_time_us()-still_since_us>=250000u;
            if (still && motor_runtime_arm(1u)) {
                last_action.reason=MOTOR_ACTION_WAIT_START;
                if (!motor_runtime_pulse()) { last_action.reason=MOTOR_ACTION_REJECTED; last_action.ended_us=board_get_time_us(); last_action.timing_flags |= 0x40u; }
            } else { last_action.ended_us=board_get_time_us(); last_action.timing_flags |= 0x40u; }
        }
    }
    board_leave_critical(mask);
}

bool motor_runtime_output_is_busy(void)
{
    uint32_t mask=board_enter_critical();
    bool busy=output_is_protected();
    board_leave_critical(mask); return busy;
}

bool motor_runtime_restart_zero(void)
{
    uint32_t mask=board_enter_critical();
    bool allowed=runtime_status.state==MOTOR_RUNTIME_DISABLED && last_action.id==0u
        && still_seen && board_get_time_us()-still_since_us>=250000u && input_data_are_valid();
    if (allowed) {
        (void)sensor_offset_init(&runtime_status.estimated_offset);
        runtime_status.offset_is_aborted=false; runtime_status.offset_rejected_count=0u;
        runtime_status.calibration_verified=false; zero_began_us=0u; zero_ended_us=0u;
        zero_explicit=true;
    }
    board_leave_critical(mask); return allowed;
}

bool motor_runtime_read_zero(motor_log_snapshot_t *status, motor_zero_snapshot_t *zero)
{
    if (status==NULL || zero==NULL) { return false; }
    uint32_t mask=board_enter_critical();
    bool valid=motor_runtime_read_log(status);
    zero->offset=runtime_status.estimated_offset;
    zero->began_us=zero_began_us; zero->ended_us=zero_ended_us;
    zero->profile_id=BOARD_SAMPLING_PROFILE_ID;
    zero->scale_nA=(uint32_t)(BOARD_NOMINAL_A_PER_COUNT*1000000000.0f+0.5f);
    zero->flags=(uint8_t)(8u | (zero_explicit ? 16u : 0u)
        | (zero->offset.accepted_count==SENSOR_OFFSET_SAMPLE_COUNT ? 1u : 0u)
        | (runtime_status.offset_is_aborted ? 2u : 0u)
        | (runtime_status.state==MOTOR_RUNTIME_DISABLED && runtime_status.fault_flags==0u && still_seen
           && board_get_time_us()-still_since_us>=250000u && input_data_are_valid() ? 4u : 0u));
    board_leave_critical(mask); return valid;
}

bool motor_runtime_read_current(motor_log_snapshot_t *status, motor_zero_snapshot_t *zero,
                                motor_current_snapshot_t *current)
{
    if (status==NULL || zero==NULL || current==NULL) { return false; }
    uint32_t mask=board_enter_critical();
    bool valid=motor_runtime_read_zero(status,zero);
    *current=current_capture;
    board_leave_critical(mask); return valid;
}
