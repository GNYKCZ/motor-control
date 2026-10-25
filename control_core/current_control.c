/** 位置式PI与最终联合限幅反馈；按M2b契约独立实现，不复制例程PID/USER CODE。
 * 除调用者的两个积分外无持久状态；M2a直接复用，无第二套变换/调制算法。 */
#include "current_control.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

#if defined(__FAST_MATH__)
#error "current_control requires finite-input checks; do not enable fast-math"
#endif

/** 检查单轴显式增益，不对合法增益自动整定或补默认值。 */
static bool current_control_axis_is_valid(current_control_axis_t axis)
{
    return isfinite(axis.kp_V_per_A) && axis.kp_V_per_A >= 0.0f
        && isfinite(axis.ki_V_per_A_s) && axis.ki_V_per_A_s >= 0.0f
        && isfinite(axis.tracking_gain_per_s) && axis.tracking_gain_per_s >= 0.0f;
}

/** 公开状态/请求必须能表示为有限float；不把溢出静默裁成另一个控制器。 */
static bool current_control_fits_float(double value)
{
    return isfinite(value) && fabs(value) <= FLT_MAX;
}

bool current_control_init(current_control_state_t *state)
{
    return current_control_reset(state);
}

bool current_control_reset(current_control_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    *state = (current_control_state_t){0};
    return true;
}

bool current_control_step(current_control_state_t *state, current_control_config_t config,
    current_control_input_t input, current_control_output_t *output)
{
    if (output == NULL) {
        return false;
    }
    *output = (current_control_output_t){0};
    if (state == NULL || !current_control_axis_is_valid(config.d) || !current_control_axis_is_valid(config.q)
        || !isfinite(config.reference_limit_A) || config.reference_limit_A <= 0.0f
        || !isfinite(config.modulation_limit) || config.modulation_limit <= 0.0f || config.modulation_limit > 1.0f
        || !isfinite(config.minimum_dt_s) || config.minimum_dt_s <= 0.0f
        || !isfinite(config.maximum_dt_s) || config.maximum_dt_s < config.minimum_dt_s
        || !isfinite(input.reference_d_A) || !isfinite(input.reference_q_A)
        || !isfinite(input.dt_s) || input.dt_s < config.minimum_dt_s || input.dt_s > config.maximum_dt_s
        || !isfinite(state->integral_d_V) || !isfinite(state->integral_q_V)) {
        return false;
    }
    current_control_output_t calculated = {0};
    if (!foc_transform_current(input.current_u_A, input.current_v_A,
            input.electrical_angle_rad, &calculated.measured)) {
        return false;
    }
    calculated.limited_reference_d_A = fminf(config.reference_limit_A,
        fmaxf(-config.reference_limit_A, input.reference_d_A));
    calculated.limited_reference_q_A = fminf(config.reference_limit_A,
        fmaxf(-config.reference_limit_A, input.reference_q_A));
    calculated.is_reference_limited = calculated.limited_reference_d_A != input.reference_d_A
        || calculated.limited_reference_q_A != input.reference_q_A;
    double error_d_A = (double)calculated.limited_reference_d_A - calculated.measured.current_d_A;
    double error_q_A = (double)calculated.limited_reference_q_A - calculated.measured.current_q_A;
    double requested_d_V = (double)config.d.kp_V_per_A * error_d_A + state->integral_d_V;
    double requested_q_V = (double)config.q.kp_V_per_A * error_q_A + state->integral_q_V;
    if (!current_control_fits_float(requested_d_V) || !current_control_fits_float(requested_q_V)) {
        return false;
    }
    calculated.requested_voltage_d_V = (float)requested_d_V;
    calculated.requested_voltage_q_V = (float)requested_q_V;
    if (!foc_modulate_voltage(calculated.requested_voltage_d_V, calculated.requested_voltage_q_V,
            input.electrical_angle_rad, input.bus_voltage_V, config.modulation_limit, &calculated.modulation)) {
        return false;
    }
    /* 使用实际传给调制器的float请求及它发布的最终受限float电压。
     * 两轴先算到临时量；即使只有q轴溢出，也绝不能提前提交d轴。 */
    double next_d_V = state->integral_d_V + (double)config.d.ki_V_per_A_s * input.dt_s * error_d_A
        + (double)config.d.tracking_gain_per_s * input.dt_s
            * ((double)calculated.modulation.limited_voltage_d_V - calculated.requested_voltage_d_V);
    double next_q_V = state->integral_q_V + (double)config.q.ki_V_per_A_s * input.dt_s * error_q_A
        + (double)config.q.tracking_gain_per_s * input.dt_s
            * ((double)calculated.modulation.limited_voltage_q_V - calculated.requested_voltage_q_V);
    if (!current_control_fits_float(next_d_V) || !current_control_fits_float(next_q_V)) {
        return false;
    }
    current_control_state_t next_state = {(float)next_d_V, (float)next_q_V};
    calculated.is_valid = true;
    *state = next_state;
    *output = calculated;
    return true;
}
