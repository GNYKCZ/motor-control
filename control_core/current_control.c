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


/** 有界板级实验的单精度等价实现；保留通用double入口作为独立数值对照。
 * 共享一次sin/cos，未超圆界时不求根；所有参数受限，避免以优化为名删除非法输入门槛。 */
static bool cc_in_range(float value,float low,float high) { return value>=low && value<=high; }
static float cc_clip(float value,float limit) { return value>limit ? limit : (value< -limit ? -limit : value); }
bool current_control_step_bounded(current_control_state_t *state,current_control_config_t c,
    current_control_input_t x,current_control_output_t *output)
{
    if (output==NULL) { return false; }
    *output=(current_control_output_t){0};
    if (state==NULL || !cc_in_range(c.d.kp_V_per_A,0.0f,2.0f) || !cc_in_range(c.q.kp_V_per_A,0.0f,2.0f)
        || !cc_in_range(c.d.ki_V_per_A_s,0.0f,500.0f) || !cc_in_range(c.q.ki_V_per_A_s,0.0f,500.0f)
        || !cc_in_range(c.d.tracking_gain_per_s,0.0f,1000.0f) || !cc_in_range(c.q.tracking_gain_per_s,0.0f,1000.0f)
        || !cc_in_range(c.reference_limit_A,0.001f,0.05f) || !cc_in_range(c.modulation_limit,0.001f,0.1f)
        || !cc_in_range(c.minimum_dt_s,0.0001f,0.002f) || !cc_in_range(c.maximum_dt_s,c.minimum_dt_s,0.002f)
        || !cc_in_range(x.dt_s,c.minimum_dt_s,c.maximum_dt_s) || !cc_in_range(x.bus_voltage_V,9.0f,15.0f)
        || !cc_in_range(x.current_u_A,-0.2f,0.2f) || !cc_in_range(x.current_v_A,-0.2f,0.2f)
        || !cc_in_range(x.electrical_angle_rad,0.0f,6.2831853072f)
        || !cc_in_range(x.reference_d_A,-1.0f,1.0f) || !cc_in_range(x.reference_q_A,-1.0f,1.0f)
        || !cc_in_range(state->integral_d_V,-2.0f,2.0f) || !cc_in_range(state->integral_q_V,-2.0f,2.0f)) { return false; }
    current_control_output_t y={0};
    float sn=sinf(x.electrical_angle_rad),cs=cosf(x.electrical_angle_rad);
    float beta=(x.current_u_A+2.0f*x.current_v_A)*0.5773502692f;
    y.measured.current_d_A=x.current_u_A*cs+beta*sn;
    y.measured.current_q_A=-x.current_u_A*sn+beta*cs;y.measured.is_valid=true;
    y.limited_reference_d_A=cc_clip(x.reference_d_A,c.reference_limit_A);
    y.limited_reference_q_A=cc_clip(x.reference_q_A,c.reference_limit_A);
    y.is_reference_limited=y.limited_reference_d_A!=x.reference_d_A || y.limited_reference_q_A!=x.reference_q_A;
    float ed=y.limited_reference_d_A-y.measured.current_d_A,eq=y.limited_reference_q_A-y.measured.current_q_A;
    float vd=c.d.kp_V_per_A*ed+state->integral_d_V,vq=c.q.kp_V_per_A*eq+state->integral_q_V;
    y.requested_voltage_d_V=vd;y.requested_voltage_q_V=vq;
    float limit=c.modulation_limit*x.bus_voltage_V*0.5773502692f;
    float squared=vd*vd+vq*vq;
    y.modulation.is_voltage_limited=squared>limit*limit;
    if (y.modulation.is_voltage_limited) { float scale=limit/sqrtf(squared);vd*=scale;vq*=scale; }
    y.modulation.limited_voltage_d_V=vd;y.modulation.limited_voltage_q_V=vq;
    float alpha_v=vd*cs-vq*sn,beta_v=vd*sn+vq*cs;
    float u=alpha_v,v=-0.5f*alpha_v+0.8660254038f*beta_v,w=-0.5f*alpha_v-0.8660254038f*beta_v;
    float high=u>v ? u : v;high=high>w ? high : w;
    float low=u<v ? u : v;low=low<w ? low : w;
    float common=0.5f*(high+low),inverse_bus=1.0f/x.bus_voltage_V;
    y.modulation.duty_u=0.5f+(u-common)*inverse_bus;
    y.modulation.duty_v=0.5f+(v-common)*inverse_bus;
    y.modulation.duty_w=0.5f+(w-common)*inverse_bus;
    current_control_state_t next={state->integral_d_V+c.d.ki_V_per_A_s*x.dt_s*ed
        +c.d.tracking_gain_per_s*x.dt_s*(vd-y.requested_voltage_d_V),
        state->integral_q_V+c.q.ki_V_per_A_s*x.dt_s*eq
        +c.q.tracking_gain_per_s*x.dt_s*(vq-y.requested_voltage_q_V)};
    if (!cc_in_range(next.integral_d_V,-2.0f,2.0f) || !cc_in_range(next.integral_q_V,-2.0f,2.0f)
        || !cc_in_range(y.modulation.duty_u,0.0f,1.0f) || !cc_in_range(y.modulation.duty_v,0.0f,1.0f)
        || !cc_in_range(y.modulation.duty_w,0.0f,1.0f)) { return false; }
    y.modulation.is_valid=true;y.is_valid=true;*state=next;*output=y;return true;
}
