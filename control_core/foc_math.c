/** 幅值不变Clarke/Park、联合电压限制和中点注入SVPWM。
 * 按docs/m2a_foc_contract.md方程独立实现，不复制例程控制源码/查表。
 * 只有标准C数学依赖，无状态所有者/初始化/复位；输出只写调用者给定的值对象。 */
#include "foc_math.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

#if defined(__FAST_MATH__)
#error "foc_math requires finite-input checks; do not enable fast-math"
#endif

#define FOC_INVERSE_SQRT_THREE 0.57735026918962576451 /**< 1/sqrt(3)，无量纲。 */
#define FOC_SQRT_THREE_OVER_TWO 0.86602540378443864676 /**< sqrt(3)/2，相电压投影系数。 */
#define FOC_DUTY_ROUNDOFF_LIMIT (8.0 * DBL_EPSILON) /**< 仅修复double中间量端点舍入，非调制裕量。 */

/** 将理论[0,1]内的double占空比发布为float。
 * duty只接受本函数内非空临时字段；超出纯舍入误差边界即失败，不掩盖真正过调制。 */
static bool foc_store_duty(double value, float *duty)
{
    if (!isfinite(value) || value < -FOC_DUTY_ROUNDOFF_LIMIT
        || value > 1.0 + FOC_DUTY_ROUNDOFF_LIMIT) {
        return false;
    }
    *duty = (float)fmin(1.0, fmax(0.0, value));
    return true;
}

bool foc_transform_current(float current_u_A, float current_v_A,
    float electrical_angle_rad, foc_current_t *result)
{
    if (result == NULL) {
        return false;
    }
    *result = (foc_current_t){0};
    if (!isfinite(current_u_A) || !isfinite(current_v_A) || !isfinite(electrical_angle_rad)) {
        return false;
    }
    /* u/v对应已校准电流。先升为double，避免2*v或加法在有限float输入下先溢出。 */
    double current_alpha_A = current_u_A;
    double current_beta_A = ((double)current_u_A + 2.0 * current_v_A) * FOC_INVERSE_SQRT_THREE;
    double sine = sin((double)electrical_angle_rad);
    double cosine = cos((double)electrical_angle_rad);
    double current_d_A = current_alpha_A * cosine + current_beta_A * sine;
    double current_q_A = -current_alpha_A * sine + current_beta_A * cosine;
    if (!isfinite(current_d_A) || !isfinite(current_q_A)
        || fabs(current_d_A) > FLT_MAX || fabs(current_q_A) > FLT_MAX) {
        return false;
    }
    result->current_d_A = (float)current_d_A;
    result->current_q_A = (float)current_q_A;
    result->is_valid = true;
    return true;
}

bool foc_modulate_voltage(float target_voltage_d_V, float target_voltage_q_V,
    float electrical_angle_rad, float bus_voltage_V, float modulation_limit,
    foc_modulation_t *result)
{
    if (result == NULL) {
        return false;
    }
    *result = (foc_modulation_t){0};
    if (!isfinite(target_voltage_d_V) || !isfinite(target_voltage_q_V)
        || !isfinite(electrical_angle_rad) || !isfinite(bus_voltage_V)
        || !isfinite(modulation_limit) || bus_voltage_V <= 0.0f
        || modulation_limit <= 0.0f || modulation_limit > 1.0f) {
        return false;
    }
    double voltage_d_V = target_voltage_d_V;
    double voltage_q_V = target_voltage_q_V;
    double requested_magnitude_V = hypot(voltage_d_V, voltage_q_V);
    double maximum_magnitude_V = (double)modulation_limit * bus_voltage_V * FOC_INVERSE_SQRT_THREE;
    bool is_voltage_limited = requested_magnitude_V > maximum_magnitude_V;
    if (is_voltage_limited) {
        /* 同一比例同时缩放d/q，保留电压矢量方向；正上界保证此分支分母不为0。 */
        double scale = maximum_magnitude_V / requested_magnitude_V;
        voltage_d_V *= scale;
        voltage_q_V *= scale;
    }
    double sine = sin((double)electrical_angle_rad);
    double cosine = cos((double)electrical_angle_rad);
    double voltage_alpha_V = voltage_d_V * cosine - voltage_q_V * sine;
    double voltage_beta_V = voltage_d_V * sine + voltage_q_V * cosine;
    double phase_u_V = voltage_alpha_V;
    double phase_v_V = -0.5 * voltage_alpha_V + FOC_SQRT_THREE_OVER_TWO * voltage_beta_V;
    double phase_w_V = -0.5 * voltage_alpha_V - FOC_SQRT_THREE_OVER_TWO * voltage_beta_V;
    double maximum_phase_V = fmax(phase_u_V, fmax(phase_v_V, phase_w_V));
    double minimum_phase_V = fmin(phase_u_V, fmin(phase_v_V, phase_w_V));
    double common_voltage_V = 0.5 * (maximum_phase_V + minimum_phase_V);
    /* 公共电压不改变相间线电压；联合圆界保证max相-min相<=bus，故占空比合法。
     * 三个结果先写私有临时对象，全部成功后再发布，错误不能遗留部分有效字段。 */
    foc_modulation_t calculated = {0};
    if (!foc_store_duty(0.5 + (phase_u_V - common_voltage_V) / bus_voltage_V, &calculated.duty_u)
        || !foc_store_duty(0.5 + (phase_v_V - common_voltage_V) / bus_voltage_V, &calculated.duty_v)
        || !foc_store_duty(0.5 + (phase_w_V - common_voltage_V) / bus_voltage_V, &calculated.duty_w)) {
        return false;
    }
    calculated.limited_voltage_d_V = (float)voltage_d_V;
    calculated.limited_voltage_q_V = (float)voltage_q_V;
    calculated.is_voltage_limited = is_voltage_limited;
    calculated.is_valid = true;
    *result = calculated;
    return true;
}
