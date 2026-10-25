/** 按M2c手算公式独立实现；没有复制参考ADC USER CODE或硬件轮询。 */
#include "sensor_input.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#if defined(__FAST_MATH__)
#error "sensor_input requires finite-input checks; do not enable fast-math"
#endif

bool sensor_offset_init(sensor_offset_t *offset)
{
    if (offset == NULL) { return false; }
    *offset = (sensor_offset_t){0};
    return true;
}

bool sensor_offset_add(sensor_offset_t *offset, uint16_t raw_u, uint16_t raw_v, bool frame_valid)
{
    if (offset == NULL || !frame_valid || raw_u > 4095u || raw_v > 4095u
        || offset->accepted_count >= SENSOR_OFFSET_SAMPLE_COUNT
        || offset->sum_u_count > offset->accepted_count * 4095u
        || offset->sum_v_count > offset->accepted_count * 4095u) { return false; }
    sensor_offset_t next = *offset;
    if (next.accepted_count == 0u) {
        next.minimum_u_count = raw_u; next.maximum_u_count = raw_u;
        next.minimum_v_count = raw_v; next.maximum_v_count = raw_v;
    } else {
        if (next.minimum_u_count > next.maximum_u_count || next.maximum_u_count > 4095u
            || next.minimum_v_count > next.maximum_v_count || next.maximum_v_count > 4095u
            || next.sum_u_count < (uint32_t)next.minimum_u_count * next.accepted_count
            || next.sum_u_count > (uint32_t)next.maximum_u_count * next.accepted_count
            || next.sum_v_count < (uint32_t)next.minimum_v_count * next.accepted_count
            || next.sum_v_count > (uint32_t)next.maximum_v_count * next.accepted_count) { return false; }
        if (raw_u < next.minimum_u_count) { next.minimum_u_count = raw_u; }
        if (raw_u > next.maximum_u_count) { next.maximum_u_count = raw_u; }
        if (raw_v < next.minimum_v_count) { next.minimum_v_count = raw_v; }
        if (raw_v > next.maximum_v_count) { next.maximum_v_count = raw_v; }
    }
    next.accepted_count++; next.sum_u_count += raw_u; next.sum_v_count += raw_v;
    *offset = next;
    return true;
}

bool sensor_input_convert(uint16_t raw_u, uint16_t raw_v, uint16_t raw_angle,
    bool frame_valid, sensor_input_config_t config, sensor_input_t *result)
{
    if (result == NULL) { return false; }
    *result = (sensor_input_t){0};
    if (!frame_valid || raw_u > 4095u || raw_v > 4095u || raw_angle > 16383u
        || !isfinite(config.estimated_offset_u_count) || config.estimated_offset_u_count < 0.0f
        || config.estimated_offset_u_count > 4095.0f || !isfinite(config.estimated_offset_v_count)
        || config.estimated_offset_v_count < 0.0f || config.estimated_offset_v_count > 4095.0f
        || !isfinite(config.nominal_A_per_count) || config.nominal_A_per_count <= 0.0f) { return false; }
    double u = ((double)raw_u - config.estimated_offset_u_count) * config.nominal_A_per_count;
    double v = ((double)raw_v - config.estimated_offset_v_count) * config.nominal_A_per_count;
    if (!isfinite(u) || !isfinite(v) || fabs(u) > FLT_MAX || fabs(v) > FLT_MAX) { return false; }
    sensor_input_t calculated = {(float)u, (float)v,
        (float)((double)raw_angle * (6.283185307179586476925286766559 / 16384.0)), true};
    *result = calculated;
    return true;
}
