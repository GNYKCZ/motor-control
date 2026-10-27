/* 独立实现机械计数展开与真实时间差分，公共输入/状态/错误契约集中在feedback.h。
 * 由调用者串行提供完整帧，结果用于主机比较或板端反馈；不依赖/控制任何外设。
 * 以下只解释实现依据，状态所有权始终属于调用者。 */
#include "feedback.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#ifdef __FAST_MATH__
#error "feedback requires finite-input checks"
#endif

#define COUNT_RAD (6.283185307179586476925286766559 / 16384.0) /* 14 bit编码器一计数对应的机械rad。 */
#define HALF_TIME UINT32_C(2147483648) /* uint32时钟半周期us，有界模差的方向可辨范围边界。 */

/* 检查值配置的有限性/范围；不修改调用者，保证后续取模索引及时间比较有合法参数。 */
static bool config_valid(feedback_config_t c)
{
    return isfinite(c.max_speed_rad_s) && c.max_speed_rad_s > 0.0 &&
           c.maximum_gap_us > 0u && c.maximum_gap_us < HALF_TIME &&
           c.maximum_read_us < HALF_TIME && c.window_samples >= 2u &&
           c.window_samples <= FEEDBACK_CAPACITY &&
           (c.direction == 1 || c.direction == -1);
}

/* 先验证再清空，避免无效配置破坏旧轨迹；成功时复制配置并等待首帧。 */
bool feedback_init(feedback_state_t *state, feedback_config_t config)
{
    return feedback_init_spaced(state, config, 1u);
}

bool feedback_init_spaced(feedback_state_t *state, feedback_config_t config,
                          uint32_t sequence_step)
{
    if (state == NULL || !config_valid(config) || sequence_step == 0u || sequence_step >= HALF_TIME) {
        return false;
    }
    memset(state, 0, sizeof(*state));
    state->config = config;
    state->sequence_step = sequence_step;
    return true;
}

/* 配置按值传递后复用init，所以清空状态不会丢失待保留的配置。 */
bool feedback_reset(feedback_state_t *state)
{
    return state != NULL && feedback_init_spaced(state, state->config, state->sequence_step);
}

/* 单帧顺序：清空输出 -> 检查时标/模差 -> 副本累计 -> 更新窗口 -> 一次提交。
 * 失败分支均在提交前返回，已生成的局部速度不会泄漏为有效输出。 */
bool feedback_step(feedback_state_t *state, feedback_sample_t sample,
                   feedback_result_t *result)
{
    /* 只暂存本拍会改变的标量；32点历史在全部检查通过后才写一个槽。
     * 仍保持失败事务性，避免在无FPU MCU前台也复制整个历史到栈上。 */
    struct {
        int64_t continuous_count;
        uint64_t elapsed_us;
        uint8_t next_index;
        uint8_t history_count;
        bool initialized;
    } next;
    feedback_result_t out;
    uint32_t duration, midpoint;
    if (result == NULL) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    memset(&out, 0, sizeof(out));
    if (state == NULL || !config_valid(state->config) || state->sequence_step == 0u ||
        state->sequence_step >= HALF_TIME || !sample.is_valid ||
        sample.raw_angle_count >= 16384u) {
        return false;
    }
    /* 无符号相减允许一次时钟回绕；maximum_read_us<半周期，反向短读会被拒绝。
     * 软件读取区间中点只是观测近似，奇数us向下取整，不声称传感器精确采样沿。 */
    duration = sample.read_completed_us - sample.read_started_us;
    if (duration > state->config.maximum_read_us) {
        return false;
    }
    midpoint = sample.read_started_us + duration / 2u;
    /* 暂存变化标量；所有检查通过后一次提交，失败不能污染历史。 */
    next.continuous_count = state->continuous_count;
    next.elapsed_us = state->elapsed_us;
    next.next_index = state->next_index;
    next.history_count = state->history_count;
    next.initialized = state->initialized;
    if (state->initialized) {
        uint32_t dt = midpoint - state->last_midpoint_us;
        uint32_t separation = sample.read_started_us - state->last_completed_us;
        int32_t delta = (int32_t)sample.raw_angle_count - (int32_t)state->last_raw;
        double bound;
        if (sample.sequence - state->last_sequence != state->sequence_step || dt == 0u ||
            dt > state->config.maximum_gap_us || separation >= HALF_TIME ||
            delta == 8192 || delta == -8192) {
            return false;
        }
        /* 已排除恰好半圈，用加减把raw差映射到唯一开区间，避免带符号移位。 */
        if (delta > 8192) {
            delta -= 16384;
        } else if (delta < -8192) {
            delta += 16384;
        }
        /* +1是量化包络，只作判据，不能裁剪估计值。 */
        bound = state->config.max_speed_rad_s * ((double)dt / 1000000.0) / COUNT_RAD + 1.0;
        if (!isfinite(bound) || bound >= 8192.0 || fabs((double)delta) > bound) {
            return false;
        }
        delta *= state->config.direction;
        if ((delta > 0 && state->continuous_count > INT64_MAX - delta) ||
            (delta < 0 && state->continuous_count < INT64_MIN - delta) ||
            state->elapsed_us > UINT64_MAX - dt) {
            return false;
        }
        next.continuous_count += delta;
        next.elapsed_us += dt;
        out.dt_us = dt;
        out.direct_speed_rad_s = (double)delta * COUNT_RAD * 1000000.0 / dt;
        out.direct_is_valid = true;
    } else {
        out.is_first_sample = true;
        next.initialized = true;
    }
    /* 预计算写入后的索引：预热时历史从0填充；满窗后next_index指向最旧保留点。
     * 窗口速度=(当前累计计数-最旧累计计数)×COUNT_RAD/跨窗实际秒数，
     * elapsed_us用uint64累加，避免跨窗间隔超过uint32周期时误取模。 */
    next.next_index = (uint8_t)((next.next_index + 1u) % state->config.window_samples);
    if (next.history_count < state->config.window_samples) {
        next.history_count++;
    }
    if (next.history_count == state->config.window_samples) {
        /* window>=2，next_index不会指向即将写入的槽；无需先修改历史。 */
        feedback_point_t oldest = state->history[next.next_index];
        int64_t current = next.continuous_count;
        /* 跨窗口减法也显式检查，避免长寿命连续计数边界的有符号溢出。 */
        if ((oldest.count < 0 && current > INT64_MAX + oldest.count) ||
            (oldest.count > 0 && current < INT64_MIN + oldest.count) ||
            next.elapsed_us <= oldest.elapsed_us) {
            return false;
        }
        out.window_delta_count = current - oldest.count;
        out.window_dt_us = next.elapsed_us - oldest.elapsed_us;
        out.window_speed_rad_s = (double)out.window_delta_count * COUNT_RAD *
                                1000000.0 / (double)out.window_dt_us;
        out.window_is_valid = true;
    }
    out.raw_mechanical_angle_rad = sample.raw_angle_count * COUNT_RAD;
    out.continuous_count = next.continuous_count;
    out.continuous_angle_rad = (double)next.continuous_count * COUNT_RAD;
    out.midpoint_us = midpoint;
    out.window_sample_count = next.history_count;
    out.is_valid = true;
    state->history[state->next_index].count = next.continuous_count;
    state->history[state->next_index].elapsed_us = next.elapsed_us;
    state->continuous_count = next.continuous_count;
    state->elapsed_us = next.elapsed_us;
    state->next_index = next.next_index;
    state->history_count = next.history_count;
    state->initialized = next.initialized;
    state->last_raw = sample.raw_angle_count;
    state->last_sequence = sample.sequence;
    state->last_midpoint_us = midpoint;
    state->last_completed_us = sample.read_completed_us;
    *result = out;
    return true;
}
