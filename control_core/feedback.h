#ifndef CONTROL_CORE_FEEDBACK_H
#define CONTROL_CORE_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#define FEEDBACK_CAPACITY 32u

/* 机械单位rad、rad/s，时间1us。配置由init复制，运行中不可修改。 */
typedef struct {
    double max_speed_rad_s;
    uint32_t maximum_gap_us;
    uint32_t maximum_read_us;
    uint8_t window_samples;
    int8_t direction;
} feedback_config_t;

typedef struct {
    uint16_t raw_angle_count;
    uint32_t sequence;
    uint32_t read_started_us;
    uint32_t read_completed_us;
    bool is_valid;
} feedback_sample_t;

typedef struct {
    int64_t count;
    uint64_t elapsed_us;
} feedback_point_t;

/* 调用者独占；字段公开仅供固定存储，不允许运行中篡改。 */
typedef struct {
    feedback_config_t config;
    feedback_point_t history[FEEDBACK_CAPACITY];
    int64_t continuous_count;
    uint64_t elapsed_us;
    uint32_t last_sequence;
    uint32_t last_midpoint_us;
    uint32_t last_completed_us;
    uint16_t last_raw;
    uint8_t next_index;
    uint8_t history_count;
    bool initialized;
} feedback_state_t;

typedef struct {
    double raw_mechanical_angle_rad;
    double continuous_angle_rad;
    double direct_speed_rad_s;
    double window_speed_rad_s;
    int64_t continuous_count;
    int64_t window_delta_count;
    uint64_t window_dt_us;
    uint32_t midpoint_us;
    uint32_t dt_us;
    uint8_t window_sample_count;
    bool is_valid;
    bool is_first_sample;
    bool direct_is_valid;
    bool window_is_valid;
} feedback_result_t;

/* 无效配置不写state；reset保留配置。对象不可重叠，NULL返回false。 */
bool feedback_init(feedback_state_t *state, feedback_config_t config);
bool feedback_reset(feedback_state_t *state);
/* 失败清零result且不提交state任何字节；失败后由调用方故障/显式reset。 */
bool feedback_step(feedback_state_t *state, feedback_sample_t sample,
                   feedback_result_t *result);

#endif
