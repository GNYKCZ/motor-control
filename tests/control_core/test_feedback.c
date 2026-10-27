/* 主机独立验收：从整数真值构造帧，由本进程串行调用feedback；不连接固件/硬件。
 * 测试自身持有全部夹具与计数，输出断言及合成CSV供runner保存，禁止用DUT生成期望。 */
#include "feedback.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 独立高精度单位；期望从预生成展开整数真值及时间计算，不调用DUT回绕逻辑。 */
#define RAD (6.2831853071795864769252867665590057683943388L / 16384.0L)
static unsigned checks; /* 本测试进程累计执行的断言数，不属于被测模块状态。 */
/* 检查失败即打印位置并非零退出；不使用可被NDEBUG删除的assert。 */
#define CHECK(c) do { checks++; if (!(c)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

/* 按实施前固定的2e-12相对/绝对容差比较独立高精度期望，拒绝非有限结果。 */
static void near_value(double actual, long double expected)
{
    CHECK(isfinite(actual));
    CHECK(fabsl((long double)actual - expected) <= 2e-12L * fmaxl(1.0L, fabsl(expected)));
}

/* 构造测试配置：窗口n点、方向±1；固定速度/读取/间隔上限不是板端推荐参数。 */
static feedback_config_t config(unsigned n, int direction)
{
    feedback_config_t c = {20.0, 100000u, 40u, (uint8_t)n, (int8_t)direction};
    return c;
}

/* 从展开整数真值truth生成raw；seq为完成序号，midpoint单位us。
 * 固定读取区间21us，使奇数时长向下取整后正好回到给定中点。 */
static feedback_sample_t sample(int64_t truth, uint32_t seq, uint32_t midpoint)
{
    int64_t raw = truth % 16384;
    feedback_sample_t s;
    if (raw < 0) {
        raw += 16384;
    }
    memset(&s, 0, sizeof(s));
    s.raw_angle_count = (uint16_t)raw;
    s.sequence = seq;
    s.read_started_us = midpoint - 10u;
    s.read_completed_us = midpoint + 11u; /* 奇数21us，中点向下取整。 */
    s.is_valid = true;
    return s;
}

/* 验证指定帧失败、整个状态对象字节不变且结果全零；不会替换真实step路径。 */
static void rejected(feedback_state_t *state, feedback_sample_t s)
{
    unsigned char before[sizeof(*state)];
    feedback_result_t result, zero;
    memcpy(before, state, sizeof(before));
    memset(&result, 0xa5, sizeof(result));
    memset(&zero, 0, sizeof(zero));
    CHECK(!feedback_step(state, s, &result));
    CHECK(memcmp(before, state, sizeof(before)) == 0);
    CHECK(memcmp(&result, &zero, sizeof(zero)) == 0);
}

/* 多圈、双向、非均匀时间、uint32时钟/序号跨界；整数oracle独立展开。 */
static void trajectories(void)
{
    unsigned n, d, kind;
    for (n = 2; n <= 32; n += (n == 2 ? 7 : 23)) {
        for (d = 0; d < 2; d++) {
            for (kind = 0; kind < 4; kind++) {
                feedback_state_t state;
                feedback_result_t out;
                int direction = d ? -1 : 1;
                int64_t counts[400];
                uint64_t times[400];
                unsigned i;
                CHECK(feedback_init(&state, config(n, direction)));
                for (i = 0; i < 400; i++) {
                    int64_t increment = kind == 0 ? 79 : kind == 1 ? -83 :
                                        kind == 2 ? 0 : (i % 8u == 0u ? 1 : 0);
                    feedback_sample_t s;
                    counts[i] = i == 0 ? 16383 : counts[i - 1] + increment;
                    times[i] = i == 0 ? 0u : times[i - 1] + 7000u + (i * 53u) % 3000u;
                    s = sample(counts[i], UINT32_MAX - 4u + i,
                               UINT32_MAX - 22000u + (uint32_t)times[i]);
                    CHECK(feedback_step(&state, s, &out));
                    CHECK(out.is_valid && out.is_first_sample == (i == 0));
                    CHECK(out.direct_is_valid == (i != 0));
                    CHECK(out.window_is_valid == (i + 1 >= n));
                    CHECK(out.midpoint_us == UINT32_MAX - 22000u + (uint32_t)times[i]);
                    CHECK(out.window_sample_count == (i + 1 < n ? i + 1 : n));
                    CHECK(out.continuous_count == direction * (counts[i] - counts[0]));
                    near_value(out.raw_mechanical_angle_rad, s.raw_angle_count * RAD);
                    near_value(out.continuous_angle_rad, direction * (counts[i] - counts[0]) * RAD);
                    if (i != 0) {
                        CHECK(out.dt_us == times[i] - times[i - 1]);
                        near_value(out.direct_speed_rad_s,
                                   direction * (counts[i] - counts[i - 1]) * RAD *
                                   1000000.0L / (times[i] - times[i - 1]));
                    } else {
                        CHECK(out.dt_us == 0 && out.direct_speed_rad_s == 0.0);
                    }
                    if (i + 1 >= n) {
                        unsigned first = i + 1 - n;
                        CHECK(out.window_delta_count == direction * (counts[i] - counts[first]));
                        CHECK(out.window_dt_us == times[i] - times[first]);
                        near_value(out.window_speed_rad_s,
                                   direction * (counts[i] - counts[first]) * RAD *
                                   1000000.0L / (times[i] - times[first]));
                    } else {
                        CHECK(out.window_dt_us == 0 && out.window_delta_count == 0 &&
                              out.window_speed_rad_s == 0.0);
                    }
                }
            }
        }
    }
    puts("PASS independent trajectories: wraps, signs, sparse/static, jitter, windows 2/9/32");
}

/* 覆盖无效配置/帧、时间/序号、速度量化与别名边界及reset/NULL生命周期。 */
static void failures(void)
{
    feedback_state_t state;
    feedback_result_t out, zero;
    feedback_config_t c = config(9, 1), bad;
    feedback_sample_t first = sample(100, 8, 1000), next = sample(101, 9, 11000), s;
    unsigned char before[sizeof(state)];
    unsigned i;
    CHECK(feedback_init(&state, c));
    s = first; s.is_valid = false; rejected(&state, s);
    s = first; s.raw_angle_count = 16384; rejected(&state, s);
    s = first; s.read_completed_us += 20; rejected(&state, s);
    s = first; s.read_completed_us = s.read_started_us - 1u; rejected(&state, s);
    CHECK(feedback_step(&state, first, &out));
    for (i = 0; i < 15; i++) {
        s = next;
        switch (i) {
        case 0: s.sequence = 8; break;
        case 1: s.sequence = 10; break;
        case 2: s = sample(101, 9, 1000); break;
        case 3: s = sample(101, 9, 999); break;
        case 4: s = sample(101, 9, 101001); break;
        case 5: s = sample(101, 9, 1010); break; /* 读取区间重叠 */
        case 6: s.raw_angle_count = 8292; break;
        case 7: s.raw_angle_count = 8292; s.sequence = 9; break;
        case 8: s.raw_angle_count = 900; break; /* B约521，800count过大 */
        case 9: s.is_valid = false; break;
        case 10: s.raw_angle_count = UINT16_MAX; break;
        case 11: s.read_completed_us += 100; break;
        case 12: s.read_completed_us = s.read_started_us - 1u; break;
        case 13: s = sample(101, 9, 1000u + UINT32_C(2147483648)); break;
        default: s.sequence = 7; break;
        }
        rejected(&state, s);
    }
    /* 负方向raw半圈，和接近半圈的物理先验别名保护。 */
    CHECK(feedback_reset(&state));
    CHECK(feedback_step(&state, sample(8292, 8, 1000), &out));
    rejected(&state, sample(100, 9, 11000));
    c.max_speed_rad_s = (double)(8192.0L * RAD * 100.0L);
    CHECK(feedback_init(&state, c));
    CHECK(feedback_step(&state, first, &out));
    rejected(&state, sample(100, 9, 11000));
    /* 量化1count允许但报告值可大于max_speed；2count不允许。 */
    c.max_speed_rad_s = (double)(0.25L * RAD * 100.0L);
    CHECK(feedback_init(&state, c));
    CHECK(feedback_step(&state, first, &out));
    rejected(&state, sample(102, 9, 11000));
    CHECK(feedback_step(&state, next, &out));
    CHECK(out.direct_speed_rad_s > c.max_speed_rad_s);
    /* B+1阈值两侧和8192别名边界两侧，间隔1e-6 count固定。 */
    for (i = 0; i < 2; i++) {
        c.max_speed_rad_s = (double)((1.0L + (i ? 1e-6L : -1e-6L)) * RAD * 100.0L);
        CHECK(feedback_init(&state, c)); CHECK(feedback_step(&state, first, &out));
        if (i) { CHECK(feedback_step(&state, sample(102, 9, 11000), &out)); }
        else { rejected(&state, sample(102, 9, 11000)); }
        c.max_speed_rad_s = (double)((8191.0L + (i ? 1e-6L : -1e-6L)) * RAD * 100.0L);
        CHECK(feedback_init(&state, c)); CHECK(feedback_step(&state, first, &out));
        if (i) { rejected(&state, sample(100, 9, 11000)); }
        else { CHECK(feedback_step(&state, sample(100, 9, 11000), &out)); }
    }
    /* 最大gap、read、相接读取区间均包含端点；零duration允许。 */
    c = config(2, 1); c.maximum_gap_us = 21;
    CHECK(feedback_init(&state, c)); CHECK(feedback_step(&state, first, &out));
    CHECK(feedback_step(&state, sample(100, 9, 1021), &out));
    s = sample(100, 10, 1042); s.read_completed_us = s.read_started_us;
    CHECK(feedback_step(&state, s, &out));
    c = config(9, 1);
    CHECK(feedback_init(&state, c));
    s = sample(100, 1, 100); s.read_started_us = 80; s.read_completed_us = 120;
    CHECK(feedback_step(&state, s, &out));
    CHECK(feedback_reset(&state));
    CHECK(!state.initialized && state.history_count == 0);
    CHECK(feedback_step(&state, next, &out)); CHECK(out.is_first_sample && !out.direct_is_valid);
    memcpy(before, &state, sizeof(state));
    for (i = 0; i < 11; i++) {
        bad = c;
        switch (i) {
        case 0: bad.max_speed_rad_s = NAN; break;
        case 1: bad.max_speed_rad_s = INFINITY; break;
        case 2: bad.max_speed_rad_s = 0; break;
        case 3: bad.max_speed_rad_s = -1; break;
        case 4: bad.maximum_gap_us = 0; break;
        case 5: bad.maximum_gap_us = UINT32_C(2147483648); break;
        case 6: bad.maximum_read_us = UINT32_C(2147483648); break;
        case 7: bad.window_samples = 1; break;
        case 8: bad.window_samples = 33; break;
        case 9: bad.direction = 0; break;
        default: bad.direction = 2; break;
        }
        CHECK(!feedback_init(&state, bad)); CHECK(memcmp(before, &state, sizeof(state)) == 0);
    }
    CHECK(!feedback_init(NULL, c)); CHECK(!feedback_reset(NULL));
    CHECK(!feedback_step(&state, next, NULL)); CHECK(memcmp(before, &state, sizeof(state)) == 0);
    memset(&zero, 0, sizeof(zero)); memset(&out, 0xa5, sizeof(out));
    CHECK(!feedback_step(NULL, next, &out)); CHECK(memcmp(&out, &zero, sizeof(out)) == 0);
    puts("PASS failure transactions, configuration, time/sequence bounds, quantization and alias gates");
}

/* 通过平移已初始化的历史到整数边界，保留相对关系，触达不可实际跑满的寿命边界。 */
static void overflow(void)
{
    feedback_state_t state;
    feedback_result_t out;
    int sign;
    for (sign = -1; sign <= 1; sign += 2) {
        CHECK(feedback_init(&state, config(2, 1)));
        CHECK(feedback_step(&state, sample(100, 1, 1000), &out));
        state.continuous_count = sign > 0 ? INT64_MAX : INT64_MIN;
        state.history[0].count = state.continuous_count;
        rejected(&state, sample(100 + sign, 2, 11000));
        /* 向边界内部运动可成功，检查减法及转换不溢出。 */
        CHECK(feedback_step(&state, sample(100 - sign, 2, 11000), &out));
        CHECK(out.window_delta_count == -sign);
    }
    CHECK(feedback_init(&state, config(2, 1)));
    CHECK(feedback_step(&state, sample(100, 1, 1000), &out));
    state.elapsed_us = UINT64_MAX - 9999u;
    state.history[0].elapsed_us = state.elapsed_us;
    rejected(&state, sample(100, 2, 11000));
    puts("PASS signed count and unsigned elapsed overflow; transactional state unchanged");
}

/* 以静止轨迹验证跨窗累计时间>2^32，并验证0/16383相邻计数正反跨圈。 */
static void long_time_and_exact_wraps(void)
{
    feedback_state_t state;
    feedback_result_t out;
    feedback_config_t c = config(32, 1);
    unsigned i;
    c.maximum_gap_us = 200000000u;
    c.max_speed_rad_s = 1e-9;
    CHECK(feedback_init(&state, c));
    for (i = 0; i < 40; i++) {
        uint64_t elapsed = (uint64_t)i * 200000000u;
        CHECK(feedback_step(&state, sample(10, i, (uint32_t)elapsed), &out));
        if (i >= 31) {
            CHECK(out.window_dt_us == UINT64_C(6200000000));
            CHECK(out.window_speed_rad_s == 0.0);
        }
    }
    CHECK(state.elapsed_us == UINT64_C(7800000000));
    for (i = 0; i < 2; i++) {
        CHECK(feedback_init(&state, config(2, 1)));
        CHECK(feedback_step(&state, sample(i ? 0 : 16383, 0, 1000), &out));
        CHECK(feedback_step(&state, sample(i ? -1 : 16384, 1, 11000), &out));
        CHECK(out.continuous_count == (i ? -1 : 1));
    }
    puts("PASS exact adjacent raw wraps and window elapsed beyond uint32 period");
}

/* 两套独立配置先各自运行再交错运行，比对值输出，查找意外共享可变状态。 */
static void isolation(void)
{
    feedback_state_t isolated[2], interleaved[2];
    feedback_result_t expected[2][64], actual;
    unsigned i, j;
    for (j = 0; j < 2; j++) {
        CHECK(feedback_init(&isolated[j], config(j ? 32 : 2, j ? -1 : 1)));
        CHECK(feedback_init(&interleaved[j], config(j ? 32 : 2, j ? -1 : 1)));
        for (i = 0; i < 64; i++) {
            CHECK(feedback_step(&isolated[j], sample(16000 + (int64_t)i * (j ? -17 : 19),
                                i, 1000 + i * 10000), &expected[j][i]));
        }
    }
    for (i = 0; i < 64; i++) {
        for (j = 0; j < 2; j++) {
            CHECK(feedback_step(&interleaved[j], sample(16000 + (int64_t)i * (j ? -17 : 19),
                                i, 1000 + i * 10000), &actual));
            CHECK(memcmp(&actual, &expected[j][i], sizeof(actual)) == 0);
        }
    }
    puts("PASS two independently configured instances interleaved");
}

/* 同一预定轨迹同时评价两种速度；独立真值生成稀疏/周期噪声/阶跃工况，
 * 保留所有样本输出，固定120点统计区间，报告RMS与完全响应拍号。 */
static void comparison(void)
{
    static const int noise[8] = {0, 1, 0, -1, 0, 1, 0, -1}; /* 预定8拍周期计数误差，仅合成对照。 */
    unsigned trajectory, i;
    for (trajectory = 0; trajectory < 3; trajectory++) {
        feedback_state_t state;
        feedback_result_t out;
        int64_t counts[129];
        long double direct_square = 0, window_square = 0;
        unsigned measured = 0, direct_plateau = 0, window_plateau = 0;
        CHECK(feedback_init(&state, config(9, 1)));
        for (i = 0; i < 129; i++) {
            long double truth_speed;
            if (trajectory == 0) {
                counts[i] = 16000 + i / 8;
                truth_speed = RAD * 12.5L;
            } else if (trajectory == 1) {
                counts[i] = 16000 + noise[i % 8];
                truth_speed = 0;
            } else {
                counts[i] = 16000 + (i < 64 ? i : 63 + 2 * (i - 63));
                truth_speed = RAD * (i < 64 ? 100.0L : 200.0L);
            }
            CHECK(feedback_step(&state, sample(counts[i], i, 1000 + i * 10000), &out));
            if (i) {
                near_value(out.direct_speed_rad_s, (counts[i] - counts[i - 1]) * RAD * 100.0L);
            }
            if (i >= 8) {
                long double de, we;
                near_value(out.window_speed_rad_s, (counts[i] - counts[i - 8]) * RAD * 12.5L);
                de = out.direct_speed_rad_s - truth_speed;
                we = out.window_speed_rad_s - truth_speed;
                /* 8..127正好120点=15个周期，末点仅用于CSV。 */
                if (i < 128) { direct_square += de * de; window_square += we * we; measured++; }
            }
            if (trajectory == 2 && i >= 64) {
                if (!direct_plateau && fabsl(out.direct_speed_rad_s - truth_speed) < 1e-12L) { direct_plateau = i; }
                if (!window_plateau && fabsl(out.window_speed_rad_s - truth_speed) < 1e-12L) { window_plateau = i; }
            }
            printf("SYNTHETIC_CSV,%u,%u,%ld,%u,%.17g,%.17g,%.17g,%u,%u\n",
                   trajectory, i, (long)counts[i], 1000 + i * 10000,
                   out.direct_speed_rad_s, out.window_speed_rad_s, (double)truth_speed,
                   (unsigned)out.direct_is_valid, (unsigned)out.window_is_valid);
        }
        if (trajectory == 0) {
            near_value((double)sqrtl(direct_square / measured), sqrtl(0.125L * 0.875L) * RAD * 100.0L);
            near_value((double)sqrtl(window_square / measured), 0);
        } else if (trajectory == 1) {
            near_value((double)sqrtl(direct_square / measured), RAD * 100.0L);
            near_value((double)sqrtl(window_square / measured), 0);
        } else {
            CHECK(direct_plateau == 64 && window_plateau == 71);
        }
        printf("METRIC,%u,%.17g,%.17g,%u,%u,%u\n", trajectory,
               (double)sqrtl(direct_square / measured), (double)sqrtl(window_square / measured),
               measured, direct_plateau, window_plateau);
    }
    puts("PASS synthetic quantization/noise/step comparison against independent oracle");
}

/* 按组执行完整主机验收，全部通过才返回0，打印总断言数供证据核对。 */
int main(void)
{
    trajectories(); failures(); overflow(); long_time_and_exact_wraps(); isolation(); comparison();
    printf("PASS feedback: %u checks; synthetic host only\n", checks);
    return 0;
}
