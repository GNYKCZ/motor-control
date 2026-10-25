/** M2a 独立主机验收。直接调用真实纯函数，不mock算法、不链接板级。
 * oracle以三相投影、极坐标和线电压重建判断，不调用待测函数来产生expected。
 * 所有输入为合成数据；失败打印具体位置并非零退出，不触发Windows断言弹窗。 */
#include "foc_math.h"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_FOC_PI 3.14159265358979323846264338327950288 /**< 测试独立圆周率，rad。 */
#define TEST_FOC_CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(EXIT_FAILURE); \
} } while (0)

static uint32_t current_case_count; /**< 当前运行的电流向量数量，不属于控制模块状态。 */
static uint32_t voltage_case_count; /**< 当前运行的调制向量数量。 */

/** 检查差值有限且未超预先固定的绝对误差，不在失败后修改容差。 */
static void test_check_near(double actual, double expected, double tolerance)
{
    if (!isfinite(actual) || !isfinite(expected) || fabs(actual - expected) > tolerance) {
        fprintf(stderr, "FAIL numeric: actual=%.17g expected=%.17g tolerance=%.17g\n",
            actual, expected, tolerance);
        exit(EXIT_FAILURE);
    }
}

/** 独立三相投影：重建W，再将U/V/W投到三个相差120度的旋转轴。
 * 与待测的两相Clarke后Park计算图不同；只共享可信的标准数学库。 */
static void test_current_oracle(float phase_u_A, float phase_v_A, float angle_rad)
{
    double phase_w_A = -(double)phase_u_A - phase_v_A;
    double cosine = cos((double)angle_rad);
    double sine = sin((double)angle_rad);
    double half_sqrt_three = sqrt(3.0) / 2.0;
    /* 使用角差恒等式，避免极大角度再减120度时在double里丢失该偏移。 */
    double cosine_v = -0.5 * cosine + half_sqrt_three * sine;
    double cosine_w = -0.5 * cosine - half_sqrt_three * sine;
    double sine_v = -0.5 * sine - half_sqrt_three * cosine;
    double sine_w = -0.5 * sine + half_sqrt_three * cosine;
    double expected_d_A = (2.0 / 3.0) * (phase_u_A * cosine + phase_v_A * cosine_v + phase_w_A * cosine_w);
    double expected_q_A = -(2.0 / 3.0) * (phase_u_A * sine + phase_v_A * sine_v + phase_w_A * sine_w);
    double scale_A = fmax(1.0, fabs(phase_u_A) + fabs(phase_v_A) + fabs(phase_w_A));
    double tolerance_A = 8.0 * FLT_EPSILON * scale_A;
    foc_current_t result;
    TEST_FOC_CHECK(foc_transform_current(phase_u_A, phase_v_A, angle_rad, &result));
    TEST_FOC_CHECK(result.is_valid);
    test_check_near(result.current_d_A, expected_d_A, tolerance_A);
    test_check_near(result.current_q_A, expected_q_A, tolerance_A);
    double phase_square_A2 = (double)phase_u_A * phase_u_A + (double)phase_v_A * phase_v_A + phase_w_A * phase_w_A;
    double result_square_A2 = (double)result.current_d_A * result.current_d_A + (double)result.current_q_A * result.current_q_A;
    test_check_near(result_square_A2, (2.0 / 3.0) * phase_square_A2,
        32.0 * FLT_EPSILON * fmax(1.0, phase_square_A2));
    current_case_count++;
}

/** 独立极坐标oracle：请求d/q→幅值/方向→三相电压；仅从公开占空比重建线电压。
 * 不复用待测中点注入步骤来生成expected，另验max/min中心化不变量和受限d/q方向。 */
static void test_voltage_oracle(float target_d_V, float target_q_V, float angle_rad,
    float bus_voltage_V, float modulation_limit)
{
    double requested_radius_V = sqrt((double)target_d_V * target_d_V + (double)target_q_V * target_q_V);
    double maximum_radius_V = (double)modulation_limit * bus_voltage_V / sqrt(3.0);
    double expected_radius_V = fmin(requested_radius_V, maximum_radius_V);
    double requested_direction = atan2((double)target_q_V, (double)target_d_V);
    double orientation = atan2(sin((double)angle_rad), cos((double)angle_rad));
    double expected_phase_u_V = expected_radius_V * cos(orientation + requested_direction);
    double expected_phase_v_V = expected_radius_V * cos(orientation + requested_direction - 2.0 * TEST_FOC_PI / 3.0);
    double expected_phase_w_V = expected_radius_V * cos(orientation + requested_direction + 2.0 * TEST_FOC_PI / 3.0);
    double voltage_tolerance_V = 16.0 * FLT_EPSILON * bus_voltage_V;
    double field_tolerance_V = voltage_tolerance_V + nextafterf(0.0f, 1.0f);
    foc_modulation_t result;
    TEST_FOC_CHECK(foc_modulate_voltage(target_d_V, target_q_V, angle_rad, bus_voltage_V, modulation_limit, &result));
    TEST_FOC_CHECK(result.is_valid);
    TEST_FOC_CHECK(isfinite(result.duty_u) && result.duty_u >= 0.0f && result.duty_u <= 1.0f);
    TEST_FOC_CHECK(isfinite(result.duty_v) && result.duty_v >= 0.0f && result.duty_v <= 1.0f);
    TEST_FOC_CHECK(isfinite(result.duty_w) && result.duty_w >= 0.0f && result.duty_w <= 1.0f);
    double largest_duty = fmax(result.duty_u, fmax(result.duty_v, result.duty_w));
    double smallest_duty = fmin(result.duty_u, fmin(result.duty_v, result.duty_w));
    test_check_near(largest_duty + smallest_duty, 1.0, 4.0 * FLT_EPSILON);
    test_check_near((double)bus_voltage_V * ((double)result.duty_u - result.duty_v),
        expected_phase_u_V - expected_phase_v_V, voltage_tolerance_V);
    test_check_near((double)bus_voltage_V * ((double)result.duty_v - result.duty_w),
        expected_phase_v_V - expected_phase_w_V, voltage_tolerance_V);
    test_check_near(result.limited_voltage_d_V, expected_radius_V * cos(requested_direction), field_tolerance_V);
    test_check_near(result.limited_voltage_q_V, expected_radius_V * sin(requested_direction), field_tolerance_V);
    double published_radius_V = hypot((double)result.limited_voltage_d_V, (double)result.limited_voltage_q_V);
    TEST_FOC_CHECK(published_radius_V <= maximum_radius_V + 2.0 * field_tolerance_V);
    if (fabs(requested_radius_V - maximum_radius_V)
        > 64.0 * DBL_EPSILON * fmax(requested_radius_V, maximum_radius_V)) {
        TEST_FOC_CHECK(result.is_voltage_limited == (requested_radius_V > maximum_radius_V));
    }
    /* 从三路已量化占空比独立去公共量，再还原alpha/beta及d/q，与公开受限值核对。 */
    double mean_duty = ((double)result.duty_u + result.duty_v + result.duty_w) / 3.0;
    double reconstructed_alpha_V = bus_voltage_V * ((double)result.duty_u - mean_duty);
    double reconstructed_beta_V = bus_voltage_V * ((double)result.duty_v - result.duty_w) / sqrt(3.0);
    double reconstructed_d_V = reconstructed_alpha_V * cos((double)angle_rad) + reconstructed_beta_V * sin((double)angle_rad);
    double reconstructed_q_V = -reconstructed_alpha_V * sin((double)angle_rad) + reconstructed_beta_V * cos((double)angle_rad);
    test_check_near(reconstructed_d_V, result.limited_voltage_d_V, field_tolerance_V);
    test_check_near(reconstructed_q_V, result.limited_voltage_q_V, field_tolerance_V);
    voltage_case_count++;
}

/** 手算轴向量与零矢量，固定q轴符号、幅值与已知参考归一化电压的对应。 */
static void test_known_vectors(void)
{
    foc_current_t current;
    TEST_FOC_CHECK(foc_transform_current(1.0f, -0.5f, 0.0f, &current));
    test_check_near(current.current_d_A, 1.0, 4.0 * FLT_EPSILON);
    test_check_near(current.current_q_A, 0.0, 4.0 * FLT_EPSILON);
    TEST_FOC_CHECK(foc_transform_current(1.0f, -0.5f, (float)(TEST_FOC_PI / 2.0), &current));
    test_check_near(current.current_d_A, 0.0, 4.0 * FLT_EPSILON);
    test_check_near(current.current_q_A, -1.0, 4.0 * FLT_EPSILON);
    foc_modulation_t voltage;
    TEST_FOC_CHECK(foc_modulate_voltage(0.0f, 0.0f, 123.0f, 12.0f, 0.9f, &voltage));
    TEST_FOC_CHECK(voltage.duty_u == 0.5f && voltage.duty_v == 0.5f && voltage.duty_w == 0.5f);
    TEST_FOC_CHECK(!voltage.is_voltage_limited);
    TEST_FOC_CHECK(foc_modulate_voltage((float)(12.0 / (2.0 * sqrt(3.0))), 0.0f, 0.0f, 12.0f, 1.0f, &voltage));
    test_check_near(voltage.duty_u, 0.7165063509461097, 4.0 * FLT_EPSILON);
    test_check_near(voltage.duty_v, 0.2834936490538903, 4.0 * FLT_EPSILON);
    test_check_near(voltage.duty_w, 0.2834936490538903, 4.0 * FLT_EPSILON);
    float axis_boundary_V = (float)(12.0 / sqrt(3.0));
    TEST_FOC_CHECK(foc_modulate_voltage(nextafterf(axis_boundary_V, 0.0f), 0.0f, 0.0f, 12.0f, 1.0f, &voltage));
    TEST_FOC_CHECK(!voltage.is_voltage_limited);
    TEST_FOC_CHECK(foc_modulate_voltage(nextafterf(axis_boundary_V, INFINITY), 0.0f, 0.0f, 12.0f, 1.0f, &voltage));
    TEST_FOC_CHECK(voltage.is_voltage_limited);
    puts("PASS known vectors: q sign, zero vector, reference voltage scale, limit threshold");
}

/** 穷举角度网格与多组电流，另用平衡三相正弦检验幅值/相序。 */
static void test_current_grid(void)
{
    const float phase_pairs[][2] = {{0.0f, 0.0f}, {1.0f, -0.5f}, {-1.0f, 0.5f},
        {0.0f, 1.0f}, {0.0f, -1.0f}, {3.0f, -2.0f}, {-8.0f, 3.0f}, {20.0f, 20.0f}, {0.0001f, -0.0002f}};
    for (int degree = -360; degree <= 360; ++degree) {
        float angle = (float)(degree * TEST_FOC_PI / 180.0);
        for (size_t index = 0u; index < sizeof(phase_pairs) / sizeof(phase_pairs[0]); ++index) {
            test_current_oracle(phase_pairs[index][0], phase_pairs[index][1], angle);
        }
        float phase_u = (float)cos((double)angle);
        float phase_v = (float)cos((double)angle - 2.0 * TEST_FOC_PI / 3.0);
        foc_current_t current;
        TEST_FOC_CHECK(foc_transform_current(phase_u, phase_v, angle, &current));
        test_check_near(current.current_d_A, 1.0, 8.0 * FLT_EPSILON);
        test_check_near(current.current_q_A, 0.0, 8.0 * FLT_EPSILON);
        test_current_oracle(phase_v, phase_u, angle); /* 交换U/V，让oracle发现相序接错。 */
    }
    printf("PASS current oracle: %lu three-phase projections and balanced-wave checks\n", (unsigned long)current_case_count);
}

/** 密集电压网格覆盖方向、幅值、母线与显式调制限额，六扇区两侧另行检验连续性。 */
static void test_voltage_grid(void)
{
    const float buses[] = {1.0f, 12.0f, 48.0f, 500.0f};
    const float limits[] = {0.1f, 0.9f, 1.0f};
    const double levels[] = {0.0, 0.1, 0.57, 1.0, 2.0, 10.0};
    for (int degree = -360; degree <= 360; ++degree) {
        float angle = (float)(degree * TEST_FOC_PI / 180.0);
        for (int direction = 0; direction < 8; ++direction) {
            double phi = direction * TEST_FOC_PI / 4.0;
            for (size_t bus_index = 0u; bus_index < sizeof(buses) / sizeof(buses[0]); ++bus_index) {
                for (size_t limit_index = 0u; limit_index < sizeof(limits) / sizeof(limits[0]); ++limit_index) {
                    for (size_t level_index = 0u; level_index < sizeof(levels) / sizeof(levels[0]); ++level_index) {
                        double radius = levels[level_index] * buses[bus_index];
                        test_voltage_oracle((float)(radius * cos(phi)), (float)(radius * sin(phi)),
                            angle, buses[bus_index], limits[limit_index]);
                    }
                }
            }
        }
    }
    for (int sector = 0; sector <= 6; ++sector) {
        float boundary = (float)(sector * TEST_FOC_PI / 3.0);
        float before = nextafterf(boundary, -INFINITY);
        float after = nextafterf(boundary, INFINITY);
        foc_modulation_t first;
        foc_modulation_t second;
        test_voltage_oracle(8.0f, 0.0f, before, 12.0f, 1.0f);
        test_voltage_oracle(8.0f, 0.0f, boundary, 12.0f, 1.0f);
        test_voltage_oracle(8.0f, 0.0f, after, 12.0f, 1.0f);
        TEST_FOC_CHECK(foc_modulate_voltage(8.0f, 0.0f, before, 12.0f, 1.0f, &first));
        TEST_FOC_CHECK(foc_modulate_voltage(8.0f, 0.0f, after, 12.0f, 1.0f, &second));
        double continuity_bound = 2.0 * fabs((double)after - before) + 8.0 * FLT_EPSILON;
        test_check_near(first.duty_u, second.duty_u, continuity_bound);
        test_check_near(first.duty_v, second.duty_v, continuity_bound);
        test_check_near(first.duty_w, second.duty_w, continuity_bound);
    }
    printf("PASS voltage oracle: %lu polar/line-voltage vectors including sector boundaries\n", (unsigned long)voltage_case_count);
}

/** 断言失败输出逐字段为确定的无效值，不用memcmp比较未规定的结构体填充。 */
static void test_invalid_voltage(const foc_modulation_t *result)
{
    TEST_FOC_CHECK(!result->is_valid && !result->is_voltage_limited);
    TEST_FOC_CHECK(result->duty_u == 0.0f && result->duty_v == 0.0f && result->duty_w == 0.0f);
    TEST_FOC_CHECK(result->limited_voltage_d_V == 0.0f && result->limited_voltage_q_V == 0.0f);
}

/** 注入每个位置的NaN/Inf与非法范围；成功调用后再失败，检查没有残留有效结果。 */
static void test_error_contracts(void)
{
    const float invalid_values[] = {NAN, INFINITY, -INFINITY};
    for (size_t invalid = 0u; invalid < sizeof(invalid_values) / sizeof(invalid_values[0]); ++invalid) {
        for (unsigned int field = 0u; field < 5u; ++field) {
            float inputs[5] = {1.0f, 2.0f, 0.1f, 12.0f, 0.9f};
            inputs[field] = invalid_values[invalid];
            foc_modulation_t result;
            TEST_FOC_CHECK(foc_modulate_voltage(30.0f, 20.0f, 0.1f, 12.0f, 0.9f, &result));
            TEST_FOC_CHECK(result.is_voltage_limited);
            TEST_FOC_CHECK(!foc_modulate_voltage(inputs[0], inputs[1], inputs[2], inputs[3], inputs[4], &result));
            test_invalid_voltage(&result);
        }
        for (unsigned int field = 0u; field < 3u; ++field) {
            float inputs[3] = {1.0f, 2.0f, 0.1f};
            inputs[field] = invalid_values[invalid];
            foc_current_t result;
            TEST_FOC_CHECK(foc_transform_current(1.0f, 2.0f, 0.1f, &result));
            TEST_FOC_CHECK(!foc_transform_current(inputs[0], inputs[1], inputs[2], &result));
            TEST_FOC_CHECK(!result.is_valid && result.current_d_A == 0.0f && result.current_q_A == 0.0f);
        }
    }
    const float invalid_buses[] = {0.0f, -0.0f, -1.0f};
    for (size_t index = 0u; index < sizeof(invalid_buses) / sizeof(invalid_buses[0]); ++index) {
        foc_modulation_t result;
        TEST_FOC_CHECK(!foc_modulate_voltage(0.0f, 0.0f, 0.0f, invalid_buses[index], 1.0f, &result));
        test_invalid_voltage(&result);
    }
    const float invalid_limits[] = {0.0f, -0.0f, -0.1f, nextafterf(1.0f, INFINITY), 2.0f};
    for (size_t index = 0u; index < sizeof(invalid_limits) / sizeof(invalid_limits[0]); ++index) {
        foc_modulation_t result;
        TEST_FOC_CHECK(!foc_modulate_voltage(1.0f, 2.0f, 0.0f, 12.0f, invalid_limits[index], &result));
        test_invalid_voltage(&result);
    }
    TEST_FOC_CHECK(!foc_transform_current(1.0f, 2.0f, 0.0f, NULL));
    TEST_FOC_CHECK(!foc_modulate_voltage(1.0f, 2.0f, 0.0f, 12.0f, 1.0f, NULL));
    foc_current_t overflow = {1.0f, 1.0f, true};
    TEST_FOC_CHECK(!foc_transform_current(FLT_MAX, FLT_MAX, 0.0f, &overflow));
    TEST_FOC_CHECK(!overflow.is_valid && overflow.current_d_A == 0.0f && overflow.current_q_A == 0.0f);
    puts("PASS invalid inputs: all nonfinite fields, bus/limit range, NULL, output clearing, overflow");
}

/** 极端有限值验证double中间量和float发布边界；不声明亚分辨率物理精度。 */
static void test_extreme_ranges(void)
{
    float smallest = nextafterf(0.0f, 1.0f);
    const float buses[] = {smallest, FLT_MIN, 12.0f, FLT_MAX};
    const float limits[] = {smallest, FLT_MIN, 0.9f, 1.0f};
    const float angles[] = {0.0f, (float)(TEST_FOC_PI / 2.0), FLT_MAX, -FLT_MAX};
    for (size_t bus = 0u; bus < sizeof(buses) / sizeof(buses[0]); ++bus) {
        for (size_t limit = 0u; limit < sizeof(limits) / sizeof(limits[0]); ++limit) {
            for (size_t angle = 0u; angle < sizeof(angles) / sizeof(angles[0]); ++angle) {
                test_voltage_oracle(FLT_MAX, -FLT_MAX, angles[angle], buses[bus], limits[limit]);
                test_voltage_oracle(smallest, -smallest, angles[angle], buses[bus], limits[limit]);
            }
        }
    }
    test_current_oracle(FLT_MAX, -0.5f * FLT_MAX, 0.0f);
    test_current_oracle(1.0f, -0.5f, FLT_MAX);
    test_current_oracle(1.0f, -0.5f, -FLT_MAX);
    test_current_oracle(smallest, -smallest, 0.0f);
    puts("PASS extreme finite ranges: subnormal/big bus and commands, large angles, finite outputs");
}

/** 两个调用者对象交错使用同一模块；逐字段核对重复结果，不靠共享上下文保存状态。 */
static void test_interleaved_callers(void)
{
    for (int index = 0; index < 1000; ++index) {
        float angle = (float)(index / 100.0);
        foc_modulation_t first;
        foc_modulation_t unrelated;
        foc_modulation_t repeated;
        TEST_FOC_CHECK(foc_modulate_voltage(2.0f, -3.0f, angle, 12.0f, 0.9f, &first));
        TEST_FOC_CHECK(foc_modulate_voltage(-100.0f, 20.0f, -angle, 48.0f, 0.5f, &unrelated));
        foc_current_t current;
        TEST_FOC_CHECK(foc_transform_current(-2.0f, 5.0f, angle, &current));
        TEST_FOC_CHECK(foc_modulate_voltage(2.0f, -3.0f, angle, 12.0f, 0.9f, &repeated));
        TEST_FOC_CHECK(first.duty_u == repeated.duty_u && first.duty_v == repeated.duty_v && first.duty_w == repeated.duty_w);
        TEST_FOC_CHECK(first.limited_voltage_d_V == repeated.limited_voltage_d_V
            && first.limited_voltage_q_V == repeated.limited_voltage_q_V);
        TEST_FOC_CHECK(first.is_valid == repeated.is_valid && first.is_voltage_limited == repeated.is_voltage_limited);
    }
    puts("PASS independence: 1000 interleaved caller sequences, no shared mutable state");
}

/** 唯一主机测试入口，只有全部独立检查通过才返回0。 */
int main(void)
{
    test_known_vectors();
    test_current_grid();
    test_voltage_grid();
    test_error_contracts();
    test_extreme_ranges();
    test_interleaved_callers();
    printf("TOTAL independent oracle cases: current=%lu voltage=%lu; interleavings=1000\n",
        (unsigned long)current_case_count, (unsigned long)voltage_case_count);
    puts("PASS all M2a FOC math checks (synthetic host data only; firmware/hardware not connected)");
    return EXIT_SUCCESS;
}
