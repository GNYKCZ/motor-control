/** M2b独立主机oracle：三相投影/极坐标、手算递推、状态事务与交错轨迹。
 * 只调用真实core，不mock M2a；所有物理输入均合成，不代表实际电机闭环。 */
#include "current_control.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
#define PI 3.14159265358979323846264338327950288
static unsigned oracle_cases; /**< 测试计数，不属于控制核可变状态。 */
static unsigned invalid_cases;

/** 有限性与实施前固定的绝对误差门禁，失败打印原始数值。 */
static void near(double actual, double expected, double tolerance)
{
    if (!isfinite(actual) || !isfinite(expected) || fabs(actual - expected) > tolerance) {
        fprintf(stderr, "FAIL actual=%.17g expected=%.17g tolerance=%.17g\n", actual, expected, tolerance);
        exit(1);
    }
}

/** 测试参数只是合成数值，不是电机推荐增益。 */
static current_control_config_t configuration(void)
{
    current_control_config_t c = {{2,4,2}, {3,2,1}, 10, 1, 0.0001f, 0.5f};
    return c;
}

/** 独立三相投影，不调用M2a或被测step产生expected。 */
static void project(current_control_input_t in, float *d, float *q, double *tolerance)
{
    double currents[3] = {in.current_u_A, in.current_v_A, -(double)in.current_u_A - in.current_v_A};
    double sum_d = 0, sum_q = 0;
    for (unsigned phase = 0; phase < 3; ++phase) {
        double angle = in.electrical_angle_rad - 2.0 * PI * phase / 3.0;
        sum_d += currents[phase] * cos(angle);
        sum_q -= currents[phase] * sin(angle);
    }
    *d = (float)(sum_d * (2.0 / 3.0));
    *q = (float)(sum_q * (2.0 / 3.0));
    *tolerance = 8.0 * FLT_EPSILON * fmax(1.0, fabs(currents[0]) + fabs(currents[1]) + fabs(currents[2]));
}

/** 每拍独立double方程从当前float状态出发；极坐标求最终电压，线电压检查占空比。
 * 逐拍误差检查不让长轨迹误差累积后靠扩大容差蒙混。 */
static current_control_output_t check_step(current_control_state_t *state, current_control_config_t c,
    current_control_input_t in)
{
    current_control_state_t old = *state;
    float measured_d, measured_q;
    double current_tolerance;
    project(in, &measured_d, &measured_q, &current_tolerance);
    float references[2] = {in.reference_d_A, in.reference_q_A};
    for (unsigned axis = 0; axis < 2; ++axis) {
        if (references[axis] > c.reference_limit_A) { references[axis] = c.reference_limit_A; }
        if (references[axis] < -c.reference_limit_A) { references[axis] = -c.reference_limit_A; }
    }
    double errors[2] = {(double)references[0] - measured_d, (double)references[1] - measured_q};
    current_control_axis_t gains[2] = {c.d,c.q};
    double integrals[2] = {old.integral_d_V,old.integral_q_V};
    float requested[2];
    for (unsigned a = 0; a < 2; ++a) { requested[a] = (float)(gains[a].kp_V_per_A * errors[a] + integrals[a]); }
    double magnitude = hypot(requested[0], requested[1]);
    double bound = (double)c.modulation_limit * in.bus_voltage_V / sqrt(3.0);
    double limited_magnitude = magnitude > bound ? bound : magnitude;
    double direction = atan2(requested[1], requested[0]);
    float limited[2] = {(float)(limited_magnitude * cos(direction)), (float)(limited_magnitude * sin(direction))};
    current_control_output_t out;
    CHECK(current_control_step(state,c,in,&out));
    CHECK(out.is_valid && out.measured.is_valid && out.modulation.is_valid);
    near(out.measured.current_d_A,measured_d,current_tolerance);
    near(out.measured.current_q_A,measured_q,current_tolerance);
    CHECK(out.limited_reference_d_A == references[0] && out.limited_reference_q_A == references[1]);
    CHECK(out.is_reference_limited == (references[0] != in.reference_d_A || references[1] != in.reference_q_A));
    double actual_requested[2] = {out.requested_voltage_d_V,out.requested_voltage_q_V};
    double actual_limited[2] = {out.modulation.limited_voltage_d_V,out.modulation.limited_voltage_q_V};
    double actual_integrals[2] = {state->integral_d_V,state->integral_q_V};
    for (unsigned a = 0; a < 2; ++a) {
        double request_tolerance = gains[a].kp_V_per_A * current_tolerance
            + 8.0 * FLT_EPSILON * fmax(1.0,fabs(gains[a].kp_V_per_A * errors[a]) + fabs(integrals[a]));
        near(actual_requested[a],requested[a],request_tolerance);
        near(actual_limited[a],limited[a],request_tolerance + 16.0 * FLT_EPSILON * in.bus_voltage_V);
        double integral_term = (double)gains[a].ki_V_per_A_s * in.dt_s * errors[a];
        double tracking_term = (double)gains[a].tracking_gain_per_s * in.dt_s * ((double)limited[a] - requested[a]);
        double expected_next = (float)(integrals[a] + integral_term + tracking_term);
        double integral_tolerance = gains[a].ki_V_per_A_s * (double)in.dt_s * current_tolerance
            + gains[a].tracking_gain_per_s * (double)in.dt_s * (2.0 * request_tolerance + 16.0 * FLT_EPSILON * in.bus_voltage_V)
            + 8.0 * FLT_EPSILON * fmax(1.0,fabs(integrals[a]) + fabs(integral_term) + fabs(tracking_term));
        near(actual_integrals[a],expected_next,integral_tolerance);
    }
    // 在远离浮点圆界的网格核对标志；接近圆界时单独检查电压/占空比，不用容差改标志。
    if (fabs(magnitude-bound) > 64.0*FLT_EPSILON*fmax(1.0,magnitude)) {
        CHECK(out.modulation.is_voltage_limited == (magnitude > bound));
    }
    double phase_voltage[3];
    double duties[3] = {out.modulation.duty_u,out.modulation.duty_v,out.modulation.duty_w};
    for (unsigned phase = 0; phase < 3; ++phase) {
        phase_voltage[phase] = limited_magnitude * cos(in.electrical_angle_rad + direction - 2.0*PI*phase/3.0);
        CHECK(duties[phase] >= 0.0 && duties[phase] <= 1.0);
    }
    double high = fmax(phase_voltage[0],fmax(phase_voltage[1],phase_voltage[2]));
    double low = fmin(phase_voltage[0],fmin(phase_voltage[1],phase_voltage[2]));
    for (unsigned phase = 0; phase < 3; ++phase) {
        near(duties[phase],0.5+(phase_voltage[phase]-(high+low)/2)/in.bus_voltage_V,8.0*FLT_EPSILON);
        unsigned next = (phase+1)%3;
        near((duties[phase]-duties[next])*in.bus_voltage_V,phase_voltage[phase]-phase_voltage[next],
            32.0*FLT_EPSILON*in.bus_voltage_V);
    }
    ++oracle_cases;
    return out;
}

/** 确定无效值逐字段清零，不能用包含padding的memcmp假设C结构体对象表示。 */
static void check_zero(current_control_output_t out)
{
    CHECK(!out.is_valid && !out.is_reference_limited && !out.measured.is_valid);
    CHECK(!out.modulation.is_valid && !out.modulation.is_voltage_limited);
    CHECK(out.measured.current_d_A==0 && out.measured.current_q_A==0);
    CHECK(out.limited_reference_d_A==0 && out.limited_reference_q_A==0);
    CHECK(out.requested_voltage_d_V==0 && out.requested_voltage_q_V==0);
    CHECK(out.modulation.limited_voltage_d_V==0 && out.modulation.limited_voltage_q_V==0);
    CHECK(out.modulation.duty_u==0 && out.modulation.duty_v==0 && out.modulation.duty_w==0);
}

static void expect_failure(current_control_state_t *state, current_control_config_t c, current_control_input_t in)
{
    unsigned char original[sizeof(*state)];
    memcpy(original,state,sizeof(*state));
    current_control_output_t out;
    memset(&out,0xa5,sizeof(out));
    CHECK(!current_control_step(state,c,in,&out));
    CHECK(memcmp(original,state,sizeof(*state))==0);
    check_zero(out);
    ++invalid_cases;
}

/** 手算二进制分数序列固定PI使用旧积分的时间索引；3/4联合圆界可识别逐轴预裁剪。 */
static void hand_and_saturation(void)
{
    current_control_config_t c=configuration();
    current_control_state_t state;
    CHECK(current_control_init(&state));
    current_control_input_t in={0,0,0,1,-2,100,0.25f};
    current_control_output_t out=check_step(&state,c,in);
    CHECK(out.requested_voltage_d_V==2 && out.requested_voltage_q_V==-6);
    CHECK(state.integral_d_V==1 && state.integral_q_V==-1);
    out=check_step(&state,c,in);
    CHECK(out.requested_voltage_d_V==3 && out.requested_voltage_q_V==-7);
    CHECK(state.integral_d_V==2 && state.integral_q_V==-2);
    CHECK(current_control_reset(&state));
    c.d=(current_control_axis_t){1,0,2};c.q=c.d;
    in=(current_control_input_t){0,0,0,3,4,(float)sqrt(3.0),0.25f};
    out=check_step(&state,c,in);
    CHECK(out.modulation.is_voltage_limited);
    near(out.modulation.limited_voltage_d_V,0.6,1e-6);
    near(out.modulation.limited_voltage_q_V,0.8,1e-6);
    near(state.integral_d_V,-1.2,1e-6);near(state.integral_q_V,-1.6,1e-6);
    CHECK(current_control_reset(&state));
    c.d=(current_control_axis_t){2,5,20};c.q=c.d;in.dt_s=0.01f;
    for(unsigned i=0;i<500;++i){out=check_step(&state,c,in);CHECK(out.modulation.is_voltage_limited);}
    near(state.integral_d_V,-4.65,2e-5);near(state.integral_q_V,-6.2,2e-5);
    // 解除电压约束后tracking项归零；改变误差方向后原积分也须按独立递推更新。
    in.bus_voltage_V=100;
    out=check_step(&state,c,in);CHECK(!out.modulation.is_voltage_limited);
    in.reference_d_A=-3;in.reference_q_A=-4;
    for(unsigned i=0;i<100;++i){check_step(&state,c,in);}
    CHECK(current_control_reset(&state));in.reference_d_A=0;in.reference_q_A=0;
    out=check_step(&state,c,in);CHECK(out.modulation.duty_u==0.5f && state.integral_d_V==0);
    puts("PASS hand sequences, joint saturation entry/hold/exit, reverse error and reset");
}

/** 多角度/不同dt/母线/目标/初始状态，全部按独立投影及极坐标逐拍核对。 */
static void grids_and_interleaving(void)
{
    current_control_config_t c=configuration();
    const float angles[]={-12.0f,-3.14159f,-1.57079f,0,0.5f,1.57079f,3.14159f,9.0f};
    const float steps[]={0.0001f,0.001f,0.125f,0.5f};
    for(unsigned a=0;a<8;++a)for(unsigned t=0;t<4;++t)for(int u=-3;u<=3;++u)
    for(int v=-3;v<=3;++v)for(unsigned bus=0;bus<3;++bus){
        current_control_state_t s={(float)u*0.25f,(float)v*-0.5f};
        c.modulation_limit=bus==0?0.2f:1.0f;
        current_control_input_t in={(float)u,(float)v,angles[a],u*5.0f,v*-4.0f,bus==2?48.0f:3.0f,steps[t]};
        check_step(&s,c,in);
    }
    current_control_state_t a={0},b={1,-1},separate_a=a,separate_b=b;
    current_control_state_t expected_a[1000],expected_b[1000];
    c=configuration();current_control_config_t other=c;other.q.kp_V_per_A=0.25f;
    current_control_input_t x={0.1f,-0.2f,0.3f,1,2,12,0.001f};
    current_control_input_t y={-0.3f,0.1f,-0.8f,-2,1,24,0.0001f};
    current_control_output_t separate_output;
    // 先分别跑完整轨迹，再交错重放，避免两条“参考”也按相同交错顺序运行。
    for(unsigned i=0;i<1000;++i){
        CHECK(current_control_step(&separate_a,c,x,&separate_output));expected_a[i]=separate_a;
    }
    for(unsigned i=0;i<1000;++i){
        CHECK(current_control_step(&separate_b,other,y,&separate_output));expected_b[i]=separate_b;
    }
    for(unsigned i=0;i<1000;++i){
        check_step(&a,c,x);check_step(&b,other,y);
        CHECK(a.integral_d_V==expected_a[i].integral_d_V && a.integral_q_V==expected_a[i].integral_q_V);
        CHECK(b.integral_d_V==expected_b[i].integral_d_V && b.integral_q_V==expected_b[i].integral_q_V);
    }
    puts("PASS independent projection/polar/line-voltage grid and 1000 interleaved contexts");
}

/** 所有浮点字段的非有限/非法边界，及仅q轴后期失败时的两轴原子提交。 */
static void invalid_and_extreme(void)
{
    current_control_config_t c=configuration();current_control_state_t s={0.125f,-0.25f};
    current_control_input_t in={0,0,0,1,2,12,0.125f};
    float *configuration_fields[]={&c.d.kp_V_per_A,&c.d.ki_V_per_A_s,&c.d.tracking_gain_per_s,
        &c.q.kp_V_per_A,&c.q.ki_V_per_A_s,&c.q.tracking_gain_per_s,&c.reference_limit_A,
        &c.modulation_limit,&c.minimum_dt_s,&c.maximum_dt_s};
    const float bad[]={NAN,INFINITY,-INFINITY,-1.0f};
    for(unsigned f=0;f<10;++f){float old=*configuration_fields[f];
        for(unsigned v=0;v<4;++v){*configuration_fields[f]=bad[v];expect_failure(&s,c,in);}
        *configuration_fields[f]=old;
    }
    float *input_fields[]={&in.current_u_A,&in.current_v_A,&in.electrical_angle_rad,
        &in.reference_d_A,&in.reference_q_A,&in.bus_voltage_V,&in.dt_s};
    for(unsigned f=0;f<7;++f){float old=*input_fields[f];
        for(unsigned v=0;v<3;++v){*input_fields[f]=bad[v];expect_failure(&s,c,in);}
        *input_fields[f]=old;
    }
    c.modulation_limit=1.01f;expect_failure(&s,c,in);c=configuration();
    c.reference_limit_A=0;expect_failure(&s,c,in);c=configuration();
    c.modulation_limit=0;expect_failure(&s,c,in);c=configuration();
    c.minimum_dt_s=0;expect_failure(&s,c,in);c=configuration();
    c.maximum_dt_s=c.minimum_dt_s/2;expect_failure(&s,c,in);c=configuration();
    in.dt_s=nextafterf(c.minimum_dt_s,0);expect_failure(&s,c,in);
    in.dt_s=nextafterf(c.maximum_dt_s,INFINITY);expect_failure(&s,c,in);in.dt_s=0.125f;
    in.bus_voltage_V=0;expect_failure(&s,c,in);in.bus_voltage_V=-1;expect_failure(&s,c,in);in.bus_voltage_V=12;
    s.integral_d_V=NAN;expect_failure(&s,c,in);s=(current_control_state_t){0,INFINITY};expect_failure(&s,c,in);
    CHECK(current_control_reset(&s));
    in.current_u_A=FLT_MAX;in.current_v_A=FLT_MAX;expect_failure(&s,c,in);in.current_u_A=0;in.current_v_A=0;
    c.d.kp_V_per_A=FLT_MAX;in.reference_d_A=2;expect_failure(&s,c,in);
    c=configuration();c.q.kp_V_per_A=0;c.q.ki_V_per_A_s=FLT_MAX;c.reference_limit_A=FLT_MAX;
    in.reference_d_A=1;in.reference_q_A=FLT_MAX;expect_failure(&s,c,in);
    c=configuration();in.reference_q_A=2;
    current_control_output_t out;CHECK(!current_control_step(NULL,c,in,&out));check_zero(out);
    current_control_state_t old=s;CHECK(!current_control_step(&s,c,in,NULL));CHECK(memcmp(&s,&old,sizeof(s))==0);
    CHECK(!current_control_init(NULL) && !current_control_reset(NULL));
    // 允许显式零增益/跟踪；dt闭区间与相同上下限；失败后须由调用者显式复位。
    c.d=(current_control_axis_t){0,0,0};c.q=c.d;c.minimum_dt_s=0.125f;c.maximum_dt_s=0.125f;
    CHECK(current_control_reset(&s));out=check_step(&s,c,in);CHECK(out.modulation.duty_u==0.5f);
    // 有限极值允许但不必溢出：零误差/增益时，不凭数值大就拒绝合法计算。
    c.d=(current_control_axis_t){FLT_MAX,FLT_MAX,FLT_MAX};c.q=c.d;
    in=(current_control_input_t){0,0,0,0,0,FLT_MAX,0.125f};
    out=check_step(&s,c,in);CHECK(s.integral_d_V==0 && out.modulation.duty_u==0.5f);
    c.d=(current_control_axis_t){0,0,0};c.q=c.d;s=(current_control_state_t){FLT_MAX,0};
    out=check_step(&s,c,in);CHECK(s.integral_d_V==FLT_MAX && out.modulation.is_voltage_limited);
    puts("PASS invalid fields, boundary dt, float overflow and no partial state commit");
}

int main(void)
{
    hand_and_saturation();grids_and_interleaving();invalid_and_extreme();
    printf("PASS M2b: %u independent step cases, %u invalid/rollback cases (synthetic host only)\n",oracle_cases,invalid_cases);
    return 0;
}
