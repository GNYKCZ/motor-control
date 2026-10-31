/** 实际runtime/board/core闭环路径，独立星形RL对象；不运行历史main。 */
#include <cmath>
#include <initializer_list>
extern "C" {
#include "current_loop.h"
}
#define main historical_gate_main
#include "test_pwm_gate.cpp"
#undef main
static uint32_t feedback_delay;
extern "C" void __real_motor_runtime_poll_feedback(void);
extern "C" void __wrap_motor_runtime_poll_feedback(void) { __real_motor_runtime_poll_feedback();now_us+=feedback_delay; }
static uint32_t HAL_GetTick(void) { return now_us/1000u; }
static void telemetry_poll_status(uint32_t) {}
#ifdef TEST_OLD_DQ_ORDER
#include "dq_production_loop_old.h"
#else
#include "dq_production_loop.h"
#endif

static void dq_prepare(uint16_t angle=7453u)
{
    reset();fixed_angle=true;
    while (fixed_count!=angle) {
        unsigned delta=(unsigned)angle-fixed_count;fixed_count=(uint16_t)(fixed_count+(delta>16u ? 16u : delta));button_steps(false,1u);
    }
    for (unsigned i=0u;i<32u;++i) { current_loop_poll(); }
    CHECK(current_loop_is_ready());
    button_steps(false,300u);CHECK(motor_runtime_restart_zero());button_steps(false,512u);
    button_steps(true,21u);button_steps(false,21u);CHECK(status().state==MOTOR_RUNTIME_STARTING);
    open_window();CHECK(status().state==MOTOR_RUNTIME_ACTIVE && enables==1u && early_enables==0u);
    CHECK(active_ticks[0]==900u && active_ticks[1]==900u && active_ticks[2]==900u);
}
static void dq_frame(uint32_t event,uint16_t u=2027u,uint16_t v=2028u,bool compute=true)
{
    now_us=event;natural_update();TIM1_UP_IRQHandler();now_us+=227u;sample(true,u,v);
    if (compute) { current_loop_poll(); }
}
int main(void)
{
    for (uint16_t angle : {0u,4096u,7453u,16000u}) {
        dq_prepare(angle);motor_log_snapshot_t log;motor_action_t action;CHECK(motor_runtime_read_reply(&log,&action));
        uint32_t event=action.started_us;double iu=0,iv=0,previous_u=0,previous_v=0;
        for (unsigned k=0u;k<=250u;++k) {
            now_us=event+k*1000u;
            if (k) { natural_update();TIM1_UP_IRQHandler(); }
            double mean=(active_ticks[0]+active_ticks[1]+active_ticks[2])/5400.0;
            double vu=12.0*(active_ticks[0]/1800.0-mean),vv=12.0*(active_ticks[1]/1800.0-mean);
            double a=std::exp(-5.5*0.000773/0.0008),b=std::exp(-5.5*0.000227/0.0008);
            iu=a*iu+(1-a)*previous_u/5.5;iu=b*iu+(1-b)*vu/5.5;
            iv=a*iv+(1-a)*previous_v/5.5;iv=b*iv+(1-b)*vv/5.5;previous_u=vu;previous_v=vv;
            now_us+=227u;
            sample(true,(uint16_t)std::lround(2027.0+iu/BOARD_NOMINAL_A_PER_COUNT),(uint16_t)std::lround(2028.0+iv/BOARD_NOMINAL_A_PER_COUNT));
            current_loop_poll();
            if (k<250u) { CHECK(status().state==MOTOR_RUNTIME_ACTIVE); }
        }
        CHECK(status().state==MOTOR_RUNTIME_DISABLED && status().fault_flags==0u);check_closed();
        CHECK(motor_runtime_read_reply(&log,&action) && action.reason==MOTOR_ACTION_DURATION);
        CHECK(action.ended_us-action.started_us==250227u);
        for (unsigned i=0u;i<260u;++i) { if (current_loop_report_poll()) { std::fputs(scan_wire,stdout); } }
        button_steps(true,21u);button_steps(false,21u);CHECK(enables==1u);
    }
    // STOP/Break在预装待生效阶段必须撤销；后续UEV不重开。
    dq_prepare();motor_log_snapshot_t log;motor_action_t action;CHECK(motor_runtime_read_reply(&log,&action));
    dq_frame(action.started_us+1000u);motor_runtime_stop();natural_update();TIM1_UP_IRQHandler();check_closed();
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));dq_frame(action.started_us+1000u);
    pwm_tim1.BDTR.value &= ~TIM_BDTR_MOE;pwm_tim1.SR.value |= TIM_SR_BIF;TIM1_BRK_IRQHandler();natural_update();TIM1_UP_IRQHandler();check_closed();
    // 不计算，下一帧不能覆盖旧邮箱。
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));dq_frame(action.started_us+1000u,2027u,2028u,false);
    dq_frame(action.started_us+2000u,2027u,2028u,false);CHECK(status().state==MOTOR_RUNTIME_FAULT);check_closed();
    // 原始诊断和更小运动包络；零指令不是硬件停机。
    dq_prepare();now_us+=227u;sample(true,2156u,2028u);check_closed();
    dq_prepare();now_us+=227u;fixed_count=(uint16_t)((fixed_count+256u)&16383u);sample();check_closed();
    dq_prepare();now_us+=3500u;motor_runtime_check_deadline(now_us);check_closed();
    // 配置失配、迟到、陈旧帧和无输出基准超预算不能授予新提交。
    reset();pwm_tim1.RCR.value=3u;CHECK(!board_output_is_ready());
    reset();pwm_tim1.CCMR1.value=0u;CHECK(!board_output_is_ready());
    reset();clock_step=201u;current_loop_poll();clock_step=0u;CHECK(!current_loop_is_ready());
    CHECK(current_loop_timing_failed());capture_flow=true;command_enable_local_flow();automatic_steps(false,80u);
    CHECK(count_flow("P1F1,FAILED_CONTROL_TIMING")==1u && count_flow("P1F1,FAILED_ZERO_INVALID")==0u);
    CHECK(enables==0u);
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));now_us+=227u;sample(true,2027u,2028u);
    now_us+=500u;current_loop_poll();CHECK(status().state==MOTOR_RUNTIME_FAULT);check_closed();
    // 真提交的Break/STOP/绝对期限交错；无额外使能，撤销后任何UEV不重开。
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));now_us+=227u;sample(true,2027u,2028u);
    trace=true;access_count=0u;current_loop_poll();trace=false;unsigned accesses=access_count;CHECK(accesses>10u);
    motor_runtime_stop();check_closed();
    for (unsigned kind=2u;kind<=4u;++kind) for (unsigned at=1u;at<=accesses;++at) {
        dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));now_us+=227u;sample(true,2027u,2028u);
        forced_deadline=action.requested_us+270000u;trace=true;access_count=0u;inject_kind=kind;inject_at=at;
        current_loop_poll();trace=false;if (kind==2u) { TIM1_BRK_IRQHandler(); }
        motor_runtime_stop();natural_update();TIM1_UP_IRQHandler();check_closed();CHECK(enables==1u);
    }
    // 直接编译main USER3实际前台顺序，低优先任务耗时300us不阻碍先提交电流包。
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));now_us+=227u;sample(false,2027u,2028u);
    feedback_delay=300u;production_poll_iteration();feedback_delay=0u;
    CHECK(status().state==MOTOR_RUNTIME_ACTIVE);now_us=action.started_us+1000u;natural_update();TIM1_UP_IRQHandler();
    CHECK(status().state==MOTOR_RUNTIME_ACTIVE);motor_runtime_stop();check_closed();
    std::puts("PASS actual DQ runtime/board/current_control: zero and fixed signed references, four angles, bounded RL, trace, STOP/Break, mailbox overwrite, raw trip, movement, deadline, one gesture");
    return 0;
}
