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

static void dq_arm(uint16_t angle=7453u,bool break_before_release=false)
{
    reset();fixed_angle=true;
    while (fixed_count!=angle) {
        unsigned delta=(unsigned)angle-fixed_count;fixed_count=(uint16_t)(fixed_count+(delta>16u ? 16u : delta));button_steps(false,1u);
    }
    for (unsigned i=0u;i<32u;++i) { current_loop_poll(); }
    CHECK(current_loop_is_ready());
    button_steps(false,300u);CHECK(motor_runtime_restart_zero());button_steps(false,512u);
    button_steps(true,21u);
    if (break_before_release) { pwm_tim1.RCR.value=3u; } /* 启动在board_prepare_pulse校验阶段就被拒。 */
    button_steps(false,21u);
    if (!break_before_release) { CHECK(status().state==MOTOR_RUNTIME_STARTING); }
}
static void dq_prepare(uint16_t angle=7453u)
{
    dq_arm(angle);
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
        for (unsigned k=0u;k<=(BOARD_DQ_VECTOR_DIAGNOSTIC ? 19u : 250u);++k) {
            if (BOARD_DQ_VECTOR_DIAGNOSTIC) { timer_tick(event+k*1000u-now_us); }
            else { now_us=event+k*1000u; }
            if (k) { natural_update();TIM1_UP_IRQHandler(); }
            double mean=(active_ticks[0]+active_ticks[1]+active_ticks[2])/5400.0;
            double vu=12.0*(active_ticks[0]/1800.0-mean),vv=12.0*(active_ticks[1]/1800.0-mean);
            double a=std::exp(-5.5*0.000773/0.0008),b=std::exp(-5.5*0.000227/0.0008);
            iu=a*iu+(1-a)*previous_u/5.5;iu=b*iu+(1-b)*vu/5.5;
            iv=a*iv+(1-a)*previous_v/5.5;iv=b*iv+(1-b)*vv/5.5;previous_u=vu;previous_v=vv;
            if (BOARD_DQ_VECTOR_DIAGNOSTIC) { timer_tick(227u); }
            else { now_us+=227u; }
            sample(true,(uint16_t)std::lround(2027.0+iu/BOARD_NOMINAL_A_PER_COUNT),(uint16_t)std::lround(2028.0+iv/BOARD_NOMINAL_A_PER_COUNT));
            current_loop_poll();
            if (k<(BOARD_DQ_VECTOR_DIAGNOSTIC ? 19u : 250u)) { CHECK(status().state==MOTOR_RUNTIME_ACTIVE); }
            if (BOARD_DQ_VECTOR_DIAGNOSTIC) { CHECK(active_ticks[0]==900u && active_ticks[1]==900u && active_ticks[2]==900u); }
        }
        CHECK(status().state==MOTOR_RUNTIME_DISABLED && status().fault_flags==0u);check_closed();
        CHECK(motor_runtime_read_reply(&log,&action) && action.reason==MOTOR_ACTION_DURATION);
        CHECK(BOARD_DQ_VECTOR_DIAGNOSTIC ? action.ended_us-action.requested_us==20000u : action.ended_us-action.started_us==250227u);
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
    // 实际FAULT仍以1kHz采样：头帧/首次保护各一次，有限四点必须完整导出。
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));
    for (unsigned k=0u;k<4u;++k) { dq_frame(action.started_us+k*1000u); }
    dq_frame(action.started_us+4000u,2157u,2028u);CHECK(status().state==MOTOR_RUNTIME_FAULT);
    capture_flow=true;flow_count=0u;
    for (unsigned k=0u;k<12u;++k) { now_us+=1000u;sample();(void)current_loop_report_poll(); }
    CHECK(count_flow("P1D0,")==1u && count_flow("P1E1,")==1u && count_flow("P1D1,")==4u);
    std::fputs("DQ_FAILURE_CAPTURE_BEGIN\n",stderr);
    for (unsigned k=0u;k<flow_count;++k) { std::fputs(flow_lines[k],stderr); }
    std::fputs("DQ_FAILURE_CAPTURE_END\n",stderr);
    check_closed();
#if BOARD_DQ_VECTOR_DIAGNOSTIC
    // 非零观测误差也不能让PI改变固定三相PWM，积分不采纳。
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));
    for (unsigned k=0u;k<4u;++k) {
        dq_frame(action.started_us+k*1000u,2040u,2045u);
        CHECK(status().state==MOTOR_RUNTIME_ACTIVE);
        CHECK(pwm_tim1.CCR1.value==900u && pwm_tim1.CCR2.value==900u && pwm_tim1.CCR3.value==900u);
    }
    motor_runtime_stop();capture_flow=true;flow_count=0u;
    for (unsigned k=0u;k<8u;++k) { (void)current_loop_report_poll(); }
    CHECK(count_flow("P1D0,")==1u && count_flow("P1D1,")==4u);
    std::fputs("DQ_VECTOR_OBSERVER_BEGIN\n",stderr);
    for (unsigned k=0u;k<flow_count;++k) { std::fputs(flow_lines[k],stderr); }
    std::fputs("DQ_VECTOR_OBSERVER_END\n",stderr);check_closed();
#endif
    // 直接编译main USER3实际前台顺序，低优先任务耗时300us不阻碍先提交电流包。
    dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));now_us+=227u;sample(false,2027u,2028u);
    feedback_delay=300u;production_poll_iteration();feedback_delay=0u;
    CHECK(status().state==MOTOR_RUNTIME_ACTIVE);now_us=action.started_us+1000u;natural_update();TIM1_UP_IRQHandler();
    CHECK(status().state==MOTOR_RUNTIME_ACTIVE);motor_runtime_stop();check_closed();
#if BOARD_DQ_VECTOR_DIAGNOSTIC
    /* 原始诊断记录五条终止路径：正常截止、首次故障、STOP、Break、恢复样本缺失。
     * 每条路径只导出一次；解析与完整性判定由tools/dq_report.py独立完成。 */
    {
        auto drain=[&](const char *tag,bool feed_samples) {
            capture_flow=true;flow_count=0u;
            for (unsigned i=0u;i<400u;++i) {
                if (current_loop_report_poll()) { continue; }
                now_us+=1000u;
                if (feed_samples) { sample(); }
            }
            capture_flow=false;
            std::fprintf(stderr,"DQ_TRACE_%s_BEGIN\n",tag);
            for (unsigned i=0u;i<flow_count;++i) { std::fputs(flow_lines[i],stderr); }
            std::fprintf(stderr,"DQ_TRACE_%s_END\n",tag);
        };
        auto run_20ms=[&]() {
            motor_log_snapshot_t l;motor_action_t a;CHECK(motor_runtime_read_reply(&l,&a));
            uint32_t ev=a.started_us;
            for (unsigned k=0u;k<=19u;++k) {
                timer_tick(ev+k*1000u-now_us);
                if (k) { natural_update();TIM1_UP_IRQHandler(); }
                timer_tick(227u);sample(true,2027u,2028u);current_loop_poll();
            }
        };
        dq_prepare();run_20ms();
        CHECK(status().state==MOTOR_RUNTIME_DISABLED && status().fault_flags==0u);
        drain("NORMAL",true);
        dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));
        for (unsigned k=0u;k<4u;++k) { dq_frame(action.started_us+k*1000u); }
        dq_frame(action.started_us+4000u,2157u,2028u);CHECK(status().state==MOTOR_RUNTIME_FAULT);
        drain("TRIP",true);
        dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));dq_frame(action.started_us+1000u);
        motor_runtime_stop();CHECK(status().state==MOTOR_RUNTIME_DISABLED);
        drain("STOP",true);
        dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));dq_frame(action.started_us+1000u);
        pwm_tim1.BDTR.value&=~TIM_BDTR_MOE;pwm_tim1.SR.value|=TIM_SR_BIF;TIM1_BRK_IRQHandler();
        CHECK(status().state==MOTOR_RUNTIME_FAULT);
        drain("BREAK",true);
        dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));
        for (unsigned k=0u;k<4u;++k) { dq_frame(action.started_us+k*1000u); }
        dq_frame(action.started_us+4000u,2027u,2158u);CHECK(status().state==MOTOR_RUNTIME_FAULT);
        drain("TRIPV",true); /* 第二类采样故障：仅V越限，首因必须是2。 */
        dq_arm();pwm_tim1.RCR.value=3u;open_window(); /* 破坏已验收配置，启动事务必须拒绝。 */
        CHECK(status().state==MOTOR_RUNTIME_FAULT);
        drain("STARTFAIL",true);
        dq_arm(7453u,true); /* 更早的拒绝点：arm之后、动作代次之前即失败，不应产生任何记录。 */
        CHECK(status().state==MOTOR_RUNTIME_DISABLED && status().fault_flags==0u);
        drain("NOACTION",true);
        dq_prepare();CHECK(motor_runtime_read_reply(&log,&action));
        for (unsigned k=0u;k<4u;++k) { dq_frame(action.started_us+k*1000u); }
        dq_frame(action.started_us+4000u,2157u,2028u);CHECK(status().state==MOTOR_RUNTIME_FAULT);
        /* 恢复窗口内没有样本：超时后按实际条数导出；导出开始后才恢复采样，新样本必须被拒绝追加。 */
        for (unsigned i=0u;i<20u;++i) { now_us+=1000u; }
        capture_flow=true;flow_count=0u;
        for (unsigned i=0u;i<400u;++i) {
            if (current_loop_report_poll()) { continue; }
            now_us+=1000u;sample();
        }
        capture_flow=false;
        std::fputs("DQ_TRACE_NORECOVERY_BEGIN\n",stderr);
        for (unsigned i=0u;i<flow_count;++i) { std::fputs(flow_lines[i],stderr); }
        std::fputs("DQ_TRACE_NORECOVERY_END\n",stderr);
        check_closed();
    }
#endif
    std::puts(BOARD_DQ_VECTOR_DIAGNOSTIC ? "PASS fixed equal-PWM diagnostic: no PI application, independent20ms deadline, biased observer, unchanged trips/STOP/Break; no closed-loop acceptance" : "PASS actual DQ runtime/board/current_control: zero and fixed signed references, four angles, bounded RL, trace, STOP/Break, mailbox overwrite, raw trip, movement, deadline, one gesture");
    return 0;
}
