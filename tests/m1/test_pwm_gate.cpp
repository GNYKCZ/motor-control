/** 同一生产board_pwm、runtime、command；枚举访问点的自然UEV/异步Break/排队STOP。
 * 合成profile和寄存器模型仅用于主机；不证明桥电气行为、延迟或电流安全。 */
#include "linkage.h"
#include "main.h"
#include "board_config.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr,"FAIL gate %d: %s\n",__LINE__,#x); std::exit(1); } } while (0)
PwmTimer pwm_tim1(true),pwm_tim3(false);
PwmGpio pwm_gpioa,pwm_gpiob;
static uint32_t now_us,mask,sequence,clock_step;
static uint32_t active_ticks[3];
static bool fixed_angle, delay_after_moe;
static uint16_t fixed_count;
static unsigned access_count,inject_at,inject_kind,enables,early_enables,ug_writes;
static uint32_t forced_deadline;
static bool injecting,queued_stop,trace;
static unsigned advancing_ticks, fraction_ticks;

/** 独立压力时钟：访问耗时是显式测试输入，不是MCU测得周期。CNT和设备us一起推进。 */
static void advance_timer(unsigned ticks)
{
    for (unsigned i=0u;i<ticks;++i) {
        fraction_ticks+=10u; /* PSC9：一个TIM1 tick占10个72MHz周期。 */
        if (fraction_ticks>=72u) { fraction_ticks-=72u; ++now_us; }
        if (pwm_tim1.CR1.value & TIM_CR1_DIR) {
            if (pwm_tim1.CNT.value>0u) { --pwm_tim1.CNT.value; }
            if (pwm_tim1.CNT.value==0u) { pwm_tim1.CR1.value &= ~TIM_CR1_DIR; }
        } else {
            ++pwm_tim1.CNT.value;
            if (pwm_tim1.CNT.value>=pwm_tim1.ARR.value) { pwm_tim1.CR1.value |= TIM_CR1_DIR; }
        }
    }
}

static void natural_update()
{
    active_ticks[0]=pwm_tim1.CCR1.value; active_ticks[1]=pwm_tim1.CCR2.value; active_ticks[2]=pwm_tim1.CCR3.value;
    pwm_tim1.SR.value|=TIM_SR_UIF;
}

void pwm_model_access(unsigned, bool)
{
    if (advancing_ticks) { advance_timer(advancing_ticks); }
    if (!trace || injecting) { return; }
    ++access_count;
    if (access_count!=inject_at) { return; }
    injecting=true;
    if (inject_kind==1u) { natural_update(); }
    if (inject_kind==2u) { /* 短Break低→高，BIF锁存，硬件异步清MOE；CPU IRQ待解锁。 */
        pwm_tim1.BDTR.value&=~TIM_BDTR_MOE; pwm_tim1.SR.value|=TIM_SR_BIF;
    }
    if (inject_kind==3u) { queued_stop=true; }
    if (inject_kind==4u) { now_us=forced_deadline; }
    injecting=false;
}

void pwm_model_written(unsigned id, uint32_t value)
{
    if (id==R_EGR) { ++ug_writes; }
    if (id==R_BDTR && (value & TIM_BDTR_MOE)) {
        ++enables;
        #ifdef MOTOR_GATE_TEST_PROFILES
        if (active_ticks[0]!=100u || active_ticks[1]!=200u || active_ticks[2]!=300u
#elif defined(MOTOR_LOCAL_PULSE_BASELINE)
        if (active_ticks[0]!=0u || active_ticks[1]!=60u || active_ticks[2]!=0u
#elif BOARD_ALIGNMENT_MODE == 1
        if (active_ticks[0]!=60u || active_ticks[1]!=60u || active_ticks[2]!=0u
#elif BOARD_ALIGNMENT_MODE == -1
        if (active_ticks[0]!=60u || active_ticks[1]!=0u || active_ticks[2]!=60u
#else
        if (active_ticks[0]!=60u || active_ticks[1]!=0u || active_ticks[2]!=0u
#endif
            || pwm_tim1.CNT.value<=300u || (pwm_tim1.CR1.value & TIM_CR1_DIR)) { ++early_enables; }
        if (delay_after_moe) { pwm_tim1.CNT.value=1700u; }
    }
    if (id==R_CRH && (value & 0xfffu)==0xaaau) {
        CHECK(!(pwm_tim1.BDTR.value & TIM_BDTR_MOE));
        CHECK(pwm_tim1.CCER.value==0x111u);
    }
}

uint32_t board_enter_critical(void) { uint32_t old=mask;mask=1u;return old; }
void board_leave_critical(uint32_t old)
{
    mask=old;
    if (!mask && queued_stop) { queued_stop=false; motor_runtime_stop(); }
}
uint32_t board_get_time_us(void) { if (advancing_ticks) { advance_timer(36u); } uint32_t old=now_us;now_us+=clock_step;return old; }
#if BOARD_SCAN_ENABLED
static char scan_wire[257];
static bool capture_flow;
static unsigned flow_count;
static char flow_lines[16][257];
static unsigned count_flow(const char *prefix)
{
    unsigned count=0u;
    for (unsigned i=0u;i<flow_count;++i) { if (std::strncmp(flow_lines[i],prefix,std::strlen(prefix))==0) { ++count; } }
    return count;
}
bool board_serial_is_busy(void) { return false; }
bool board_start_serial_write(const char *bytes,uint32_t length)
{ CHECK(length<=256u);std::memcpy(scan_wire,bytes,length);scan_wire[length]='\0';
  if (capture_flow) { CHECK(flow_count<16u);std::memcpy(flow_lines[flow_count++],scan_wire,length+1u); }
  return true; }
#else
bool board_serial_is_busy(void) { return true; }
bool board_start_serial_write(const char *,uint32_t) { return false; }
#endif

static motor_status_t status()
{
    motor_status_t s;CHECK(motor_runtime_read_status(&s));return s;
}
static void sample(bool poll=true,uint16_t raw_u=2027u,uint16_t raw_v=2028u)
{
    ++sequence;
    board_encoder_frame_t e={};
    e.sequence=sequence;e.raw_angle_count=fixed_angle ? fixed_count : (uint16_t)(sequence & 16383u);
    e.read_started_us=now_us-50u;e.read_completed_us=now_us-28u;e.is_valid=true;
    motor_sample_t s=board_make_sample(sequence,now_us,raw_u,raw_v,&e);
    motor_runtime_handle_sample(&s);
    if (poll) { motor_runtime_poll_feedback(); }
}
static void reset(uint32_t base=0u)
{
#if BOARD_SCAN_ENABLED
    capture_flow=false;flow_count=0u;
#endif
    trace=false;inject_at=0u;inject_kind=0u;access_count=0u;queued_stop=false;mask=0u;
    advancing_ticks=0u;fraction_ticks=0u;
    fixed_angle=false; delay_after_moe=false; fixed_count=0u;
    clock_step=0u;enables=0u;early_enables=0u;ug_writes=0u;sequence=0u;now_us=base;
    pwm_tim1.BDTR.value=TIM_BDTR_BKE | TIM_BDTR_OSSI | TIM_BDTR_OSSR;
    pwm_tim1.CCER.value=0u;pwm_tim1.DIER.value=0u;pwm_tim1.SR.value=0u;
    pwm_tim1.ARR.value=1799u;pwm_tim1.CR1.value=TIM_CR1_CEN;pwm_tim1.CNT.value=0u;
    pwm_tim1.CCR1.value=0u;pwm_tim1.CCR2.value=0u;pwm_tim1.CCR3.value=0u;
    pwm_gpiob.IDR.value=GPIO_PIN_12;pwm_gpioa.CRH.value=0xa5bcd222u;
    motor_runtime_init();command_init();board_initialize_output();motor_runtime_finish_boot(true);
    for (unsigned i=0u;i<100u;++i) { now_us+=100u;sample(); }
    CHECK(status().state==MOTOR_RUNTIME_DISABLED && status().feedback.window_is_valid);
    CHECK((pwm_gpioa.CRH.value & ~0xfffu)==0xa5bcd000u);
}
static void open_window()
{
    natural_update();TIM1_UP_IRQHandler();
    pwm_tim1.CR1.value&=~TIM_CR1_DIR;pwm_tim1.CNT.value=1049u;
    pwm_tim1.SR.value|=TIM_SR_CC4IF;TIM1_CC_IRQHandler();
}
static void check_closed()
{
    CHECK(!(pwm_tim1.BDTR.value & TIM_BDTR_MOE));
    CHECK(pwm_tim1.CCER.value==0u && (pwm_gpioa.CRH.value & 0xfffu)==0x222u);
    CHECK(early_enables==0u && ug_writes==0u && mask==0u);
}
static void timer_tick(uint32_t delta)
{
    for (uint32_t i=0u;i<delta;++i) {
        ++now_us;pwm_tim3.CNT.value=(pwm_tim3.CNT.value+1u)&65535u;
        if ((pwm_tim3.DIER.value & TIM_DIER_CC1IE) && pwm_tim3.CNT.value==pwm_tim3.CCR1.value) {
            pwm_tim3.SR.value|=TIM_SR_CC1IF;TIM3_IRQHandler();
        }
    }
}
/** 1ms前台轮询，期间真实runtime继续收100us样本，不借大时间跳跃补认手势。 */
static void button_steps(bool pressed,unsigned milliseconds)
{
    for (unsigned ms=0u;ms<milliseconds;++ms) {
        now_us+=1000u;sample(); /* 测试配置约1kHz真实输入节奏。 */
        motor_runtime_poll_button(pressed,now_us);
    }
}
static void still_ready()
{
    reset();fixed_angle=true;button_steps(false,300u);
}
#if BOARD_ALIGNMENT_MODE
static void automatic_steps(bool pressed, unsigned count, bool noisy=false)
{
    for (unsigned i=0u;i<count;++i) {
        now_us+=1000u;sample(true,(uint16_t)(2027u+(noisy && (i & 1u) ? 40u : 0u)),2028u);motor_runtime_poll_button(pressed,now_us);
        for (unsigned j=0u;j<8u;++j) { (void)command_poll(now_us); }
    }
}
#endif
int main()
{
    (void)timer_tick; (void)button_steps; (void)still_ready;
#if BOARD_SCAN_ENABLED
    (void)count_flow;
#endif
    reset();
#if defined(MOTOR_LOCAL_PULSE_BASELINE)
    // 完整低频零偏批：显式重开不使能，512个1ms样本，时间窗/整数和独立核对。
    still_ready();CHECK(motor_runtime_restart_zero());unsigned zero_enables=enables;
    motor_log_snapshot_t zero_log; motor_zero_snapshot_t zero;
    CHECK(motor_runtime_read_zero(&zero_log,&zero) && zero.offset.accepted_count==0u);
    uint32_t zero_first=now_us+1000u;
    button_steps(false,512u);
    CHECK(motor_runtime_read_zero(&zero_log,&zero));
    CHECK(zero.offset.accepted_count==512u && zero.offset.sum_u_count==2027u*512u
        && zero.offset.sum_v_count==2028u*512u && zero.flags==0x1du);
    CHECK(zero.began_us==zero_first && zero.ended_us-zero.began_us==511000u && enables==zero_enables);
    CHECK(!status().calibration_verified);
    CHECK(motor_runtime_restart_zero());button_steps(false,10u);fixed_count=16u;button_steps(false,1u);
    CHECK(motor_runtime_read_zero(&zero_log,&zero) && (zero.flags & 2u) && zero.offset.accepted_count==10u);
    fixed_count=0u;button_steps(false,600u);
    CHECK(motor_runtime_read_zero(&zero_log,&zero) && (zero.flags & 2u) && zero.offset.accepted_count==10u);
    reset();CHECK(!motor_runtime_restart_zero()); /* 没有静止暖机不能重开。 */
    // 比较事件+300tick进入延迟，每次寄存器访问4tick、取时36tick的明确压力场景。
    // 从实际CCR4起算，不把CNT固定在一个永不消失的窗口中。
    still_ready();button_steps(true,21u);button_steps(false,21u);
    natural_update();TIM1_UP_IRQHandler();
    pwm_tim1.CNT.value=pwm_tim1.CCR4.value;advance_timer(300u);
    pwm_tim1.SR.value|=TIM_SR_CC4IF;advancing_ticks=4u;TIM1_CC_IRQHandler();advancing_ticks=0u;
    CHECK(status().state==MOTOR_RUNTIME_ACTIVE && enables==1u && early_enables==0u);
    motor_runtime_stop();check_closed();
    // 更晚进入仍必须拒绝，不能靠提前比较放宽窗口或延长请求。
    still_ready();button_steps(true,21u);button_steps(false,21u);
    natural_update();TIM1_UP_IRQHandler();pwm_tim1.CNT.value=1400u;
    pwm_tim1.SR.value|=TIM_SR_CC4IF;advancing_ticks=4u;TIM1_CC_IRQHandler();advancing_ticks=0u;
    check_closed();CHECK(status().fault_flags & MOTOR_FAULT_START_WINDOW);
    reset();
    CHECK(motor_runtime_output_profile_count()==1u);
    open_window();check_closed();CHECK(status().state==MOTOR_RUNTIME_DISABLED);
    // 生产profile不可经API调用，只有原始PA12电平的新完整手势可到达。
    fixed_angle=true;
    for (unsigned ms=0u;ms<300u;++ms) {
        for (unsigned i=0u;i<10u;++i) { now_us+=100u;sample(); }
        motor_runtime_poll_button(false,now_us);
    }
    for (unsigned ms=0u;ms<21u;++ms) {
        for (unsigned i=0u;i<10u;++i) { now_us+=100u;sample(); }
        motor_runtime_poll_button(true,now_us);
    }
    for (unsigned ms=0u;ms<21u;++ms) {
        for (unsigned i=0u;i<10u;++i) { now_us+=100u;sample(); }
        motor_runtime_poll_button(false,now_us);
    }
    CHECK(status().state==MOTOR_RUNTIME_STARTING);open_window();
    CHECK(status().state==MOTOR_RUNTIME_ACTIVE && enables==1u && early_enables==0u);
    motor_log_snapshot_t log; motor_action_t action;
    CHECK(motor_runtime_read_reply(&log,&action));
    CHECK(action.id==1u && action.reason==MOTOR_ACTION_ACTIVE && (action.timing_flags & 0x80u));
    for (unsigned i=0u;i<19u;++i) { timer_tick(1000u);sample(true,2100u,1990u); }
    timer_tick(999u);CHECK(status().state==MOTOR_RUNTIME_ACTIVE);
    timer_tick(1u);check_closed();CHECK(status().state==MOTOR_RUNTIME_DISABLED);
    CHECK(motor_runtime_read_reply(&log,&action));
    CHECK(action.reason==MOTOR_ACTION_DURATION && action.timing_flags==0xc0u);
    CHECK(action.ended_us-action.requested_us==20000u);
    motor_zero_snapshot_t current_zero;motor_current_snapshot_t current;
    CHECK(motor_runtime_read_current(&log,&current_zero,&current));
    CHECK(current.samples.accepted_count==19u && current.samples.sum_u_count==19u*2100u
        && current.samples.sum_v_count==19u*1990u && !current.overflow && current.action_id==action.id);
    CHECK(current.began_us-action.started_us==1000u && current.ended_us-current.began_us==18000u);
    now_us+=1000u;sample();CHECK(motor_runtime_read_current(&log,&current_zero,&current));
    CHECK(current.samples.accepted_count==19u); /* 关闭后的样本不能污染诊断。 */
    for (unsigned ms=0u;ms<60u;++ms) {
        now_us+=1000u;motor_runtime_poll_button(ms<30u,now_us);
    }
    open_window();check_closed();CHECK(enables==1u);
    // 超容量只标诊断溢出，不覆盖前32帧，也不改变已有动作门控。
    still_ready();button_steps(true,21u);button_steps(false,21u);open_window();
    for (unsigned i=0u;i<33u;++i) { now_us+=100u;sample(true,2100u,1990u); }
    CHECK(motor_runtime_read_current(&log,&current_zero,&current));
    CHECK(current.samples.accepted_count==32u && current.samples.sum_u_count==32u*2100u && current.overflow==1u);
    motor_runtime_stop();check_closed();
    // RV-001：采集从开门前开始、ADC完成却在开门后；极端原始值不得污染带流统计。
    still_ready();button_steps(true,21u);button_steps(false,21u);open_window();
    now_us+=1u;sample(true,4095u,0u);
    CHECK(motor_runtime_read_current(&log,&current_zero,&current));
    CHECK(current.samples.accepted_count==0u);
    now_us+=1000u;sample(true,2100u,1990u);
    CHECK(motor_runtime_read_current(&log,&current_zero,&current));
    CHECK(current.samples.accepted_count==1u && current.samples.sum_u_count==2100u
        && current.samples.sum_v_count==1990u);
    motor_runtime_stop();check_closed();
    // 内部资格/调度调用有耗时，20000us仍从物理释放接受时起算。
    still_ready();button_steps(true,21u);clock_step=2u;button_steps(false,21u);clock_step=0u;
    CHECK(motor_runtime_read_reply(&log,&action));open_window();
    while (now_us-action.requested_us<19800u) { now_us+=100u;sample(); }
    now_us=action.requested_us+19999u;sample();CHECK(status().state==MOTOR_RUNTIME_ACTIVE);
    now_us=action.requested_us+20000u;sample();check_closed();
    CHECK(motor_runtime_read_reply(&log,&action) && action.reason==MOTOR_ACTION_DURATION);
    // 写MOE后计数推进到拒绝窗，必须由本次复核关闭，不调用测试补救STOP。
    still_ready();button_steps(true,21u);button_steps(false,21u);delay_after_moe=true;open_window();
    check_closed();CHECK(enables==1u && status().state==MOTOR_RUNTIME_FAULT);
    CHECK(status().fault_flags & MOTOR_FAULT_START_WINDOW);
    CHECK(motor_runtime_read_reply(&log,&action) && action.reason==MOTOR_ACTION_FAULT && action.timing_flags==0xc0u);
    // RV-001：在CC开门路径的每个寄存器访问点跨过绝对期限，不能保留ACTIVE或残留重开。
    still_ready();button_steps(true,21u);button_steps(false,21u);
    natural_update();TIM1_UP_IRQHandler();pwm_tim1.CNT.value=1049u;pwm_tim1.SR.value|=TIM_SR_CC4IF;
    trace=true;access_count=0u;TIM1_CC_IRQHandler();trace=false;unsigned deadline_points=access_count;
    for (unsigned at=1u;at<=deadline_points;++at) {
        still_ready();button_steps(true,21u);button_steps(false,21u);
        CHECK(motor_runtime_read_reply(&log,&action));forced_deadline=action.requested_us+20000u;
        for (unsigned i=0u;i<199u;++i) { now_us+=100u;sample(); }
        now_us=forced_deadline-1u;
        natural_update();TIM1_UP_IRQHandler();pwm_tim1.CNT.value=1049u;pwm_tim1.SR.value|=TIM_SR_CC4IF;
        trace=true;inject_kind=4u;inject_at=at;access_count=0u;TIM1_CC_IRQHandler();trace=false;
        check_closed();CHECK(status().state!=MOTOR_RUNTIME_ACTIVE);
        CHECK(motor_runtime_read_reply(&log,&action) && (action.timing_flags & 0x40u));
        unsigned prior=enables;open_window();CHECK(enables==prior);check_closed();
    }
    still_ready();button_steps(true,21u);motor_runtime_stop();button_steps(false,21u);
    open_window();CHECK(enables==0u && status().state==MOTOR_RUNTIME_DISABLED);
    CHECK(motor_runtime_read_reply(&log,&action) && action.id==0u);
    // 超5ms的采样空档不能把未完整的按下手势补成一次触发。
    still_ready();button_steps(true,21u);now_us+=6000u;
    motor_runtime_poll_button(false,now_us);button_steps(false,21u);open_window();CHECK(enables==0u);
    // 开机长按，松开只建立READY；必须再完成一个新手势。
    reset();fixed_angle=true;button_steps(true,300u);button_steps(false,21u);open_window();CHECK(enables==0u);
    button_steps(true,21u);button_steps(false,21u);CHECK(status().state==MOTOR_RUNTIME_STARTING);
    motor_runtime_stop();check_closed();
    // 弹跳低/高均短于20ms，不得产生尝试。
    still_ready();
    for (unsigned i=0u;i<8u;++i) { button_steps(true,2u);button_steps(false,2u); }
    CHECK(motor_runtime_read_reply(&log,&action) && action.id==0u);open_window();CHECK(enables==0u);
    // 资格失败也消耗启动机会，后续安静/重新手势不能偷偷重试。
    reset();button_steps(false,21u);button_steps(true,21u);button_steps(false,21u);
    CHECK(motor_runtime_read_reply(&log,&action) && action.id==1u && action.reason==MOTOR_ACTION_REJECTED);
    fixed_angle=true;button_steps(false,300u);button_steps(true,21u);button_steps(false,21u);open_window();CHECK(enables==0u);
    // 累计原始位移达到46count，实际生产IRQ路径必须撤销，并保留起止有效标志。
    still_ready();button_steps(true,21u);button_steps(false,21u);open_window();
    fixed_count=46u;now_us+=100u;sample();check_closed();
    CHECK(status().state==MOTOR_RUNTIME_FAULT && (status().fault_flags & MOTOR_FAULT_EXPERIMENT));
    CHECK(motor_runtime_read_reply(&log,&action) && action.reason==MOTOR_ACTION_FAULT && action.timing_flags==0xc0u);
    uint32_t ended=action.ended_us;motor_runtime_stop();
    CHECK(motor_runtime_read_reply(&log,&action) && action.ended_us==ended);
    // 本地释放事务寄存器访问中排队STOP，解锁后直接关闭；无补救STOP洗掉证据。
    still_ready();button_steps(true,21u);trace=true;inject_kind=3u;inject_at=1u;access_count=0u;
    button_steps(false,21u);trace=false;open_window();check_closed();CHECK(!queued_stop);
    // 故障拒绝也有一次动作记录；USB-only不能通过按键绕过设备故障。
    still_ready();motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);button_steps(false,21u);
    button_steps(true,21u);button_steps(false,21u);open_window();check_closed();
    CHECK(motor_runtime_read_reply(&log,&action) && action.id==1u && action.reason==MOTOR_ACTION_REJECTED);
    std::puts("PASS production: local released gesture only, one attempt/boot, twenty-ms deadline and valid action timestamps");
#elif BOARD_SCAN_ENABLED
    fixed_angle=true;button_steps(false,300u);button_steps(true,21u);button_steps(false,21u);
    open_window();check_closed();CHECK(enables==0u); /* 未显式零偏不得启动长扫描。 */
    reset(0xfff00000u);fixed_angle=true;button_steps(false,300u);CHECK(motor_runtime_restart_zero());
    button_steps(false,512u);
    button_steps(true,21u);button_steps(false,21u);open_window();
    CHECK(status().state==MOTOR_RUNTIME_ACTIVE && enables==1u);
    const unsigned oracle[7][3]={{60,0,0},{60,60,0},{0,60,0},{0,60,60},{0,0,60},{60,0,60},{60,0,0}};
    motor_log_snapshot_t log;motor_scan_snapshot_t scan;
    for (unsigned point=0u;point<BOARD_SEQUENCE_POINTS;++point) {
        unsigned index=BOARD_ALIGNMENT_MODE ? (point==0u ? (BOARD_ALIGNMENT_MODE==1 ? 1u : 5u) : 0u) : (BOARD_SCAN_DIRECTION==1 ? point : (6u-point)%6u);
        for (unsigned c=0u;c<3u;++c) { CHECK(active_ticks[c]==oracle[index][c]); }
        int position=(BOARD_ALIGNMENT_MODE ? -BOARD_ALIGNMENT_MODE : BOARD_SCAN_DIRECTION)*(int)point*390;
        fixed_count=(uint16_t)(position & 16383);
        for (unsigned ms=0u;ms<200u;++ms) {
            now_us+=1000u;pwm_tim1.CNT.value=1000u;sample();
        }
        CHECK(motor_runtime_read_scan(&log,&scan));
        CHECK(scan.point[point].count==50u && scan.point[point].sum==position*50);
        CHECK(scan.point[point].minimum==scan.point[point].maximum);
        if (point<BOARD_SEQUENCE_POINTS-1u) {
            CHECK(status().state==MOTOR_RUNTIME_ACTIVE);
            CHECK(scan.step==point); /* 写预装不是已生效，必须等自然UEV确认。 */
            now_us+=1999u;natural_update();TIM1_UP_IRQHandler(); /* 提交期限前1us仍可接受，总期限不后移。 */
            CHECK(motor_runtime_read_scan(&log,&scan) && scan.step==point+1u);
        }
    }
    check_closed();CHECK(scan.done==1u && scan.reason==1u && scan.mask==((1u<<BOARD_SEQUENCE_POINTS)-1u) && enables==1u);
    CHECK(scan.ended_us-scan.started_us<BOARD_SEQUENCE_LIMIT_US);
    const char request[]="P1C1,00000001,08,1B7E\r\n";
    for (unsigned i=0u;i<23u;++i) { command_receive_byte((uint8_t)request[i],now_us); }
    for (unsigned i=0u;i<250u;++i) { (void)command_poll(now_us); }
    CHECK(std::strlen(scan_wire)==(BOARD_ALIGNMENT_MODE ? 183u : 248u));std::fputs(scan_wire,stdout);
    auto start_scan=[]() {
        still_ready();CHECK(motor_runtime_restart_zero());button_steps(false,512u);
        button_steps(true,21u);button_steps(false,21u);open_window();
        CHECK(status().state==MOTOR_RUNTIME_ACTIVE && enables==1u);
    };
    // 扫描RV-001：SSI在150ms前、ADC在150ms完成，不得算入稳定窗口；同时跨32位时标回绕。
    reset(0xfff00000u);fixed_angle=true;button_steps(false,300u);CHECK(motor_runtime_restart_zero());
    button_steps(false,512u);button_steps(true,21u);button_steps(false,21u);open_window();
    for (unsigned i=0u;i<149u;++i) { now_us+=1000u;sample(); }
    now_us+=1000u;fixed_count=15u;sample();
    CHECK(motor_runtime_read_scan(&log,&scan) && scan.point[0].count==0u);
    fixed_count=0u;
    for (unsigned i=0u;i<49u;++i) { now_us+=1000u;sample(); }
    now_us+=1000u;fixed_count=14u;pwm_tim1.CNT.value=1000u;sample();
    CHECK(motor_runtime_read_scan(&log,&scan) && scan.point[0].count==50u && scan.point[0].sum==14);
    CHECK(scan.point[0].maximum==14 && (scan.mask & 1u)); /* SSI结束在200ms前、ADC到200ms才发布的最后帧保留。 */
    motor_runtime_stop();check_closed();
    start_scan();
    for (unsigned i=0u;i<199u;++i) { now_us+=1000u;sample(); }
    now_us+=1028u;fixed_count=15u;pwm_tim1.CNT.value=1000u;sample(); /* SSI结束恰好200000us，排除。 */
    CHECK(motor_runtime_read_scan(&log,&scan) && scan.point[0].count==49u && scan.point[0].sum==0);
    motor_runtime_stop();check_closed();
    // 整个最后50ms只有25帧，不得用延时或其他点补齐。
    start_scan();
    for (unsigned i=0u;i<150u;++i) { now_us+=1000u;pwm_tim1.CNT.value=1000u;sample(); }
    for (unsigned i=0u;i<25u;++i) { now_us+=2000u;sample(); }
    check_closed();CHECK(motor_runtime_read_scan(&log,&scan) && scan.reason==7u && !scan.done);
    // 40帧边界可提交，64帧存储边界保持有效，第65帧必须中止。
    start_scan();
    for (unsigned i=0u;i<150u;++i) { now_us+=1000u;sample(); }
    for (unsigned i=0u;i<40u;++i) { now_us+=1250u;sample(); }
    CHECK(motor_runtime_read_scan(&log,&scan) && scan.point[0].count==40u && (scan.mask & 1u));
    motor_runtime_stop();check_closed();
    start_scan();
    for (unsigned i=0u;i<150u;++i) { now_us+=1000u;sample(); }
    for (unsigned i=0u;i<64u;++i) { now_us+=100u;sample(); }
    CHECK(motor_runtime_read_scan(&log,&scan) && scan.point[0].count==64u && !scan.reason);
    now_us+=100u;sample();check_closed();CHECK(motor_runtime_read_scan(&log,&scan) && scan.reason==8u);
    start_scan();
    for (unsigned i=0u;i<200u;++i) { now_us+=1000u;fixed_count=(i%2u) ? 8u : 16376u;sample(); }
    check_closed();CHECK(motor_runtime_read_scan(&log,&scan) && scan.reason==7u);
    start_scan();now_us+=1000u;fixed_count=3999u;sample();CHECK(status().state==MOTOR_RUNTIME_ACTIVE);
    now_us+=1000u;fixed_count=4000u;sample();check_closed();
    CHECK(status().fault_flags==MOTOR_FAULT_EXPERIMENT);
    // 绝对期限不会因为板级提交/前台停顿而延长；实际生产runtime负责撤销。
    if (BOARD_ALIGNMENT_MODE) {
        start_scan();motor_action_t action;motor_log_snapshot_t snapshot;
        CHECK(motor_runtime_read_reply(&snapshot,&action));
        now_us=action.requested_us+420000u;
        motor_runtime_check_deadline(now_us);check_closed();
        CHECK(motor_runtime_read_scan(&snapshot,&scan) && !scan.done);
        // 同一启动的第二次手势不能再次开门。
        button_steps(true,21u);button_steps(false,21u);CHECK(enables==1u);
    }
    // 点请求未生效，独立TIM3提交期限必须截止，即使ADC与前台都不再推进。
    start_scan();
    for (unsigned i=0u;i<200u;++i) { now_us+=1000u;pwm_tim1.CNT.value=1000u;sample(); }
    timer_tick(2000u);check_closed();CHECK(status().fault_flags & MOTOR_FAULT_CALIBRATION);
    // CCR写入与UEV/Break/排队STOP逐访问点交错：不能提交混合矢量，也不得再次开MOE。
    start_scan();
    for (unsigned i=0u;i<199u;++i) { now_us+=1000u;sample(); }
    now_us+=1000u;trace=true;access_count=0u;pwm_tim1.CNT.value=1000u;sample();trace=false;
    unsigned update_points=access_count;CHECK(update_points>10u);
    for (unsigned kind=1u;kind<=3u;++kind) {
        for (unsigned at=1u;at<=update_points;++at) {
            start_scan();
            for (unsigned i=0u;i<199u;++i) { now_us+=1000u;sample(); }
            now_us+=1000u;pwm_tim1.CNT.value=1000u;
            trace=true;access_count=0u;inject_at=at;inject_kind=kind;sample();trace=false;
            if (status().state==MOTOR_RUNTIME_ACTIVE && kind==1u) {
                TIM1_UP_IRQHandler();
                CHECK(motor_runtime_read_scan(&log,&scan));
                if (scan.step==1u) {
                    unsigned phase=BOARD_ALIGNMENT_MODE ? 0u : (BOARD_SCAN_DIRECTION==1 ? 1u : 5u);
                    for (unsigned c=0u;c<3u;++c) { CHECK(active_ticks[c]==oracle[phase][c]); }
                } else {
                    now_us+=100u;natural_update();TIM1_UP_IRQHandler();
                    CHECK(motor_runtime_read_scan(&log,&scan) && scan.step==1u);
                }
                motor_runtime_stop();
            }
            if (kind == 2u) { TIM1_BRK_IRQHandler(); } /* 硬件已清MOE，服务待决Break IRQ后才检查引脚接管。 */
            check_closed();CHECK(enables==1u);
            natural_update();TIM1_UP_IRQHandler();TIM1_CC_IRQHandler();check_closed();
        }
    }
    // 电流诊断越界立即撤销，参数未标校准；旧样本不能让长动作继续。
    still_ready();CHECK(motor_runtime_restart_zero());
    button_steps(false,512u);
    button_steps(true,21u);button_steps(false,21u);open_window();
    now_us+=1000u;sample(true,2283u,2028u);CHECK(status().state==MOTOR_RUNTIME_ACTIVE); /* 256count等号 */
    now_us+=1000u;sample(true,2284u,2028u);check_closed(); /* 257count */
    CHECK(status().fault_flags==MOTOR_FAULT_CALIBRATION);
    CHECK(motor_runtime_read_scan(&log,&scan) && scan.reason==5u && !scan.done);
    start_scan();now_us+=1000u;sample(true,2227u,2228u);check_closed(); /* 单路200count，但推算W为-400count。 */
    CHECK(motor_runtime_read_scan(&log,&scan) && scan.reason==5u && !scan.done);
#if BOARD_ALIGNMENT_MODE
    // 自动准备穿越设备时标回绕；启动按住PA12仍不能开门。
    reset(0xfff00000u);fixed_angle=true;capture_flow=true;command_enable_local_flow();
    automatic_steps(true,1100u);
    CHECK(enables==0u && motor_runtime_zero_is_ready());
    CHECK(count_flow("P1F1,READY_PRESS_PA12")==1u && count_flow("P1Z1,")==1u);
    automatic_steps(true,200u);CHECK(count_flow("P1F1,READY_PRESS_PA12")==1u && enables==0u);
    automatic_steps(false,21u);automatic_steps(true,21u);automatic_steps(false,21u);
    open_window();CHECK(status().state==MOTOR_RUNTIME_ACTIVE && enables==1u);
    fixed_count=(uint16_t)((BOARD_ALIGNMENT_MODE*390)&16383);automatic_steps(false,200u);
    now_us+=100u;natural_update();TIM1_UP_IRQHandler();
    fixed_count=0u;automatic_steps(false,200u);check_closed();automatic_steps(false,80u);
    CHECK(count_flow("P1A2,00000000,08,")==1u && count_flow("P1F1,ACTION_DONE")==1u);
    for (unsigned i=0u;i<flow_count;++i) { std::fputs(flow_lines[i],stdout); }
    automatic_steps(true,21u);automatic_steps(false,21u);automatic_steps(false,100u);
    CHECK(enables==1u && count_flow("P1A2,")==1u);
    // 停止准备也不自动重开零偏；自动回执不占用用户命令序号。
    reset();fixed_angle=true;capture_flow=true;command_enable_local_flow();
    const char stop_request[]="P1C1,00000002,02,1A67\r\n";
    for (unsigned i=0u;i<23u;++i) { command_receive_byte((uint8_t)stop_request[i],now_us); }
    automatic_steps(false,1200u);
    motor_zero_snapshot_t zero;CHECK(motor_runtime_read_zero(&log,&zero) && !(zero.flags & 16u));
    CHECK(enables==0u && count_flow("P1F1,FAILED_STOP")==1u);
    CHECK(count_flow("P1R2,00000002,02,00,")==1u);
    // 准备中的传感器故障和一直移动都只能报告失败，不进行动作或重试。
    reset();fixed_angle=true;capture_flow=true;command_enable_local_flow();automatic_steps(false,50u);
    motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);automatic_steps(false,1200u);check_closed();
    CHECK(enables==0u && count_flow("P1F1,FAILED_INPUT")==1u && count_flow("P1A2,")==0u);
    reset();fixed_angle=true;capture_flow=true;command_enable_local_flow();automatic_steps(false,1200u,true);check_closed();
    CHECK(enables==0u && count_flow("P1F1,FAILED_ZERO_INVALID")==1u);
    reset();capture_flow=true;command_enable_local_flow();automatic_steps(false,3300u);check_closed();
    CHECK(enables==0u && count_flow("P1F1,FAILED_NOT_STILL")==1u);
    std::puts("PASS automatic alignment flow: no commands, held-at-boot no motion, zero/READY once, new PA12 only, unsolicited result once, STOP/fault/moving failure, no rearm, time wrap");
#endif
    std::puts(BOARD_ALIGNMENT_MODE ? "PASS production alignment: two applied points, preset side, 420ms bound, one gesture, SSI window and current/position/submit/Break/STOP checks; no hardware" : "PASS production scan: seven applied points, fixed direction, explicit zero, bounded statistics/current trip; no hardware");
#else
    CHECK(motor_runtime_output_profile_count()==1u);
    CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());
    CHECK(!(pwm_tim1.BDTR.value & TIM_BDTR_MOE));
    open_window();CHECK(status().state==MOTOR_RUNTIME_ACTIVE && enables==1u && early_enables==0u);
    motor_runtime_stop();open_window();check_closed();CHECK(status().state==MOTOR_RUNTIME_DISABLED);
    /* 每个生产寄存器访问点注入：自然UEV、短Break、IRQ屏蔽期间排队STOP。 */
    reset();CHECK(motor_runtime_arm(1u));trace=true;CHECK(motor_runtime_pulse());open_window();trace=false;
    unsigned points=access_count;
    for (unsigned kind=1u;kind<=3u;++kind) {
        for (unsigned at=1u;at<=points;++at) {
            reset();CHECK(motor_runtime_arm(1u));trace=true;inject_at=at;inject_kind=kind;
            (void)motor_runtime_pulse();open_window();trace=false;
            if (kind==2u) { TIM1_BRK_IRQHandler(); }
            if (kind==2u || kind==3u) {
                /* RV-001：这里没有补救STOP；必须直接观察交错本身的撤销结果。 */
                if (kind==2u) { CHECK(status().state==MOTOR_RUNTIME_FAULT && (status().fault_flags & MOTOR_FAULT_DEVICE)); }
                else { CHECK(!queued_stop && status().state!=MOTOR_RUNTIME_STARTING && status().state!=MOTOR_RUNTIME_ACTIVE); }
                check_closed();
                unsigned before=enables;
                open_window();open_window();TIM1_UP_IRQHandler();TIM1_CC_IRQHandler();TIM1_BRK_IRQHandler();
                CHECK(enables==before);check_closed();
            } else {
                /* 自然UEV不是撤销事件，单独清理此条正常请求。 */
                motor_runtime_stop();open_window();check_closed();
            }
        }
    }
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());
    natural_update();TIM1_UP_IRQHandler();pwm_tim1.CNT.value=1700u;pwm_tim1.CR1.value|=TIM_CR1_DIR;
    pwm_tim1.SR.value|=TIM_SR_CC4IF;TIM1_CC_IRQHandler();check_closed();CHECK(status().state==MOTOR_RUNTIME_FAULT);
    CHECK(status().fault_flags==(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_WINDOW));
    /* main完全停滞，TIM3在raw年龄301us独立停机；300us仍未超过旧门限。 */
    reset();CHECK(motor_runtime_arm(1u));
    timer_tick(3000u);CHECK(status().state==MOTOR_RUNTIME_ARMED);
    timer_tick(1u);check_closed();CHECK(status().fault_flags & MOTOR_FAULT_SAMPLE);
    /* ADC仍推进而不调用前台deadline：PULSE目标1000us不可因样本/STATUS延长。 */
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    for (unsigned i=0u;i<9u;++i) { timer_tick(100u);sample(); }
    timer_tick(99u);CHECK(status().state==MOTOR_RUNTIME_ACTIVE);timer_tick(1u);check_closed();
    CHECK(status().state==MOTOR_RUNTIME_DISABLED && !motor_runtime_pulse());
    reset(0xffffd800u);CHECK(motor_runtime_arm(1u));
    timer_tick(3001u);check_closed();CHECK(status().state==MOTOR_RUNTIME_FAULT);
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    command_receive_error();check_closed();CHECK(status().fault_flags & MOTOR_FAULT_COMMUNICATION);
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    const char *stop="P1C1,00000002,02,1A67\r\n";
    for (unsigned i=0u;i<std::strlen(stop);++i) { command_receive_byte((uint8_t)stop[i],now_us); }
    check_closed();CHECK(status().state==MOTOR_RUNTIME_DISABLED);
    reset();CHECK(motor_runtime_arm(1u));CHECK(!motor_runtime_arm(1u));check_closed();
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());CHECK(!motor_runtime_pulse());open_window();check_closed();
    /* 资格到期与反馈持续推进；不能因新样本把ARM寿命延长。 */
    reset();CHECK(motor_runtime_arm(1u));
    for (unsigned i=0u;i<99u;++i) { timer_tick(100u);sample(); }
    timer_tick(99u);CHECK(status().state==MOTOR_RUNTIME_ARMED);timer_tick(1u);
    CHECK(status().state==MOTOR_RUNTIME_DISABLED);check_closed();
    /* 原始帧仍来但前台不取反馈邮箱，受保护状态仍必须检测积压并停机。 */
    reset();CHECK(motor_runtime_arm(1u));
    for (unsigned i=0u;i<11u;++i) { timer_tick(100u);sample(false); }
    CHECK(status().state==MOTOR_RUNTIME_FAULT && (status().fault_flags & MOTOR_FAULT_FEEDBACK));check_closed();
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    motor_runtime_handle_sample(NULL);CHECK(status().state==MOTOR_RUNTIME_FAULT);check_closed();
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    const char *bad="P1C1,00000002,02,0000\r\n";
    for (unsigned i=0u;i<std::strlen(bad);++i) { command_receive_byte((uint8_t)bad[i],now_us); }
    CHECK(status().fault_flags & MOTOR_FAULT_COMMUNICATION);check_closed();
    for (unsigned stage=0u;stage<3u;++stage) {
        reset();CHECK(motor_runtime_arm(1u));
        if (stage>0u) { CHECK(motor_runtime_pulse()); }
        if (stage>1u) { open_window(); }
        motor_runtime_init();open_window();CHECK(status().state==MOTOR_RUNTIME_BOOT);check_closed();
    }
    reset();pwm_gpiob.IDR.value=0u;CHECK(!motor_runtime_arm(1u));check_closed();
    reset();now_us+=3001u;CHECK(!motor_runtime_arm(1u));check_closed();
    reset();board_pwm_ticks_t invalid={65535u,0u,0u,40u};CHECK(!board_prepare_pulse(invalid,1u));check_closed();
    reset();invalid.u=0u;invalid.margin_ticks=0u;CHECK(!board_prepare_pulse(invalid,1u));check_closed();
    reset();pwm_tim3.CNT.value=65530u;CHECK(board_schedule_deadline(now_us+70000u));
    CHECK(pwm_tim3.CCR1.value==65529u);board_cancel_deadline();
    reset();clock_step=2u;CHECK(!board_schedule_deadline(now_us+1u));clock_step=0u;
    CHECK(!(pwm_tim3.DIER.value & TIM_DIER_CC1IE));
    std::printf("PASS synthetic-only gate: %u access points x 3 events; natural preload/common-low/Break/STOP/deadline/wrap/communication\n",points);
#endif
    return 0;
}
