/** 同一生产board_pwm、runtime、command；枚举访问点的自然UEV/异步Break/排队STOP。
 * 合成profile和寄存器模型仅用于主机；不证明桥电气行为、延迟或电流安全。 */
#include "linkage.h"
#include "main.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr,"FAIL gate %d: %s\n",__LINE__,#x); std::exit(1); } } while (0)
PwmTimer pwm_tim1(true),pwm_tim3(false);
PwmGpio pwm_gpioa,pwm_gpiob;
static uint32_t now_us,mask,sequence,clock_step;
static uint32_t active_ticks[3];
static unsigned access_count,inject_at,inject_kind,enables,early_enables,ug_writes;
static bool injecting,queued_stop,trace;

static void natural_update()
{
    active_ticks[0]=pwm_tim1.CCR1.value; active_ticks[1]=pwm_tim1.CCR2.value; active_ticks[2]=pwm_tim1.CCR3.value;
    pwm_tim1.SR.value|=TIM_SR_UIF;
}

void pwm_model_access(unsigned, bool)
{
    if (!trace || injecting) { return; }
    ++access_count;
    if (access_count!=inject_at) { return; }
    injecting=true;
    if (inject_kind==1u) { natural_update(); }
    if (inject_kind==2u) { /* 短Break低→高，BIF锁存，硬件异步清MOE；CPU IRQ待解锁。 */
        pwm_tim1.BDTR.value&=~TIM_BDTR_MOE; pwm_tim1.SR.value|=TIM_SR_BIF;
    }
    if (inject_kind==3u) { queued_stop=true; }
    injecting=false;
}

void pwm_model_written(unsigned id, uint32_t value)
{
    if (id==R_EGR) { ++ug_writes; }
    if (id==R_BDTR && (value & TIM_BDTR_MOE)) {
        ++enables;
        if (active_ticks[0]!=100u || active_ticks[1]!=200u || active_ticks[2]!=300u
            || pwm_tim1.CNT.value<=300u || (pwm_tim1.CR1.value & TIM_CR1_DIR)) { ++early_enables; }
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
uint32_t board_get_time_us(void) { uint32_t old=now_us;now_us+=clock_step;return old; }
bool board_serial_is_busy(void) { return true; }
bool board_start_serial_write(const char *,uint32_t) { return false; }

static motor_status_t status()
{
    motor_status_t s;CHECK(motor_runtime_read_status(&s));return s;
}
static void sample(bool poll=true)
{
    ++sequence;
    board_encoder_frame_t e={};
    e.sequence=sequence;e.raw_angle_count=(uint16_t)(sequence & 16383u);
    e.read_started_us=now_us-50u;e.read_completed_us=now_us-28u;e.is_valid=true;
    motor_sample_t s=board_make_sample(sequence,now_us,2027u,2028u,&e);
    motor_runtime_handle_sample(&s);
    if (poll) { motor_runtime_poll_feedback(); }
}
static void reset(uint32_t base=0u)
{
    trace=false;inject_at=0u;inject_kind=0u;access_count=0u;queued_stop=false;mask=0u;
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
int main()
{
    (void)timer_tick;
    reset();
#ifndef MOTOR_GATE_TEST_PROFILES
    CHECK(motor_runtime_output_profile_count()==0u);
    for (unsigned id=0u;id<256u;++id) { CHECK(!motor_runtime_arm(id));CHECK(!motor_runtime_pulse()); }
    open_window();check_closed();CHECK(status().state==MOTOR_RUNTIME_DISABLED);
    std::puts("PASS production: 0 profiles, all ARM IDs/PULSE/unsolicited IRQs denied; no MOE enable");
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
            motor_runtime_stop();open_window();check_closed();
        }
    }
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());
    natural_update();TIM1_UP_IRQHandler();pwm_tim1.CNT.value=1700u;pwm_tim1.CR1.value|=TIM_CR1_DIR;
    pwm_tim1.SR.value|=TIM_SR_CC4IF;TIM1_CC_IRQHandler();check_closed();CHECK(status().state==MOTOR_RUNTIME_FAULT);
    /* main完全停滞，TIM3在raw年龄301us独立停机；300us仍未超过旧门限。 */
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    timer_tick(300u);CHECK(status().state==MOTOR_RUNTIME_ACTIVE);
    timer_tick(1u);check_closed();CHECK(status().fault_flags & MOTOR_FAULT_SAMPLE);
    /* ADC仍推进而不调用前台deadline：PULSE目标1000us不可因样本/STATUS延长。 */
    reset();CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    for (unsigned i=0u;i<9u;++i) { timer_tick(100u);sample(); }
    timer_tick(99u);CHECK(status().state==MOTOR_RUNTIME_ACTIVE);timer_tick(1u);check_closed();
    CHECK(status().state==MOTOR_RUNTIME_DISABLED && !motor_runtime_pulse());
    reset(0xffffd800u);CHECK(motor_runtime_arm(1u));CHECK(motor_runtime_pulse());open_window();
    timer_tick(301u);check_closed();CHECK(status().state==MOTOR_RUNTIME_FAULT);
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
    reset();now_us+=301u;CHECK(!motor_runtime_arm(1u));check_closed();
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
