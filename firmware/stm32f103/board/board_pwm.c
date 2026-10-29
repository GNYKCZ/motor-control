/** 唯一PWM/引脚门控所有者；生成配置负责初始化，运行期无UG、无UDIS、无逐相HAL启动。
 * 所有事务有界，保存/恢复PRIMASK。MOE0只停止调制，不代表固定高EN的功率桥高阻。 */
#include "board_io.h"
#include "motor_runtime.h"
#include "main.h"

static uint32_t pending_generation;
static uint16_t largest_tick, window_margin;
static unsigned pending_phase; /**< 0无请求，1等自然UEV，2等上数共同低窗。 */
static bool deadline_clock_ready;
#define PHASE_CHANNELS (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E)

void board_disable_output(void)
{
    uint32_t mask=board_enter_critical();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    bool had_request=pending_phase!=0u || (TIM1->BDTR & TIM_BDTR_MOE)!=0u;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    if (had_request && deadline_clock_ready) { motor_runtime_record_gate(false,board_get_time_us()); }
    pending_phase=0u;
    TIM1->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC4IE);
    /* CCxE仍保持idle低，先接管GPIO再关通道，避免AF无驱动窗口。 */
    GPIOA->BRR=GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIOA->CRH=(GPIOA->CRH & ~0xfffu) | 0x222u;
    TIM1->CCER=0u;
    board_leave_critical(mask);
}

void board_prepare_output_idle(void)
{
    TIM1->BDTR=(TIM1->BDTR & ~(TIM_BDTR_MOE | TIM_BDTR_AOE)) | TIM_BDTR_OSSI | TIM_BDTR_OSSR;
    TIM1->CCER=PHASE_CHANNELS; /* 三路OIS由CubeMX置低；此处MOE始终为0。 */
}

void board_initialize_output(void)
{
    board_disable_output();
    TIM1->SR=~TIM_SR_BIF;
    TIM1->DIER |= TIM_DIER_BIE;
    TIM3->SR=0u;
    TIM3->CNT=0u;
    TIM3->DIER=0u;
    TIM3->CR1 |= TIM_CR1_CEN;
    deadline_clock_ready=true;
}

bool board_output_is_ready(void)
{
    return deadline_clock_ready && (GPIOB->IDR & GPIO_PIN_12)!=0u
        && (TIM1->SR & TIM_SR_BIF)==0u && (TIM1->CR1 & TIM_CR1_CEN)!=0u
        && (TIM1->BDTR & (TIM_BDTR_BKE | TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_AOE | TIM_BDTR_BKP))
            == (TIM_BDTR_BKE | TIM_BDTR_OSSI | TIM_BDTR_OSSR);
}

bool board_prepare_pulse(board_pwm_ticks_t ticks, uint32_t generation)
{
    uint32_t maximum=ticks.u;
    if (ticks.v>maximum) { maximum=ticks.v; }
    if (ticks.w>maximum) { maximum=ticks.w; }
    if (!board_output_is_ready() || ticks.margin_ticks==0u
        || maximum+2u*ticks.margin_ticks>=TIM1->ARR || TIM1->ARR!=1799u) { return false; }
    board_disable_output();
    board_prepare_output_idle();
    TIM1->CCR1=ticks.u; TIM1->CCR2=ticks.v; TIM1->CCR3=ticks.w;
    largest_tick=(uint16_t)maximum; window_margin=ticks.margin_ticks;
    TIM1->CCR4=(maximum+TIM1->ARR)/2u;
    pending_generation=generation;
    /* 写入间自然UEV可出现：MOE0。清旧UIF后只认下一次UEV，确保全部预装值已转移。 */
    TIM1->SR=~TIM_SR_UIF;
    pending_phase=1u;
    GPIOA->CRH=(GPIOA->CRH & ~0xfffu) | 0xaaau; /* 三路2MHz AF；此刻idle低。 */
    TIM1->DIER |= TIM_DIER_UIE;
    return true;
}

void TIM1_UP_IRQHandler(void)
{
    uint32_t mask=board_enter_critical();
    if ((TIM1->SR & TIM_SR_UIF)!=0u && (TIM1->DIER & TIM_DIER_UIE)!=0u) {
        TIM1->SR=~TIM_SR_UIF;
        TIM1->DIER &= ~TIM_DIER_UIE;
        if (pending_phase==1u) {
            pending_phase=2u;
            TIM1->SR=~TIM_SR_CC4IF;
            TIM1->DIER |= TIM_DIER_CC4IE;
        }
    }
    board_leave_critical(mask);
}

/** 开门前后均要求上数共同低窗，余量来自profile；迟到直接拒绝，不等待或重试。 */
static bool in_start_window(void)
{
    return (TIM1->CR1 & TIM_CR1_DIR)==0u && TIM1->CNT>largest_tick+window_margin
        && TIM1->CNT+window_margin<TIM1->ARR;
}

void TIM1_CC_IRQHandler(void)
{
    uint32_t mask=board_enter_critical();
    if ((TIM1->SR & TIM_SR_CC4IF)!=0u && (TIM1->DIER & TIM_DIER_CC4IE)!=0u) {
        TIM1->SR=~TIM_SR_CC4IF;
        TIM1->DIER &= ~TIM_DIER_CC4IE;
        if (pending_phase==2u) {
            pending_phase=0u;
            if (board_output_is_ready() && in_start_window()
                && motor_runtime_commit_output(pending_generation) && in_start_window()) {
                TIM1->BDTR |= TIM_BDTR_MOE;
                uint32_t opened_us=board_get_time_us();
                motor_runtime_record_gate(true,opened_us);
                /* 短Break可能发生在最终检查与MOE写之间，低窗内再次检查，禁止陈旧事务重开。 */
                if (!board_output_is_ready() || !in_start_window()) {
                    motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
                }
            } else { motor_runtime_latch_fault(MOTOR_FAULT_DEVICE); }
        }
    }
    board_leave_critical(mask);
}

void TIM1_BRK_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_BIF)!=0u) {
        motor_runtime_latch_fault(MOTOR_FAULT_DEVICE); /* 先停再清已处理的BIF，不恢复MOE。 */
        TIM1->DIER &= ~TIM_DIER_BIE; /* 故障锁存，无需反复服务持续低电平；BKE仍有效。 */
        TIM1->SR=~TIM_SR_BIF;
    }
}

void board_cancel_deadline(void)
{
    if (deadline_clock_ready) { TIM3->DIER &= ~TIM_DIER_CC1IE; }
}

bool board_schedule_deadline(uint32_t deadline_us)
{
    if (!deadline_clock_ready) { return false; }
    uint32_t now=board_get_time_us();
    uint32_t remaining=deadline_us-now;
    if (remaining==0u || remaining>=0x80000000u) { return false; }
    if (remaining>65535u) { remaining=65535u; }
    TIM3->DIER &= ~TIM_DIER_CC1IE;
    TIM3->SR=~TIM_SR_CC1IF;
    TIM3->CCR1=(TIM3->CNT+remaining)&65535u;
    TIM3->DIER |= TIM_DIER_CC1IE;
    /* 系统调用/IRQ屏蔽期间比较点已越过时不能等下一圈。 */
    if (board_get_time_us()-now>=remaining) { board_cancel_deadline(); return false; }
    return true;
}

void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_CC1IF)!=0u && (TIM3->DIER & TIM_DIER_CC1IE)!=0u) {
        TIM3->SR=~TIM_SR_CC1IF;
        motor_runtime_check_deadline(board_get_time_us());
    }
}
