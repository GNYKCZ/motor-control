/** 唯一PWM/引脚门控所有者；生成配置负责初始化，运行期无UG、无UDIS、无逐相HAL启动。
 * 所有事务有界，保存/恢复PRIMASK。MOE0只停止调制，不代表固定高EN的功率桥高阻。 */
#include "board_io.h"
#include "motor_runtime.h"
#include "main.h"
#include "board_config.h"
#include "current_loop.h"
#include "diag_trace.h"

static uint32_t pending_generation;
#if BOARD_DQ_CURRENT_LOOP
static uint32_t pending_current_sequence,pending_sample_us;
#endif
static unsigned pending_scan_step;
static uint16_t largest_tick, window_margin;
static unsigned pending_phase; /**< 0无请求，1等零值UEV，4等单U励磁UEV；旧模式2等共同低窗、3等点提交。 */
static bool deadline_clock_ready;
static bool output_was_enabled; /**< 本代次是否真正开过MOE；Break会由硬件清位，不能用实时MOE判断停机阶段。 */
static bool stop_snapshot_needed; /**< 启动已发起但尚未通过校验：失败时也要留下停机配置快照。 */
#define PHASE_CHANNELS (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E)

/** 阶段配置快照：寄存器在此读取后交给记录模块，记录本身不访问设备。
 * CCR是预装寄存器值；该阶段是否已装入有效比较值由ccr_is_preload单独标注。 */
#if BOARD_DQ_VECTOR_DIAGNOSTIC
static void record_config(unsigned stage,uint32_t generation,uint8_t ccr_is_preload)
{
    diag_config_t config;
    config.generation=generation;
    config.cr1=(uint16_t)TIM1->CR1;
    config.cr2=(uint16_t)TIM1->CR2;
    config.ccmr1=(uint16_t)TIM1->CCMR1;
    config.ccmr2=(uint16_t)TIM1->CCMR2;
    config.ccer=(uint16_t)TIM1->CCER;
    config.bdtr=(uint16_t)TIM1->BDTR;
    config.psc=(uint16_t)TIM1->PSC;
    config.arr=(uint16_t)TIM1->ARR;
    config.rcr=(uint16_t)TIM1->RCR;
    config.ccr1=(uint16_t)TIM1->CCR1;
    config.ccr2=(uint16_t)TIM1->CCR2;
    config.ccr3=(uint16_t)TIM1->CCR3;
    config.gpioa_crh=GPIOA->CRH;
    config.afio_mapr=AFIO->MAPR;
    config.gpioa_idr=GPIOA->IDR;
    config.read_us=board_get_time_us();
    config.stage=(uint8_t)stage;
    config.ccr_is_preload=ccr_is_preload;
    diag_trace_config(&config);
}
#else
/** 非固定零矢量诊断构建不付寄存器快照成本；调用点保持同一形状。 */
#define record_config(stage,generation,preload) ((void)(stage),(void)(generation),(void)(preload))
#endif

void board_disable_output(void)
{
    uint32_t mask=board_enter_critical();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    bool had_request=pending_phase!=0u || (TIM1->BDTR & TIM_BDTR_MOE)!=0u;
    bool was_outputting=output_was_enabled; /* Break已由硬件清MOE，只有软件标志能证明曾输出。 */
    bool was_starting=stop_snapshot_needed; /* 启动已发起但在校验阶段被拒：同样要留快照。 */
    output_was_enabled=false;
    stop_snapshot_needed=false;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    if (had_request && deadline_clock_ready) { motor_runtime_record_gate(false,board_get_time_us()); }
    uint32_t stopped_generation=pending_generation;
    pending_phase=0u;
    TIM1->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC4IE);
    /* CCxE仍保持idle低，先接管GPIO再关通道，避免AF无驱动窗口。 */
    GPIOA->BRR=GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIOA->CRH=(GPIOA->CRH & ~0xfffu) | 0x222u;
    TIM1->CCER=0u;
    if ((had_request || was_outputting || was_starting) && deadline_clock_ready) {
        record_config(DIAG_STAGE_STOPPED,stopped_generation,1u);
    }
    board_leave_critical(mask);
}

void board_prepare_output_idle(void)
{
    TIM1->BDTR=(TIM1->BDTR & ~(TIM_BDTR_MOE | TIM_BDTR_AOE)) | TIM_BDTR_OSSI | TIM_BDTR_OSSR;
    TIM1->CCER=PHASE_CHANNELS; /* 三路OIS由CubeMX置低；此处MOE始终为0。 */
}

void board_initialize_output(void)
{
    output_was_enabled=false; /* 初始化不得沿用上一会话的“开过MOE”标志。 */
    stop_snapshot_needed=false;
    board_disable_output();
    TIM1->SR=~TIM_SR_BIF;
    TIM1->DIER |= TIM_DIER_BIE;
    TIM3->SR=0u;
    TIM3->CNT=0u;
    TIM3->DIER=0u;
    TIM3->CR1 |= TIM_CR1_CEN;
    deadline_clock_ready=true;
    record_config(DIAG_STAGE_IDLE,0u,0u); /* 启动初始化后的空闲配置基线。 */
}

bool board_output_is_ready(void)
{
    bool timing_ready;
#if BOARD_SINGLE_U
    timing_ready=TIM1->PSC==0u && TIM1->ARR==1800u && board_sampling_phase_is_ready();
#if BOARD_DQ_CURRENT_LOOP
    timing_ready=timing_ready && TIM1->RCR==39u && (TIM1->CR1 & TIM_CR1_CMS)==TIM_CR1_CMS
        && (TIM1->CCMR1 & (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE))==(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE)
        && (TIM1->CCMR2 & TIM_CCMR2_OC3PE)!=0u;
#endif
#else
    timing_ready=TIM1->PSC==9u && TIM1->ARR==1799u;
#endif
    return timing_ready && deadline_clock_ready && (GPIOB->IDR & GPIO_PIN_12)!=0u
        && (TIM1->SR & TIM_SR_BIF)==0u && (TIM1->CR1 & TIM_CR1_CEN)!=0u
        && (TIM1->BDTR & (TIM_BDTR_BKE | TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_AOE | TIM_BDTR_BKP))
            == (TIM_BDTR_BKE | TIM_BDTR_OSSI | TIM_BDTR_OSSR);
}

bool board_prepare_pulse(board_pwm_ticks_t ticks, uint32_t generation)
{
    /* 启动已发起：即使在本函数校验阶段被拒，后续停机也要留下能显示坏配置的快照。 */
    stop_snapshot_needed=true;
    uint32_t maximum=ticks.u;
    if (ticks.v>maximum) { maximum=ticks.v; }
    if (ticks.w>maximum) { maximum=ticks.w; }
    if (!board_output_is_ready() || ticks.margin_ticks==0u
        || maximum+2u*ticks.margin_ticks>=TIM1->ARR || TIM1->ARR!=(BOARD_SINGLE_U ? 1800u : 1799u)
        || (BOARD_SINGLE_U && (ticks.u!=900u || ticks.v!=(BOARD_DQ_CURRENT_LOOP ? 900u : 0u) || ticks.w!=(BOARD_DQ_CURRENT_LOOP ? 900u : 0u) || ticks.margin_ticks!=144u))) { return false; }
    stop_snapshot_needed=false; /* 这里的清场不是停机，不应产生STOPPED快照。 */
    board_disable_output();
    board_prepare_output_idle();
    TIM1->CCR1=BOARD_SINGLE_U ? 0u : ticks.u; TIM1->CCR2=BOARD_SINGLE_U ? 0u : ticks.v; TIM1->CCR3=BOARD_SINGLE_U ? 0u : ticks.w;
    largest_tick=(uint16_t)maximum; window_margin=ticks.margin_ticks;
    /* 在同一允许窗口的首个tick发比较事件，为IRQ进入/资格复核留出余量。
     * 不改变窗口边界；迟到仍拒绝，本次请求不等下一圈重试。 */
    TIM1->CCR4=maximum+ticks.margin_ticks+1u;
    pending_generation=generation;
    /* 写入间自然UEV可出现：MOE0。清旧UIF后只认下一次UEV，确保全部预装值已转移。 */
    TIM1->SR=~TIM_SR_UIF;
    pending_phase=1u;
    GPIOA->CRH=(GPIOA->CRH & ~0xfffu) | 0xaaau; /* 三路2MHz AF；此刻idle低。 */
    TIM1->DIER |= TIM_DIER_UIE;
    return true;
}

bool board_update_scan_point(board_pwm_ticks_t ticks, uint32_t generation, unsigned step)
{
    if (BOARD_SINGLE_U) { board_disable_output(); return false; }
    /* 调用者已持runtime锁。影子寄存器不同步写不是原子提交：检测写间UEV，不能洗掉标志。 */
    uint32_t high=(ticks.u==60u)+(ticks.v==60u)+(ticks.w==60u);
    if ((BOARD_ALIGNMENT_MODE && (step!=1u || ticks.u!=60u || ticks.v!=0u || ticks.w!=0u))
        || step==0u || step>=BOARD_SEQUENCE_POINTS || pending_phase!=0u || generation!=pending_generation
        || (ticks.u!=0u && ticks.u!=60u) || (ticks.v!=0u && ticks.v!=60u)
        || (ticks.w!=0u && ticks.w!=60u) || high==0u || high==3u
        || !board_output_is_ready() || !(TIM1->BDTR & TIM_BDTR_MOE)
        || TIM1->CNT<=494u) { board_disable_output(); return false; }
    uint32_t began=board_get_time_us();
    TIM1->SR=~TIM_SR_UIF;
    TIM1->CCR1=ticks.u; TIM1->CCR2=ticks.v; TIM1->CCR3=ticks.w;
    if ((TIM1->SR & TIM_SR_UIF)!=0u || TIM1->CNT<=494u
        || board_get_time_us()-began>=50u || !board_output_is_ready() || !(TIM1->BDTR & TIM_BDTR_MOE)) {
        board_disable_output(); return false;
    }
    pending_scan_step=step; pending_phase=3u;
    TIM1->DIER |= TIM_DIER_UIE;
    return true;
}

void TIM1_UP_IRQHandler(void)
{
    uint32_t mask=board_enter_critical();
    if ((TIM1->SR & TIM_SR_UIF)!=0u && (TIM1->DIER & TIM_DIER_UIE)!=0u) {
        TIM1->SR=~TIM_SR_UIF;
        TIM1->DIER &= ~TIM_DIER_UIE;
#if BOARD_SINGLE_U
#if BOARD_DQ_CURRENT_LOOP
        if (pending_phase==5u) {
            uint32_t event=0u;
            if (!board_get_update_time_us(&event) || !board_output_is_ready() || !(TIM1->BDTR & TIM_BDTR_MOE)
                || !motor_runtime_current_is_active(pending_generation) || event-pending_sample_us<740u
                || event-pending_sample_us>810u
                || !current_loop_applied(pending_generation,pending_current_sequence,event)) {
                motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
            } else { pending_phase=0u; }
        } else
#endif
        if (pending_phase==1u || pending_phase==4u) {
            uint32_t event_us=0u;
            if (!board_get_update_time_us(&event_us) || !board_output_is_ready()
                || !motor_runtime_start_is_valid_at(pending_generation,board_get_time_us_locked())) {
                motor_runtime_latch_fault((motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_TIME));
            } else if (pending_phase==1u) {
                /* 三路有效CCR已由本次UEV统一装入0。此时使能无励磁，再排下次硬件提交。 */
                TIM1->BDTR |= TIM_BDTR_MOE;
                output_was_enabled=true;
                if (!board_output_is_ready() || !(TIM1->BDTR & TIM_BDTR_MOE)) {
                    motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
                } else {
                    TIM1->CCR1=900u;
#if BOARD_DQ_CURRENT_LOOP
                    TIM1->CCR2=900u;TIM1->CCR3=900u; /* 启动为三路50%零电压矢量。 */
#endif
                    uint32_t checked_event=0u;
                    if (!board_get_update_time_us(&checked_event) || (TIM1->SR & TIM_SR_UIF)!=0u
                        || (int32_t)(checked_event-event_us)>4 || (int32_t)(checked_event-event_us)<-4) {
                        motor_runtime_latch_fault((motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_TIME));
                    } else { pending_phase=4u; TIM1->DIER |= TIM_DIER_UIE;
                        record_config(DIAG_STAGE_ZERO,pending_generation,1u); }
                }
            } else {
                uint32_t now=board_get_time_us_locked();
                if (!(TIM1->BDTR & TIM_BDTR_MOE) || !motor_runtime_commit_output_at(pending_generation,now)) {
                    motor_runtime_latch_fault((motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_INPUT));
                } else {
                    pending_phase=0u;
                    motor_runtime_record_gate(true,event_us); /* 实际励磁UEV时标，而非迟到IRQ时刻。 */
                    record_config(DIAG_STAGE_FIFTY,pending_generation,0u); /* 本次UEV已装入三路900。 */
                }
            }
        }
#else
        if (BOARD_SCAN_ENABLED && pending_phase==3u) {
            pending_phase=0u;
            if (!board_output_is_ready() || !(TIM1->BDTR & TIM_BDTR_MOE)
                || !motor_runtime_commit_scan_point(pending_generation,pending_scan_step,board_get_time_us())) {
                motor_runtime_latch_fault(MOTOR_FAULT_CALIBRATION);
            }
        } else if (pending_phase==1u) {
            pending_phase=2u;
            TIM1->SR=~TIM_SR_CC4IF;
            TIM1->DIER |= TIM_DIER_CC4IE;
        }
#endif
    }
    board_leave_critical(mask);
}

/** 开门前后均要求上数共同低窗，余量来自profile；迟到直接拒绝，不等待或重试。 */
#if !BOARD_SINGLE_U
static bool in_start_window(void)
{
    return (TIM1->CR1 & TIM_CR1_DIR)==0u && TIM1->CNT>largest_tick+window_margin
        && TIM1->CNT+window_margin<TIM1->ARR;
}
#endif
void TIM1_CC_IRQHandler(void)
{
    uint32_t mask=board_enter_critical();
    if ((TIM1->SR & TIM_SR_CC4IF)!=0u && (TIM1->DIER & TIM_DIER_CC4IE)!=0u) {
        TIM1->SR=~TIM_SR_CC4IF;
        TIM1->DIER &= ~TIM_DIER_CC4IE;
        if (pending_phase==2u) {
            pending_phase=0u;
#if BOARD_SINGLE_U
            /* 单U不使用CC4软件开门；仅由自然UEV提交，陈旧CC4不能启动。 */
            motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
#else
            uint32_t transaction_started=board_get_time_us();
            motor_fault_t reject=MOTOR_FAULT_DEVICE;
            /* 逗号表达式只标记当前检查阶段，保持原有短路顺序；不增加寄存器读取。 */
            if (board_output_is_ready()
                && (reject=(motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_WINDOW),in_start_window())
                && (reject=(motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_INPUT),motor_runtime_commit_output(pending_generation))
                && (reject=(motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_WINDOW),in_start_window())
                && (reject=(motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_TIME),!motor_runtime_output_deadline_reached(board_get_time_us()))
                && (!BOARD_SINGLE_U || board_get_time_us()-transaction_started<8u)) {
                TIM1->BDTR |= TIM_BDTR_MOE;
                uint32_t opened_us=board_get_time_us();
                motor_runtime_record_gate(true,opened_us);
                /* 短Break可能发生在最终检查与MOE写之间，低窗内再次检查，禁止陈旧事务重开。 */
                bool invalid_driver=!board_output_is_ready();
                bool invalid_window=invalid_driver || !in_start_window()
                    || (BOARD_SINGLE_U && board_get_time_us()-transaction_started>=8u);
                uint32_t checked_us=board_get_time_us();
                bool expired=motor_runtime_output_deadline_reached(checked_us);
                if (expired || invalid_window) {
                    /* 先清MOE，避免状态清理/时标记录挤占共同低窗。 */
                    TIM1->BDTR &= ~TIM_BDTR_MOE;
                    motor_runtime_record_gate(false,board_get_time_us());
                }
                if (expired) {
                    motor_runtime_check_deadline(checked_us);
                } else if (invalid_window) {
                    motor_runtime_latch_fault(invalid_driver ? MOTOR_FAULT_DEVICE
                        : (motor_fault_t)(MOTOR_FAULT_DEVICE|MOTOR_FAULT_START_WINDOW));
                }
            } else { motor_runtime_latch_fault(reject); }
#endif
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

#if BOARD_DQ_CURRENT_LOOP
bool board_submit_current(board_pwm_ticks_t ticks,uint32_t generation,uint32_t sequence,uint32_t sample_us)
{
    uint32_t event=0u;uint32_t began=board_get_time_us_locked();
    /* ADC在225us附近完成。只在本UEV后至500us前写入；距离下一UEV至少500us。
     * 三个APB写无等待/调用；20us复核是故障检测，不能靠放宽它支持迟到提交。 */
    if (pending_phase || generation!=pending_generation || !board_output_is_ready()
        || !(TIM1->BDTR & TIM_BDTR_MOE) || !motor_runtime_current_sample_matches(generation,sequence,sample_us)
        || !board_get_update_time_us(&event) || began-event<225u || began-event>500u
        || began-sample_us>450u || ticks.u<848u || ticks.u>952u || ticks.v<848u || ticks.v>952u
        || ticks.w<848u || ticks.w>952u) { return false; }
    TIM1->SR=~TIM_SR_UIF;
    TIM1->CCR1=ticks.u;TIM1->CCR2=ticks.v;TIM1->CCR3=ticks.w;
    if ((TIM1->SR & TIM_SR_UIF) || board_get_time_us_locked()-began>20u
        || !board_output_is_ready() || !(TIM1->BDTR & TIM_BDTR_MOE)) { board_disable_output();return false; }
    pending_current_sequence=sequence;pending_sample_us=sample_us;pending_phase=5u;TIM1->DIER |= TIM_DIER_UIE;
    return true;
}
#endif
