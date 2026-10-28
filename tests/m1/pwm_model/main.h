/** 主机寄存器语义：preload/自然UEV、SR写0清、异步Break；不模拟实际电气波形或WCET。 */
#ifndef PWM_MODEL_MAIN_H
#define PWM_MODEL_MAIN_H
#include <stdint.h>
enum { R_OTHER, R_BDTR, R_CCR1, R_CCR2, R_CCR3, R_CCR4, R_SR1, R_CRH, R_CCER, R_SR3, R_CCR31, R_EGR };
void pwm_model_access(unsigned id, bool write);
void pwm_model_written(unsigned id, uint32_t value);
struct PwmRegister {
    uint32_t value; unsigned id;
    explicit PwmRegister(unsigned kind=R_OTHER):value(0),id(kind) {}
    operator uint32_t() const { pwm_model_access(id,false); return value; }
    PwmRegister &operator=(uint32_t v) {
        pwm_model_access(id,true);
        value=(id==R_SR1 || id==R_SR3) ? value & v : v;
        pwm_model_written(id,value); return *this;
    }
    PwmRegister &operator|=(uint32_t v) { uint32_t old=*this; return operator=(old|v); }
    PwmRegister &operator&=(uint32_t v) { uint32_t old=*this; return operator=(old&v); }
};
struct PwmTimer {
    PwmRegister BDTR,CCR1,CCR2,CCR3,CCR4,SR,CCER,CR1,DIER,CNT,ARR,EGR;
    explicit PwmTimer(bool first):BDTR(first?R_BDTR:R_OTHER),CCR1(first?R_CCR1:R_CCR31),
        CCR2(R_CCR2),CCR3(R_CCR3),CCR4(R_CCR4),SR(first?R_SR1:R_SR3),CCER(R_CCER),EGR(R_EGR) {}
};
struct PwmGpio { PwmRegister CRH,BRR,IDR; PwmGpio():CRH(R_CRH) {} };
extern PwmTimer pwm_tim1,pwm_tim3;
extern PwmGpio pwm_gpioa,pwm_gpiob;
#define TIM1 (&pwm_tim1)
#define TIM3 (&pwm_tim3)
#define GPIOA (&pwm_gpioa)
#define GPIOB (&pwm_gpiob)
#define GPIO_PIN_8 (1u<<8)
#define GPIO_PIN_9 (1u<<9)
#define GPIO_PIN_10 (1u<<10)
#define GPIO_PIN_12 (1u<<12)
#define TIM_BDTR_MOE (1u<<15)
#define TIM_BDTR_AOE (1u<<14)
#define TIM_BDTR_BKP (1u<<13)
#define TIM_BDTR_BKE (1u<<12)
#define TIM_BDTR_OSSR (1u<<11)
#define TIM_BDTR_OSSI (1u<<10)
#define TIM_CCER_CC1E (1u<<0)
#define TIM_CCER_CC2E (1u<<4)
#define TIM_CCER_CC3E (1u<<8)
#define TIM_DIER_UIE (1u<<0)
#define TIM_DIER_CC1IE (1u<<1)
#define TIM_DIER_CC4IE (1u<<4)
#define TIM_DIER_BIE (1u<<7)
#define TIM_SR_UIF (1u<<0)
#define TIM_SR_CC1IF (1u<<1)
#define TIM_SR_CC4IF (1u<<4)
#define TIM_SR_BIF (1u<<7)
#define TIM_CR1_CEN (1u<<0)
#define TIM_CR1_DIR (1u<<4)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM1_CLK_ENABLE() ((void)0)
extern "C" { void TIM1_UP_IRQHandler(void); void TIM1_CC_IRQHandler(void); void TIM1_BRK_IRQHandler(void); void TIM3_IRQHandler(void); }
#endif
