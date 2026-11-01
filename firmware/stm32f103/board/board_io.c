/** STM32F103C8 板级适配：使用生成的外设句柄，拥有 DMA 缓冲、设备时间和 UART 借用状态。
 * 完成事件进入 runtime；SPI 接收缓冲永不直接向前台公开，前台只读 runtime 快照。
 * GPIO/PWM门控由board_pwm唯一拥有；本地按键受限profile，无法控制驱动 EN/nSLEEP。 */
#include "board_io.h"
#include "board_config.h"
#include "sample_frame.h"
#include "motor_runtime.h"
#include "main.h"
#include "adc.h"
#include <stddef.h>

static volatile uint8_t encoder_receive_bytes[3];/**< DMA 写入的硬件内存；volatile 强制实际读取，不承担帧一致性。 */
static const uint8_t encoder_clock_bytes[3] = {0u, 0u, 0u}; /**< 仅用于产生 24 个时钟。 */
static board_encoder_frame_t completed_encoder; /**< 只在 RX TC 之后整体替换。 */
static uint32_t encoder_sequence;       /**< DMA 完成序号，校验失败也递增。 */
static uint32_t sample_sequence;        /**< 完整双 ADC 序号。 */
static uint32_t encoder_started_us;     /**< 当前 DMA 的 CS 下降沿附近设备时间。 */
static volatile bool encoder_is_busy;   /**< TIM2 开始，RX TC/异常结束；短临界区保护。 */
static volatile uint32_t device_time_high_us; /**< TIM4 溢出的高位，模 2^32。 */

uint32_t board_enter_critical(void)
{
    uint32_t previous_mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return previous_mask;
}

void board_leave_critical(uint32_t previous_mask)
{
    __DMB();
    __set_PRIMASK(previous_mask);
}

uint32_t board_get_time_us(void)
{
    uint32_t previous_mask = board_enter_critical();
    uint32_t high_us = device_time_high_us;
    uint32_t low_us = TIM4->CNT;
    if ((TIM4->SR & TIM_SR_UIF) != 0u) {
        /* 溢出 IRQ 尚未处理时补一次高位；不清 UIF，交给唯一 IRQ 写入者处理。
         * 需要 IRQ 最长屏蔽小于 65.536 ms；本批不宣称已测得实时预算。 */
        high_us += 65536u;
        low_us = TIM4->CNT;
    }
    board_leave_critical(previous_mask);
    return high_us + low_us;
}

/** 生成初始化完成后的运行准备；不再配置时钟/引脚/ADC 通道或定时器参数。
 * 校准是上电运行操作；清初始化 UG 的状态位再启动 TIM4 时间基准，避免伪溢出。
 * SPI HAL_Init 不置 SPE，board 在实际 DMA 事务前启用 SPI；无 PWM 通道启动。 */
bool board_initialize_devices(void)
{
    board_disable_output();
    board_initialize_output();
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK
        || HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
        return false;
    }
    SPI1->CR1 |= SPI_CR1_SPE;
    TIM1->SR = 0u;
    TIM2->SR = 0u;
    TIM4->SR = 0u;
    TIM4->CNT = 0u;
    TIM4->DIER = TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    board_start_serial_receive();
    return true;
}

bool board_start_sampling(void)
{
    board_disable_output();
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK
        || HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        return false;
    }
    ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    TIM2->CNT = 0u;
    TIM2->DIER = TIM_DIER_CC1IE;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM1->CNT = 0u;
    TIM1->CR1 |= TIM_CR1_CEN;
    return true;
}

/** TIM2 比较事件启动一帧；忙/异常不等待、不覆写半帧，锁存故障。
 * 短临界区防止 ADC 读取 completed_encoder 时被发布打断；DMA 私有缓冲无读者。 */
static void board_start_encoder_read(void)
{
    uint32_t previous_mask = board_enter_critical();
    if (encoder_is_busy) {
        motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
        board_leave_critical(previous_mask);
        return;
    }
    encoder_is_busy = true;
    SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    DMA1_Channel2->CCR = 0u;
    DMA1_Channel3->CCR = 0u;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel2->CMAR = (uint32_t)encoder_receive_bytes;
    DMA1_Channel2->CNDTR = 3u;
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_TEIE;
    DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel3->CMAR = (uint32_t)encoder_clock_bytes;
    DMA1_Channel3->CNDTR = 3u;
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
    encoder_started_us = board_get_time_us();
    GPIOA->BRR = GPIO_PIN_4;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    DMA1_Channel3->CCR |= DMA_CCR_EN;
    __DMB();
    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
    board_leave_critical(previous_mask);
}

/** SPI RX DMA 完成：仅处理 TC/TE，拒绝半帧；无 BSY 轮询或 HAL 超时等待。
 * 关闭 DMA 后再解析；CS 在下个 ADC 事件（距完成有余量）拉高，满足 SSI 尾保持。
 * ADC 的高优先级可能先于本 IRQ，此时只能看到上一完整帧并按序号/年龄拒绝。 */
void DMA1_Channel2_IRQHandler(void)
{
    uint32_t flags = DMA1->ISR;
    if ((flags & (DMA_ISR_TCIF2 | DMA_ISR_TEIF2)) == 0u) {
        return;
    }
    uint32_t previous_mask = board_enter_critical();
    SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    bool is_complete = encoder_is_busy && (flags & DMA_ISR_TCIF2) != 0u
        && (flags & DMA_ISR_TEIF2) == 0u && (DMA1->ISR & DMA_ISR_TEIF3) == 0u
        && DMA1_Channel2->CNDTR == 0u && (SPI1->SR & (SPI_SR_BSY | SPI_SR_OVR | SPI_SR_MODF)) == 0u;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    encoder_sequence++;
    __DMB();
    /* 完成事件+关闭 DMA+临界区负责一致性；volatile 只保证读取硬件刚写入的字节。 */
    uint8_t finished_bytes[3] = {encoder_receive_bytes[0], encoder_receive_bytes[1], encoder_receive_bytes[2]};
    completed_encoder = board_decode_encoder_frame(finished_bytes,
        is_complete ? 3u : 0u, encoder_sequence, encoder_started_us, board_get_time_us());
    encoder_is_busy = false;
    if (!is_complete) {
        motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
    }
    board_leave_critical(previous_mask);
}

/** 双 ADC IRQ：两个 JEOC 都存在才读取 JDR1 并发布一个完整包。
 * 只调用有界整数校验/runtime；不做 FOC，不格式化、不发送、不等待另一路 ADC。 */
void ADC1_2_IRQHandler(void)
{
    bool both_are_complete = (ADC1->SR & ADC_SR_JEOC) != 0u && (ADC2->SR & ADC_SR_JEOC) != 0u;
    uint32_t completed_us = board_get_time_us();
    ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    if (!both_are_complete) {
        motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
        return;
    }
    GPIOA->BSRR = GPIO_PIN_4;
    if ((GPIOB->IDR & GPIO_PIN_12) == 0u) {
        motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
    }
    /* DMA 发布也持同一临界区；复制完成对象，不接触正由 DMA 改写的三字节缓冲。 */
    uint32_t previous_mask = board_enter_critical();
    board_encoder_frame_t encoder = completed_encoder;
    board_leave_critical(previous_mask);
    sample_sequence++;
    motor_sample_t sample = board_make_sample(sample_sequence, completed_us,
        (uint16_t)ADC1->JDR1, (uint16_t)ADC2->JDR1, &encoder);
    motor_runtime_handle_sample(&sample);
}

/** TIM2 每个 TRGO 周期仅在 CCR1 到达时启动一次 SSI；清标志后执行有界启动。 */
void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_CC1IF) != 0u) {
        TIM2->SR = ~TIM_SR_CC1IF;
        board_start_encoder_read();
    }
}

/** TIM4 每 65.536 ms 扩展微秒计数；与读取时的 UIF 补偿配合，不改采样时间。 */
void TIM4_IRQHandler(void)
{
    if ((TIM4->SR & TIM_SR_UIF) != 0u) {
        TIM4->SR = ~TIM_SR_UIF;
        device_time_high_us += 65536u;
    }
}
