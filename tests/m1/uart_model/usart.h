/** RV-001寄存器语义模型：SR写0清RXNE/TC，SR读改写之间可注入硬件新RX。
 * 这是主机受控交错，不模拟CPU指令时长、NVIC或物理串口。 */
#ifndef TEST_UART_MODEL_H
#define TEST_UART_MODEL_H
#include <stdint.h>
#define USART_SR_PE (1u<<0)
#define USART_SR_FE (1u<<1)
#define USART_SR_NE (1u<<2)
#define USART_SR_ORE (1u<<3)
#define USART_SR_RXNE (1u<<5)
#define USART_SR_TC (1u<<6)
#define USART_SR_TXE (1u<<7)
#define USART_CR1_RXNEIE (1u<<5)
#define USART_CR1_TCIE (1u<<6)
#define USART_CR1_TXEIE (1u<<7)
#define USART_CR1_PEIE (1u<<8)
#define USART_CR3_EIE (1u<<0)
extern uint32_t uart_status, uart_data, sr_writes;
extern bool arrival_pending;
extern uint8_t arrival_byte;
inline void deliver_arrival()
{
    uart_data=arrival_byte; uart_status|=USART_SR_RXNE; arrival_pending=false;
}
struct StatusRegister {
    operator uint32_t() const { return uart_status; }
    StatusRegister &operator=(uint32_t value) {
        ++sr_writes;
        if (arrival_pending) { deliver_arrival(); }
        /* RXNE/TC写1保持，写0清；其他位不在本竞态模型范围。 */
        uart_status &= value | ~(USART_SR_RXNE|USART_SR_TC);
        return *this;
    }
    StatusRegister &operator&=(uint32_t bits) {
        uint32_t stale=uart_status; /* 模拟读指令已经看到RXNE=0。 */
        return operator=(stale & bits); /* 新字节在随后写指令前由硬件到达。 */
    }
};
struct DataRegister {
    operator uint32_t() const {
        uint32_t value=uart_data;
        uart_status &= ~(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE);
        return value;
    }
    DataRegister &operator=(uint32_t value) {
        uart_data=value;
        uart_status &= ~USART_SR_TC; /* 生产ISR已经先读SR，随后写DR清TC。 */
        return *this;
    }
};
struct ModelUsart { StatusRegister SR; DataRegister DR; uint32_t CR1, CR2, CR3; };
extern ModelUsart model_usart;
#define USART2 (&model_usart)
extern "C" void USART2_IRQHandler(void);
#endif
