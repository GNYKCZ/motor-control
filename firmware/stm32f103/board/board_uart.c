/** USART2唯一寄存器入口：每次IRQ最多处理1个RX和1个TX事件，不等待或格式化。
 * 拆出小型硬件适配，使主机可用同一ISR验证SR/DR错误与TX借用交错。 */
#include "board_io.h"
#include "board_config.h"
#include "command.h"
#include "usart.h"
#include <stddef.h>

static volatile bool serial_is_busy;
static const char *serial_bytes;
static uint32_t serial_length, serial_position;

void board_start_serial_receive(void)
{
    /* 初始化时排掉旧SR/DR，随后才启用RX；运行中错误也必须先读SR再读DR清除。 */
    volatile uint32_t status=USART2->SR;
    volatile uint32_t data=USART2->DR;
    (void)status; (void)data;
    USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_PEIE;
    USART2->CR3 |= USART_CR3_EIE;
}

bool board_serial_is_busy(void) { return serial_is_busy; }

bool board_start_serial_write(const char *bytes, uint32_t length)
{
    if (bytes==NULL || length==0u || length>BOARD_SERIAL_BUFFER_BYTES) { return false; }
    uint32_t mask=board_enter_critical();
    if (serial_is_busy) { board_leave_critical(mask); return false; }
    serial_bytes=bytes; serial_length=length; serial_position=0u; serial_is_busy=true;
    USART2->CR1 |= USART_CR1_TXEIE;
    board_leave_critical(mask);
    return true;
}

void USART2_IRQHandler(void)
{
    uint32_t status=USART2->SR;
    uint32_t errors=status & (USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE);
    if ((status & USART_SR_RXNE)!=0u || errors!=0u) {
        uint8_t byte=(uint8_t)USART2->DR; /* SR在前、DR在后；即使错误也必须读DR。 */
        if (errors!=0u) { command_receive_error(); }
        else { command_receive_byte(byte,board_get_time_us()); }
    }
    if ((USART2->CR1 & USART_CR1_TXEIE)!=0u && (status & USART_SR_TXE)!=0u) {
        USART2->DR=(uint8_t)serial_bytes[serial_position++];
        if (serial_position==serial_length) {
            USART2->CR1=(USART2->CR1 & ~USART_CR1_TXEIE)|USART_CR1_TCIE;
        }
    } else if ((USART2->CR1 & USART_CR1_TCIE)!=0u && (status & USART_SR_TC)!=0u) {
        USART2->CR1 &= ~USART_CR1_TCIE;
        /* RV-001：只关TCIE，不对SR读改写。陈旧SR的RXNE=0写回会吞掉新到达字节。
         * TC保留不再触发IRQ；下一次TXE路径先读SR、再写DR，由硬件清TC。 */
        serial_is_busy=false;
    }
}
