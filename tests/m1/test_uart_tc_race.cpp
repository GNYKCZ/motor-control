/** 编译同一生产board_uart.c，使用W0C寄存器模型复现RV-001，非重写ISR。 */
#include "linkage.h"
#include "usart.h"
#include <cstdio>
#include <cstdlib>
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr,"FAIL RV-001 %s:%d: %s\n",__FILE__,__LINE__,#x); std::exit(1); } } while (0)
uint32_t uart_status, uart_data, sr_writes;
bool arrival_pending;
uint8_t arrival_byte;
ModelUsart model_usart;
static uint32_t irq_mask, now_us, disables;
uint32_t board_enter_critical(void) { uint32_t old=irq_mask; irq_mask=1u; return old; }
void board_leave_critical(uint32_t old) { irq_mask=old; }
uint32_t board_get_time_us(void) { return now_us; }
void board_disable_output(void) { ++disables; }

static void finish_one_byte_tx(void)
{
    CHECK(board_start_serial_write("x",1u));
    uart_status|=USART_SR_TXE;
    USART2_IRQHandler();
    CHECK(board_serial_is_busy()); /* 写DR还不是TC。 */
    CHECK((uart_status & USART_SR_TC)==0u); /* 上次保留TC由SR读/DR写清除。 */
    uart_status=USART_SR_TC;
}

int main(void)
{
    const char stop[]="P1C1,00000002,02,1A67\r\n";
    for (unsigned overlap=0u;overlap<23u;++overlap) {
        CHECK(!board_serial_is_busy());
        uart_status=0u; uart_data=0u; sr_writes=0u; arrival_pending=false;
        model_usart.CR1=0u; model_usart.CR3=0u; now_us=0u;
        motor_runtime_init(); command_init(); motor_runtime_finish_boot(true); board_start_serial_receive();
        uint32_t before=disables;
        for (unsigned i=0u;i<23u;++i) {
            now_us+=100u;
            if (i==overlap) {
                finish_one_byte_tx();
                arrival_byte=(uint8_t)stop[i]; arrival_pending=true;
                USART2_IRQHandler();
                CHECK(!board_serial_is_busy() && !(model_usart.CR1 & USART_CR1_TCIE));
                /* 旧版SR读改写在模型内部注入后写0丢RXNE；新版无SR写，在IRQ退出处到达。 */
                if (arrival_pending) { deliver_arrival(); }
                if (uart_status & USART_SR_RXNE) { USART2_IRQHandler(); }
            } else {
                uart_data=(uint8_t)stop[i]; uart_status|=USART_SR_RXNE; USART2_IRQHandler();
            }
        }
        CHECK(disables==before+1u); /* 每个STOP字节与TC重叠仍完整执行一次，不靠前台poll。 */
        CHECK(sr_writes==0u); /* 不存在能用陈旧RXNE=0覆盖新字节的SR写入。 */
        CHECK((uart_status & USART_SR_TC)!=0u); /* 关TCIE后保留TC不会影响接收。 */
        finish_one_byte_tx(); USART2_IRQHandler(); CHECK(!board_serial_is_busy());
    }
    std::puts("PASS RV-001: same production ISR, W0C/read-write arrival model, all 23 STOP-byte/TC overlaps, TC retention and next-TX clearing");
    return 0;
}
