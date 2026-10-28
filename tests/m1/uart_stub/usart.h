/** 主机UART寄存器替身；真实board_uart.c不变，不模拟电气波形或NVIC延迟。 */
#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { volatile uint32_t SR, DR, CR1, CR2, CR3; } test_usart_t;
extern test_usart_t test_usart;
#define USART2 (&test_usart)
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
void USART2_IRQHandler(void);
#endif
