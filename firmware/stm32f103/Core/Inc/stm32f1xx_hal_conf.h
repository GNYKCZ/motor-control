/** 自编 HAL 配置：只启用本批真实调用的组件；厂家实现保持原样。
 * 输入为编译期宏，输出给厂家头文件；无状态，时钟值与 board_config 一致。 */
#ifndef STM32F1XX_HAL_CONF_H
#define STM32F1XX_HAL_CONF_H
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HSE_VALUE 8000000u            /**< 板卡外部晶振标称 8 MHz。 */
#define HSE_STARTUP_TIMEOUT 100u       /**< 仅启动阶段等待外部晶振，ms。 */
#define HSI_VALUE 8000000u            /**< STM32 内部振荡器标称值。 */
#define LSI_VALUE 40000u              /**< 未启用 LSI，仅满足 HAL 编译配置。 */
#define LSE_VALUE 32768u             /**< 未启用 LSE，仅满足 HAL 编译配置。 */
#define LSE_STARTUP_TIMEOUT 5000u    /**< LSE 配置上限，本批不启用。 */
#define VDD_VALUE 3300u              /**< 标称 3.3 V，不是实测值。 */
#define TICK_INT_PRIORITY 3u         /**< SysTick 低于采样/计时 IRQ。 */
#define USE_RTOS 0u                  /**< 无 RTOS。 */
#define PREFETCH_ENABLE 1u           /**< 72 MHz 启用预取。 */
#define USE_HAL_ADC_REGISTER_CALLBACKS 0u  /**< 使用静态回调约定。 */
#define USE_HAL_SPI_REGISTER_CALLBACKS 0u  /**< 运行 DMA 不走 HAL 阻塞回调。 */
#define USE_HAL_UART_REGISTER_CALLBACKS 0u /**< UART 自编 TXE/TC 路径。 */
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_uart.h"
/* 保留 HAL 的参数断言入口；本批运行契约在 runtime 中显式检查，不依赖此空宏。 */
#define assert_param(expression) ((void)0u)
#endif
