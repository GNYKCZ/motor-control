/** 本批固定参数：来自板卡连接、STM32 配置和 MT6701 协议；配置值不等于实测值。
 * 输入：编译期常量；输出：board、runtime、测试共用同一约束；不持有动态状态。 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_DEVICE_TIME_HZ 1000000u       /**< 设备时间：每 tick 为 1 us。 */
#define BOARD_TIMER_CLOCK_HZ 72000000u      /**< APB 定时器输入时钟，需板端确认。 */
#define BOARD_SAMPLE_TIMER_RELOAD 1799u     /**< 保留参考 TIM1 ARR；中心对齐半周期为 ARR tick。 */
#define BOARD_SAMPLE_TIMER_REPEAT 3u        /**< 每 4 个溢出/下溢边界产生一次更新。 */
#define BOARD_ENCODER_START_DELAY_US 50u    /**< 更新后 50 us 读取角度，为下拍 ADC 留余量。 */
#define BOARD_ENCODER_TRANSFER_LIMIT_US 40u /**< 24 bit / 1.125 MHz 约 21.34 us，40 us 为软件上限。 */
#define BOARD_ENCODER_AGE_LIMIT_US 200u     /**< 从读取开始到 ADC 完成的最大年龄，未实测。 */
#define BOARD_SAMPLE_GAP_LIMIT_US 300u      /**< 正常约 99.94 us；超出即判输入失步。 */
#define BOARD_BOOT_SAMPLE_LIMIT_US 1000u    /**< 首个完整有效帧最多等待 1 ms，时间由设备计数。 */
#define BOARD_TELEMETRY_PERIOD_MS 100u      /**< 前台每 100 ms 尝试导出一次，不影响 ADC 频率。 */
#define BOARD_SERIAL_BUFFER_BYTES 256u     /**< 单行导出容量，静态分配，无动态内存。 */
#define BOARD_ADC_MAX_COUNT 4095u          /**< 12 bit ADC 原始值最大值，不代表电流已标定。 */
#define BOARD_ENCODER_MAX_COUNT 16383u     /**< 14 bit 单圈原始计数最大值。 */

#endif
