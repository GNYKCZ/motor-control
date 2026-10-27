/** 本批固定参数：来自板卡连接、STM32 配置和 MT6701 协议；配置值不等于实测值。
 * 输入：编译期常量；输出：board、runtime、测试共用同一约束；不持有动态状态。 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_DEVICE_TIME_HZ 1000000u       /**< 设备时间：每 tick 为 1 us。 */
/* 外设时钟/ARR/RCR/比较值只由 m1_sampling.ioc 生成，不在本头复制第二套参数。 */
#define BOARD_ENCODER_TRANSFER_LIMIT_US 40u /**< 24 bit / 1.125 MHz 约 21.34 us，40 us 为软件上限。 */
#define BOARD_ENCODER_AGE_LIMIT_US 200u     /**< 从读取开始到 ADC 完成的最大年龄，未实测。 */
#define BOARD_SAMPLE_GAP_LIMIT_US 300u      /**< 正常约 99.94 us；超出即判输入失步。 */
#define BOARD_BOOT_SAMPLE_LIMIT_US 1000u    /**< 首个完整有效帧最多等待 1 ms，时间由设备计数。 */
#define BOARD_TELEMETRY_PERIOD_MS 100u      /**< 前台每 100 ms 尝试导出一次，不影响 ADC 频率。 */
#define BOARD_SERIAL_BUFFER_BYTES 1536u    /**< M3B2增加有限观测计时，仍小于capture单行2048B上限。 */
#define BOARD_FEEDBACK_DIVIDER 10u         /**< 每10个完整有效原始帧交接一次，非严格1kHz调度保证。 */
#define BOARD_FEEDBACK_GAP_LIMIT_US 3000u  /**< 抽帧实际中点间隔上限，不能覆盖原始流失帧。 */
#define BOARD_FEEDBACK_MAX_SPEED_RAD_S 100.0 /**< 仅估算先验：约955rpm，非实测速度保护。 */
#define BOARD_FEEDBACK_WINDOW 9u           /**< 9点窗口，正常跨8个约1ms间隔；真实dt另记。 */
#define BOARD_ADC_MAX_COUNT 4095u          /**< 12 bit ADC 原始值最大值，不代表电流已标定。 */
#define BOARD_ENCODER_MAX_COUNT 16383u     /**< 14 bit 单圈原始计数最大值。 */
/* 来自参考conf.h的标称参数，非本板实测；ADC满量程分母4096，非4095。
 * INA199A1名义50 V/V、20mΩ；无独立电流基准/母线ADC，电气正方向尚未验证。 */
#define BOARD_NOMINAL_ADC_REFERENCE_V 3.3f
#define BOARD_NOMINAL_SHUNT_OHM 0.02f
#define BOARD_NOMINAL_AMPLIFIER_GAIN 50.0f
#define BOARD_NOMINAL_A_PER_COUNT (BOARD_NOMINAL_ADC_REFERENCE_V / (4096.0f * BOARD_NOMINAL_SHUNT_OHM * BOARD_NOMINAL_AMPLIFIER_GAIN))

#endif
