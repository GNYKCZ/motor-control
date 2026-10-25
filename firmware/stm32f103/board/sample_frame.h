/** 完整帧契约。输入来自 ADC 双完成和 SPI/DMA 完成，输出交给 runtime。
 * 本模块只做纯数据校验，不读寄存器；帧对象由 board 拥有，函数不保留传入指针。
 * 调用者负责在有界临界区内发布/复制整个对象，不能逐字段跨上下文读取。 */
#ifndef SAMPLE_FRAME_H
#define SAMPLE_FRAME_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t sequence;             /**< 完整 DMA 接收次数，含校验失败的帧，模 2^32。 */
    uint32_t read_started_us;      /**< CS 拉低时的设备时间，不是传感器采样时刻。 */
    uint32_t read_completed_us;    /**< DMA 完成被处理时的设备时间，含 IRQ 延迟。 */
    uint16_t raw_angle_count;      /**< 0..16383，未经方向/零位校正的机械角计数。 */
    uint8_t magnetic_status;       /**< Mg[3:0] 原样保留；本批只接受全部为 0。 */
    bool is_valid;                /**< 长度、传输时长、CRC、磁场状态均合格。 */
} board_encoder_frame_t;

typedef struct {
    uint32_t sequence;             /**< 完整双 ADC 发布序号，模 2^32。 */
    uint32_t current_completed_us; /**< 双 ADC 完成被处理时间，非精确采样沿。 */
    uint16_t raw_current_u_count;  /**< ADC1/PA0 原始 12 bit 值，未换算为 A。 */
    uint16_t raw_current_v_count;  /**< ADC2/PA1 原始 12 bit 值，未换算为 A。 */
    board_encoder_frame_t encoder;/**< 同一次完整发布的编码器帧副本。 */
    bool is_valid;                /**< 双 ADC 范围、编码器完整性和年龄均通过。 */
} motor_sample_t;

/** 解析 24 bit SSI 包。bytes 必须指向至少 completed_bytes 字节；允许 NULL，结果无效。
 * 只接受 completed_bytes==3；序号/微秒时间由调用者提供，持续时间须 <=40 us。
 * 返回完整值对象，失败仍保留时间/序号，is_valid=false；不写调用者内存/全局状态。 */
board_encoder_frame_t board_decode_encoder_frame(const uint8_t *bytes,
    uint32_t completed_bytes, uint32_t sequence, uint32_t read_started_us,
    uint32_t read_completed_us);

/** 从两个已完成的 ADC 值和一个稳定编码器副本构造帧。
 * raw_current_u_count/raw_current_v_count 为 0..4095；completed_us 为 1 MHz 32 bit 设备计数。
 * encoder 可 NULL（输出无效）；读取开始到 ADC 完成须 <=200 us。
 * 返回值不保留指针，不修改输入；调用者仅在双 ADC 确实完成时调用。 */
motor_sample_t board_make_sample(uint32_t sequence, uint32_t completed_us,
    uint16_t raw_current_u_count, uint16_t raw_current_v_count, const board_encoder_frame_t *encoder);

/** 判断 newer 是否在 older 之后；只接受模差 [1,2^31)，相同/倒序返回 false。
 * 无状态，适用于序号和设备时间，跨越半周期以上的情况视为无效。 */
bool board_sequence_is_newer(uint32_t newer, uint32_t older);
#endif
