/** 纯数据帧构造与 SSI 校验。由 board 完成事件调用，主机测试调用同一实现。
 * 不持有状态，不阻塞、不访问 HAL；序号和时间的发布由 board 所有。 */
#include "sample_frame.h"
#include "board_config.h"
#include <stddef.h>

/** 按 MT6701 手册 x^6+x+1 计算 18 bit 数据余数；初值 0，MSB 先入，无最终异或。
 * data 对应 D[13:0] 拼接 Mg[3:0]，反馈位对应多项式最高项。 */
static uint8_t board_calculate_encoder_crc(uint32_t data)
{
    uint8_t remainder = 0u;
    for (int bit_index = 17; bit_index >= 0; --bit_index) {
        uint8_t feedback = (uint8_t)(((data >> bit_index) & 1u) ^ (remainder >> 5));
        remainder = (uint8_t)((remainder << 1) & 0x3fu);
        if (feedback != 0u) {
            remainder ^= 0x03u;
        }
    }
    return remainder;
}

bool board_sequence_is_newer(uint32_t newer, uint32_t older)
{
    uint32_t difference = newer - older;
    return difference != 0u && difference < 0x80000000u;
}

board_encoder_frame_t board_decode_encoder_frame(const uint8_t *bytes,
    uint32_t completed_bytes, uint32_t sequence, uint32_t read_started_us,
    uint32_t read_completed_us)
{
    board_encoder_frame_t frame = {0};
    frame.sequence = sequence;
    frame.read_started_us = read_started_us;
    frame.read_completed_us = read_completed_us;
    if (bytes == NULL || completed_bytes != 3u) {
        return frame;
    }
    uint32_t packet = ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[2];
    uint32_t data = packet >> 6;
    frame.raw_angle_count = (uint16_t)(packet >> 10);
    frame.magnetic_status = (uint8_t)(data & 0x0fu);
    /* 模减法允许一次 32 bit 时间回绕；倒序会形成巨大差值，被上限拒绝。 */
    frame.is_valid = read_completed_us - read_started_us <= BOARD_ENCODER_TRANSFER_LIMIT_US
        && board_calculate_encoder_crc(data) == (packet & 0x3fu)
        && frame.magnetic_status == 0u;
    return frame;
}

motor_sample_t board_make_sample(uint32_t sequence, uint32_t completed_us,
    uint16_t raw_current_u_count, uint16_t raw_current_v_count, const board_encoder_frame_t *encoder)
{
    motor_sample_t sample = {0};
    sample.sequence = sequence;
    sample.current_completed_us = completed_us;
    sample.raw_current_u_count = raw_current_u_count;
    sample.raw_current_v_count = raw_current_v_count;
    if (encoder != NULL) {
        sample.encoder = *encoder;
        uint32_t read_duration_us = encoder->read_completed_us - encoder->read_started_us;
        uint32_t age_upper_us = completed_us - encoder->read_started_us;
        sample.is_valid = encoder->is_valid
            && encoder->raw_angle_count <= BOARD_ENCODER_MAX_COUNT
            && encoder->magnetic_status == 0u
            && read_duration_us <= BOARD_ENCODER_TRANSFER_LIMIT_US
            && age_upper_us >= read_duration_us && age_upper_us <= BOARD_ENCODER_AGE_LIMIT_US
            && raw_current_u_count <= BOARD_ADC_MAX_COUNT && raw_current_v_count <= BOARD_ADC_MAX_COUNT;
    }
    return sample;
}
