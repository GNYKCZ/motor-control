/** 原始采样→名义物理量与零偏候选统计；无HAL、串口或内部可变状态。
 * 只估计两相A与机械单圈rad，不产生电角度、校准通过或任何输出使能。 */
#ifndef SENSOR_INPUT_H
#define SENSOR_INPUT_H
#include <stdbool.h>
#include <stdint.h>
#define SENSOR_OFFSET_SAMPLE_COUNT 512u /**< 一次候选批次的完整连续帧数，非校准可信度阈值。 */

typedef struct {
    float estimated_offset_u_count; /**< U相零偏候选，ADC count，有限[0,4095]。 */
    float estimated_offset_v_count; /**< V相候选，允许平均后的非整数count。 */
    float nominal_A_per_count;      /**< 名义A/count，有限正值；不代表独立电流基准校准。 */
} sensor_input_config_t;

typedef struct {
    float nominal_current_u_A; /**< (raw_u-offset_u)*nominal_A_per_count，A，未经实物精度验证。 */
    float nominal_current_v_A; /**< V相名义A，未定义相线正负方向的板端确认。 */
    float mechanical_angle_rad;/**< raw_angle*2*pi/16384，机械单圈[0,2*pi)，未经方向/零位校正。 */
    bool is_valid;             /**< 数学转换有效，不能据此称calibration_verified或用于换相。 */
} sensor_input_t;

typedef struct {
    uint32_t accepted_count; /**< 已计入完整帧0..512，满批冻结。 */
    uint32_t sum_u_count;    /**< U累计原始count，最大512*4095，不使用浮点或长循环。 */
    uint32_t sum_v_count;    /**< V累计count。 */
    uint16_t minimum_u_count;/**< 本批U最小count；accepted_count=0时无物理含义。 */
    uint16_t maximum_u_count;/**< 本批U最大count。 */
    uint16_t minimum_v_count;/**< 本批V最小count。 */
    uint16_t maximum_v_count;/**< 本批V最大count。 */
} sensor_offset_t;

/** 显式初始化一批候选统计；NULL返回false，无内部状态，不自动重开被中断批次。 */
bool sensor_offset_init(sensor_offset_t *offset);
/** 累计一个已由调用者验证连续性/状态/时序的完整帧，O(1)整数操作。
 * frame_valid=false、raw越界、状态不合法或已满512返回false且整个状态不变。
 * 不负责序号/故障锁存；调用者收到异常须冻结整批，不能跳过异常后继续补满。 */
bool sensor_offset_add(sensor_offset_t *offset, uint16_t raw_u, uint16_t raw_v, bool frame_valid);
/** 显式零偏/比例和完整有效原始12bit ADC、14bit角度→名义A/机械rad。
 * result独占且不与其他对象重叠；失败返回false，非空输出清零无效，配置按值不修改。
 * 不要求候选已验证；调用者必须另标来源和calibration_verified=false，不能将结果用于闭环。 */
bool sensor_input_convert(uint16_t raw_u, uint16_t raw_v, uint16_t raw_angle,
    bool frame_valid, sensor_input_config_t config, sensor_input_t *result);
#endif
