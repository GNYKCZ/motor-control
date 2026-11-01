#ifndef RESEARCH_BIAS_COMPENSATION_H
#define RESEARCH_BIAS_COMPENSATION_H
#include <stdint.h>
/* 研究候选，不接入现有固件。b0包含名义1.65V换算后的全部固定偏差。
 * certified必须来自独立标定有效性检查，不能由闭环“接近零”生成。
 * error_A包含偏置残差、噪声、增益误差；此处另外加入半个ADC LSB。
 */
typedef struct {
    float b0_A[2], k_A_per_duty[2], error_A[2];
    float duty_min, duty_max, bus_min_V, bus_max_V;
    uint32_t calibration_id;
    uint8_t model; /* 1=自身duty；2=三相平均duty。M3未经适配不可调用。 */
    uint8_t certified;
} bias_calibration_t;
typedef struct { uint16_t adc[2]; float duty[3], bus_V; } bias_input_t;
typedef struct { float corrected_A[3], upper_A[3]; uint8_t permitted; } bias_output_t;
/* 0=允许；1=非法/未知标定；2=饱和；3=越域；4=电流区间越限。 */
int bias_compensate(const bias_calibration_t *cal, const bias_input_t *in, bias_output_t *out);
#endif
