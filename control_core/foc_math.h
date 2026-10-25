/** FOC 纯数学接口：输入已校准电流/已知电角度或目标电压，输出值对象交给调用者。
 * 本模块无可变全局状态、HAL/设备依赖或使能能力；当前只由主机测试调用。
 * 调用者独占输出对象，函数不保留指针；可用不同对象交错调用，不需要init/reset。 */
#ifndef FOC_MATH_H
#define FOC_MATH_H
#include <stdbool.h>

typedef struct {
    float current_d_A; /**< d轴电流，A；幅值不变变换的结果，非新增硬件测量。 */
    float current_q_A; /**< q轴电流，A；正q方向遵循 iq=-alpha*sin(theta)+beta*cos(theta)。 */
    bool is_valid;    /**< true才可使用；错误时数值清零且此标志false。 */
} foc_current_t;

typedef struct {
    float duty_u;              /**< U相占空比，[0,1]无量纲；没有硬件使能含义。 */
    float duty_v;              /**< V相占空比，[0,1]；相序U/V/W。 */
    float duty_w;              /**< W相占空比，[0,1]；有效零矢量为三路0.5。 */
    float limited_voltage_d_V; /**< 等比限幅后模型d轴电压，V；不是实测端电压。 */
    float limited_voltage_q_V; /**< 等比限幅后模型q轴电压，V；与d轴共同受圆形上界约束。 */
    bool is_voltage_limited;   /**< 请求矢量幅值超过显式上界时true，未超限时false。 */
    bool is_valid;             /**< 数学结果有效；失败时所有数值0、两标志false，不等于停机命令。 */
} foc_modulation_t;

/** 两相电流→d/q。输入current_u_A/current_v_A为已校准有限A，假设三相电流和为0；
 * electrical_angle_rad为有限电角度rad，正方向alpha→beta，alpha与U同向。
 * result须为调用者独占的可写对象；NULL返回false且不写内存。
 * 成功返回true并置is_valid；NaN/Inf或结果超float范围返回false，非空输出全部清零。
 * 不修改输入/其他状态，不校准ADC、不估计电零位；大量程角度的输入精度由调用者负责。 */
bool foc_transform_current(float current_u_A, float current_v_A,
    float electrical_angle_rad, foc_current_t *result);

/** d/q电压→联合限幅、逆Park、连续SVPWM。目标d/q为有限V，角度为有限电角度rad；
 * bus_voltage_V为有限正V，modulation_limit必须有限且在(0,1]，不暗设0.9。
 * 矢量上界为modulation_limit*bus_voltage_V/sqrt(3)，超限等比缩放，不逐轴裁剪。
 * result须为调用者独占可写对象；NULL返回false。成功置is_valid并返回true。
 * 非法输入/数值异常返回false，非空输出全部清零、标志false；失败数值不能作为硬件命令。
 * 内部double、公开float，返回受限模型电压；量化误差见数学契约，不代表真实施加电压。
 * 无任何设备写入、使能或可变内部状态，当前没有接入固件控制路径。 */
bool foc_modulate_voltage(float target_voltage_d_V, float target_voltage_q_V,
    float electrical_angle_rad, float bus_voltage_V, float modulation_limit,
    foc_modulation_t *result);
#endif
