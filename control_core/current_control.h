/** 主机可验证的双轴电流PI：显式A/V/rad/s，无设备或使能能力。
 * 每个调用者独占一份积分状态；参数/样本按值传入，不保留输入/输出指针。 */
#ifndef CURRENT_CONTROL_H
#define CURRENT_CONTROL_H
#include "foc_math.h"

typedef struct {
    float kp_V_per_A;           /**< 比例增益V/A，有限非负，0允许。 */
    float ki_V_per_A_s;         /**< 积分增益V/(A·s)，有限非负，乘显式dt。 */
    float tracking_gain_per_s;  /**< 最终限幅反馈增益1/s，有限非负；0为显式关闭跟踪。 */
} current_control_axis_t;

typedef struct {
    current_control_axis_t d;   /**< d轴参数，调用者显式提供，不与q轴共用可变状态。 */
    current_control_axis_t q;   /**< q轴参数。 */
    float reference_limit_A;    /**< 每轴参考电流绝对上限A，有限正值，不是矢量圆界。 */
    float modulation_limit;     /**< 联合电压圆界系数，(0,1]，直接交给M2a。 */
    float minimum_dt_s;         /**< 最小允许采样间隔s，有限正值，含端点。 */
    float maximum_dt_s;         /**< 最大允许间隔s，有限且>=minimum，含端点。 */
} current_control_config_t;

typedef struct {
    float integral_d_V;         /**< d轴积分电压V；当前拍输出使用旧积分，下拍使用成功提交的新值。 */
    float integral_q_V;         /**< q轴积分电压V；失败不提交任一轴。 */
} current_control_state_t;

typedef struct {
    float current_u_A;          /**< 合成或已校准U相A，不能传raw ADC计数冒充A。 */
    float current_v_A;          /**< V相A，假设U+V+W=0。 */
    float electrical_angle_rad; /**< 已知电角度rad，沿用M2a相序与q符号。 */
    float reference_d_A;        /**< 本拍d轴目标A，显式输入，无默认电流。 */
    float reference_q_A;        /**< 本拍q轴目标A。 */
    float bus_voltage_V;        /**< 有限正母线V，当前不是实际板端测量接口。 */
    float dt_s;                 /**< 本次采样间隔s，须在配置闭区间内。 */
} current_control_input_t;

typedef struct {
    foc_current_t measured;       /**< M2a测得的d/q A及有效标志。 */
    float limited_reference_d_A;  /**< 按每轴电流上限裁剪后的d轴A。 */
    float limited_reference_q_A;  /**< 裁剪后的q轴A。 */
    float requested_voltage_d_V;  /**< 比例+旧积分的d轴请求V，联合限制前。 */
    float requested_voltage_q_V;  /**< q轴请求V。 */
    foc_modulation_t modulation;  /**< M2a最终受限d/q V、三相占空比与限幅标志。 */
    bool is_reference_limited;    /**< 任一目标被每轴电流上限裁剪时true。 */
    bool is_valid;                /**< 仅完整成功才true；失败清零全部字段，不代表停机命令。 */
} current_control_output_t;

/** 首次建立状态：清两轴积分，无默认参数/电流，NULL返回false。 */
bool current_control_init(current_control_state_t *state);
/** 显式复位两轴积分为0；用于调用者处理故障/切换后的清理，不自动使能，NULL返回false。 */
bool current_control_reset(current_control_state_t *state);
/** 一拍计算。state和output须为独占、不重叠的有效对象；同一state禁止并发写。
 * 先M2a变换，PI请求共同送M2a调制，再以最终受限电压差跟踪积分。
 * 成功同时发布output和两轴新积分；任一参数/输入/积分/数值无效均返回false，
 * 原state逐字节不变，非空output清零无效；NULL output也不提交状态。
 * 不校准、不碰硬件、不锁存/解除故障；失败后调用者须锁存故障并显式reset。
 * 算术采用double中间量、float发布；完整公式/参数范围见M2b契约。 */
bool current_control_step(current_control_state_t *state, current_control_config_t config,
    current_control_input_t input, current_control_output_t *output);
#endif
