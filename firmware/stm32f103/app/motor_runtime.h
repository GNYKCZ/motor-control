/** M1 运行管理：唯一拥有生命周期/快照的模块；只有 BOOT、DISABLED、FAULT。
 * 输入来自 board 完成包，输出为一致状态；初始化在前台，采样在 ADC IRQ，快照在前台。
 * 所有共享状态修改/读取均保存恢复 PRIMASK；不保留调用者指针，不存在使能/对齐入口。 */
#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H
#include "sample_frame.h"
#include "sensor_input.h"
#include "feedback.h"

typedef enum {
    MOTOR_RUNTIME_BOOT = 0,     /**< 尚未确认外设初始化完成，禁止 PWM。 */
    MOTOR_RUNTIME_DISABLED = 1, /**< 只采样，永久禁止 PWM。 */
    MOTOR_RUNTIME_FAULT = 2     /**< 故障锁存，仍禁止 PWM，本批无自动复位。 */
} motor_runtime_state_t;

typedef enum {
    MOTOR_FAULT_NONE = 0u,         /**< 无已锁存故障，不代表硬件已验证正常。 */
    MOTOR_FAULT_INITIALIZATION = 1u,/**< 时钟/外设初始化或启动失败。 */
    MOTOR_FAULT_SAMPLE = 2u,       /**< 完整帧内容、CRC、时间或序号无效。 */
    MOTOR_FAULT_DEVICE = 4u,       /**< DMA/ADC 调度或驱动 nFAULT 异常。 */
    MOTOR_FAULT_FEEDBACK = 8u      /**< 原始流断续、抽帧邮箱积压或反馈估算拒绝；锁存不自动恢复。 */
} motor_fault_t;

typedef struct {
    motor_runtime_state_t state; /**< 生命周期；只由本模块写入。 */
    uint32_t fault_flags;        /**< 锁存的 motor_fault_t 位组合。 */
    uint32_t sample_count;       /**< 处理的完整 ADC 包数，含无效包。 */
    uint32_t rejected_count;     /**< 被拒的无效/重复/陈旧包数。 */
    motor_sample_t sample;       /**< 最近一次收到的完整包；无效时也保留原始证据。 */
    bool has_valid_sample;      /**< 本次启动以来是否至少收到一个有效包。 */
    bool pwm_is_permanently_blocked; /**< 本批恒 true；不宣称功率桥高阻关闭。 */
    sensor_offset_t estimated_offset; /**< DISABLED中完整连续帧的零偏候选整数统计。 */
    uint32_t offset_rejected_count; /**< 候选批被首个无效帧中止的次数，最多1；完整拒收总数仍见rejected_count。 */
    bool offset_is_aborted;     /**< 任一无效/重复/时序错误/故障会冻结未完成批次，不补数。 */
    bool calibration_verified;  /**< 本批恒false；收齐512帧不证明静止、无电流或换算精度。 */
    feedback_result_t feedback; /**< 最近发布的前台估算，独立于sample时标；故障时清零无效。 */
    uint32_t feedback_sequence; /**< 该估算使用的真实encoder完成序号，未发布时0。 */
    uint32_t feedback_completed_us; /**< 该估算输入的真实encoder读取完成时刻。 */
    uint16_t feedback_raw_count; /**< 该估算输入raw，与最新sample可能不同。 */
    uint32_t feedback_overruns; /**< 邮箱满时未能交接的抽帧数；不覆盖未消费输入。 */
} motor_status_t;

/** 前台首次初始化；归零状态，先禁止 PWM，再建立 BOOT；不得在采样启动后重入。 */
void motor_runtime_init(void);
/** 前台完成初始化握手；devices_are_ready=true 才从 BOOT 转 DISABLED。
 * false 锁存初始化故障；重复调用/在其他状态调用锁存故障，不自动恢复。 */
void motor_runtime_finish_boot(bool devices_are_ready);
/** ADC 完成路径提交一个完整 sample（允许 NULL，以故障处理）。
 * 重新验证值域、时间/序号；无效输入不绕过门控。首次帧最多等待 1 ms。
 * 不调用串口，不保存指针；故障后仍可记录样本但不自动退出 FAULT。 */
void motor_runtime_handle_sample(const motor_sample_t *sample);
/** 前台检查采样是否失联；now_us 必须取设备 1 MHz 时间。
 * 启动超过 1 ms 无有效帧、已有有效帧后超过 300 us 无新包，锁存采样故障。
 * 短临界区内只比较整数；BOOT 阶段不检查；故障不自动清除。 */
void motor_runtime_check_deadline(uint32_t now_us);
/** 仅前台调用：短临界区取单槽邮箱，在临界区外计算double反馈，再原子发布。
 * 每10个连续有效帧产生一个输入；前台尽快消费，无严格1kHz/WCET声明。
 * 重入/并发前台调用不允许；状态独占，任何中途IRQ故障均阻止结果重新有效。 */
void motor_runtime_poll_feedback(void);
/** IRQ 故障入口：先禁止 PWM，再锁存非零 fault；不清故障，不使能输出。 */
void motor_runtime_latch_fault(motor_fault_t fault);
/** 前台读取完整状态副本到 status；NULL 返回 false，不写内存。
 * 成功返回 true，短临界区内一次复制；调用者不能获得可写的内部状态指针。 */
bool motor_runtime_read_status(motor_status_t *status);
#endif
