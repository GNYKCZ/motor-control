/** 运行管理：唯一拥有生命周期/快照的模块；生产仅由本地新按键手势可进入一次受限动作。
 * 输入来自 board 完成包，输出为一致状态；初始化在前台，采样在 ADC IRQ，快照在前台。
 * 所有共享状态修改/读取均保存恢复 PRIMASK；profile只读，不保留串口指针，无自动对齐。 */
#ifndef MOTOR_RUNTIME_H
#define MOTOR_RUNTIME_H
#include "sample_frame.h"
#include "sensor_input.h"
#include "feedback.h"

typedef enum {
    MOTOR_RUNTIME_BOOT = 0,     /**< 尚未确认外设初始化完成，禁止 PWM。 */
    MOTOR_RUNTIME_DISABLED = 1, /**< 当前只采样，不使能PWM；新本地手势仍须完整资格检查。 */
    MOTOR_RUNTIME_FAULT = 2,    /**< 故障锁存，仍禁止 PWM，本批无自动复位。 */
    MOTOR_RUNTIME_ARMED = 3,    /**< 一次性资格，无PWM；仅本地受限profile建立，串口不可到达。 */
    MOTOR_RUNTIME_STARTING = 4, /**< 资格已消耗，等自然UEV与共同低窗。 */
    MOTOR_RUNTIME_ACTIVE = 5    /**< 已通过最终事务；非电气输出证明。 */
} motor_runtime_state_t;

typedef enum {
    MOTOR_FAULT_NONE = 0u,         /**< 无已锁存故障，不代表硬件已验证正常。 */
    MOTOR_FAULT_INITIALIZATION = 1u,/**< 时钟/外设初始化或启动失败。 */
    MOTOR_FAULT_SAMPLE = 2u,       /**< 完整帧内容、CRC、时间或序号无效。 */
    MOTOR_FAULT_DEVICE = 4u,       /**< DMA/ADC 调度或驱动 nFAULT 异常。 */
    MOTOR_FAULT_FEEDBACK = 8u,     /**< 原始流断续、抽帧邮箱积压或反馈估算拒绝；锁存不自动恢复。 */
    MOTOR_FAULT_COMMUNICATION = 16u, /**< 受保护状态中的UART/帧错误，立即撤销资格/输出。 */
    MOTOR_FAULT_EXPERIMENT = 32u, /**< 本次固定实验累计位移达到46count，非物理最大转角保证。 */
    MOTOR_FAULT_START_WINDOW = 64u, /**< 启动事务共同低窗检查拒绝；同时保留DEVICE位。 */
    MOTOR_FAULT_START_INPUT = 128u, /**< 最终代次/输入/资格检查拒绝；同时保留DEVICE位。 */
    MOTOR_FAULT_START_TIME = 256u /**< 最终开门前期限检查拒绝；同时保留DEVICE位。 */
} motor_fault_t;

typedef struct {
    motor_runtime_state_t state; /**< 生命周期；只由本模块写入。 */
    uint32_t fault_flags;        /**< 锁存的 motor_fault_t 位组合。 */
    uint32_t sample_count;       /**< 处理的完整 ADC 包数，含无效包。 */
    uint32_t rejected_count;     /**< 被拒的无效/重复/陈旧包数。 */
    motor_sample_t sample;       /**< 最近一次收到的完整包；无效时也保留原始证据。 */
    bool has_valid_sample;      /**< 本次启动以来是否至少收到一个有效包。 */
    bool pwm_is_permanently_blocked; /**< profile可用不等于正在输出；仍不宣称功率桥高阻关闭。 */
    sensor_offset_t estimated_offset; /**< DISABLED中完整连续帧的零偏候选整数统计。 */
    uint32_t offset_rejected_count; /**< 候选批被首个无效帧中止的次数，最多1；完整拒收总数仍见rejected_count。 */
    bool offset_is_aborted;     /**< 任一无效/重复/时序错误/故障会冻结未完成批次，不补数。 */
    bool calibration_verified;  /**< 本批恒false；收齐512帧不证明静止、无电流或换算精度。 */
    feedback_result_t feedback; /**< 最近发布的前台估算，独立于sample时标；故障时清零无效。 */
    uint32_t feedback_sequence; /**< 该估算使用的真实encoder完成序号，未发布时0。 */
    uint32_t feedback_completed_us; /**< 该估算输入的真实encoder读取完成时刻。 */
    uint16_t feedback_raw_count; /**< 该估算输入raw，与最新sample可能不同。 */
    uint32_t feedback_overruns; /**< 邮箱满时未能交接的抽帧数；不覆盖未消费输入。 */
    uint32_t feedback_poll_max_us; /**< 有输入的前台反馈处理最大观测墙钟us，含抢占，非WCET。 */
    uint32_t mailbox_wait_max_us; /**< 消费等待或溢出时未消费输入年龄的同一最大值，故障保留。 */
} motor_status_t;

/** 周期日志专用紧凑快照：只复制本轮验收实际发送的数据，不含double/历史/零偏统计。 */
typedef struct {
    motor_runtime_state_t state;
    uint32_t fault_flags;
    uint32_t sequence;           /**< 最近ADC完成序号，含失败包，uint32模回绕。 */
    uint32_t completed_us;       /**< 该ADC完成时刻，1MHz设备时间。 */
    uint16_t raw_u, raw_v, raw_angle; /**< 原始count；失败包也保留，不冒充名义电流。 */
    bool sample_is_valid;
    bool position_is_valid;
    bool direct_is_valid;        /**< 本轮只观察估算器就绪，不发送/验证速度精度。 */
    bool window_is_valid;
    uint32_t feedback_overruns;
    uint32_t feedback_max_us;    /**< 反馈poll最大观测墙钟，含抢占，非WCET。 */
    uint32_t mailbox_max_us;     /**< 最大等待/未消费年龄，起点为输入所属ADC完成时刻。 */
} motor_log_snapshot_t;

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
/** IRQ/前台幂等STOP：立即禁止调制，撤销资格/待开始，不等待回执、不清故障/采样历史。 */
void motor_runtime_stop(void);
/** 生产固定本地profile数量1；测试专用profile只在主机宏构建，不能进入ARM目标。 */
unsigned motor_runtime_output_profile_count(void);
#ifdef MOTOR_GATE_TEST_PROFILES
/** host合成profile入口；生产函数为runtime私有，只允许本地手势调用。 */
bool motor_runtime_arm(unsigned profile_id);
/** 消耗一次资格，开始等待UEV/低窗；绝不续期，失败也不返还资格。 */
bool motor_runtime_pulse(void);
#endif
/** 前台约1ms采样PA12低有效电平。非阻塞去抖/新鲜释放，启动按住不触发，每启动最多1次尝试。 */
void motor_runtime_poll_button(bool pressed, uint32_t now_us);
/** 前台周期日志资格：受保护输出状态暂停旧M3B3，真实回执用P1R2。 */
bool motor_runtime_output_is_busy(void);
/** board已持锁，写MOE之后/清MOE之后的设备时间标记；不证明电气边沿已被外部观测。 */
void motor_runtime_record_gate(bool opened, uint32_t now_us);
typedef enum { MOTOR_ACTION_NONE=0, MOTOR_ACTION_REJECTED=1, MOTOR_ACTION_WAIT_START=2,
    MOTOR_ACTION_ACTIVE=3, MOTOR_ACTION_DURATION=4, MOTOR_ACTION_STOP=5, MOTOR_ACTION_FAULT=6 } motor_action_reason_t;
typedef struct { uint32_t id, requested_us, started_us, ended_us; motor_action_reason_t reason;
    uint8_t timing_flags; /**< 0x80开始时标有效，0x40结束时标有效；0时刻也合法。 */
} motor_action_t;
/** 同一短锁同时冻结旧状态与最近一次动作槽，供P1R2回执；不格式化、不提供可写内部指针。 */
bool motor_runtime_read_reply(motor_log_snapshot_t *status, motor_action_t *action);
/** 完整512帧零偏候选的只读快照；不自动应用或标为已校准。 */
typedef struct { sensor_offset_t offset; uint32_t began_us, ended_us, profile_id, scale_nA;
    uint8_t flags; /**< bit0满批、bit1中止、bit2静止辅助、bit3名义增益、bit4显式重开批次。 */
} motor_zero_snapshot_t;
bool motor_runtime_read_zero(motor_log_snapshot_t *status, motor_zero_snapshot_t *zero);
/** 固定U矢量输出期间的有界原始统计；只用于诊断，不应用零偏、不作为闭环/峰值保护。 */
typedef struct { sensor_offset_t samples; uint32_t began_us, ended_us, action_id;
    uint16_t first_angle, last_angle; uint8_t overflow; } motor_current_snapshot_t;
/** 与零偏/运行态在同一短锁冻结；输出开始前/关闭后的样本不加入统计。 */
bool motor_runtime_read_current(motor_log_snapshot_t *status, motor_zero_snapshot_t *zero,
                                motor_current_snapshot_t *current);
/** 仅DISABLED/无故障/新鲜静止/尚无动作时重开候选批，不改变输出资格。 */
bool motor_runtime_restart_zero(void);
/** 仅board CC4在持锁且共同低窗口内调用；代次与输入仍有效才转ACTIVE。 */
bool motor_runtime_commit_output(uint32_t generation);
/** board持锁：开门事务前后核对绝对期限，不调度、不复制状态。 */
bool motor_runtime_output_deadline_reached(uint32_t now_us);
/** UART错误/格式/CRC/超时：仅ARMED/STARTING/ACTIVE锁存通信故障并先停。 */
void motor_runtime_communication_error(void);
/** IRQ 故障入口：先禁止 PWM，再锁存非零 fault；不清故障，不使能输出。 */
void motor_runtime_latch_fault(motor_fault_t fault);
/** 前台读取完整状态副本到 status；NULL 返回 false，不写内存。
 * 成功返回 true，短临界区内一次复制；调用者不能获得可写的内部状态指针。 */
bool motor_runtime_read_status(motor_status_t *status);
/** 直接在短临界区复制小快照；不先读取完整status，不换算、不格式化。 */
bool motor_runtime_read_log(motor_log_snapshot_t *snapshot);
#endif
