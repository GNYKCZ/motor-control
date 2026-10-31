/** 固定零矢量诊断的有界RAM记录；只复制整数原始值，不改变任何保护或输出。
 * ADC完成路径在runtime校验/保护判定之前调用 diag_trace_sample，不做格式化、浮点或等待。
 * 记录是IRQ读取时刻的快照，不是ADC采样孔径时刻，也不是功率桥电气同步证明。
 * 完整性由导出头帧/末帧的计数字段与解析器判定；缺帧、重复、错序不自动补齐。
 * 仅在 BOARD_DQ_VECTOR_DIAGNOSTIC 构建中分配RAM并导出，其它构建提供空实现。 */
#ifndef APP_DIAG_TRACE_H
#define APP_DIAG_TRACE_H
#include "sample_frame.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define DIAG_TRACE_FORMAT_VERSION 1u     /**< 帧内格式版本；布局变化必须递增。 */
#define DIAG_TRACE_PRE_SLOTS 16u         /**< 动作前环形样本，动作建立时冻结，不再覆盖。 */
#define DIAG_TRACE_BODY_SLOTS 48u        /**< 启动/动作/首次故障/停机后恢复的线性样本。 */
#define DIAG_TRACE_CONFIG_SLOTS 4u       /**< 空闲、零矢量开门、50%生效、首次停机各一次。 */
#define DIAG_TRACE_RECOVERY_SAMPLES 8u   /**< 停机后恢复窗口，有限；缺失即报不完整。 */
#define DIAG_TRACE_RECOVERY_WAIT_US 12000u/**< 8条1kHz恢复样本加余量的等待上限；超时如实报不完整。 */
#define DIAG_TRACE_MAX_FRAME 168u        /**< 导出单帧上限，含CRC4与CRLF。 */
#define DIAG_TRACE_NO_INDEX 0xFFFFFFFFu  /**< 首次故障样本未落在记录内。 */

/** 记录条目的阶段位。PRE/无位为动作前，POST为停机后恢复窗口。 */
#define DIAG_FLAG_VALID 0x01u           /**< 该帧Runtime值域校验通过；不证明电流准确。 */
#define DIAG_FLAG_EDGE_UNCERTAIN 0x02u  /**< 读前读后TIM1方向不同或时间差超2us。 */
#define DIAG_FLAG_PRE 0x04u             /**< 动作建立之前的样本。 */
#define DIAG_FLAG_POST 0x08u            /**< 首次停机之后的恢复样本。 */

/** 阶段编码；用于把配置快照和样本时间对齐。 */
typedef enum {
    DIAG_STAGE_IDLE = 0u,    /**< 初始化完成、尚无输出。 */
    DIAG_STAGE_ZERO = 1u,    /**< 零矢量UEV刚打开MOE，本UEV有效比较值为0。 */
    DIAG_STAGE_FIFTY = 2u,   /**< 下一UEV把三路900装入，才开始50%零矢量。 */
    DIAG_STAGE_STOPPED = 3u  /**< 首次停机后，MOE已清。 */
} diag_stage_t;

/** 一次ADC完成时由板级同时读取的时序/引脚快照；调用者须在保护判定之前取得。 */
typedef struct {
    uint32_t before_us;  /**< 读取引脚/定时器之前的设备时间。 */
    uint32_t after_us;   /**< 读取之后的设备时间；差值>2us即标记不确定。 */
    uint16_t tim1_cnt;   /**< 原样计数，不换算成占空比。 */
    uint16_t tim2_cnt;   /**< 距上一次TIM1更新事件的1MHz计数（TIM2由TRGO复位），只在本轮约1ms内可比。 */
    uint8_t pins;        /**< bit0..2为PA8/9/10同一次IDR读数的电平，bit3为MOE。 */
    uint8_t dir;         /**< 读取后的TIM1方向：0向上计数，1向下计数。 */
    uint8_t dir_changed; /**< 1=读前读后TIM1方向不同，本次快照跨边沿。 */
} diag_irq_snapshot_t;

/** 单条记录：只有整数原始值与时序，不含换算安培或控制量。
 * 记录的是IRQ读取时刻，不是ADC采样孔径时刻，也不是功率端同步证据。 */
typedef struct {
    uint32_t sequence;      /**< 双ADC发布序号，模2^32。 */
    uint32_t completed_us;  /**< 该帧ADC完成被处理时刻。 */
    uint32_t before_us;     /**< 读取引脚/定时器之前的设备时间。 */
    uint32_t after_us;      /**< 读取之后的设备时间。 */
    uint16_t raw_u, raw_v;  /**< 原始12bit计数，未换算、未减零偏。 */
    uint16_t angle;         /**< 14bit机械角原始计数。 */
    uint16_t tim1_cnt;      /**< 原样计数，不换算占空比。 */
    uint16_t tim2_cnt;      /**< 距上次TIM1更新事件的1MHz计数低16位。 */
    uint8_t pins;           /**< bit0..2引脚电平，bit3为MOE。 */
    uint8_t flags;          /**< DIAG_FLAG_* 组合。 */
    uint8_t dir;            /**< TIM1方向：0向上，1向下。 */
    uint8_t stage;          /**< 写入时最近一次配置阶段。 */
} diag_sample_t;

/** 阶段配置快照；CCR读数是预装寄存器值，不是内部有效比较值的直接读回。 */typedef struct {
    uint32_t generation;  /**< 读取时的输出代次；0表示尚无动作。 */
    uint32_t read_us;     /**< 寄存器读取后的设备时间。 */
    uint16_t cr1, cr2, ccmr1, ccmr2, ccer, bdtr, psc, arr, rcr;
    uint16_t ccr1, ccr2, ccr3;
    uint32_t gpioa_crh, afio_mapr, gpioa_idr; /**< 32位保留SWJ_CFG等高位，不截断。 */
    uint8_t stage;          /**< diag_stage_t。 */
    uint8_t ccr_is_preload; /**< 本阶段CCR只是预装值，尚未在本次UEV生效。 */
} diag_config_t;

/** 归零全部记录状态；只在前台初始化调用，不清除任何故障或输出资格。 */
void diag_trace_init(void);
/** ADC完成IRQ、runtime校验与保护判定之前调用；只做整数复制和有界写入。 */
void diag_trace_sample(const diag_irq_snapshot_t *snapshot, const motor_sample_t *sample);
/** board_pwm在明确阶段调用；寄存器值由调用者读好传入，本模块不访问设备。 */
void diag_trace_config(const diag_config_t *config);
/** 动作代次建立：冻结动作前环，清零启动/动作记录；不改变输出。 */
void diag_trace_begin(uint32_t generation);
/** 输出停止（正常截止、STOP或故障）：安排有限恢复窗口，不恢复输出。 */
void diag_trace_stop(void);
/** 首次保护只冻结一次；后续故障采样不能清空记录或重复安排头帧。 */
void diag_trace_freeze(unsigned reason, uint32_t sequence);
/** 前台只读：是否还有待导出内容，含恢复窗口的有限等待。 */
bool diag_trace_pending(void);
/** 停机后前台按固定顺序导出；串口忙时重试同一帧。返回true表示本次写了一帧。 */
bool diag_trace_report_poll(void);
#ifdef __cplusplus
}
#endif
#endif
