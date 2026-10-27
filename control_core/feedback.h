#ifndef CONTROL_CORE_FEEDBACK_H
#define CONTROL_CORE_FEEDBACK_H

/* 主机反馈模块：接收调用者提供的完整编码器帧，输出机械角和两种速度供离线比较。
 * 不读取外设、不驱动输出、不拥有全局状态；每个调用者持有独立配置/状态/结果对象。
 * 当前由主机测试串行调用；同一状态不可并发访问，未验收在MCU中断内的执行时间。
 * 调用顺序：init -> step逐帧处理；失败由调用者故障处理或reset后重新建立原点。
 * 公共契约在此集中说明；合成实验及来源对应见docs/m3a_feedback_contract.md。 */
#include <stdbool.h>
#include <stdint.h>

/* 环形历史的最大点数；实际使用前window_samples个槽，不进行动态分配。 */
#define FEEDBACK_CAPACITY 32u

/* 机械单位rad、rad/s，时间1us。配置由init复制，运行中不可修改。 */
typedef struct {
    double max_speed_rad_s;       /* 真实机械速度绝对值的先验上限rad/s，有限且>0；不是软件保护实测值。 */
    uint32_t maximum_gap_us;      /* 相邻成功帧近似中点的最大间隔us，范围[1,2^31-1]，包含上界。 */
    uint32_t maximum_read_us;     /* 一帧读取起止的最大时长us，范围[0,2^31-1]；0仅接受瞬时读取。 */
    uint8_t window_samples;      /* 完整窗口点数[2,32]，包含当前点；满窗才有窗口速度。 */
    int8_t direction;             /* 仅+1或-1，作用于相对连续角及速度；不改变raw单圈角正向。 */
} feedback_config_t;

/* 单帧输入值副本，来自M2c完整编码器帧的同名信息，不依赖板级头文件。
 * 调用者负责提供一致快照；本模块只保存所需值，不保留输入指针。 */
typedef struct {
    uint16_t raw_angle_count;     /* 未定机械方向/电零位的14 bit原始角计数[0,16383]。 */
    uint32_t sequence;            /* 编码器完成序号；首帧任意，其后必须模2^32严格加1。 */
    uint32_t read_started_us;     /* 本帧读取开始的1MHz设备时间，uint32模2^32回绕。 */
    uint32_t read_completed_us;   /* 本帧读取结束时间，同一设备时钟；不是精确传感器采样沿。 */
    bool is_valid;               /* 调用者对完整帧的有效性判定；false拒绝，true仍需通过模块边界检查。 */
} feedback_sample_t;

/* 内部历史点的固定存储表示；仅在状态的已填充窗口中有效。 */
typedef struct {
    int64_t count;                /* 该成功帧相对首帧的有向累计计数，不是单圈raw。 */
    uint64_t elapsed_us;          /* 该帧距首帧近似中点的累计us，可跨多个uint32时钟周期。 */
} feedback_point_t;

/* 调用者独占；必须先init，不能只依赖零初始化。字段公开仅供固定存储，
 * 运行中不可篡改或共享写入；有效期从init/reset到下一次init/reset结束该段轨迹。
 * step只有成功才写状态，失败时包括填充字节在内保持对象表示不变。 */
typedef struct {
    feedback_config_t config;    /* init复制的有效配置；更改参数须重新init。 */
    feedback_point_t history[FEEDBACK_CAPACITY]; /* 成功帧环形历史，仅使用前window_samples个槽。 */
    int64_t continuous_count;    /* 当前相对首帧的有向累计计数；首帧为0，溢出前拒绝新帧。 */
    uint64_t elapsed_us;         /* 成功帧中点间隔的累计us；首帧为0，溢出前拒绝新帧。 */
    uint32_t last_sequence;      /* 上一成功帧序号，仅initialized=true时有效。 */
    uint32_t last_midpoint_us;   /* 上一成功帧的近似中点设备时间，初始化前无效。 */
    uint32_t last_completed_us;  /* 上一成功帧读取结束时间，用于拒绝读取区间重叠/倒序。 */
    uint16_t last_raw;           /* 上一成功帧raw计数，用于计算一拍模差，初始化前无效。 */
    uint8_t next_index;          /* 下一写入槽[0,window_samples-1]；满窗时也指向当前最旧点。 */
    uint8_t history_count;       /* 已填点数[0,window_samples]；未满窗时从槽0连续存放，不输出窗口速度。 */
    bool initialized;           /* 是否已有首个成功帧；init/reset后false，首帧成功后true。 */
} feedback_state_t;

/* 本次调用的值输出，调用者持有；下次调用前可保存副本。
 * is_valid只表示位置/本次帧有效，两速度必须分别检查有效标志；失败输出全零。 */
typedef struct {
    double raw_mechanical_angle_rad; /* raw×2π/16384 rad，[0,2π)，位置有效时可用，不受direction影响。 */
    double continuous_angle_rad; /* 相对首帧的有向机械角rad；累计计数绝对值>2^53后不保证逐计数精度。 */
    double direct_speed_rad_s;   /* 相邻成功帧差分的机械速度rad/s，仅direct_is_valid时有效，否则0。 */
    double window_speed_rad_s;   /* 完整窗口首尾差分的机械速度rad/s，仅window_is_valid时有效，否则0。 */
    int64_t continuous_count;    /* 相对首帧的精确有向累计计数，位置有效时可用；首帧0。 */
    int64_t window_delta_count;  /* 完整窗口最新/最旧有向累计计数差，窗口未就绪时0。 */
    uint64_t window_dt_us;       /* 完整窗口首尾中点的实际间隔us，可>2^32；未就绪时0。 */
    uint32_t midpoint_us;        /* start+floor(duration/2)的模2^32近似时标，位置有效时可用。 */
    uint32_t dt_us;              /* 与上一成功帧中点间隔us，直接速度有效时>0，首帧/失败时0。 */
    uint8_t window_sample_count; /* 当前已填点数[1,window_samples]；成功即报告预热进度，失败时0。 */
    bool is_valid;              /* 本次帧与位置是否有效；true不代表两种速度都已就绪。 */
    bool is_first_sample;       /* 本次是否为init/reset后的首个成功帧；连续原点0且两速度无效。 */
    bool direct_is_valid;       /* 当前帧与上一帧连续且全部检查通过；首帧/失败时false。 */
    bool window_is_valid;       /* 当前成功且已填满配置窗口；预热/失败时false。 */
} feedback_result_t;

/* 初始化/重建轨迹：state指向调用者的可写独占对象，config为值传递的完整配置，
 * 范围见各字段。true时复制配置、清空所有历史/计数，等待首帧建立原点；
 * state为NULL或配置无效返回false且不修改状态。无外设/分配副作用。 */
bool feedback_init(feedback_state_t *state, feedback_config_t config);

/* 显式复位：state必须曾成功init且配置未被篡改；true时保留配置并清空轨迹，
 * 下一成功帧重新设相对原点0。NULL/无效配置返回false，非NULL状态不变。 */
bool feedback_reset(feedback_state_t *state);

/* 处理一个完整帧：state须已成功init且由当前调用独占；sample是输入值副本，
 * result为调用者可写输出对象，与state不可重叠；同一状态不得并发调用。
 * 读取duration=(end-start)模2^32须<=maximum_read_us，允许0；中点向下取整us。
 * 后续序号严格加1，读取区间不可重叠（端点相接允许），中点dt在[1,maximum_gap_us]。
 * 调用者保证真实读取时长/相邻间隔<2^31 us；模时间不能检测完整时钟周期丢失。
 * raw模差取(-8192,8192)；半圈拒绝。B=max_speed_rad_s×dt_us/1e6÷(2π/16384)，
 * abs(raw模差)<=B+1才接受；B+1>=8192整拍不可辨而拒绝。+1是量化包络，
 * 不裁剪报告速度，真实运动遵守速度先验由调用者保证，软件无法证明。
 * true时提交状态和结果；首帧仅建立位置原点，窗口满window_samples点后才有窗口速度。
 * false时state任何字节不变，非NULL result全零；NULL result也不提交state。
 * 失败原因包括上述边界、无效帧/配置/指针、累计整数溢出；调用方须故障处理或显式reset，
 * 不应跳过坏帧后假装序列连续。无外设/打印/分配副作用，仅提供主机软件证据。 */
bool feedback_step(feedback_state_t *state, feedback_sample_t sample,
                   feedback_result_t *result);

#endif
