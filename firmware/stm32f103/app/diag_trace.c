/** 固定零矢量诊断记录实现：写入只在ADC完成IRQ与明确的启动阶段，导出只在前台。
 * 本模块不读外设寄存器、不改PWM/MOE/门限/期限，也不把记录成功当作保护前提。
 * 非固定零矢量诊断构建不占用记录RAM，也不输出P1T帧。 */
#include "diag_trace.h"
#include "board_config.h"
#include "board_io.h"
#include <string.h>

#if !BOARD_DQ_VECTOR_DIAGNOSTIC

void diag_trace_init(void) {}
void diag_trace_sample(const diag_irq_snapshot_t *snapshot,const motor_sample_t *sample)
{ (void)snapshot;(void)sample; }
void diag_trace_config(const diag_config_t *config) { (void)config; }
void diag_trace_begin(uint32_t generation) { (void)generation; }
void diag_trace_stop(void) {}
void diag_trace_freeze(unsigned reason,uint32_t sequence) { (void)reason;(void)sequence; }
bool diag_trace_pending(void) { return false; }
bool diag_trace_report_poll(void) { return false; }

#else

static diag_sample_t pre_samples[DIAG_TRACE_PRE_SLOTS];
static diag_sample_t body_samples[DIAG_TRACE_BODY_SLOTS];
static diag_config_t configurations[DIAG_TRACE_CONFIG_SLOTS];
static unsigned pre_next,pre_used;      /**< 动作前环形写指针与已写入条数。 */
static unsigned pre_count,pre_start;    /**< 冻结时的有效条数与最旧位置。 */
static unsigned body_used,config_used;
static bool pre_frozen,armed,frozen,recovery_closed,overflowed;
static unsigned recovery_left;
static unsigned freeze_reason,trip_index;
static uint32_t action_generation,freeze_sequence,freeze_us;
static uint8_t last_config_stage;
/* 导出开始时一次锁存：此后统计口径不变，恢复窗口也不再接收新样本。 */
static bool sealed;
static unsigned sealed_pre,sealed_body;
static bool sealed_complete,sealed_overflow;

enum { EXPORT_IDLE=0u,EXPORT_HEADER,EXPORT_TIMER,EXPORT_TIMING,EXPORT_GPIO,EXPORT_SAMPLE,EXPORT_TAIL,EXPORT_DONE };
static unsigned export_stage,export_config,export_sample;
static char line[DIAG_TRACE_MAX_FRAME];
static unsigned line_length;

void diag_trace_init(void)
{
    memset(pre_samples,0,sizeof(pre_samples));
    memset(body_samples,0,sizeof(body_samples));
    memset(configurations,0,sizeof(configurations));
    pre_next=pre_used=pre_count=pre_start=body_used=config_used=0u;
    pre_frozen=armed=frozen=recovery_closed=overflowed=false;
    recovery_left=0u;freeze_reason=trip_index=DIAG_TRACE_NO_INDEX;
    action_generation=freeze_sequence=freeze_us=0u;last_config_stage=(uint8_t)DIAG_STAGE_IDLE;
    sealed=false;sealed_pre=sealed_body=0u;sealed_complete=sealed_overflow=false;
    export_stage=EXPORT_IDLE;export_config=export_sample=0u;
}

static void fill_record(diag_sample_t *record,const diag_irq_snapshot_t *snapshot,const motor_sample_t *sample)
{
    record->sequence=sample->sequence;
    record->completed_us=sample->current_completed_us;
    record->before_us=snapshot->before_us;
    record->after_us=snapshot->after_us;
    record->raw_u=sample->raw_current_u_count;
    record->raw_v=sample->raw_current_v_count;
    record->angle=sample->encoder.raw_angle_count;
    record->tim1_cnt=snapshot->tim1_cnt;
    record->tim2_cnt=snapshot->tim2_cnt;
    record->pins=snapshot->pins;
    record->flags=0u;
    record->dir=(uint8_t)(snapshot->dir & 1u);
    record->stage=last_config_stage;
    if (sample->is_valid) { record->flags|=DIAG_FLAG_VALID; }
    if (snapshot->dir_changed || snapshot->after_us-snapshot->before_us>2u) { record->flags|=DIAG_FLAG_EDGE_UNCERTAIN; }
}

void diag_trace_sample(const diag_irq_snapshot_t *snapshot,const motor_sample_t *sample)
{
    if (snapshot==NULL || sample==NULL) { return; }
    diag_sample_t record;
    fill_record(&record,snapshot,sample);
    uint32_t mask=board_enter_critical();
    if (sealed) { board_leave_critical(mask);return; } /* 导出已开始：不再追加，避免头尾计数漂移。 */
    if (!pre_frozen) {
        /* 动作前只保留最近16条；起止位置在冻结时确定，导出仍按到达顺序。 */
        record.flags|=DIAG_FLAG_PRE;
        pre_samples[pre_next]=record;
        pre_next=(pre_next+1u)%DIAG_TRACE_PRE_SLOTS;
        if (pre_used<DIAG_TRACE_PRE_SLOTS) { ++pre_used; }
    } else if (!frozen || recovery_left!=0u) {
        if (frozen) { record.flags|=DIAG_FLAG_POST; }
        if (body_used>=DIAG_TRACE_BODY_SLOTS) { overflowed=true; }
        else {
            body_samples[body_used]=record;++body_used;
            if (frozen) {
                --recovery_left;
                if (recovery_left==0u) { recovery_closed=true; }
            }
        }
    } /* 恢复窗口结束后不再新增：持续故障不能覆盖首次证据。 */
    board_leave_critical(mask);
}

void diag_trace_config(const diag_config_t *config)
{
    if (config==NULL) { return; }
    uint32_t mask=board_enter_critical();
    if (sealed) { board_leave_critical(mask);return; } /* 导出已锁存：配置也不得再追加。 */
    last_config_stage=config->stage; /* 只做8位写；ADC IRQ只读该字节。 */
    /* 同一阶段重复出现（例如多次停机路径）只更新该阶段，不挤占其它阶段证据。 */
    if (config_used!=0u && configurations[config_used-1u].stage==config->stage) {
        configurations[config_used-1u]=*config;
    } else if (config_used<DIAG_TRACE_CONFIG_SLOTS) { configurations[config_used]=*config;++config_used; }
    board_leave_critical(mask);
}

void diag_trace_begin(uint32_t generation)
{
    uint32_t mask=board_enter_critical();
    /* 每启动只允许一次动作：重复begin不能把上次的动作前样本冒充成本次证据。 */
    if (armed) { pre_count=0u; }
    else { pre_count=pre_used;pre_start=(pre_used<DIAG_TRACE_PRE_SLOTS) ? 0u : pre_next; }
    armed=true;action_generation=generation;pre_frozen=true;
    body_used=0u;frozen=false;recovery_closed=false;overflowed=false;recovery_left=0u;
    freeze_reason=DIAG_TRACE_NO_INDEX;trip_index=DIAG_TRACE_NO_INDEX;
    freeze_sequence=freeze_us=0u;
    sealed=false;sealed_pre=sealed_body=0u;sealed_complete=sealed_overflow=false;
    export_stage=EXPORT_IDLE;export_config=export_sample=0u;
    board_leave_critical(mask);
}

/** 停机后把阶段推进到STOPPED：即使该路径没有配置快照，恢复样本也不能标成动作段。 */
static void enter_stopped_stage(void)
{
    last_config_stage=(uint8_t)DIAG_STAGE_STOPPED;
}

void diag_trace_freeze(unsigned reason,uint32_t sequence)
{
    uint32_t mask=board_enter_critical();
    if (!frozen && armed) {
        frozen=true;freeze_reason=reason;freeze_sequence=sequence;freeze_us=board_get_time_us();
        recovery_left=DIAG_TRACE_RECOVERY_SAMPLES;recovery_closed=false;
        enter_stopped_stage();
        /* 触发保护的那一帧通常已由本次ADC路径写入；只接受末条匹配，不猜测。 */
        trip_index=(body_used!=0u && body_samples[body_used-1u].sequence==sequence)
            ? body_used-1u : DIAG_TRACE_NO_INDEX;
    }
    board_leave_critical(mask);
}

void diag_trace_stop(void)
{
    uint32_t mask=board_enter_critical();
    if (armed && !frozen) {
        frozen=true;freeze_reason=DIAG_TRACE_NO_INDEX;freeze_sequence=0u;freeze_us=board_get_time_us();
        recovery_left=DIAG_TRACE_RECOVERY_SAMPLES;recovery_closed=false;trip_index=DIAG_TRACE_NO_INDEX;
        enter_stopped_stage();
    }
    board_leave_critical(mask);
}

/** 完整：恢复窗口已收满且没有溢出；故障原因缺失不影响本判定，由头帧单独报告。 */
static bool records_complete(void) { return recovery_closed && !overflowed; }
static unsigned sample_total(void) { return sealed_pre+sealed_body; }

static void sample_at(unsigned index,diag_sample_t *record)
{
    if (index<sealed_pre) { *record=pre_samples[(pre_start+index)%DIAG_TRACE_PRE_SLOTS]; }
    else { *record=body_samples[index-sealed_pre]; }
}

bool diag_trace_pending(void)
{
    if (export_stage==EXPORT_DONE || !armed) { return false; }
    if (export_stage==EXPORT_IDLE) {
        if (!frozen) { return false; } /* 只在首次停机之后导出，不依赖调用者的忙碌门控。 */
        if (!recovery_closed && board_get_time_us()-freeze_us<DIAG_TRACE_RECOVERY_WAIT_US) {
            return false; /* 有限等待恢复窗口；超时后按实际条数导出并如实报不完整。 */
        }
    }
    return true;
}

static uint16_t crc_byte(uint16_t crc,uint8_t byte)
{
    crc^=(uint16_t)byte<<8;
    for (unsigned i=0u;i<8u;++i) { crc=(uint16_t)((crc & 0x8000u) ? (crc<<1)^0x1021u : (uint32_t)crc<<1); }
    return crc;
}
static void word(uint32_t value)
{
    static const char hex[]="0123456789ABCDEF";
    for (unsigned j=0u;j<8u;++j) { line[line_length++]=hex[(value>>(28u-4u*j))&15u]; }
    line[line_length++]=',';
}
static bool send_line(void)
{
    uint16_t crc=0xffffu;
    for (unsigned i=0u;i<line_length;++i) { crc=crc_byte(crc,(uint8_t)line[i]); }
    static const char hex[]="0123456789ABCDEF";
    for (unsigned j=0u;j<4u;++j) { line[line_length++]=hex[(crc>>(12u-4u*j))&15u]; }
    line[line_length++]='\r';line[line_length++]='\n';
    return board_start_serial_write(line,line_length);
}

static void build_header(void)
{
    uint32_t first=0u;
    diag_sample_t record;
    if (sample_total()!=0u) { sample_at(0u,&record);first=record.sequence; }
    memcpy(line,"P1T0,",5u);line_length=5u;
    word(DIAG_TRACE_FORMAT_VERSION);word(sealed_pre);word(sealed_body);word(sample_total());
    word(sealed_complete ? 1u : 0u);word(sealed_overflow ? 1u : 0u);word(frozen ? 1u : 0u);
    /* trip_index按导出序号给出：动作前样本占前sealed_pre条，避免与body索引混淆。 */
    word(trip_index==DIAG_TRACE_NO_INDEX ? DIAG_TRACE_NO_INDEX : trip_index+sealed_pre);
    word(first);word(freeze_reason);word(freeze_sequence);
    word(config_used);word(action_generation);
}

static void build_config_timer(const diag_config_t *c)
{
    memcpy(line,"P1T2,",5u);line_length=5u;
    word(c->stage);word(c->generation);word(c->read_us);word(c->cr1);word(c->cr2);
    word(c->ccmr1);word(c->ccmr2);word(c->ccer);word(c->bdtr);
}
static void build_config_timing(const diag_config_t *c)
{
    memcpy(line,"P1T3,",5u);line_length=5u;
    word(c->stage);word(c->read_us);word(c->psc);word(c->arr);word(c->rcr);
    word(c->ccr1);word(c->ccr2);word(c->ccr3);
}
static void build_config_gpio(const diag_config_t *c)
{
    memcpy(line,"P1T4,",5u);line_length=5u;
    word(c->stage);word(c->read_us);word(c->gpioa_crh);word(c->afio_mapr);
    word(c->gpioa_idr);word((c->bdtr & 0x8000u)!=0u ? 1u : 0u);word(c->ccr_is_preload);
}
static void build_sample(unsigned index,const diag_sample_t *r)
{
    memcpy(line,"P1T1,",5u);line_length=5u;
    word(index);word(r->sequence);word(r->completed_us);word(r->before_us);word(r->after_us);
    word(r->raw_u);word(r->raw_v);word(r->angle);word(r->tim1_cnt);word(r->tim2_cnt);
    word(r->pins);word(r->flags);word(r->dir);word(r->stage);
}
static void build_tail(void)
{
    memcpy(line,"P1T5,",5u);line_length=5u;
    word(export_sample);word(sample_total());word(sealed_pre);word(sealed_body);
    word(sealed_complete ? 1u : 0u);word(sealed_overflow ? 1u : 0u);
}

static unsigned next_stage_after_config(unsigned next)
{
    if (next<config_used) { return EXPORT_TIMER; }
    return sample_total()!=0u ? EXPORT_SAMPLE : EXPORT_TAIL;
}

/** 首次导出前锁存统计口径并停止接收恢复样本，避免导出期间头/尾帧不一致。 */
static void seal_export(void)
{
    uint32_t mask=board_enter_critical();
    sealed_pre=pre_count;sealed_body=body_used;
    sealed_complete=records_complete();sealed_overflow=overflowed;
    recovery_left=0u;
    sealed=true;
    board_leave_critical(mask);
}

bool diag_trace_report_poll(void)
{
    if (!diag_trace_pending()) { return false; }
    if (export_stage==EXPORT_IDLE) { seal_export();export_stage=EXPORT_HEADER;export_config=export_sample=0u; }
    switch (export_stage) {
    case EXPORT_HEADER: build_header();break;
    case EXPORT_TIMER: build_config_timer(&configurations[export_config]);break;
    case EXPORT_TIMING: build_config_timing(&configurations[export_config]);break;
    case EXPORT_GPIO: build_config_gpio(&configurations[export_config]);break;
    case EXPORT_SAMPLE: {
        diag_sample_t record;
        sample_at(export_sample,&record);
        build_sample(export_sample,&record);
        break;
    }
    case EXPORT_TAIL: build_tail();break;
    default: return false;
    }
    if (!send_line()) { return false; } /* 串口忙：不推进状态，下一轮重发同一帧。 */
    switch (export_stage) {
    case EXPORT_HEADER: export_stage=next_stage_after_config(0u);break;
    case EXPORT_TIMER: export_stage=EXPORT_TIMING;break;
    case EXPORT_TIMING: export_stage=EXPORT_GPIO;break;
    case EXPORT_GPIO:
        ++export_config;
        export_stage=next_stage_after_config(export_config);
        break;
    case EXPORT_SAMPLE:
        ++export_sample;
        if (export_sample>=sample_total()) { export_stage=EXPORT_TAIL; }
        break;
    case EXPORT_TAIL: export_stage=EXPORT_DONE;break;
    default: break;
    }
    return true;
}
#endif
