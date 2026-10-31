/** 当前闭环实验仅使用明确候选p7/d+1/z，非已验证标定；无速度/位置闭环。 */
#include "current_loop.h"
#include "current_control.h"
#include "board_io.h"
#include "board_config.h"
#include "motor_runtime.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#if BOARD_DQ_CURRENT_LOOP
#define TRACE_COUNT 256u
/* 只保留一次250ms瞬态，7168字节；导出后不返还PA12资格。 */
typedef struct { uint32_t t,sequence; int16_t id,iq,reference,vd,vq; uint16_t compute,latency,flags,angle; } loop_point_t;
static loop_point_t points[TRACE_COUNT],candidate_point;
static volatile unsigned used,benchmark_step;
static unsigned exported;
static volatile uint32_t generation;
static uint32_t last_sequence,last_sample_us,requested_sequence;
static uint32_t maximum_compute,maximum_latency,action_start;
static sensor_offset_t offset;
/* 显式加载到RAM的实验候选；来源为31-17-27保持角7453，不作Flash持久标定。 */
static struct { uint32_t source_sampling_profile; uint16_t zero_count; uint8_t pole_pairs; int8_t direction; } calibration;
static motor_sample_t mailbox;
static volatile bool queued,inflight,benchmark_failed;
static volatile bool header_pending,trip_pending;
static struct { uint32_t reason,sequence,t,u,v,angle;int32_t detail; } first_trip;
static current_control_state_t state,pending_state;
static const current_control_config_t config={{0.5f,150.0f,200.0f},{0.5f,150.0f,200.0f},
    0.04f,0.0577350269f,0.00099f,0.00101f};
static char line[160];
static unsigned line_length;
static int16_t milli(float value) { return (int16_t)lroundf(value*1000.0f); }

void current_loop_init(void)
{
    used=exported=benchmark_step=0u;generation=last_sequence=last_sample_us=0u;
    calibration.source_sampling_profile=0x00030001u;calibration.zero_count=7453u;
    calibration.pole_pairs=7u;calibration.direction=1;
    maximum_compute=maximum_latency=action_start=requested_sequence=0u;offset=(sensor_offset_t){0};
    queued=inflight=benchmark_failed=false;header_pending=true;trip_pending=false;first_trip.reason=0u;
    state=(current_control_state_t){0};pending_state=state;
}
bool current_loop_timing_failed(void) { return benchmark_failed; }
bool current_loop_is_ready(void) { return benchmark_step==32u && !benchmark_failed
    && calibration.source_sampling_profile==BOARD_SAMPLING_PROFILE_ID && calibration.pole_pairs==7u
    && calibration.direction==1 && calibration.zero_count==7453u; }
void current_loop_begin(uint32_t next_generation,sensor_offset_t next_offset)
{
    generation=next_generation;offset=next_offset;used=exported=0u;last_sequence=last_sample_us=0u;
    action_start=0u;queued=inflight=false;trip_pending=false;first_trip.reason=0u;state=(current_control_state_t){0};pending_state=state;
}
void current_loop_abort(void)
{
    bool had_request=generation!=0u || queued || inflight;
    generation=0u;queued=inflight=false;state=(current_control_state_t){0};pending_state=state;
    if (had_request) { header_pending=true; } /* 后续FAULT采样不能反复抢占有限记录导出。 */
}
void current_loop_capture_trip(unsigned reason,const motor_sample_t *sample,int32_t detail)
{
    uint32_t mask=board_enter_critical();
    board_disable_output(); /* 不让记录工作延迟MOE撤销；不改变runtime故障/资格所有权。 */
    if (generation && !first_trip.reason && sample!=NULL) {
        first_trip.reason=reason;first_trip.sequence=sample->sequence;first_trip.t=sample->current_completed_us;
        first_trip.u=sample->raw_current_u_count;first_trip.v=sample->raw_current_v_count;
        first_trip.angle=sample->encoder.raw_angle_count;first_trip.detail=detail;trip_pending=true;
    }
    board_leave_critical(mask);
}

void current_loop_queue(const motor_sample_t *sample)
{
    if (!generation) { return; }
    if (queued || inflight || sample==NULL) { motor_runtime_latch_fault(MOTOR_FAULT_FEEDBACK);return; }
    mailbox=*sample;queued=true;
}

void current_loop_poll(void)
{
    if (!generation && benchmark_step<32u && !benchmark_failed) {
        /* 前台单片计时，IRQ保持开放；包含共享单精度sin/cos/限幅的真实生产核心。 */
        current_control_state_t trial={(benchmark_step & 1u) ? 1.0f : 0.0f,(benchmark_step & 2u) ? -1.0f : 0.0f};current_control_output_t result;
        current_control_input_t input={0.01f,-0.02f,(float)benchmark_step*0.2f,0.0f,0.04f,12.0f,0.001f};
        uint32_t began=board_get_time_us();bool ok=current_control_step_bounded(&trial,config,input,&result);
        uint32_t elapsed=board_get_time_us()-began;
        if (elapsed>maximum_compute) { maximum_compute=elapsed; }
        if (!ok || elapsed>200u) { benchmark_failed=true;header_pending=true;return; }
        ++benchmark_step;if (benchmark_step==32u) { header_pending=true; }return;
    }
    uint32_t mask=board_enter_critical();
    if (!generation || !queued || inflight) { board_leave_critical(mask);return; }
    motor_sample_t sample=mailbox;uint32_t mine=generation;queued=false;
    current_control_state_t next=state;board_leave_critical(mask);
    uint32_t began=board_get_time_us();
    uint32_t dt=last_sample_us ? sample.current_completed_us-last_sample_us : 1000u;
    if ((last_sample_us && sample.sequence-last_sequence!=1u) || dt<990u || dt>1010u
        || began-sample.current_completed_us>250u) { motor_runtime_latch_fault(MOTOR_FAULT_SAMPLE);return; }
    motor_log_snapshot_t status;motor_action_t action;
    if (!motor_runtime_read_reply(&status,&action) || status.state!=MOTOR_RUNTIME_ACTIVE || status.fault_flags) { return; }
    uint32_t elapsed=sample.current_completed_us-action.started_us;
    float ramp=elapsed<25000u ? (float)elapsed/25000.0f : (elapsed<225000u ? 1.0f : (250000.0f-(float)elapsed)/25000.0f);
    if (ramp<0.0f) { ramp=0.0f; }
    /* U/V符号+1由先前独立固定矢量响应支持；名义增益不等于绝对安培校准。 */
    float u=((float)sample.raw_current_u_count-(float)offset.sum_u_count/512.0f)*BOARD_NOMINAL_A_PER_COUNT;
    float v=((float)sample.raw_current_v_count-(float)offset.sum_v_count/512.0f)*BOARD_NOMINAL_A_PER_COUNT;
    float theta=fmodf((float)((int32_t)sample.encoder.raw_angle_count-calibration.zero_count)
        *calibration.pole_pairs*calibration.direction*0.000383495197f,6.2831853072f);
    if (theta<0.0f) { theta+=6.2831853072f; }
    current_control_input_t input={u,v,theta,0.0f,(float)BOARD_DQ_REFERENCE_MA*0.001f*ramp,12.0f,(float)dt*0.000001f};
    current_control_output_t result;bool ok=current_control_step_bounded(&next,config,input,&result);
    uint32_t computed=board_get_time_us();uint32_t cost=computed-began;
    if (cost>maximum_compute) { maximum_compute=cost; }
    mask=board_enter_critical();
    bool still_current=generation==mine && motor_runtime_current_is_active(mine);
    board_leave_critical(mask);
    if (!still_current) { return; }
    if (!ok || cost>200u || computed-sample.current_completed_us>450u || used>=TRACE_COUNT) {
        motor_runtime_latch_fault(MOTOR_FAULT_FEEDBACK);return;
    }
    if (fabsf(result.measured.current_d_A)>0.08f || fabsf(result.measured.current_q_A)>0.08f
        || (elapsed>100000u && fabsf(result.measured.current_q_A-input.reference_q_A)>0.04f)) {
        unsigned why=fabsf(result.measured.current_d_A)>0.08f ? 7u : (fabsf(result.measured.current_q_A)>0.08f ? 8u : 9u);
        float detail=why==7u ? result.measured.current_d_A : (why==8u ? result.measured.current_q_A : result.measured.current_q_A-input.reference_q_A);
        current_loop_capture_trip(why,&sample,milli(detail));
        motor_runtime_latch_fault(MOTOR_FAULT_EXPERIMENT);return;
    }
#if BOARD_DQ_VECTOR_DIAGNOSTIC
    /* 只取当前坐标观测；不采用PI计算的电压或积分，不再提交三相CCR更新。
     * 启动后CCR固定900/900/900；诊断记录latency0表示没有新的控制提交。 */
    mask=board_enter_critical();
    if (generation==mine && motor_runtime_current_sample_matches(mine,sample.sequence,sample.current_completed_us)) {
        state=(current_control_state_t){0};pending_state=state;
        points[used++]=(loop_point_t){sample.current_completed_us,sample.sequence,milli(result.measured.current_d_A),
            milli(result.measured.current_q_A),0,0,0,(uint16_t)cost,0u,0u,sample.encoder.raw_angle_count};
        last_sequence=sample.sequence;last_sample_us=sample.current_completed_us;action_start=action.started_us;
    }
    board_leave_critical(mask);return;
#endif
    board_pwm_ticks_t ticks={(uint16_t)lroundf(result.modulation.duty_u*1800.0f),
        (uint16_t)lroundf(result.modulation.duty_v*1800.0f),(uint16_t)lroundf(result.modulation.duty_w*1800.0f),144u};
    mask=board_enter_critical();
    if (generation!=mine || !motor_runtime_current_is_active(mine)) { board_leave_critical(mask);return; }
    pending_state=next;requested_sequence=sample.sequence;
    candidate_point=(loop_point_t){sample.current_completed_us,sample.sequence,milli(result.measured.current_d_A),
        milli(result.measured.current_q_A),milli(input.reference_q_A),milli(result.modulation.limited_voltage_d_V),
        milli(result.modulation.limited_voltage_q_V),(uint16_t)cost,0u,(uint16_t)result.modulation.is_voltage_limited,sample.encoder.raw_angle_count};
    inflight=true;
    if (!board_submit_current(ticks,mine,sample.sequence,sample.current_completed_us)) {
        motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
    } else { last_sequence=sample.sequence;last_sample_us=sample.current_completed_us;action_start=action.started_us; }
    board_leave_critical(mask);
}

bool current_loop_applied(uint32_t mine,uint32_t sequence,uint32_t event_us)
{
    if (!generation || generation!=mine || !inflight || sequence!=requested_sequence || used>=TRACE_COUNT
        || event_us-candidate_point.t<740u || event_us-candidate_point.t>810u) { return false; }
    state=pending_state;candidate_point.latency=(uint16_t)(event_us-candidate_point.t);
    if (candidate_point.latency>maximum_latency) { maximum_latency=candidate_point.latency; }
    points[used++]=candidate_point;inflight=false;return true;
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
bool current_loop_report_poll(void)
{
    if (motor_runtime_output_is_busy() || board_serial_is_busy()) { return false; }
    if (!header_pending && !trip_pending && exported>=used) { return false; }
    bool sending_header=header_pending,sending_trip=!sending_header && trip_pending;
    line_length=5u;
    if (sending_header) {
        memcpy(line,"P1D0,",5u);word(BOARD_DQ_PROFILE_ID);word((uint32_t)BOARD_DQ_REFERENCE_MA);word(used);
        word(maximum_compute);word(maximum_latency);word(benchmark_step);word(benchmark_failed);word(action_start);
        word(offset.sum_u_count);word(offset.sum_v_count);word(calibration.zero_count);word(calibration.pole_pairs);
        motor_log_snapshot_t status;motor_action_t action;
        if (!motor_runtime_read_reply(&status,&action)) { return false; }
        word(status.fault_flags);word((uint32_t)action.reason);
    } else if (sending_trip) {
        memcpy(line,"P1E1,",5u);word(first_trip.reason);word(first_trip.sequence);word(first_trip.t);
        word(first_trip.u);word(first_trip.v);word(first_trip.angle);word((uint32_t)first_trip.detail);
    } else {
        loop_point_t point=points[exported];memcpy(line,"P1D1,",5u);word(exported);word(point.sequence);word(point.t);
        word((uint32_t)(int32_t)point.reference);word((uint32_t)(int32_t)point.id);word((uint32_t)(int32_t)point.iq);
        word((uint32_t)(int32_t)point.vd);word((uint32_t)(int32_t)point.vq);word(point.compute);word(point.latency);word(point.flags);word(point.angle);
    }
    uint16_t crc=0xffffu;for (unsigned i=0u;i<line_length;++i) { crc=crc_byte(crc,(uint8_t)line[i]); }
    static const char hex[]="0123456789ABCDEF";
    for (unsigned j=0u;j<4u;++j) { line[line_length++]=hex[(crc>>(12u-4u*j))&15u]; }
    line[line_length++]='\r';line[line_length++]='\n';
    if (board_start_serial_write(line,line_length)) { if (sending_header) { header_pending=false; } else if (sending_trip) { trip_pending=false; } else { ++exported; }return true; }
    return false;
}
#endif
