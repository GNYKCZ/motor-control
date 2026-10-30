/** 固定P1C1 23B / P1R2 95B / P1Z1 138B / P1V1 221B；无printf、浮点或动态分配。
 * RX状态仅在UART IRQ/短临界区修改；TX缓冲只由前台修改，TC前保持借用。 */
#include "command.h"
#include "board_io.h"
#include "motor_runtime.h"
#include "board_config.h"
#include <stddef.h>

enum { REPLY_OK=0, REPLY_FORMAT=1, REPLY_CRC=2, REPLY_SEQUENCE=3,
       REPLY_DENIED=4, REPLY_UART=5, REPLY_TIMEOUT=6, REPLY_SNAPSHOT=7 };
enum { REQUEST_STATUS=1, REQUEST_STOP=2, REQUEST_ZERO_STATUS=5, REQUEST_ZERO_RESTART=6,
       REQUEST_CURRENT_STATUS=7, REQUEST_SCAN_STATUS=8, REPLY_LOST=128 };
typedef struct { uint32_t sequence; uint8_t code, result; } reply_event_t;
static unsigned rx_index;
static uint32_t rx_sequence, rx_started_us, rx_last_us, last_sequence;
static uint16_t rx_crc, rx_received_crc;
static uint8_t rx_code;
static bool rx_bad, rx_drop, have_sequence;
static bool reply_pending;
static reply_event_t pending_reply;
static char reply_line[248];
static unsigned reply_length;
static unsigned reply_index;
static bool reply_building, reply_borrowed;
static uint16_t reply_crc;
static reply_event_t reply;
static motor_log_snapshot_t reply_status;
static motor_action_t reply_action;
static motor_zero_snapshot_t reply_zero;
static motor_current_snapshot_t reply_current;
static motor_scan_snapshot_t reply_scan;
static uint32_t scan_fields[43];
static const uint8_t scan_widths[43]={8,2,2,2,8,8,2,2,2,2,2,4,8,8,8,
    2,8,4,4,2,8,4,4,2,8,4,4,2,8,4,4,2,8,4,4,2,8,4,4,2,8,4,4};
static unsigned scan_field, scan_digits;

/** CCITT-FALSE: poly1021/initFFFF/no reflection/xorout0，最多8步。 */
static uint16_t crc_byte(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (unsigned i=0u; i<8u; ++i) {
        crc = (uint16_t)((crc & 0x8000u) ? ((uint32_t)crc << 1)^0x1021u : (uint32_t)crc << 1);
    }
    return crc;
}

/** 已持锁/IRQ：单槽可丢回执，STOP优先；高位标记前一待回执被丢，不承诺必达。 */
static void queue_reply(uint32_t sequence, uint8_t code, uint8_t result)
{
    if (reply_pending) {
        result |= REPLY_LOST;
        if (pending_reply.code==REQUEST_STOP && code!=REQUEST_STOP) {
            pending_reply.result |= REPLY_LOST;
            return;
        }
    }
    pending_reply = (reply_event_t){sequence,code,result};
    reply_pending=true;
}

static void reset_frame(void)
{
    rx_index=0u; rx_sequence=0u; rx_code=0u; rx_crc=0xffffu; rx_received_crc=0u; rx_bad=false;
}

void command_init(void)
{
    uint32_t mask=board_enter_critical();
    reset_frame(); rx_drop=false; have_sequence=false; last_sequence=0u;
    reply_pending=false; reply_building=false; reply_borrowed=false; reply_index=0u;
    board_leave_critical(mask);
}

/** 已持锁/IRQ，超时帧必须丢至LF；不是遇到新前缀就自动重同步。 */
static void expire_frame(uint32_t now_us)
{
    if (rx_index!=0u && !rx_drop &&
        ((now_us-rx_last_us>25000u && now_us-rx_last_us<0x80000000u) ||
         (now_us-rx_started_us>250000u && now_us-rx_started_us<0x80000000u))) {
        queue_reply(rx_sequence,rx_code,REPLY_TIMEOUT);
        motor_runtime_communication_error();
        reset_frame(); rx_drop=true;
    }
}

void command_receive_error(void)
{
    motor_runtime_communication_error();
    queue_reply(0u,0u,REPLY_UART);
    reset_frame(); rx_drop=true;
}

/** 每次只解析一个hex位；小写/非hex拒绝，不能忽略非法字符再拼出命令。 */
static bool hex_value(uint8_t byte, uint8_t *value)
{
    if (byte>='0' && byte<='9') { *value=(uint8_t)(byte-'0'); return true; }
    if (byte>='A' && byte<='F') { *value=(uint8_t)(byte-'A'+10u); return true; }
    return false;
}

void command_receive_byte(uint8_t byte, uint32_t now_us)
{
    static const char prefix[]="P1C1,";
    expire_frame(now_us);
    if (rx_drop) {
        if (byte=='\n') { rx_drop=false; reset_frame(); }
        return;
    }
    if (rx_index==0u) { rx_started_us=now_us; }
    rx_last_us=now_us;
    unsigned i=rx_index;
    uint8_t digit=0u;
    if (i<5u) { if (byte!=(uint8_t)prefix[i]) { rx_bad=true; } }
    else if (i>=5u && i<=12u) {
        if (!hex_value(byte,&digit)) { rx_bad=true; }
        rx_sequence=(rx_sequence<<4)|digit;
    } else if (i==13u || i==16u) { if (byte!=',') { rx_bad=true; } }
    else if (i==14u || i==15u) {
        if (!hex_value(byte,&digit)) { rx_bad=true; }
        rx_code=(uint8_t)((rx_code<<4)|digit);
    } else if (i>=17u && i<=20u) {
        if (!hex_value(byte,&digit)) { rx_bad=true; }
        rx_received_crc=(uint16_t)((rx_received_crc<<4)|digit);
    } else if (i==21u) { if (byte!='\r') { rx_bad=true; } }
    else if (i==22u) { if (byte!='\n') { rx_bad=true; } }
    if (i<17u) { rx_crc=crc_byte(rx_crc,byte); }
    if (byte=='\n') {
        uint8_t result=REPLY_OK;
        if (i!=22u || rx_bad) { result=REPLY_FORMAT; }
        else if (rx_crc!=rx_received_crc) { result=REPLY_CRC; }
        else {
            /* STOP在完整CRC正确时先执行；即使序号重复/过期、邮箱满、TX忙也不能等待回执。 */
            if (rx_code==REQUEST_STOP) { motor_runtime_stop(); }
            uint32_t advance=rx_sequence-last_sequence;
            if (have_sequence && (advance==0u || advance>=0x80000000u)) { result=REPLY_SEQUENCE; }
            else {
                have_sequence=true; last_sequence=rx_sequence;
                if (rx_code==REQUEST_ZERO_RESTART) {
                    if (!motor_runtime_restart_zero()) { result=REPLY_DENIED; }
                } else if (rx_code!=REQUEST_STATUS && rx_code!=REQUEST_STOP && rx_code!=REQUEST_ZERO_STATUS
                           && rx_code!=(BOARD_SCAN_ENABLED ? REQUEST_SCAN_STATUS : REQUEST_CURRENT_STATUS)) {
                    result=REPLY_DENIED;
                }
            }
        }
        if (result!=REPLY_OK) { motor_runtime_communication_error(); }
        queue_reply(rx_sequence,rx_code,result);
        reset_frame();
    } else if (i>=22u) {
        motor_runtime_communication_error();
        queue_reply(rx_sequence,rx_code,REPLY_FORMAT);
        reset_frame(); rx_drop=true; /* 第23字节不是LF，整条物理行都拒绝。 */
    } else { rx_index=i+1u; }
}

static char hex_digit(uint32_t value, unsigned shift)
{
    static const char hex[]="0123456789ABCDEF";
    return hex[(value>>shift)&15u];
}

/** 固定位置输出一字符；状态已经冻结，不在组帧过程中混读运行状态。 */
static char reply_character(unsigned i)
{
    if (reply.code==REQUEST_SCAN_STATUS && BOARD_SCAN_ENABLED) {
        static const char prefix[]="P1A1,";
        if (i<5u) { return prefix[i]; }
        if (i>=242u && i<=245u) { return hex_digit(reply_crc,(245u-i)*4u); }
        if (i>=246u) { return i==246u ? '\r' : '\n'; }
        if (!scan_digits) {
            scan_field++;
            if (scan_field<43u) { scan_digits=scan_widths[scan_field]; }
            return ',';
        }
        scan_digits--; return hex_digit(scan_fields[scan_field],scan_digits*4u);
    }
    if (reply.code==REQUEST_ZERO_STATUS || (reply.code==REQUEST_CURRENT_STATUS && !BOARD_SCAN_ENABLED)) {
        const char *prefix=reply.code==REQUEST_ZERO_STATUS ? "P1Z1," : "P1V1,";
        if (i<5u) { return prefix[i]; }
        if (i==13u || i==16u || i==19u || i==22u || i==31u || i==40u || i==49u
            || i==54u || i==63u || i==72u || i==77u || i==82u || i==87u || i==92u
            || i==95u || i==104u || i==113u || i==122u || i==131u) { return ','; }
        if (i<=12u) { return hex_digit(reply.sequence,(12u-i)*4u); }
        if (i<=15u) { return hex_digit(reply.code,(15u-i)*4u); }
        if (i<=18u) { return hex_digit(reply.result,(18u-i)*4u); }
        if (i<=21u) { return hex_digit((uint32_t)reply_status.state,(21u-i)*4u); }
        if (i<=30u) { return hex_digit(reply_status.fault_flags,(30u-i)*4u); }
        if (i<=39u) { return hex_digit(reply_status.sequence,(39u-i)*4u); }
        if (i<=48u) { return hex_digit(reply_status.completed_us,(48u-i)*4u); }
        if (i<=53u) { return hex_digit(reply_zero.offset.accepted_count,(53u-i)*4u); }
        if (i<=62u) { return hex_digit(reply_zero.offset.sum_u_count,(62u-i)*4u); }
        if (i<=71u) { return hex_digit(reply_zero.offset.sum_v_count,(71u-i)*4u); }
        if (i<=76u) { return hex_digit(reply_zero.offset.minimum_u_count,(76u-i)*4u); }
        if (i<=81u) { return hex_digit(reply_zero.offset.maximum_u_count,(81u-i)*4u); }
        if (i<=86u) { return hex_digit(reply_zero.offset.minimum_v_count,(86u-i)*4u); }
        if (i<=91u) { return hex_digit(reply_zero.offset.maximum_v_count,(91u-i)*4u); }
        if (i<=94u) { return hex_digit(reply_zero.flags,(94u-i)*4u); }
        if (i<=103u) { return hex_digit(reply_zero.scale_nA,(103u-i)*4u); }
        if (i<=112u) { return hex_digit(reply_zero.began_us,(112u-i)*4u); }
        if (i<=121u) { return hex_digit(reply_zero.ended_us,(121u-i)*4u); }
        if (i<=130u) { return hex_digit(reply_zero.profile_id,(130u-i)*4u); }
        if (reply.code==REQUEST_CURRENT_STATUS) {
            if (i==136u || i==145u || i==154u || i==159u || i==164u || i==169u
                || i==174u || i==183u || i==192u || i==197u || i==202u || i==205u || i==214u) { return ','; }
            if (i<=135u) { return hex_digit(reply_current.samples.accepted_count,(135u-i)*4u); }
            if (i<=144u) { return hex_digit(reply_current.samples.sum_u_count,(144u-i)*4u); }
            if (i<=153u) { return hex_digit(reply_current.samples.sum_v_count,(153u-i)*4u); }
            if (i<=158u) { return hex_digit(reply_current.samples.minimum_u_count,(158u-i)*4u); }
            if (i<=163u) { return hex_digit(reply_current.samples.maximum_u_count,(163u-i)*4u); }
            if (i<=168u) { return hex_digit(reply_current.samples.minimum_v_count,(168u-i)*4u); }
            if (i<=173u) { return hex_digit(reply_current.samples.maximum_v_count,(173u-i)*4u); }
            if (i<=182u) { return hex_digit(reply_current.began_us,(182u-i)*4u); }
            if (i<=191u) { return hex_digit(reply_current.ended_us,(191u-i)*4u); }
            if (i<=196u) { return hex_digit(reply_current.first_angle,(196u-i)*4u); }
            if (i<=201u) { return hex_digit(reply_current.last_angle,(201u-i)*4u); }
            if (i<=204u) { return hex_digit(reply_current.overflow,(204u-i)*4u); }
            if (i<=213u) { return hex_digit(reply_current.action_id,(213u-i)*4u); }
            if (i<=218u) { return hex_digit(reply_crc,(218u-i)*4u); }
            return i==219u ? '\r' : '\n';
        }
        if (i<=135u) { return hex_digit(reply_crc,(135u-i)*4u); }
        return i==136u ? '\r' : '\n';
    }
    static const char prefix[]="P1R2,";
    if (i<5u) { return prefix[i]; }
    if (i<=12u) { return hex_digit(reply.sequence,(12u-i)*4u); }
    if (i==13u || i==16u || i==19u || i==22u || i==31u || i==40u || i==49u || i==58u || i==67u || i==76u || i==85u || i==88u) { return ','; }
    if (i<=15u) { return hex_digit(reply.code,(15u-i)*4u); }
    if (i<=18u) { return hex_digit(reply.result,(18u-i)*4u); }
    if (i<=21u) { return hex_digit((uint32_t)reply_status.state,(21u-i)*4u); }
    if (i<=30u) { return hex_digit(reply_status.fault_flags,(30u-i)*4u); }
    if (i<=39u) { return hex_digit(reply_status.sequence,(39u-i)*4u); }
    if (i<=48u) { return hex_digit(reply_status.completed_us,(48u-i)*4u); }
    if (i<=57u) { return hex_digit(reply_action.id,(57u-i)*4u); }
    if (i<=66u) { return hex_digit(reply_action.requested_us,(66u-i)*4u); }
    if (i<=75u) { return hex_digit(reply_action.started_us,(75u-i)*4u); }
    if (i<=84u) { return hex_digit(reply_action.ended_us,(84u-i)*4u); }
    if (i<=87u) { return hex_digit((uint32_t)reply_action.reason|reply_action.timing_flags,(87u-i)*4u); }
    if (i<=92u) { return hex_digit(reply_crc,(92u-i)*4u); }
    return i==93u ? '\r' : '\n';
}

bool command_poll(uint32_t now_us)
{
    uint32_t mask=board_enter_critical();
    expire_frame(now_us);
    board_leave_critical(mask);
    if (reply_borrowed) {
        if (board_serial_is_busy()) { return true; }
        reply_borrowed=false;
    }
    if (!reply_building) {
        mask=board_enter_critical();
        bool pending=reply_pending;
        if (pending) { reply=pending_reply; reply_pending=false; }
        board_leave_critical(mask);
        if (!pending) { return false; }
        if (!motor_runtime_read_reply(&reply_status,&reply_action)) {
            motor_log_snapshot_t empty={0}; motor_action_t no_action={0}; reply_status=empty; reply_action=no_action; reply.result=REPLY_SNAPSHOT;
        }
        reply_length=95u;
        if (reply.code==REQUEST_ZERO_STATUS) {
            reply_length=138u;
            if (!motor_runtime_read_zero(&reply_status,&reply_zero)) { reply.result=REPLY_SNAPSHOT; }
        } else if (reply.code==REQUEST_CURRENT_STATUS && !BOARD_SCAN_ENABLED) {
            reply_length=221u;
            if (!motor_runtime_read_current(&reply_status,&reply_zero,&reply_current)) { reply.result=REPLY_SNAPSHOT; }
        } else if (reply.code==REQUEST_SCAN_STATUS && BOARD_SCAN_ENABLED) {
            reply_length=sizeof(reply_line);
            if (!motor_runtime_read_scan(&reply_status,&reply_scan)) { reply.result=REPLY_SNAPSHOT; }
            uint32_t header[15]={reply.sequence,reply.code,reply.result,(uint32_t)reply_status.state,
                reply_status.fault_flags,reply_scan.action_id,(uint8_t)reply_scan.direction,reply_scan.step,
                reply_scan.done,reply_scan.reason,reply_scan.mask,reply_scan.origin,reply_scan.started_us,
                reply_scan.ended_us,0x00010001u};
            for (unsigned j=0u;j<15u;++j) { scan_fields[j]=header[j]; }
            for (unsigned j=0u;j<7u;++j) {
                scan_fields[15u+4u*j]=reply_scan.point[j].count;
                scan_fields[16u+4u*j]=(uint32_t)reply_scan.point[j].sum;
                scan_fields[17u+4u*j]=(uint16_t)reply_scan.point[j].minimum;
                scan_fields[18u+4u*j]=(uint16_t)reply_scan.point[j].maximum;
            }
            scan_field=0u; scan_digits=scan_widths[0];
        }
        reply_index=0u; reply_crc=0xffffu; reply_building=true;
        return true;
    }
    if (board_serial_is_busy()) { return true; }
    char byte=reply_character(reply_index);
    reply_line[reply_index]=byte;
    if (reply_index<reply_length-6u) { reply_crc=crc_byte(reply_crc,(uint8_t)byte); }
    reply_index++;
    if (reply_index==reply_length) {
        reply_building=false;
        reply_borrowed=board_start_serial_write(reply_line,reply_length);
    }
    return true;
}
