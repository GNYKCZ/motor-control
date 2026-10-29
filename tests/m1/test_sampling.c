/** M1 主机验证：调用真实帧/runtime/telemetry，仅替换外设与 IRQ 屏蔽接口。
 * 判断依据来自协议多项式长除法、手工时间/状态轨迹，不由待测实现产生 expected。
 * 不模拟物理电流，不代表真实 DMA 时序、引脚电平或电机测试。 */
#include "sample_frame.h"
#include "motor_runtime.h"
#include "telemetry.h"
#include "board_io.h"
#include "board_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

static uint32_t interrupt_mask;       /**< 假设备 PRIMASK，允许验证嵌套恢复。 */
static uint32_t disable_count;        /**< 禁止调制接口调用次数。 */
static uint32_t device_now_us;        /**< 手工设备时间。 */
static bool serial_busy;              /**< 手工串口拥塞条件。 */
static const char *borrowed_line;     /**< 验证忙期间缓冲内容不变。 */
static char captured_line[BOARD_SERIAL_BUFFER_BYTES]; /**< 最近成功发送的副本。 */
static motor_sample_t pending_sample; /**< 中断屏蔽期间等待发布的下一完整包。 */
static bool has_pending_sample;       /**< 解除屏蔽时触发一次模拟 ADC 完成。 */
static uint32_t advance_after_unlock_us; /**< RV-001：取邮箱解锁后推进真实假设备时钟。 */
static uint32_t clock_read_increment_us; /**< 仅计时测试：每次读时钟固定推进，独立于真实执行时间。 */

/** 外设替身：记录门控调用，不冒充测量实际功率桥。 */
void board_disable_output(void) { disable_count++; }
/** 模拟 Cortex 的保存/屏蔽操作；返回进入前的值以测试嵌套。 */
uint32_t board_enter_critical(void)
{
    uint32_t old_mask = interrupt_mask;
    interrupt_mask = 1u;
    return old_mask;
}
/** 恢复旧 mask；延迟事件只能在解除屏蔽后发生，模拟一次发布/读取交错。 */
void board_leave_critical(uint32_t old_mask)
{
    assert(interrupt_mask == 1u);
    interrupt_mask = old_mask;
    if (interrupt_mask == 0u && advance_after_unlock_us != 0u) {
        device_now_us += advance_after_unlock_us;
        advance_after_unlock_us = 0u;
    }
    if (interrupt_mask == 0u && has_pending_sample) {
        has_pending_sample = false;
        motor_runtime_handle_sample(&pending_sample);
    }
}
/** 手工时间来源；由测试设置，和电脑时钟无关。 */
uint32_t board_get_time_us(void)
{
    uint32_t result = device_now_us;
    device_now_us += clock_read_increment_us;
    return result;
}
/** 手工串口忙状态，仅用于验证前台不覆盖缓冲。 */
bool board_serial_is_busy(void) { return serial_busy; }
/** 捕获协议文本；模拟成功提交后仍持有原缓冲，直至测试解除 busy。 */
bool board_start_serial_write(const char *bytes, uint32_t length)
{
    assert(length < sizeof(captured_line));
    assert(!serial_busy);
    memcpy(captured_line, bytes, length);
    captured_line[length] = '\0';
    borrowed_line = bytes;
    serial_busy = true;
    return true;
}

/** 独立 oracle：把 18 bit 数据乘 x^6，再用 0x43 做二进制长除法。
 * 与待测实现的 6 bit 移位寄存器不同，比较完整 24 bit 多项式余数。 */
static uint8_t reference_crc(uint32_t data)
{
    uint32_t dividend = data << 6;
    for (int leading = 23; leading >= 6; --leading) {
        if ((dividend & (1u << leading)) != 0u) {
            dividend ^= 0x43u << (leading - 6);
        }
    }
    return (uint8_t)dividend;
}

/** 根据协议字段构造 wire 包；仅测试数据生成使用独立长除法 oracle。 */
static void make_packet(uint16_t angle, uint8_t status, uint8_t bytes[3])
{
    uint32_t data = ((uint32_t)angle << 4) | status;
    uint32_t packet = (data << 6) | reference_crc(data);
    bytes[0] = (uint8_t)(packet >> 16);
    bytes[1] = (uint8_t)(packet >> 8);
    bytes[2] = (uint8_t)packet;
}

/** 构造一个手工时间对齐的有效样本；序号可通过参数覆盖回绕边界。 */
static motor_sample_t make_sample(uint32_t sequence, uint32_t time_us)
{
    uint8_t bytes[3];
    make_packet((uint16_t)(sequence & 16383u), 0u, bytes);
    board_encoder_frame_t encoder = board_decode_encoder_frame(bytes, 3u,
        sequence, time_us - 50u, time_us - 28u);
    return board_make_sample(sequence, time_us, 1234u, 2345u, &encoder);
}

/** 对全部角度、状态和 24 位单 bit 损坏进行协议独立验证，另验半帧与时间上限。 */
static void test_encoder_frames(void)
{
    uint8_t bytes[3];
    for (uint32_t angle = 0u; angle <= 16383u; ++angle) {
        make_packet((uint16_t)angle, 0u, bytes);
        board_encoder_frame_t frame = board_decode_encoder_frame(bytes, 3u, 77u, 100u, 122u);
        assert(frame.is_valid && frame.raw_angle_count == angle && frame.sequence == 77u);
        for (uint32_t bit = 0u; bit < 24u; ++bit) {
            bytes[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
            frame = board_decode_encoder_frame(bytes, 3u, 78u, 100u, 122u);
            assert(!frame.is_valid);
            bytes[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
        }
    }
    for (uint8_t status = 1u; status < 16u; ++status) {
        make_packet(100u, status, bytes);
        board_encoder_frame_t frame = board_decode_encoder_frame(bytes, 3u, 1u, 0u, 22u);
        assert(!frame.is_valid && frame.magnetic_status == status);
    }
    make_packet(16383u, 0u, bytes);
    for (uint32_t length = 0u; length < 3u; ++length) {
        assert(!board_decode_encoder_frame(bytes, length, 1u, 0u, 22u).is_valid);
    }
    assert(!board_decode_encoder_frame(NULL, 3u, 1u, 0u, 22u).is_valid);
    assert(!board_decode_encoder_frame(bytes, 4u, 1u, 0u, 22u).is_valid);
    assert(board_decode_encoder_frame(bytes, 3u, 1u, 100u, 140u).is_valid);
    assert(!board_decode_encoder_frame(bytes, 3u, 1u, 100u, 141u).is_valid);
    assert(!board_decode_encoder_frame(bytes, 3u, 1u, 100u, 99u).is_valid);
    assert(board_decode_encoder_frame(bytes, 3u, 1u, UINT32_MAX - 10u, 11u).is_valid);
    puts("PASS protocol: all 16384 angles, 393216 single-bit corruptions, status/half-frame/time");
}

/** 使用独立手工边界检查 ADC 范围、完成先后与编码器年龄，不能从原始缓冲拼半帧。 */
static void test_sample_boundaries(void)
{
    motor_sample_t original = make_sample(1u, 100u);
    assert(original.is_valid);
    assert(board_make_sample(1u, 250u, 0u, 4095u, &original.encoder).is_valid);
    assert(!board_make_sample(1u, 251u, 0u, 4095u, &original.encoder).is_valid);
    assert(!board_make_sample(1u, 71u, 0u, 0u, &original.encoder).is_valid);
    assert(!board_make_sample(1u, 100u, 4096u, 0u, &original.encoder).is_valid);
    assert(!board_make_sample(1u, 100u, 0u, 4096u, &original.encoder).is_valid);
    assert(!board_make_sample(1u, 100u, 0u, 0u, NULL).is_valid);
    assert(board_sequence_is_newer(0u, UINT32_MAX));
    assert(!board_sequence_is_newer(1u, 1u));
    assert(!board_sequence_is_newer(0u, 1u));
    assert(!board_sequence_is_newer(0x80000000u, 0u));
    puts("PASS sample: ADC limits, freshness, completion order, modulo order");
}

/** 读取快照的便利入口；其返回值仍由真实 runtime 构造，expected 由测试显式断言。 */
static motor_status_t read_status(void)
{
    motor_status_t status;
    assert(motor_runtime_read_status(&status));
    assert(!status.pwm_is_permanently_blocked && motor_runtime_output_profile_count()==1u);
    return status;
}

/** 正常路径明确让假设备当前时间跟随最新ADC；延迟回归直接调用真实poll。 */
static void poll_fresh_feedback(void)
{
    device_now_us = read_status().sample.current_completed_us;
    motor_runtime_poll_feedback();
}

/** 常规导出测试驱动多次前台迭代；专门的分步测试直接调用真实poll。 */
static void send_telemetry(uint32_t now_ms)
{
    telemetry_poll_status(now_ms);
    if (serial_busy) { return; }
    for (unsigned i = 0u; i < 16u && !serial_busy; ++i) { telemetry_poll_status(now_ms); }
}

/** 建立一次新启动；检查 BOOT、初始化成功后的 DISABLED，保持软件门控。 */
static void start_runtime(void)
{
    device_now_us = 0u;
    motor_runtime_init();
    assert(read_status().state == MOTOR_RUNTIME_BOOT);
    motor_runtime_finish_boot(true);
    assert(read_status().state == MOTOR_RUNTIME_DISABLED);
}

/** 验证初始化/输入异常不使能、故障锁存、序号回绕和未收到 ADC 时的失联检测。 */
static void test_runtime_states(void)
{
    assert(!motor_runtime_read_status(NULL));
    start_runtime();
    motor_sample_t first = make_sample(1u, 100u);
    motor_runtime_handle_sample(&first);
    assert(read_status().state == MOTOR_RUNTIME_DISABLED && read_status().has_valid_sample);
    motor_runtime_check_deadline(99u); /* 时间在前台取出后 ADC 才发布，不能误报超时。 */
    assert(read_status().state == MOTOR_RUNTIME_DISABLED);
    motor_runtime_handle_sample(&first); /* 重复完整包也必须拒绝。 */
    assert(read_status().state == MOTOR_RUNTIME_FAULT);
    motor_sample_t second = make_sample(2u, 200u);
    motor_runtime_handle_sample(&second);
    assert(read_status().state == MOTOR_RUNTIME_FAULT);

    start_runtime();
    first.is_valid = false;
    motor_runtime_handle_sample(&first); /* 首次等待，不把未完成帧当有效测量。 */
    assert(!read_status().has_valid_sample && read_status().state == MOTOR_RUNTIME_DISABLED);
    motor_runtime_check_deadline(1001u);
    assert(read_status().state == MOTOR_RUNTIME_FAULT);

    start_runtime();
    motor_runtime_check_deadline(1001u); /* 完全无 ADC 事件仍能检测。 */
    assert(read_status().state == MOTOR_RUNTIME_FAULT);
    start_runtime();
    device_now_us = UINT32_MAX - 100u;
    motor_runtime_init();
    motor_runtime_finish_boot(true);
    first = make_sample(UINT32_MAX, UINT32_MAX - 49u);
    second = make_sample(0u, 50u);
    motor_runtime_handle_sample(&first);
    motor_runtime_handle_sample(&second);
    assert(read_status().state == MOTOR_RUNTIME_DISABLED && read_status().sample.sequence == 0u);
    motor_runtime_check_deadline(351u);
    assert(read_status().state == MOTOR_RUNTIME_FAULT);

    start_runtime();
    first = make_sample(1u, 1001u);
    motor_runtime_handle_sample(&first);
    assert(read_status().state == MOTOR_RUNTIME_FAULT); /* 即使首帧有效，迟到仍不能绕过期限。 */

    for (unsigned int condition = 0u; condition < 6u; ++condition) {
        start_runtime();
        first = make_sample(1u, 100u);
        motor_runtime_handle_sample(&first);
        second = make_sample(2u, 200u);
        if (condition == 0u) second.raw_current_u_count = 4096u;
        if (condition == 1u) second.encoder.raw_angle_count = 16384u;
        if (condition == 2u) second.encoder.read_completed_us = 250u;
        if (condition == 3u) second.encoder.sequence = 1u;
        if (condition == 4u) second.current_completed_us = 100u;
        if (condition == 5u) second.current_completed_us = 401u;
        motor_runtime_handle_sample(&second);
        assert(read_status().state == MOTOR_RUNTIME_FAULT);
    }
    start_runtime();
    motor_runtime_handle_sample(NULL);
    assert(read_status().state == MOTOR_RUNTIME_FAULT);
    motor_runtime_init();
    motor_runtime_finish_boot(false);
    assert(read_status().fault_flags == MOTOR_FAULT_INITIALIZATION);
    motor_runtime_init();
    motor_runtime_handle_sample(&first); /* 不得在 BOOT 绕过初始化。 */
    assert(read_status().state == MOTOR_RUNTIME_FAULT);
    assert(disable_count > 20u);
    puts("PASS lifecycle: BOOT/DISABLED/FAULT, invalid/repeated/stale/missing inputs, no auto recovery");
}

/** 在快照复制解除屏蔽的一刻注入下一包，前后快照必须各自完整且嵌套不误开 IRQ。 */
static void test_snapshot_interleaving(void)
{
    start_runtime();
    motor_sample_t first = make_sample(1u, 100u);
    motor_runtime_handle_sample(&first);
    pending_sample = make_sample(2u, 200u);
    pending_sample.raw_current_u_count = 3333u;
    has_pending_sample = true;
    motor_status_t old = read_status();
    motor_status_t current = read_status();
    assert(old.sample.sequence == 1u && old.sample.current_completed_us == 100u);
    assert(old.sample.raw_current_u_count == 1234u && old.sample.encoder.sequence == 1u);
    assert(current.sample.sequence == 2u && current.sample.current_completed_us == 200u);
    assert(current.sample.raw_current_u_count == 3333u && current.sample.encoder.sequence == 2u);
    assert(old.estimated_offset.accepted_count==1u && old.estimated_offset.sum_u_count==1234u);
    assert(current.estimated_offset.accepted_count==2u && current.estimated_offset.sum_u_count==4567u);
    interrupt_mask = 1u;
    (void)read_status();
    assert(interrupt_mask == 1u);
    motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
    assert(interrupt_mask == 1u);
    interrupt_mask = 0u;
    assert(!motor_runtime_read_log(NULL));
    assert(sizeof(motor_log_snapshot_t) <= 48u && sizeof(motor_log_snapshot_t) < sizeof(motor_status_t));
    printf("PASS compact snapshot size: %u versus full status %u bytes\n",
        (unsigned)sizeof(motor_log_snapshot_t), (unsigned)sizeof(motor_status_t));
    start_runtime();
    motor_runtime_handle_sample(&first);
    pending_sample = make_sample(2u, 200u);
    pending_sample.raw_current_u_count = 3333u;
    has_pending_sample = true;
    motor_log_snapshot_t old_log, new_log;
    assert(motor_runtime_read_log(&old_log));
    assert(motor_runtime_read_log(&new_log));
    assert(old_log.sequence == 1u && old_log.raw_u == 1234u && old_log.raw_angle == 1u);
    assert(new_log.sequence == 2u && new_log.raw_u == 3333u && new_log.raw_angle == 2u);
    interrupt_mask = 1u;
    assert(motor_runtime_read_log(&new_log) && interrupt_mask == 1u);
    interrupt_mask = 0u;
    puts("PASS snapshot: interleaved publication is coherent; nested PRIMASK restored");
}

/** 验证实际导出序号/设备时间/原始量，模拟UART长期忙时缓冲不可变且有界放弃。 */
static void test_telemetry(void)
{
    start_runtime();
    motor_sample_t sample = make_sample(123u, 1000u);
    motor_runtime_handle_sample(&sample);
    send_telemetry(99u);
    assert(!serial_busy);
    send_telemetry(100u);
    assert(serial_busy);
    assert(strstr(captured_line, "M3B3,s=1,f=0,n=123,t=1000,") == captured_line);
    assert(strstr(captured_line, "u=1234,v=2345,a=123,ok=1,") != NULL);
    assert(strcmp(captured_line + strlen(captured_line) - 2u, "\r\n") == 0);
    char saved_line[BOARD_SERIAL_BUFFER_BYTES];
    strcpy(saved_line, borrowed_line);
    for (uint32_t time_ms = 200u; time_ms <= 1000u; time_ms += 100u) {
        send_telemetry(time_ms);
        assert(strcmp(borrowed_line, saved_line) == 0);
    }
    serial_busy = false;
    send_telemetry(1100u);
    assert(strstr(captured_line, "fo=0,") != NULL);
    serial_busy = false;
    send_telemetry(UINT32_MAX - 49u);
    serial_busy = false;
    send_telemetry(50u); /* 毫秒时间回绕后相隔 100 ms。 */
    assert(serial_busy);
    puts("PASS telemetry: exact device fields, congestion bounded, buffer ownership, time wrap");
}

/** 真实runtime连续512帧统计；异常不补数，FAULT/满批冻结，不再在日志中换算名义量。 */
static void test_offset_candidates(void)
{
    start_runtime();
    for(uint32_t i=0;i<512u;++i){
        motor_sample_t sample=make_sample(i+1u,(i+1u)*100u);
        sample.raw_current_u_count=(uint16_t)(2000u+i);
        sample.raw_current_v_count=(uint16_t)(3000u-i);
        motor_runtime_handle_sample(&sample);
        poll_fresh_feedback();
    }
    motor_status_t status=read_status();
    assert(status.state==MOTOR_RUNTIME_DISABLED && !status.calibration_verified && !status.offset_is_aborted);
    assert(status.estimated_offset.accepted_count==512u);
    assert(status.estimated_offset.sum_u_count==1154816u && status.estimated_offset.sum_v_count==1405184u);
    assert(status.estimated_offset.minimum_u_count==2000u && status.estimated_offset.maximum_u_count==2511u);
    sensor_offset_t complete=status.estimated_offset;
    motor_sample_t after=make_sample(513u,51300u);motor_runtime_handle_sample(&after);
    status=read_status();assert(memcmp(&status.estimated_offset,&complete,sizeof(complete))==0);
    serial_busy=false;send_telemetry(1000u);
    assert(strstr(captured_line,"n=513,t=51300,")!=NULL);
    assert(strstr(captured_line,"cn=")==NULL);
    assert(strstr(captured_line,"iu_mA=")==NULL && strstr(captured_line,"ma_mrad=")==NULL);
    // 行为新格式保持原字段含义，且单行最坏数字容量仍小于静态借用缓冲。
    assert(strlen(captured_line)<BOARD_SERIAL_BUFFER_BYTES);
    for(unsigned condition=0;condition<5;++condition){
        start_runtime();motor_sample_t first=make_sample(1u,100u);motor_runtime_handle_sample(&first);
        sensor_offset_t partial=read_status().estimated_offset;
        motor_sample_t bad=make_sample(2u,200u);
        if(condition==0)bad.is_valid=false;
        if(condition==1)bad.sequence=1u;
        if(condition==2)bad.current_completed_us=401u;
        if(condition==3)motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
        if(condition==4)motor_runtime_check_deadline(401u);
        if(condition<3)motor_runtime_handle_sample(&bad);
        motor_sample_t good=make_sample(3u,500u);motor_runtime_handle_sample(&good);
        status=read_status();assert(status.offset_is_aborted && status.estimated_offset.accepted_count==1u);
        assert(memcmp(&status.estimated_offset,&partial,sizeof(partial))==0);
        assert(!status.calibration_verified && status.state==MOTOR_RUNTIME_FAULT);
        serial_busy=false;send_telemetry(2000u+condition*100u);assert(strstr(captured_line,"M3B3,s=2,")==captured_line);
    }
    start_runtime();motor_sample_t invalid=make_sample(1u,100u);invalid.is_valid=false;
    motor_runtime_handle_sample(&invalid);motor_sample_t good=make_sample(2u,200u);motor_runtime_handle_sample(&good);
    status=read_status();assert(status.state==MOTOR_RUNTIME_DISABLED && status.offset_is_aborted);
    assert(status.estimated_offset.accepted_count==0 && status.offset_rejected_count==1);
    for(unsigned jump=0;jump<2;++jump){
        start_runtime();motor_sample_t first=make_sample(1u,100u);motor_runtime_handle_sample(&first);
        motor_sample_t missing=make_sample(3u,200u);
        if(jump==0)missing.encoder.sequence=2u;else missing.sequence=2u;
        motor_runtime_handle_sample(&missing);status=read_status();
        assert(status.state==MOTOR_RUNTIME_FAULT && status.sample.is_valid);
        assert(status.fault_flags & MOTOR_FAULT_FEEDBACK);
        assert(status.offset_is_aborted && status.offset_rejected_count==1 && status.estimated_offset.accepted_count==1);
    }
    puts("PASS M2c runtime: continuous 512 candidates, freeze on invalid/duplicate/time/fault, no verified calibration, compact telemetry without nominal conversion");
}

/** M3b独立整数轨迹：原始序号与ADC序号错开，完整逐帧交接后核验前台结果。
 * 跨圈期望来自预定整数位移，不以另一遍feedback_step作为oracle。 */
static void test_feedback_integration(void)
{
    const double count_rad = 6.283185307179586476925286766559 / 16384.0;
    start_runtime();
    motor_sample_t warmup = make_sample(16u, 100u);
    warmup.is_valid = false;
    motor_runtime_handle_sample(&warmup);
    for (uint32_t i = 0u; i <= 80u; ++i) {
        motor_sample_t sample = make_sample(17u + i, 200u + 100u * i);
        uint8_t packet[3];
        make_packet((uint16_t)((16380u + i) % 16384u), 0u, packet);
        sample.encoder = board_decode_encoder_frame(packet, 3u, 900u + i,
            sample.current_completed_us - 50u, sample.current_completed_us - 28u);
        motor_runtime_handle_sample(&sample);
        poll_fresh_feedback();
        motor_status_t status = read_status();
        assert(status.state == MOTOR_RUNTIME_DISABLED && !status.fault_flags);
        uint32_t processed = i / 10u * 10u;
        assert(status.feedback_sequence == 900u + processed);
        assert(status.feedback.continuous_count == (int64_t)processed);
        assert(status.feedback.is_valid && status.feedback.is_first_sample == (i < 10u));
        assert(status.feedback.direct_is_valid == (i >= 10u));
        assert(status.feedback.window_is_valid == (i >= 80u));
        assert(fabs(status.feedback.continuous_angle_rad - processed * count_rad) < 2e-12);
        if (i >= 10u) {
            assert(status.feedback.dt_us == 1000u);
            assert(fabs(status.feedback.direct_speed_rad_s - 10000.0 * count_rad) < 2e-12);
        }
        if (i == 0u || i == 10u || i == 80u) {
            serial_busy = false;
            send_telemetry(3000u + i * 100u);
            assert(strstr(captured_line, "M3B3,") == captured_line);
            puts(captured_line); /* test_m1.py再用真实capture解析器复核完整链路。 */
        }
    }
    motor_status_t final = read_status();
    assert(final.feedback.window_dt_us == 8000u && final.feedback.window_delta_count == 80);
    assert(fabs(final.feedback.window_speed_rad_s - 10000.0 * count_rad) < 2e-12);
    assert(final.feedback_raw_count == 76u);
    char held[BOARD_SERIAL_BUFFER_BYTES];
    strcpy(held, borrowed_line);
    for (uint32_t i = 81u; i < 100u; ++i) {
        motor_sample_t sample = make_sample(17u + i, 200u + 100u * i);
        sample.encoder.sequence = 900u + i;
        sample.encoder.raw_angle_count = (uint16_t)((16380u + i) % 16384u);
        motor_runtime_handle_sample(&sample); poll_fresh_feedback();
        send_telemetry(12000u + i * 100u);
        assert(strcmp(held, borrowed_line) == 0);
        assert(read_status().state == MOTOR_RUNTIME_DISABLED);
    }
    motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
    assert(!read_status().feedback.is_valid && read_status().feedback_sequence == 0u);
    poll_fresh_feedback();
    assert(!read_status().feedback.is_valid);

    /* 反向跨圈与非标称但允许的300us原始间隔，速度必须按实际3000us算。 */
    start_runtime();
    for (uint32_t i = 0u; i <= 10u; ++i) {
        motor_sample_t sample = make_sample(i + 1u, 100u + i * 300u);
        sample.encoder.raw_angle_count = (uint16_t)((16388u - i) % 16384u);
        motor_runtime_handle_sample(&sample); poll_fresh_feedback();
    }
    final = read_status();
    assert(final.state == MOTOR_RUNTIME_DISABLED && final.feedback.continuous_count == -10);
    assert(final.feedback.dt_us == 3000u);
    assert(fabs(final.feedback.direct_speed_rad_s + 10.0 * count_rad / 0.003) < 2e-12);

    /* 不调用前台，第二个抽帧到达时只能锁故障，不能覆盖/吞掉第一个。 */
    start_runtime();
    for (uint32_t i = 1u; i <= 11u; ++i) {
        motor_sample_t sample = make_sample(i, i * 100u);
        motor_runtime_handle_sample(&sample);
    }
    final = read_status();
    assert(final.state == MOTOR_RUNTIME_FAULT && final.fault_flags == MOTOR_FAULT_FEEDBACK);
    assert(final.feedback_overruns == 1u && !final.feedback.is_valid);
    assert(final.mailbox_wait_max_us == 1000u);
    serial_busy = false; send_telemetry(30000u);
    assert(strstr(captured_line, "M3B3,s=2,f=8,") == captured_line);
    assert(strstr(captured_line, "fo=1,fb_us=0,mb_us=1000,") != NULL);
    poll_fresh_feedback();
    assert(!read_status().feedback.is_valid);

    /* 非抽帧点丢失也必须被每原始帧检查捕获，ADC和encoder分别覆盖。 */
    for (unsigned channel = 0u; channel < 2u; ++channel) {
        start_runtime();
        motor_sample_t first = make_sample(1u, 100u);
        motor_runtime_handle_sample(&first); poll_fresh_feedback();
        motor_sample_t jump = make_sample(2u, 200u);
        if (channel == 0u) jump.sequence = 3u; else jump.encoder.sequence = 3u;
        motor_runtime_handle_sample(&jump);
        assert(read_status().fault_flags & MOTOR_FAULT_FEEDBACK);
        assert(!read_status().feedback.is_valid);
    }
    /* 半圈歧义/超过先验速度：IRQ只交接，前台计算拒绝并锁存。 */
    for (unsigned failure = 0u; failure < 2u; ++failure) {
        start_runtime();
        for (uint32_t i = 0u; i <= 10u; ++i) {
            motor_sample_t sample = make_sample(i + 1u, (i + 1u) * 100u);
            sample.encoder.raw_angle_count = i == 10u ? (failure ? 1000u : 8192u) : 0u;
            motor_runtime_handle_sample(&sample); poll_fresh_feedback();
        }
        assert(read_status().fault_flags & MOTOR_FAULT_FEEDBACK);
        assert(!read_status().feedback.is_valid);
    }
    /* 释放取邮箱锁后立即发生设备故障：计算不能重新发布为有效。 */
    start_runtime();
    motor_sample_t first = make_sample(1u, 100u);
    motor_runtime_handle_sample(&first);
    pending_sample = make_sample(2u, 200u); pending_sample.is_valid = false;
    has_pending_sample = true;
    device_now_us = 100u;
    motor_runtime_poll_feedback();
    assert(read_status().state == MOTOR_RUNTIME_FAULT && !read_status().feedback.is_valid);

    /* 新初始化明确重建原点；时钟/原始序号回绕仍保持真实步长10。 */
    start_runtime(); device_now_us = UINT32_MAX - 500u;
    motor_runtime_init(); motor_runtime_finish_boot(true);
    for (uint32_t i = 0u; i <= 10u; ++i) {
        motor_sample_t sample = make_sample(UINT32_MAX - 5u + i, UINT32_MAX - 400u + i * 100u);
        sample.encoder.raw_angle_count = 123u;
        motor_runtime_handle_sample(&sample); poll_fresh_feedback();
    }
    final = read_status();
    assert(final.state == MOTOR_RUNTIME_DISABLED && final.feedback_sequence == 4u);
    assert(final.feedback.continuous_count == 0 && final.feedback.direct_speed_rad_s == 0.0);
    motor_runtime_check_deadline(final.sample.current_completed_us + 301u);
    assert(!read_status().feedback.is_valid);
    puts("PASS M3b integration: real stride, first/warmup/window, wrap, raw gaps, overrun, fault race, reset/deadline");
}

/** RV-001：deadline已通过后，前台取邮箱前/计算期间停采样；过期反馈不得先导出。 */
static void test_feedback_publish_age(void)
{
    for (unsigned condition = 0u; condition < 4u; ++condition) {
        start_runtime();
        uint32_t time = condition >= 2u ? UINT32_MAX - 100u : 100u;
        device_now_us = time - 50u;
        motor_runtime_init(); motor_runtime_finish_boot(true);
        motor_sample_t sample = make_sample(17u, time);
        motor_runtime_handle_sample(&sample);
        device_now_us = time;
        motor_runtime_check_deadline(device_now_us);
        assert(read_status().state == MOTOR_RUNTIME_DISABLED);
        if (condition % 2u == 0u) device_now_us += 5000u;
        else advance_after_unlock_us = 5000u; /* 只在poll取邮箱后的解锁发生。 */
        motor_runtime_poll_feedback();
        motor_status_t status = read_status();
        assert(status.state == MOTOR_RUNTIME_FAULT && (status.fault_flags & MOTOR_FAULT_SAMPLE));
        assert(status.fault_flags & MOTOR_FAULT_FEEDBACK);
        assert(!status.feedback.is_valid && !status.feedback.direct_is_valid && !status.feedback.window_is_valid);
        serial_busy = false; send_telemetry(50000u + condition * 100u);
        assert(strstr(captured_line, "M3B3,s=2,") == captured_line);
        assert(strstr(captured_line, "fp=0,fd=0,fw=0,") != NULL);
    }
    /* 原始断流门限300us也要在本轮发布时检查，即使反馈年龄尚未达3000us。 */
    for (uint32_t delay = 300u; delay <= 301u; ++delay) {
        start_runtime();
        motor_sample_t sample = make_sample(1u, 100u);
        motor_runtime_handle_sample(&sample);
        device_now_us = 100u;
        motor_runtime_check_deadline(device_now_us);
        device_now_us += delay;
        motor_runtime_poll_feedback();
        assert(read_status().feedback.is_valid == (delay == 300u));
        assert(read_status().state == (delay == 300u ? MOTOR_RUNTIME_DISABLED : MOTOR_RUNTIME_FAULT));
    }
    /* 取输入后有正常新帧发布：锁内取当前时间不能把新ADC误判成“来自未来”。 */
    start_runtime();
    motor_sample_t sample = make_sample(1u, 100u);
    motor_runtime_handle_sample(&sample);
    device_now_us = 100u;
    pending_sample = make_sample(2u, 200u);
    advance_after_unlock_us = 100u; has_pending_sample = true;
    motor_runtime_poll_feedback();
    motor_status_t status = read_status();
    assert(status.state == MOTOR_RUNTIME_DISABLED && status.feedback.is_valid);
    assert(status.sample.sequence == 2u && status.feedback_sequence == 1u);
    assert(interrupt_mask == 0u);
    puts("PASS RV-001: current-clock publication, delay before/during compute, time wrap, 300us boundary, fresh IRQ interleave");
}

/** 分步调度oracle：每次前台调用之间插入两个100us原始帧，绝不把主机耗时当板端耗时。 */
static void test_telemetry_scheduling(void)
{
    start_runtime();
    motor_sample_t first = make_sample(1u, 100u);
    motor_runtime_handle_sample(&first);
    clock_read_increment_us = 7u;
    poll_fresh_feedback();
    motor_status_t timed = read_status();
    assert(timed.feedback_poll_max_us == 14u);
    serial_busy = false;
    uint32_t sequence = 1u;
    for (unsigned part = 0u; part < 17u; ++part) {
        for (unsigned tick = 0u; tick < 2u; ++tick) {
            sequence++;
            motor_sample_t sample = make_sample(sequence, sequence * 100u);
            motor_runtime_handle_sample(&sample);
        }
        poll_fresh_feedback();
        telemetry_poll_status(100000u);
        assert(serial_busy == (part == 16u)); /* 一次poll不能吞掉后续格式步骤。 */
        assert(read_status().state == MOTOR_RUNTIME_DISABLED && !read_status().feedback_overruns);
    }
    assert(sequence == 35u);
    assert(strstr(captured_line, "M3B3,s=1,f=0,n=3,t=300,") == captured_line);
    assert(strstr(captured_line, "u=1234,v=2345,a=3,ok=1,fp=1,fd=0,fw=0,") != NULL); /* 同行各段始终是准备时那份快照。 */
    assert(strstr(captured_line, "fe=") == NULL);
    assert(strstr(captured_line, "fb_us=14,") != NULL);
    char held[BOARD_SERIAL_BUFFER_BYTES]; strcpy(held, borrowed_line);
    for (unsigned tick = 0u; tick < 20u; ++tick) {
        sequence++;
        motor_sample_t sample = make_sample(sequence, sequence * 100u);
        motor_runtime_handle_sample(&sample); poll_fresh_feedback();
        telemetry_poll_status(100100u);
        assert(strcmp(held, borrowed_line) == 0);
        assert(read_status().state == MOTOR_RUNTIME_DISABLED);
    }
    /* 组装中途原始帧失败：本行保持旧快照，下一行明确记录新故障及保留诊断。 */
    serial_busy = false;
    telemetry_poll_status(100200u);
    sequence++;
    motor_sample_t bad = make_sample(sequence, sequence * 100u); bad.is_valid = false;
    motor_runtime_handle_sample(&bad);
    assert(read_status().state == MOTOR_RUNTIME_FAULT);
    for (unsigned part = 1u; part < 17u; ++part) { telemetry_poll_status(100200u); }
    assert(serial_busy && strstr(captured_line, "M3B3,s=1,f=0,") == captured_line);
    serial_busy = false; send_telemetry(100300u);
    assert(strstr(captured_line, "M3B3,s=2,") == captured_line);
    assert(strstr(captured_line, "fp=0,fd=0,fw=0,") != NULL);
    assert(strstr(captured_line, "log_us=7,log_id=0") != NULL);
    assert(strstr(captured_line, "tp13_us=") == NULL);
    assert(read_status().feedback_poll_max_us == 14u); /* 故障失效不擦除耗时证据。 */
    clock_read_increment_us = 0u;
    puts("PASS M3b scheduling: fixed format slices, intervening raw frames, frozen snapshot, backpressure/fault, retained timing");
}

/** 计时整数oracle：观测过程跨uint32回绕，等待最大值单调且故障后保留。 */
static void test_timing_wrap(void)
{
    start_runtime();
    device_now_us = UINT32_MAX - 200u;
    motor_runtime_init(); motor_runtime_finish_boot(true);
    motor_sample_t sample = make_sample(41u, UINT32_MAX - 100u);
    motor_runtime_handle_sample(&sample);
    device_now_us = UINT32_MAX - 5u; clock_read_increment_us = 7u;
    motor_runtime_poll_feedback();
    motor_status_t status = read_status();
    assert(status.feedback.is_valid && status.mailbox_wait_max_us == 95u);
    assert(status.feedback_poll_max_us == 14u);
    for (uint32_t i = 1u; i <= 10u; ++i) {
        sample = make_sample(41u + i, UINT32_MAX - 100u + i * 100u);
        motor_runtime_handle_sample(&sample);
    }
    device_now_us = sample.current_completed_us + 50u;
    motor_runtime_poll_feedback(); status = read_status();
    assert(status.feedback.is_valid && status.mailbox_wait_max_us == 95u);
    motor_runtime_latch_fault(MOTOR_FAULT_DEVICE); status = read_status();
    assert(!status.feedback.is_valid && status.mailbox_wait_max_us == 95u && status.feedback_poll_max_us == 14u);
    clock_read_increment_us = 0u;
    motor_runtime_init(); status = read_status();
    assert(status.mailbox_wait_max_us == 0u && status.feedback_poll_max_us == 0u);
    puts("PASS timing observations: uint32 wrap, independent 7us clock, aggregate max wait, fault retention and init reset");
}

/** 主机入口；断言失败立即非零退出，全部通过才打印最终结果。 */
int main(void)
{
    test_encoder_frames();
    test_sample_boundaries();
    test_runtime_states();
    test_snapshot_interleaving();
    test_telemetry();
    test_offset_candidates();
    test_feedback_integration();
    test_feedback_publish_age();
    test_telemetry_scheduling();
    test_timing_wrap();
    puts("PASS all M1 host software checks (hardware NOT RUN)");
    return 0;
}
