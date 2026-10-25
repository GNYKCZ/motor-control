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

static uint32_t interrupt_mask;       /**< 假设备 PRIMASK，允许验证嵌套恢复。 */
static uint32_t disable_count;        /**< 禁止调制接口调用次数。 */
static uint32_t device_now_us;        /**< 手工设备时间。 */
static bool serial_busy;              /**< 手工串口拥塞条件。 */
static const char *borrowed_line;     /**< 验证忙期间缓冲内容不变。 */
static char captured_line[256];       /**< 最近成功发送的副本。 */
static motor_sample_t pending_sample; /**< 中断屏蔽期间等待发布的下一完整包。 */
static bool has_pending_sample;       /**< 解除屏蔽时触发一次模拟 ADC 完成。 */

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
    if (interrupt_mask == 0u && has_pending_sample) {
        has_pending_sample = false;
        motor_runtime_handle_sample(&pending_sample);
    }
}
/** 手工时间来源；由测试设置，和电脑时钟无关。 */
uint32_t board_get_time_us(void) { return device_now_us; }
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
    assert(status.pwm_is_permanently_blocked);
    return status;
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
    interrupt_mask = 1u;
    (void)read_status();
    assert(interrupt_mask == 1u);
    motor_runtime_latch_fault(MOTOR_FAULT_DEVICE);
    assert(interrupt_mask == 1u);
    interrupt_mask = 0u;
    puts("PASS snapshot: interleaved publication is coherent; nested PRIMASK restored");
}

/** 验证实际导出序号/设备时间/原始量，模拟 UART 长期忙时缓冲不可变且可见跳过次数。 */
static void test_telemetry(void)
{
    start_runtime();
    motor_sample_t sample = make_sample(123u, 1000u);
    motor_runtime_handle_sample(&sample);
    telemetry_poll_status(99u);
    assert(!serial_busy);
    telemetry_poll_status(100u);
    assert(serial_busy);
    assert(strstr(captured_line, "M1,s=1,f=0,n=123,t=1000,e=123,es=950,ec=972,") == captured_line);
    assert(strstr(captured_line, "u=1234,v=2345,a=123,mg=0,ok=1,bad=0,skip=0,") != NULL);
    assert(strstr(captured_line, "pwm=blocked,power=unverified\r\n") != NULL);
    char saved_line[256];
    strcpy(saved_line, borrowed_line);
    for (uint32_t time_ms = 200u; time_ms <= 1000u; time_ms += 100u) {
        telemetry_poll_status(time_ms);
        assert(strcmp(borrowed_line, saved_line) == 0);
    }
    serial_busy = false;
    telemetry_poll_status(1100u);
    assert(strstr(captured_line, "skip=9,") != NULL);
    serial_busy = false;
    telemetry_poll_status(UINT32_MAX - 49u);
    serial_busy = false;
    telemetry_poll_status(50u); /* 毫秒时间回绕后相隔 100 ms。 */
    assert(serial_busy);
    puts("PASS telemetry: exact device fields, congestion skip, buffer ownership, time wrap");
}

/** 主机入口；断言失败立即非零退出，全部通过才打印最终结果。 */
int main(void)
{
    test_encoder_frames();
    test_sample_boundaries();
    test_runtime_states();
    test_snapshot_interleaving();
    test_telemetry();
    puts("PASS all M1 host software checks (hardware NOT RUN)");
    return 0;
}
