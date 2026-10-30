/** 只替换runtime快照与板级发送，真实M3B3整数序列化独立验收。
 * 期望字符串来自协议十进制边界，不调用被测代码生成expected。 */
#include "motor_runtime.h"
/* 本单元只测codec，实际输出期间日志暂停由runtime集成用例验证。 */
static bool gate_busy;
bool motor_runtime_output_is_busy(void) { return gate_busy; }
#include "telemetry.h"
#include "board_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL compact line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
static motor_log_snapshot_t supplied;
static uint32_t device_us, read_increment;
static bool busy, allow_snapshot = true, allow_send = true;
static unsigned submissions;
static char captured[256];
static const char *borrowed;

bool motor_runtime_read_log(motor_log_snapshot_t *out)
{
    if (!allow_snapshot) { return false; }
    *out = supplied;
    return true;
}
bool board_serial_is_busy(void) { return busy; }
uint32_t board_get_time_us(void)
{
    uint32_t now = device_us;
    device_us += read_increment;
    return now;
}
bool board_start_serial_write(const char *bytes, uint32_t length)
{
    CHECK(!busy && length < sizeof(captured));
    if (!allow_send) { return false; }
    memcpy(captured, bytes, length); captured[length] = '\0';
    borrowed = bytes; busy = true; submissions++;
    return true;
}

static void emit(uint32_t ms)
{
    ms *= 10u; /* 仅把格式/拥塞工况的导出时间映射到1秒周期。 */
    for (unsigned i = 0u; i < 17u; ++i) { telemetry_poll_status(ms); }
}

int main(void)
{
    supplied.state = MOTOR_RUNTIME_DISABLED;
    supplied.sample_is_valid = true;
    supplied.position_is_valid = true;
    supplied.direct_is_valid = true;
    supplied.window_is_valid = true;
#ifdef TEST_SMALL_BUFFER
    emit(100u);
    CHECK(!busy && submissions == 0u);
    emit(200u);
    CHECK(!busy && submissions == 0u);
    puts("PASS compact negative capacity: incomplete frame never submitted");
    return 0;
#else
    static const uint32_t values[] = {0u, 9u, 10u, 99u, 100u, 999999999u, 1000000000u, UINT32_MAX};
    static const char *const expected[] = {"0", "9", "10", "99", "100", "999999999", "1000000000", "4294967295"};
    for (unsigned i = 0u; i < sizeof(values) / sizeof(values[0]); ++i) {
        supplied.sequence = values[i];
        supplied.completed_us = values[i];
        busy = false; emit((i + 1u) * 100u);
        CHECK(busy);
        char oracle[80];
        /* 测试侧标准格式只拼预先写出的期望文本，不实现十进制转换。 */
        snprintf(oracle, sizeof(oracle), "n=%s,t=%s,", expected[i], expected[i]);
        CHECK(strstr(captured, oracle) != NULL);
        CHECK(strlen(captured) < BOARD_SERIAL_BUFFER_BYTES);
    }
    supplied.state = MOTOR_RUNTIME_FAULT;
    supplied.fault_flags = UINT32_MAX;
    supplied.raw_u = supplied.raw_v = supplied.raw_angle = UINT16_MAX;
    supplied.sample_is_valid = supplied.position_is_valid = false;
    supplied.direct_is_valid = supplied.window_is_valid = false;
    supplied.feedback_overruns = supplied.feedback_max_us = supplied.mailbox_max_us = UINT32_MAX;
    busy = false;
    for (unsigned i = 0u; i < 16u; ++i) { telemetry_poll_status(10000u); }
    read_increment = UINT32_MAX; /* 最后一个字段单步观测最大值，固定ID=16。 */
    telemetry_poll_status(10000u); CHECK(busy); read_increment = 0u;
    busy = false; emit(1100u);
    CHECK(strcmp(captured,
        "M3B3,s=2,f=4294967295,n=4294967295,t=4294967295,u=65535,v=65535,a=65535,"
        "ok=0,fp=0,fd=0,fw=0,fo=4294967295,fb_us=4294967295,mb_us=4294967295,"
        "log_us=4294967295,log_id=16\r\n") == 0);
    CHECK(strlen(captured) == 169u);
    puts(captured); /* Python端再经真实parser核验极值行。 */
    char held[256]; strcpy(held, borrowed);
    emit(1200u); CHECK(strcmp(held, borrowed) == 0); /* busy不触碰借用缓冲。 */
    busy = false; unsigned before = submissions;
    allow_send = false; emit(1300u); CHECK(!busy && submissions == before);
    allow_send = true; emit(1400u); CHECK(busy && submissions == before + 1u);
    busy = false; allow_snapshot = false; before = submissions;
    emit(1500u); CHECK(!busy && submissions == before);
    allow_snapshot = true; emit(1600u); CHECK(busy);
    busy=false; before=submissions; gate_busy=true;
    emit(1700u); CHECK(submissions==before && !busy); /* ACTIVE时不把新状态塞入旧M3B3。 */
    gate_busy=false; emit(1800u); CHECK(submissions==before+1u && busy);
    puts("PASS compact integer codec: 0/decimal edges/UINT32_MAX, exact 169B, maximum-step ID, busy/failures");
    return 0;
#endif
}
