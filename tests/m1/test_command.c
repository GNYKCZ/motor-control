/** 真实runtime/command/board_uart/telemetry集成；替换寄存器/时钟，不操作板卡。
 * 命令向量由Python独立crc_hqx生成，另有明写黄金字节，不用生产CRC当expected。 */
#include "command.h"
#include "motor_runtime.h"
#include "board_io.h"
#include "telemetry.h"
#include "usart.h"
#include "command_vectors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
test_usart_t test_usart;
static uint32_t mask, clock_us, disables, sample_n;
static bool fake_pwm, live_samples, fault_on_unlock, hold_angle;
static char wire[512];
static size_t wire_length;
uint32_t board_enter_critical(void) { uint32_t old=mask; mask=1u; return old; }
void board_leave_critical(uint32_t old)
{
    CHECK(mask==1u); mask=old;
    if (!old && fault_on_unlock) { fault_on_unlock=false; motor_runtime_latch_fault(MOTOR_FAULT_DEVICE); }
}
void board_disable_output(void) { disables++; fake_pwm=false; }
uint32_t board_get_time_us(void) { return clock_us; }

static void sample(void)
{
    ++sample_n;
    uint32_t data=((hold_angle ? 1000u : sample_n & 16383u)<<4), dividend=data<<6;
    for (int i=23;i>=6;--i) { if (dividend & (1u<<i)) { dividend^=0x43u<<(i-6); } }
    uint32_t packet=(data<<6)|dividend;
    uint8_t bytes[3]={(uint8_t)(packet>>16),(uint8_t)(packet>>8),(uint8_t)packet};
    board_encoder_frame_t enc=board_decode_encoder_frame(bytes,3u,sample_n,clock_us-50u,clock_us-28u);
    motor_sample_t s=board_make_sample(sample_n,clock_us,2027u,2028u,&enc);
    motor_runtime_handle_sample(&s); motor_runtime_poll_feedback();
}

static void start(void)
{
    CHECK(!board_serial_is_busy());
    memset(&test_usart,0,sizeof(test_usart)); clock_us=0u; sample_n=0u; live_samples=false;hold_angle=false;
    motor_runtime_init(); command_init(); motor_runtime_finish_boot(true); board_start_serial_receive();
    CHECK((test_usart.CR1 & (USART_CR1_RXNEIE|USART_CR1_PEIE))==(USART_CR1_RXNEIE|USART_CR1_PEIE));
    CHECK(test_usart.CR3 & USART_CR3_EIE);
    clock_us=100u; sample(); wire_length=0u; wire[0]='\0';
}

static const char *vector(uint32_t seq,unsigned code)
{
    for (size_t i=0u;i<sizeof(vectors)/sizeof(vectors[0]);++i) {
        if (vectors[i].seq==seq && vectors[i].code==code) { return vectors[i].frame; }
    }
    CHECK(false); return NULL;
}

static void rx(uint8_t byte)
{
    clock_us+=100u;
    if (live_samples) { sample(); }
    test_usart.DR=byte; test_usart.SR=USART_SR_RXNE;
    USART2_IRQHandler();
}

static void feed(const char *text) { while (*text) { rx((uint8_t)*text++); } }

static void tx_drain(void)
{
    unsigned count=0u;
    while (test_usart.CR1 & USART_CR1_TXEIE) {
        CHECK(++count<=256u && wire_length+1u<sizeof(wire));
        test_usart.SR=USART_SR_TXE; USART2_IRQHandler();
        wire[wire_length++]=(char)test_usart.DR;
    }
    CHECK(board_serial_is_busy()); /* 末字节送DR不等于TC完成。 */
    test_usart.SR=USART_SR_TC; USART2_IRQHandler();
    CHECK(!board_serial_is_busy()); wire[wire_length]='\0';
}

static void reply_done(void)
{
    for (unsigned i=0u;i<150u;++i) {
        clock_us+=100u; if (live_samples) { sample(); }
        (void)command_poll(clock_us);
        if (board_serial_is_busy()) { tx_drain(); }
    }
}

static unsigned result(void)
{
    unsigned value=999u; CHECK(wire_length>=95u);
    CHECK(sscanf(wire+17,"%2x",&value)==1); return value;
}

int main(void)
{
    start();
    CHECK(strcmp(vector(1u,1u),"P1C1,00000001,01,A1E6\r\n")==0);
    CHECK(strcmp(vector(2u,2u),"P1C1,00000002,02,1A67\r\n")==0);
    live_samples=true; feed(vector(1u,1u)); reply_done(); CHECK(result()==0u);
    motor_log_snapshot_t snap; CHECK(motor_runtime_read_log(&snap) && snap.state==MOTOR_RUNTIME_DISABLED && !snap.fault_flags);
    CHECK(snap.sequence>50u); fputs(wire,stdout);

    /* 对黄金STOP的184种单bit破坏，必须无任何STOP动作；不使用生产CRC重算expected。 */
    for (unsigned pos=0u;pos<23u;++pos) {
        for (unsigned bit=0u;bit<8u;++bit) {
            start(); uint32_t old=disables;
            for (unsigned i=0u;i<23u;++i) {
                uint8_t byte=(uint8_t)vector(2u,2u)[i];
                rx(i==pos ? (uint8_t)(byte^(1u<<bit)) : byte);
            }
            CHECK(disables==old);
        }
    }

    /* STOP在IRQ中先执行，不需要任何command_poll；UART忙和待STATUS不会阻挡。 */
    start(); feed(vector(1u,1u)); CHECK(board_start_serial_write("old\r\n",5u));
    fake_pwm=true; uint32_t before=disables; feed(vector(2u,2u));
    CHECK(!fake_pwm && disables==before+1u && board_serial_is_busy());
    for (unsigned i=0u;i<30u;++i) { (void)command_poll(clock_us); }
    CHECK(test_usart.CR1 & USART_CR1_TXEIE); tx_drain(); CHECK(strcmp(wire,"old\r\n")==0);
    wire_length=0u; reply_done(); CHECK((result() & 127u)==0u && (result() & 128u)); fputs(wire,stdout);

    /* 已借用ACK缓冲不可因新STOP改写，STOP仍立即执行，下一行才回新请求。 */
    start(); feed(vector(1u,1u));
    for (unsigned i=0u;i<96u;++i) { (void)command_poll(clock_us); }
    CHECK(board_serial_is_busy()); fake_pwm=true; feed(vector(2u,2u)); CHECK(!fake_pwm);
    tx_drain(); CHECK(strncmp(wire,"P1R2,00000001,01,",17u)==0);
    wire_length=0u; reply_done(); CHECK(strncmp(wire,"P1R2,00000002,02,",17u)==0);

    /* 同次IRQ的RXNE+TXE：先消费收到的STOP字节，再写回执字节，不能读到自己的TX。 */
    start(); feed(vector(1u,1u));
    for (unsigned i=0u;i<96u;++i) { (void)command_poll(clock_us); }
    CHECK(board_serial_is_busy()); fake_pwm=true;
    const char *stop=vector(2u,2u);
    for (unsigned i=0u;i<23u;++i) {
        clock_us+=100u; test_usart.DR=(uint8_t)stop[i]; test_usart.SR=USART_SR_RXNE|USART_SR_TXE;
        USART2_IRQHandler(); wire[wire_length++]=(char)test_usart.DR;
    }
    CHECK(!fake_pwm); tx_drain(); CHECK(strncmp(wire,"P1R2,00000001,01,",17u)==0);
    wire_length=0u; reply_done(); CHECK(strncmp(wire,"P1R2,00000002,02,",17u)==0);

    /* 待STOP回执不能被后来的STATUS替换，丢回执标记可见。 */
    start(); feed(vector(2u,2u)); feed(vector(3u,1u)); reply_done();
    CHECK(strncmp(wire,"P1R2,00000002,02,",17u)==0 && (result() & 128u));

    start(); feed(vector(1u,3u)); reply_done(); CHECK(result()==4u); fputs(wire,stdout);
    wire_length=0u; feed(vector(2u,4u)); reply_done(); CHECK(result()==4u);
    wire_length=0u; feed(vector(3u,0x55u)); reply_done(); CHECK(result()==4u);

    /* 重复/旧/halfRange拒绝；重复STOP仍幂等禁用，FAULT不被STOP清除。 */
    start(); feed(vector(2u,2u)); reply_done();
    wire_length=0u; before=disables; feed(vector(2u,2u)); CHECK(disables==before+1u); reply_done(); CHECK(result()==3u);
    wire_length=0u; feed(vector(1u,1u)); reply_done(); CHECK(result()==3u);
    wire_length=0u; feed(vector(0x80000002u,1u)); reply_done(); CHECK(result()==3u);
    motor_runtime_latch_fault(MOTOR_FAULT_FEEDBACK); feed(vector(3u,2u)); reply_done();
    CHECK(motor_runtime_read_log(&snap) && snap.state==MOTOR_RUNTIME_FAULT && (snap.fault_flags & 8u));
    mask=1u; before=disables; motor_runtime_stop(); CHECK(mask==1u && disables==before+1u); mask=0u;

    /* seq与时间32位回绕，命令初始化丢旧半帧且不重放STOP。 */
    start(); clock_us=0xfffffff0u; feed(vector(0xfffffffeu,1u)); reply_done(); CHECK(result()==0u);
    wire_length=0u; feed(vector(0xffffffffu,1u)); reply_done(); CHECK(result()==0u);
    wire_length=0u; feed(vector(0u,1u)); reply_done(); CHECK(result()==0u);
    start(); feed("P1C1,00000002,"); before=disables; command_init(); feed("02,1A67\r\n"); CHECK(disables==before);
    reply_done(); CHECK(result()!=0u);

    /* CRC、布局、超长后嵌入的合法尾部都不能执行STOP。 */
    start(); char bad[24]; memcpy(bad,vector(2u,2u),24u); bad[17]=bad[17]=='0'?'1':'0';
    before=disables; feed(bad); CHECK(disables==before); reply_done(); CHECK(result()==2u);
    start(); feed("noise"); before=disables; feed(vector(2u,2u)); CHECK(disables==before); reply_done();
    feed(vector(2u,2u)); CHECK(disables==before+1u); reply_done();
    start(); for (unsigned i=0u;i<100u;++i) { rx('X'); }
    before=disables; feed(vector(2u,2u)); CHECK(disables==before); reply_done();
    feed(vector(2u,2u)); CHECK(disables==before+1u); reply_done();

    start(); feed("P1C1,"); clock_us+=25001u; (void)command_poll(clock_us);
    before=disables; feed(vector(2u,2u)); CHECK(disables==before); reply_done();
    feed(vector(2u,2u)); CHECK(disables==before+1u); reply_done();
    start(); const char *whole=vector(1u,1u);
    for (unsigned i=0u;i<9u;++i) { rx((uint8_t)whole[i]); }
    /* main先取时、随后RX IRQ更新last_us的真实交错：旧now不能当成超时。 */
    CHECK(!command_poll(clock_us-1000u)); feed(whole+9u); reply_done(); CHECK(result()==0u);
    start(); const char *slow=vector(2u,2u); before=disables;
    while (*slow) { clock_us+=24000u; rx((uint8_t)*slow++); }
    CHECK(disables==before); reply_done(); CHECK((result() & 127u)==6u);

    /* ORE/FE/NE/PE全部走真实board ISR错误分支；尾片段不恢复命令。 */
    const uint32_t flags[]={USART_SR_ORE,USART_SR_FE,USART_SR_NE,USART_SR_PE};
    for (unsigned i=0u;i<4u;++i) {
        start(); feed("P1C1,"); test_usart.SR=flags[i]|USART_SR_RXNE; test_usart.DR='X'; USART2_IRQHandler();
        before=disables; feed(vector(2u,2u)); CHECK(disables==before); reply_done(); CHECK((result() & 127u)==5u);
        feed(vector(2u,2u)); CHECK(disables==before+1u); reply_done();
    }

    /* 解锁时ADC故障可抢占，随后STATUS/STOP都不能恢复有效/输出。 */
    start(); feed(vector(1u,1u)); fault_on_unlock=true; reply_done();
    CHECK(motor_runtime_read_log(&snap) && snap.state==MOTOR_RUNTIME_FAULT && !snap.position_is_valid);
    feed(vector(2u,2u)); reply_done(); CHECK(motor_runtime_read_log(&snap) && snap.state==MOTOR_RUNTIME_FAULT);

    /* 已启动周期日志的完整行和ACK串行交错，真UART TC后才释放各自缓冲。 */
    start(); live_samples=true;
    for (unsigned i=0u;i<17u;++i) { clock_us+=100u; sample(); telemetry_poll_status(1000u); }
    CHECK(board_serial_is_busy()); feed(vector(1u,1u));
    (void)command_poll(clock_us); tx_drain(); CHECK(strncmp(wire,"M3B3,",5u)==0); fputs(wire,stdout);
    wire_length=0u; reply_done(); CHECK(strncmp(wire,"P1R2,",5u)==0); fputs(wire,stdout);
    CHECK(motor_runtime_read_log(&snap) && snap.state==MOTOR_RUNTIME_DISABLED && snap.feedback_overruns==0u);
    puts("PASS actual runtime/command/UART ISR/TXE/TC integration: STOP priority, CRC/sequence/boundaries/errors/wrap/backpressure/faults; no hardware");
    /* 新增只读零偏回执独立黄金：只有首帧2027/2028，查询不复位统计、不产生输出。 */
    start(); before=disables; feed(vector(1u,5u)); reply_done();
    CHECK(wire_length==138u && result()==0u && disables==before);
    CHECK(strncmp(wire,"P1Z1,",5u)==0);fputs(wire,stdout);
    start(); before=disables; feed(vector(1u,6u)); reply_done();
    CHECK(result()==4u && disables==before); /* 未静止/未暖机拒绝，不触发输出。 */
    start();hold_angle=true;
    for (unsigned i=0u;i<300u;++i) { clock_us+=1000u;sample(); }
    live_samples=true;feed(vector(1u,6u));
    motor_zero_snapshot_t zero;CHECK(motor_runtime_read_zero(&snap,&zero));
    CHECK(zero.offset.accepted_count==0u && !fake_pwm && (zero.flags & 16u));
    reply_done();CHECK(result()==0u);
    CHECK(motor_runtime_read_zero(&snap,&zero) && zero.offset.accepted_count>0u);
    uint32_t began=zero.began_us;wire_length=0u;feed(vector(1u,6u));reply_done();
    CHECK(result()==3u && motor_runtime_read_zero(&snap,&zero) && zero.began_us==began);
    CHECK(!fake_pwm); /* 重放ZERO_RESTART不能重开候选，更不能激励。 */
    return 0;
}
