"""编译实际board_io相位检查函数，独立穷举载波相位；不运行硬件。"""
import hashlib
import json
from pathlib import Path
import subprocess
import os
ROOT=Path(__file__).resolve().parents[2]
source=ROOT/'firmware/stm32f103/board/board_io.c'
text=source.read_text(encoding='utf-8')
start=text.index('static bool sampling_phase_is_valid(void)')
end=text.index('\n}\n',start)+3
function=text[start:end]
out=ROOT/'build/host_single_u_phase';out.mkdir(parents=True,exist_ok=True)
fixture='''#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#define TIM_CR1_DIR 16u
struct Timer { uint32_t CR1,CNT; } a,b;
#define TIM1 phase_timer()
#define TIM2 (&b)
static uint32_t clock_step,calls,timer_reads;
static bool change_direction;
struct Timer *phase_timer(void) {
    if (change_direction && ++timer_reads==3u) { a.CR1^=16u; }
    return &a;
}
uint32_t board_get_time_us(void) {
    uint32_t value=calls++*clock_step;
    return value;
}
'''+function+'''
int main(void) {
 for (unsigned us=175u;us<1000u;++us) {
   for (unsigned shift=0;shift<3600u;++shift) {
     unsigned phase=(us*72u+shift)%3600u;
     a.CR1=phase>1800u ? 16u : 0u;
     a.CNT=phase>1800u ? 3600u-phase : phase;b.CNT=us;
     calls=0;clock_step=0;
     assert(sampling_phase_is_valid()==(shift<=144u || shift>=3456u));
   }
 }
 a.CR1=0;a.CNT=1800;b.CNT=225;calls=0;clock_step=2;
 assert(sampling_phase_is_valid());
 calls=0;clock_step=3;assert(!sampling_phase_is_valid());
 calls=0;clock_step=0;b.CNT=1000;assert(!sampling_phase_is_valid());
 b.CNT=225;a.CNT=1801;calls=0;assert(!sampling_phase_is_valid());
 a.CNT=1800;calls=0;timer_reads=0;change_direction=true;assert(!sampling_phase_is_valid());
 return 0;
}
'''
(out/'phase.c').write_text(fixture,encoding='utf-8')
gcc=Path('D:/mingw64/bin/gcc.exe')
env=os.environ.copy();env['PATH']=str(gcc.parent)+os.pathsep+env.get('PATH','')
subprocess.run([str(gcc),'-O2','-Wall','-Wextra','-Werror',str(out/'phase.c'),'-o',str(out/'phase.exe')],check=True,env=env)
env=os.environ.copy();env['PATH']=str(gcc.parent)+os.pathsep+env.get('PATH','')
subprocess.run([str(out/'phase.exe')],check=True,env=env)
(out/'evidence.json').write_text(json.dumps({'result':'PASS','production_function_sha256':hashlib.sha256(function.encode()).hexdigest(),
    'board_io_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'oracle':'reset shift0..3599, TIM2 us175..999, exact circular <=144, 2/3us read budget, direction transition','hardware':'NOT RUN'},indent=2)+'\n')
print('PASS production phase check: 2,970,000 phase/time combinations, half-cycle rejection, 2us boundary and transition rejection')

# 编译实际时间读取函数：公共入口持锁、已持锁入口不重复加锁，两者同一溢出补偿。
clock_start=text.index("uint32_t board_get_time_us_locked(void)")
clock_end=text.index("/** 生成初始化",clock_start)
clock_source=text[clock_start:clock_end]
clock_fixture="""#include <stdint.h>
#include <assert.h>
struct Timer { uint32_t CNT,SR; } timer;
#define TIM4 (&timer)
#define TIM_SR_UIF 1u
static uint32_t device_time_high_us,mask,enters,leaves;
uint32_t board_enter_critical(void) { uint32_t previous=mask;mask=1;enters++;return previous; }
void board_leave_critical(uint32_t previous) { mask=previous;leaves++; }
"""+clock_source+"""
int main(void) {
 const uint32_t highs[]={0u,65536u,0xffff0000u};
 const uint32_t lows[]={0u,1u,65535u};
 for(unsigned h=0;h<3;h++) for(unsigned l=0;l<3;l++) for(unsigned pending=0;pending<2;pending++) {
   device_time_high_us=highs[h];timer.CNT=lows[l];timer.SR=pending;
   uint32_t expected=highs[h]+lows[l]+(pending ? 65536u : 0u);
   mask=1;enters=leaves=0;
   assert(board_get_time_us_locked()==expected && mask==1 && enters==0 && leaves==0);
   assert(timer.SR==pending); // 只读时间不消费UIF
   mask=0;assert(board_get_time_us()==expected && mask==0 && enters==1 && leaves==1);
   mask=1;assert(board_get_time_us()==expected && mask==1 && enters==2 && leaves==2);
 }
 return 0;
}
"""
(out/'clock.c').write_text(clock_fixture,encoding='utf-8')
subprocess.run([str(gcc),'-O2','-Wall','-Wextra','-Werror',str(out/'clock.c'),'-o',str(out/'clock.exe')],check=True,env=env)
subprocess.run([str(out/'clock.exe')],check=True,env=env)
print("PASS production locked/public clock: pending UIF, 16/32bit wrap, nested IRQ state and zero extra locks")
