"""使用生产board_start_sampling函数核验同固件复位引脚选择及ADC首次使能顺序。"""
from pathlib import Path
import os,subprocess
root=Path(__file__).resolve().parents[2]
board=(root/'firmware/stm32f103/board/board_io.c').read_text(encoding='utf8')
production=board.split('bool board_start_sampling(void)',1)[1].split('\n}\n',1)[0]
out=root/'build/host_phase_boot';out.mkdir(parents=True,exist_ok=True)
fixture=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define BOARD_DQ_VECTOR_DIAGNOSTIC 1
enum {TIM_CR1_CEN=1,TIM_CR1_DIR=16,TIM_BDTR_MOE=32768,GPIO_PIN_12=4096,TIM_EGR_UG=1,
      HAL_OK=0,ADC_SR_JEOC=4,ADC_SR_JSTRT=8,TIM_DIER_CC1IE=2};
struct Timer {unsigned CR1,CNT,EGR,SR,CCR2,BDTR,DIER;} a,b;
#define TIM1 (&a)
#define TIM2 (&b)
struct {unsigned IDR;} gpio;
#define GPIOA (&gpio)
struct {unsigned SR;} adc1,adc2;
#define ADC1 (&adc1)
#define ADC2 (&adc2)
static int hadc1,hadc2,order;
static unsigned phase,expected;
static void board_disable_output(void){a.BDTR=0;}
static void diag_trace_set_adc_phase(uint16_t x){assert(order==0);phase=x;}
static int HAL_ADCEx_InjectedStart(int *x){
 assert(x==&hadc2 && order==0 && phase==expected && b.CCR2==expected);
 assert(b.EGR==1 && !(b.CR1&1) && !(a.CR1&1) && !a.BDTR);order++;return 0;
}
static int HAL_ADCEx_InjectedStart_IT(int *x){assert(x==&hadc1 && order==1);order++;return 0;}
bool board_start_sampling(void)
'''+production+r'''
}
int main(void){
 for(unsigned pressed=0;pressed<2;pressed++){
   a=(struct Timer){0};b=(struct Timer){0};gpio.IDR=pressed?0:4096;
   expected=pressed?250:225;phase=0;order=0;assert(board_start_sampling());
   assert(phase==expected && b.CCR2==expected && order==2 && a.CR1==1 && b.CR1==1 && !a.BDTR);
   gpio.IDR^=4096; assert(!board_start_sampling());assert(phase==expected && order==2);
 }
}
'''
path=out/'phase_boot.c';path.write_text(fixture,encoding='utf8')
gcc=Path('D:/mingw64/bin/gcc.exe');env=os.environ.copy();env['PATH']=str(gcc.parent)+os.pathsep+env['PATH']
exe=out/'phase_boot.exe'
subprocess.run([str(gcc),'-std=c11','-Wall','-Wextra','-Werror',str(path),'-o',str(exe)],check=True,env=env)
subprocess.run([str(exe)],check=True,env=env)
print('PASS production boot phase selection: 225/250, ADC armed only after preload, running re-selection rejected')
