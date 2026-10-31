/** 单精度有界入口对通用double核心的独立数值对照；不使用硬件。 */
#include "current_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL bounded %d: %s\n",__LINE__,#x);exit(1); } } while(0)
static void near(float a,float b,float tolerance) { CHECK(isfinite(a) && isfinite(b) && fabsf(a-b)<=tolerance); }
int main(void) {
 current_control_config_t c={{.5f,150,200},{.5f,150,200},.04f,.0577350269f,.00099f,.00101f};
 const float currents[]={-.15f,-.08f,0,.08f,.15f};unsigned count=0;
 for(unsigned angle=0;angle<16384;angle+=17) for(unsigned u=0;u<5;u++) for(unsigned v=0;v<5;v++)
 for(unsigned s=0;s<4;s++) for(int q=-1;q<=1;q++) {
  current_control_state_t a={(s&1)?1.0f:0.0f,(s&2)?-1.0f:0.0f},b=a;
  current_control_input_t x={currents[u],currents[v],(float)angle*.000383495197f,0,q*.04f,12,.001f};
  current_control_output_t ya,yb;
  CHECK(current_control_step(&a,c,x,&ya));CHECK(current_control_step_bounded(&b,c,x,&yb));
  near(ya.measured.current_d_A,yb.measured.current_d_A,0.000003f);near(ya.measured.current_q_A,yb.measured.current_q_A,0.000003f);
  near(ya.modulation.duty_u,yb.modulation.duty_u,.000002f);near(ya.modulation.duty_v,yb.modulation.duty_v,.000002f);near(ya.modulation.duty_w,yb.modulation.duty_w,.000002f);
  near(ya.modulation.limited_voltage_d_V,yb.modulation.limited_voltage_d_V,.000005f);near(ya.modulation.limited_voltage_q_V,yb.modulation.limited_voltage_q_V,.000005f);
  near(a.integral_d_V,b.integral_d_V,.00001f);near(a.integral_q_V,b.integral_q_V,.00001f);++count;
 }
 /* 各配置字段的NaN拒绝及状态事务；不是只验证正常数值。 */
 for(unsigned field=0;field<10;field++) {
  current_control_config_t bad=c;float *fields[]={&bad.d.kp_V_per_A,&bad.d.ki_V_per_A_s,&bad.d.tracking_gain_per_s,
   &bad.q.kp_V_per_A,&bad.q.ki_V_per_A_s,&bad.q.tracking_gain_per_s,&bad.reference_limit_A,&bad.modulation_limit,&bad.minimum_dt_s,&bad.maximum_dt_s};
  *fields[field]=NAN;current_control_state_t a={.1f,-.1f},before=a;current_control_output_t y;
  current_control_input_t x={0,0,0,0,0,12,.001f};CHECK(!current_control_step_bounded(&a,bad,x,&y));
  CHECK(!y.is_valid && !memcmp(&a,&before,sizeof(a)));
 }
 current_control_state_t large={2,0},saved=large;current_control_output_t rejected;
 current_control_config_t more=c;more.d.ki_V_per_A_s=500;more.d.tracking_gain_per_s=0;more.maximum_dt_s=.002f;
 current_control_input_t impulse={-.2f,0,0,0,0,12,.002f};
 CHECK(!current_control_step_bounded(&large,more,impulse,&rejected));CHECK(!memcmp(&large,&saved,sizeof(large)) && !rejected.is_valid);
 const float invalid[]={NAN,INFINITY,-INFINITY,3.0f};
 for(unsigned i=0;i<4;i++) {
  current_control_state_t a={.1f,-.1f},before=a;current_control_output_t y;
  current_control_input_t x={invalid[i],0,0,0,0,12,.001f};
  CHECK(!current_control_step_bounded(&a,c,x,&y));CHECK(!y.is_valid && !memcmp(&a,&before,sizeof(a)));
 }
 printf("PASS bounded: %u double-oracle cases; current/duty/voltage/integral tolerances; invalid rollback\n",count);
 return 0;
}
