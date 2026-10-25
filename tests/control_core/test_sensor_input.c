/** M2c独立主机数值依据：二进制精确A/count、角度比例、整数级数和失败原状态。 */
#include "sensor_input.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static void invalid_conversion(uint16_t u,uint16_t v,uint16_t a,bool valid,sensor_input_config_t c)
{
    sensor_input_t out={1,2,3,true};
    assert(!sensor_input_convert(u,v,a,valid,c,&out));
    assert(!out.is_valid && out.nominal_current_u_A==0 && out.nominal_current_v_A==0 && out.mechanical_angle_rad==0);
}

int main(void)
{
    sensor_input_config_t c={1024,3072,0.25f};sensor_input_t out;
    for(uint16_t raw=0;raw<4096;++raw){
        assert(sensor_input_convert(raw,(uint16_t)(4095-raw),0,true,c,&out));
        assert(out.nominal_current_u_A==((int)raw-1024)/4.0f);
        assert(out.nominal_current_v_A==((4095-(int)raw)-3072)/4.0f);
    }
    for(uint16_t angle=0;angle<16384;++angle){
        assert(sensor_input_convert(1024,3072,angle,true,c,&out));
        double expected=(double)angle/8192.0*3.14159265358979323846;
        assert(fabs(out.mechanical_angle_rad-expected)<4.0*FLT_EPSILON*6.284);
        assert(out.mechanical_angle_rad>=0 && out.mechanical_angle_rad<6.283185307179587);
    }
    assert(sensor_input_convert(1024,3072,0,true,c,&out) && out.mechanical_angle_rad==0);
    c=(sensor_input_config_t){2047.5f,2048.5f,0.5f};
    assert(sensor_input_convert(2048,2048,8192,true,c,&out));
    assert(out.nominal_current_u_A==0.25f && out.nominal_current_v_A==-0.25f);
    float *fields[]={&c.estimated_offset_u_count,&c.estimated_offset_v_count,&c.nominal_A_per_count};
    const float bad[]={NAN,INFINITY,-INFINITY,-1.0f};
    for(unsigned f=0;f<3;++f){float old=*fields[f];for(unsigned i=0;i<4;++i){
        *fields[f]=bad[i];invalid_conversion(2048,2048,0,true,c);
    }*fields[f]=old;}
    c.estimated_offset_u_count=4095.5f;invalid_conversion(0,0,0,true,c);
    c=(sensor_input_config_t){0,4096,1};invalid_conversion(0,0,0,true,c);
    c=(sensor_input_config_t){0,0,0};invalid_conversion(0,0,0,true,c);
    c.nominal_A_per_count=FLT_MAX;invalid_conversion(0,4095,0,true,c);
    c=(sensor_input_config_t){0,0,1};
    invalid_conversion(4096,0,0,true,c);invalid_conversion(0,4096,0,true,c);
    invalid_conversion(0,0,16384,true,c);invalid_conversion(0,0,0,false,c);
    assert(!sensor_input_convert(0,0,0,true,c,NULL));
    puts("PASS sensor conversion: all 4096 ADC counts, 16384 mechanical counts, fractional offsets and invalid/overflow inputs");

    sensor_offset_t a,b;assert(sensor_offset_init(&a) && sensor_offset_init(&b));
    for(uint16_t i=0;i<512;++i){
        assert(sensor_offset_add(&a,i,(uint16_t)(4095-i),true));
        assert(sensor_offset_add(&b,100,200,true));
        assert(a.accepted_count==(uint32_t)i+1u && b.accepted_count==(uint32_t)i+1u);
    }
    assert(a.sum_u_count==130816 && a.sum_v_count==1965824);
    assert((double)a.sum_u_count/512==255.5 && (double)a.sum_v_count/512==3839.5);
    assert(a.minimum_u_count==0 && a.maximum_u_count==511 && a.minimum_v_count==3584 && a.maximum_v_count==4095);
    assert(b.sum_u_count==51200 && b.sum_v_count==102400 && b.minimum_u_count==100 && b.maximum_v_count==200);
    unsigned char saved[sizeof(a)];memcpy(saved,&a,sizeof(a));
    assert(!sensor_offset_add(&a,0,0,true) && memcmp(saved,&a,sizeof(a))==0);
    assert(sensor_offset_init(&a));assert(sensor_offset_add(&a,123,321,true));memcpy(saved,&a,sizeof(a));
    assert(!sensor_offset_add(&a,4096,0,true) && memcmp(saved,&a,sizeof(a))==0);
    assert(!sensor_offset_add(&a,0,4096,true) && memcmp(saved,&a,sizeof(a))==0);
    assert(!sensor_offset_add(&a,0,0,false) && memcmp(saved,&a,sizeof(a))==0);
    a.accepted_count=513;memcpy(saved,&a,sizeof(a));
    assert(!sensor_offset_add(&a,0,0,true) && memcmp(saved,&a,sizeof(a))==0);
    assert(!sensor_offset_init(NULL) && !sensor_offset_add(NULL,0,0,true));
    puts("PASS offset candidates: exact 512-frame sums/means/min/max, two instances, full/invalid no state commit");
    return 0;
}
