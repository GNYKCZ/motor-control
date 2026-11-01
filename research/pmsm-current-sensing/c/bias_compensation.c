#include "bias_compensation.h"
#include <float.h>
#include <stddef.h>
#if defined(__FAST_MATH__)
#error "Finite-input validation requires no fast-math"
#endif
static int finite_value(float x) { return x>=-FLT_MAX && x<=FLT_MAX; }
static float magnitude(float x) { return x<0 ? -x : x; }
int bias_compensate(const bias_calibration_t *c, const bias_input_t *x, bias_output_t *o)
{
    const float lsb=3.3f/4096.0f; /* 20mOhm*50=1V/A；硬件配置改变须重新验证。 */
    if (o==NULL) return 1;
    *o=(bias_output_t){0};
    if (c==NULL || x==NULL || !c->certified || c->calibration_id==0 || (c->model!=1 && c->model!=2)) return 1;
    if (!finite_value(c->duty_min) || !finite_value(c->duty_max) || c->duty_min<0 || c->duty_max>1 || c->duty_min>c->duty_max
        || !finite_value(c->bus_min_V) || !finite_value(c->bus_max_V) || c->bus_min_V<=0 || c->bus_min_V>c->bus_max_V) return 1;
    for (int j=0;j<2;++j) {
        if (!finite_value(c->b0_A[j]) || magnitude(c->b0_A[j])>.2f || !finite_value(c->k_A_per_duty[j]) || magnitude(c->k_A_per_duty[j])>1.0f
            || !finite_value(c->error_A[j]) || c->error_A[j]<0 || c->error_A[j]>.1f) return 1;
        if (x->adc[j]==0 || x->adc[j]>=4095) return 2;
    }
    if (!finite_value(x->bus_V) || x->bus_V<c->bus_min_V || x->bus_V>c->bus_max_V) return 3;
    for (int j=0;j<3;++j) if (!finite_value(x->duty[j]) || x->duty[j]<c->duty_min || x->duty[j]>c->duty_max) return 3;
    float mean=(x->duty[0]+x->duty[1]+x->duty[2])/3.0f;
    for (int j=0;j<2;++j) {
        float feature=c->model==1 ? x->duty[j] : mean;
        o->corrected_A[j]=x->adc[j]*lsb-1.65f-c->b0_A[j]-c->k_A_per_duty[j]*feature;
        o->upper_A[j]=magnitude(o->corrected_A[j])+c->error_A[j]+lsb*.5f+2e-6f; /* 有界float运算余量 */
    }
    o->corrected_A[2]=-o->corrected_A[0]-o->corrected_A[1];
    o->upper_A[2]=magnitude(o->corrected_A[2])+c->error_A[0]+c->error_A[1]+lsb+4e-6f;
    for (int j=0;j<3;++j) if (!finite_value(o->upper_A[j]) || o->upper_A[j]>128.0f*lsb) return 4;
    o->permitted=1;
    return 0;
}
