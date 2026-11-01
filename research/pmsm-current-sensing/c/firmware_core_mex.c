/* Host-only bridge to unchanged production current_control.c. No HAL/devices. */
#include "mex.h"
#include "current_control.h"
void mexFunction(int nlhs,mxArray **plhs,int nrhs,const mxArray **prhs)
{
 if(nrhs!=2 || nlhs!=1 || mxGetNumberOfElements(prhs[0])!=2 || mxGetNumberOfElements(prhs[1])!=7)
  mexErrMsgIdAndTxt("research:input","Expected state[2], input[u,v,theta,idref,iqref,bus,dt][7], one output.");
 const double *s=mxGetPr(prhs[0]),*x=mxGetPr(prhs[1]);
 current_control_state_t st={(float)s[0],(float)s[1]};
 current_control_config_t cfg={{.5f,150.f,200.f},{.5f,150.f,200.f},.04f,.0577350269f,.00099f,.00101f};
 current_control_input_t in={(float)x[0],(float)x[1],(float)x[2],(float)x[3],(float)x[4],(float)x[5],(float)x[6]};
 current_control_output_t y;bool ok=current_control_step_bounded(&st,cfg,in,&y);
 plhs[0]=mxCreateDoubleMatrix(14,1,mxREAL);double *z=mxGetPr(plhs[0]);
 z[0]=ok;z[1]=st.integral_d_V;z[2]=st.integral_q_V;z[3]=y.measured.current_d_A;z[4]=y.measured.current_q_A;
 z[5]=y.modulation.limited_voltage_d_V;z[6]=y.modulation.limited_voltage_q_V;
 z[7]=y.modulation.duty_u;z[8]=y.modulation.duty_v;z[9]=y.modulation.duty_w;
 z[10]=y.modulation.is_voltage_limited;z[11]=y.is_reference_limited;
 z[12]=y.requested_voltage_d_V;z[13]=y.requested_voltage_q_V;
}
