"""1kHz d/q方案的独立精确RL离散模型；不是板端或物理电流校准。"""
import itertools,json,math
from pathlib import Path

KP,KI,TRACK=0.5,150.0,200.0
DT,LIMIT,REFERENCE=0.001,0.4,0.04

def simulate(resistance,inductance,bus_ratio,sensor_gain,delay_s,polarity=1,reference=REFERENCE,offset_deg=0):
    # ADC之后的命令在下一UEV才施加；两段精确指数积分，保留前拍电压。
    value=integral=previous=0.0
    rows=[]
    for k in range(700):
        t=k*DT
        target=reference*min(t/0.025,1.0) if k<225 else reference*max(0.0,(0.250-t)/0.025)
        measured=value*sensor_gain*polarity
        error=target-measured
        requested=KP*error+integral
        voltage=max(-LIMIT,min(LIMIT,requested))
        integral+=KI*DT*error+TRACK*DT*(voltage-requested)
        first=math.exp(-resistance*delay_s/inductance)
        second=math.exp(-resistance*(DT-delay_s)/inductance)
        value=first*value+(1-first)*previous*bus_ratio/resistance
        value=second*value+(1-second)*voltage*bus_ratio/resistance
        previous=voltage
        rows.append((target,measured,value,voltage,integral))
    return rows

def main():
    worst_error=worst_peak=worst_recovery=0.0
    count=0
    for r,l,bus,gain,delay,sign in itertools.product((3.85,5.5,7.15),(0.000504,0.00072,0.0009,0.00117),(0.85,1.0,1.15),(0.8,1.0,1.2),(0.74e-3,0.775e-3,0.81e-3),(-1,1)):
        rows=simulate(r,l,bus,gain,delay,reference=sign*REFERENCE)
        error=max(abs(x[0]-x[1]) for x in rows[180:225])
        peak=max(abs(x[2]) for x in rows)
        recovery=abs(rows[-1][1])
        assert error<0.006,(r,l,bus,gain,delay,error)
        assert peak<0.065 and recovery<0.0001,(peak,recovery)
        assert all(math.isfinite(z) for row in rows for z in row)
        worst_error=max(worst_error,error);worst_peak=max(worst_peak,peak);worst_recovery=max(worst_recovery,recovery);count+=1
    bad=simulate(5.5,0.00072,1,1,0.000775,polarity=-1)
    assert max(abs(x[0]-x[1]) for x in bad[:100])>0.04
    # 几何oracle：dq电压圆界0.4V时，SVPWM duty偏离0.5至多sqrt(3)*0.4/(2*12)。
    deviation=math.sqrt(3)*LIMIT/(2*12)
    tick_max=math.ceil((0.5+deviation)*1800)
    low_edge_us=tick_max/72
    aperture_us=7.5/12
    conversion_us=20/12
    earliest=25-2
    latest_end=25+2+conversion_us
    assert tick_max<=954 and earliest-low_edge_us>9.7
    assert latest_end<50-low_edge_us
    report=dict(result='PASS',cases=count,kp_V_per_A=KP,ki_V_per_A_s=KI,tracking_per_s=TRACK,reference_A=REFERENCE,voltage_circle_V=LIMIT,
        worst_steady_measured_error_A=worst_error,worst_actual_current_A=worst_peak,worst_recovery_A=worst_recovery,
        maximum_ccr=tick_max,minimum_pre_aperture_low_us=earliest-low_edge_us,adc_aperture_us=aperture_us,adc_total_conversion_us=conversion_us,
        delay_s=[0.00074,0.00081],sample_period_s=DT,limitations='Recovery value models continued zero-reference regulation through700ms, not the physical250ms STOP profile. RL held-rotor/small speed model; no PWM ripple, nonlinear friction/backEMF, switch deadtime or analog settling certification; current tracking cannot prove electric zero',hardware='NOT RUN')
    out=Path(__file__).resolve().parents[2]/'build/dq_current';out.mkdir(parents=True,exist_ok=True)
    (out/'discrete_model.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
