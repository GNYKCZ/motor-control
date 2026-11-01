function p=parameters
% 各数值来源见docs/hardware_facts.md，假设单独标注。
p.Rs=5.5;p.Ld=.7185e-3;p.Lq=.905e-3;p.psi=.00257;p.poles=7;
p.Vbus=12;p.shunt=.02;p.gain=50;p.Vref=3.3;p.bits=12;p.ref=1.65;
p.lsb=p.Vref/2^p.bits/(p.shunt*p.gain);p.Ts=.001;p.fpwm=20000;
p.adc_clock=12e6;p.aperture=7.5/p.adc_clock;p.conversion=20/p.adc_clock;
p.bw=80e3;p.recovery_tau=20e-6;p.b0=[.004 -.003]; % 假设固定零偏A
p.k=[.123 .118]; % 合成生成器参数；不自动交给辨识器
p.kp=.5;p.ki=150;p.tracking=200;p.ref_limit=.04;p.mod_limit=.0577350269;
p.seed=2801;
end
