function c=closed_loop_config
p=parameters;c.p=p;c.dt=5e-6;c.duration=.25;c.reference=.04;c.method="offline_M1";
c.guard="original";c.bias_model="M1";c.sensor_noise=.0008;c.seed=2801;
c.delay=.000775;c.omega=0;c.motion="locked";c.J=2e-5;c.B=2e-5;c.load=0;
c.adc_phase_s=250e-6; % 既有合成M3采用高窗；明确对应第二次250us诊断，不冒充默认225us。
c.Rfactor=1;c.Lfactor=1;c.psifactor=1;c.bus=12;c.bias_factor=1;c.temp_shift=0;
a=exp(-p.Rs*p.Ts/mean([p.Ld p.Lq]));c.esoA=diag([a 1]);c.esoB=[(1-a)/p.Rs;0];c.esoC=[1 1];c.esoK=place(c.esoA',(c.esoC*c.esoA)',exp(-2*pi*20*p.Ts*[1 1.2]))';
c.certified=false;c.error_bound=[.003;.003];
root=fileparts(fileparts(mfilename('fullpath')));f=load(fullfile(root,'results','data','identification.mat'),'fits_all');c.fit=f.fits_all.M1;
end
