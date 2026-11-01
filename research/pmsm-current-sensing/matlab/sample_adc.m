function raw=sample_adc(uv,duty,t,c)
% Sensor generator, separated from estimator. Noise repeatable across methods.
p=c.p;d.duty=duty(:)';d.bus=c.bus;
phase=mod(t*p.fpwm+.5+mod(c.adc_phase_s*p.fpwm,1),1);rise=(1-d.duty(1:2))/2;fall=(1+d.duty(1:2))/2;
d.state=double(phase>=rise & phase<fall);ar=mod(phase-rise,1);af=mod(phase-fall,1);d.age=min(ar,af)/p.fpwm;d.edge_sign=2*(ar<af)-1;
b=p.b0';
if c.bias_model=="M1",b=b+c.bias_factor*p.k'.*duty(1:2);
elseif c.bias_model=="M2",b=b+c.bias_factor*p.k'*mean(duty);
elseif c.bias_model=="M3",for ch=1:2,b(ch)=bias_features(d,ch,'M3')*[p.b0(ch);p.k(ch)*.65*c.bias_factor;.02;.035];end
end
b=b+c.temp_shift;
% Deterministic pseudo-noise trace is pre-generated in config, no RNG in RHS.
j=min(size(c.noise,1),floor((t+1e-10)/p.Ts)+1);noise=c.noise(j,:)';
raw=min(4095,max(0,round((p.ref+(uv+b+noise)*p.shunt*p.gain)/p.Vref*4096)));
end
