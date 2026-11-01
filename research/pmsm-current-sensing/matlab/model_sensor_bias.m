function s=model_sensor_bias(t,current,bias,p,noise,delay)
% 信号链顺序：采样电阻->放大增益/REF->采样->ADC噪声->饱和量化。
% bias是等效输入电流偏置，noise是ADC输入等效安培噪声，delay单位s。
assert(size(current,2)==2 && isequal(size(current),size(bias),size(noise)));
ts=max(t(1),t-delay);s.truth_at_sample=interp1(t,current,ts,'linear');
bs=interp1(t,bias,ts,'linear');s.analog_V=p.ref+(s.truth_at_sample+bs)*p.shunt*p.gain;
lsbV=p.Vref/2^p.bits;unclipped=round((s.analog_V+noise*p.shunt*p.gain)/lsbV);
s.saturated=unclipped<0|unclipped>2^p.bits-1;s.raw=min(2^p.bits-1,max(0,unclipped));
s.measured=(s.raw*lsbV-p.ref)/(p.shunt*p.gain);s.t=t;s.truth=current;
end
