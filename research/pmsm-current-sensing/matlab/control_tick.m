function s=control_tick(s,input,c)
% Only raw ADC + measured angle enter controller. No true current access.
p=c.p;tick=s(20);t=tick*c.dt;M=round(p.Ts/c.dt);
if s(6) && tick==s(10),s(1:2)=s(11:12);s(3:5)=s(13:15);s(10)=inf;end
if s(6) && mod(tick,M)==0
 raw=input(1:2);theta=input(3);d.duty=s(3:5)';d.bus=c.bus;
 phase=mod(t*p.fpwm+.5+mod(c.adc_phase_s*p.fpwm,1),1);rise=(1-d.duty(1:2))/2;fall=(1+d.duty(1:2))/2;
 d.state=double(phase>=rise & phase<fall);ar=mod(phase-rise,1);af=mod(phase-fall,1);d.age=min(ar,af)/p.fpwm;d.edge_sign=2*(ar<af)-1;
 measured=(raw*p.Vref/4096-p.ref)/(p.shunt*p.gain);bh=zeros(2,1);
 if c.method=="zero_only",bh=p.b0';elseif c.method=="fixed50",bh=p.b0'+.0615;
 elseif startsWith(c.method,"offline"),bh=predict_bias(c.fit,d)';
 elseif c.method=="bias_ESO"
  phaseVoltage=12*(s(3:5)-mean(s(3:5))); % known command model, not actual bus
  for ch=1:2
   ids=24+(ch-1)*2+(1:2);prior=c.esoA*s(ids)+c.esoB*phaseVoltage(ch);
   posterior=prior+c.esoK*(measured(ch)-c.esoC*prior);s(ids)=posterior;bh(ch)=posterior(2);
  end
 end
 corrected=measured-bh;s(16:17)=corrected;s(8:9)=clarke_park(corrected',theta)';
 counts=(measured-p.b0')/p.lsb;trip=0;
 if any(raw<=0|raw>=4095),trip=1;
 elseif c.guard=="original"
  if any(abs(counts)>128),trip=2;elseif abs(sum(counts))>128,trip=3;end
 else
  valid=c.certified && all(d.duty>=.3 & d.duty<=.7) && c.bus>=10 && c.bus<=14;
  [allow,bounds,~]=compensation_guard(raw,bh,c.error_bound,p,valid);s(21)=max(bounds);
  if ~allow,trip=4;end
 end
 if t>=c.duration-1e-10,trip=8;end
 ramp=max(0,min([t/.025,1,(c.duration-t)/.025]));ref=c.reference*ramp;s(18)=ref;
 if trip==0
  y=firmware_core_mex(s(1:2),[corrected;theta;0;ref;12;p.Ts]);
  if ~y(1),trip=5;elseif any(abs(y(4:5))>.08),trip=6;elseif t>.1 && abs(y(5)-ref)>.04,trip=7;end
  if trip==0,s(11:12)=y(2:3);s(13:15)=round(y(8:10)*1800)/1800;s(10)=tick+round(c.delay/c.dt)-1;s(19)=y(11);end
 end
 if trip~=0,s(6)=0;s(7)=trip;s(1:2)=0;s(10)=inf;end
end
s(20)=tick+1;
end
