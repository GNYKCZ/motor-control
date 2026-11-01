function run_model_c
% Explicit ideal MOSFET+diode leg states and nonoverlap dead time.
% Electrical zero-sequence constraint, shunts, amplifier signal bandwidth,
% distinct assumed common-mode feedthrough, ADC aperture and conversion.
root=fileparts(fileparts(mfilename('fullpath')));p=parameters;rows={};traces=struct;
for dtdead=[0 90e-9 500e-9]
 for phase=[0 12.5e-6 25e-6 37.5e-6]
 for coupling=[0 .03]
  z=sw(p,[.3 .5 .7],dtdead,phase,coupling,25e-9,.002);
  keep=z.sample_t>.001;er=z.measured(keep,:)-z.sample_truth(keep,:);
  rows(end+1,:)={dtdead,phase,coupling,sqrt(mean(er.^2,'all')),max(abs(er),[],'all'),max(abs(sum(z.current,2))),max(z.shootthrough),mean(z.phase_voltage(z.t>.001,:),1)};
  if dtdead==90e-9&&phase==12.5e-6&&coupling==.03,traces.nominal=z;end
 end
 end
end
T=cell2table(rows,'VariableNames',{'deadtime_s','phase_s','coupling_A','sample_RMSE_A','sample_peak_A','KCL_error_A','shootthrough','mean_phase_voltage_V'});
assert(all(T.KCL_error_A<1e-12));assert(~any(T.shootthrough));
% Independent average-voltage balance; Ldi/dt+Ri=v over a whole final cycle.
z=sw(p,[.3 .5 .7],0,12.5e-6,0,25e-9,.004);ix=z.t>=.00395;meanV=mean(z.phase_voltage(ix,:),1);expected=p.Vbus*([.3 .5 .7]-.5);volterr=max(abs(meanV-expected));assert(volterr<.02);
% Independent convergence; selected aperture sufficiently away from edges.
a=sw(p,[.3 .5 .7],100e-9,12.5e-6,.03,25e-9,.002);b=sw(p,[.3 .5 .7],100e-9,12.5e-6,.03,12.5e-9,.002);
refine=max(abs(a.sample_truth-b.sample_truth),[],'all');assert(refine<.001);
zero=sw(p,[.5 .5 .5],0,12.5e-6,0,25e-9,.001);assert(max(abs(zero.current),[],'all')<1e-12);
writetable(T,fullfile(root,'results','data','model_c_sweep.csv'));writetable(table(volterr,refine),fullfile(root,'results','data','model_c_checks.csv'));
save(fullfile(root,'results','data','model_c.mat'),'T','traces','volterr','refine');
f=figure('Visible','off');tiledlayout(3,1);z=traces.nominal;ix=z.t>.0019;nexttile;plot(z.t(ix)*1e6,z.phase_voltage(ix,:));ylabel('Phase V');grid on;nexttile;plot(z.t(ix)*1e6,z.current(ix,:));ylabel('Phase A');grid on;nexttile;plot(z.sample_t*1e6,(z.measured-z.sample_truth)*1000,'o-');ylabel('Sample error mA');xlabel('Time us');grid on;exportgraphics(f,fullfile(root,'results','figures','model_c_switching.png'));close(f);
fprintf('MODEL_C_SWITCHING_PASS %d cases; average voltage error %g V; refinement %g A\n',height(T),volterr,refine);
end
function z=sw(p,duty,dead,phase,coupling,dt,duration)
t=(0:dt:duration)';n=numel(t);i=zeros(3,1);f=zeros(2,1);transient=f;lastCmd=zeros(3,1);state=lastCmd;release=zeros(3,1);oldPole=zeros(3,1);
% Equal inductance locked-rotor plant; shunts only in U/V, W has no shunt.
L=mean([p.Ld p.Lq]);Rphase=[p.Rs+p.shunt;p.Rs+p.shunt;p.Rs];projection=eye(3)-ones(3)/3;
E=[1 0;0 1;-1 -1];A=-projection(1:2,:)*diag(Rphase)*E/L;B=projection(1:2,:)/L;exact=expm([A B;zeros(3,5)]*dt);Ad=exact(1:2,1:2);Bd=exact(1:2,3:5);af=exp(-2*pi*p.bw*dt);
z.current=zeros(n,3);z.phase_voltage=z.current;z.shootthrough=false(n,1);signal=zeros(n,2);
for k=1:n
 carrier=abs(2*mod(t(k)*p.fpwm,1)-1);cmd=double(duty(:)>=carrier);
 changed=cmd~=lastCmd;release(changed)=t(k)+dead;state(changed)=-1;lastCmd=cmd;
 ready=t(k)>=release;state(ready)=cmd(ready);
 upper=(state==1);lower=(state==0);z.shootthrough(k)=any(upper&lower);
 % state 1 upper MOSFET on,0 lower on,-1 both off. Positive current
 % freewheels through lower diode, negative through upper diode.
 pole=state;off=state<0;pole(off)=double(i(off)<0);
 vp=p.Vbus*pole;neutral=mean(vp-Rphase.*i);v=vp-neutral;
 i=E*(Ad*i(1:2)+Bd*vp);f=af*f+(1-af)*i(1:2);
 transient=transient*exp(-dt/p.recovery_tau)+coupling*(pole(1:2)-oldPole(1:2));oldPole=pole;
 signal(k,:)=p.ref+(f+transient)'*p.shunt*p.gain;
 z.current(k,:)=i';z.phase_voltage(k,:)=v';
end
% ADC samples aperture average; report t at end of aperture, not conversion end.
starts=(phase:1/p.fpwm:duration-p.aperture)';valid=starts>=0;starts=starts(valid);z.sample_t=starts+p.aperture;
z.conversion_complete_t=starts+p.conversion; % 孔径终点与转换完成分开，后者不改变采样真值
z.measured=zeros(numel(starts),2);z.sample_truth=z.measured;
for j=1:numel(starts)
 ix=find(t>=starts(j)&t<=starts(j)+p.aperture);av=mean(signal(ix,:),1);
 raw=min(4095,max(0,round(av/p.Vref*4096)));z.measured(j,:)=(raw*p.Vref/4096-p.ref)/(p.shunt*p.gain);
 z.sample_truth(j,:)=mean(z.current(ix,1:2),1);
end
z.t=t;
end
