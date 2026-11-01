function run_switching_crosscheck
% Model C/B共同物理子集：锁转子、Ld=Lq均值、U/V分流器、无死区。
% 先隔离PWM离散效应；INA动态/死区已由run_model_c独立扫描。
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));
rows={};traces=struct;
for reference=[0 .04]
 a=runplant(false,25e-9,reference);b=runplant(true,25e-9,reference);fine=runplant(true,12.5e-9,reference);
 error=max(abs(a.cycle_mean-b.cycle_mean),[],'all');refine=max(abs(b.cycle_mean-fine.cycle_mean),[],'all');
 fprintf('Crosscheck diagnostics: trips %g/%g/%g error=%g refinement=%g\n',a.trip,b.trip,fine.trip,error,refine);
 save(fullfile(root,'results','data','switching_crosscheck_latest_diagnostic.mat'),'a','b','fine','error','refine');
 assert(a.trip==8 && b.trip==8 && fine.trip==8);assert(error<.01 && refine<.005);
 assert(max(abs(sum(fine.cycle_mean,2)))<1e-12);
 rows(end+1,:)={reference,error,refine,a.trip,b.trip,max(abs(b.sample_current),[],'all')};
 key=['ref_' num2str(round(reference*1000))];traces.(key)=struct('average',a,'switching',b,'fine',fine);
 fprintf('PWM crosscheck reference=%g mean_error=%g refinement=%g\n',reference,error,refine);
end
T=cell2table(rows,'VariableNames',{'reference_A','cycle_mean_difference_A','step_refinement_A','average_trip','switching_trip','sample_peak_A'});
writetable(T,fullfile(root,'results','data','switching_crosscheck.csv'));save(fullfile(root,'results','data','switching_crosscheck.mat'),'T','traces');
f=figure('Visible','off');q=traces.ref_40;plot(q.average.t,q.average.cycle_mean(:,2),q.switching.t,q.switching.cycle_mean(:,2));legend('Average inverter','Switched inverter');xlabel('s');ylabel('V phase cycle mean A');grid on;exportgraphics(f,fullfile(root,'results','figures','switching_crosscheck.png'));close(f);
fprintf('SWITCHING_CROSSCHECK_PASS\n');
end
function z=runplant(switched,dt,reference)
c=closed_loop_config;c.dt=dt;c.duration=.04;c.reference=reference;c.method="zero_only";c.bias_model="none";c.noise=zeros(100,2);p=c.p;
E=[1 0;0 1;-1 -1];P=eye(3)-ones(3)/3;L=mean([p.Ld p.Lq]);R=diag([p.Rs+p.shunt;p.Rs+p.shunt;p.Rs]);A=-P(1:2,:)*R*E/L;B=P(1:2,:)/L;
e=expm([A B;zeros(3,5)]*dt);Ad=e(1:2,1:2);Bd=e(1:2,3:5);uv=zeros(2,1);s=control_initial;M=round(p.Ts/dt);W=round(1/p.fpwm/dt);N=round(c.duration/dt)+3;
z.cycle_mean=zeros(floor((N-1)/W),3);z.t=zeros(size(z.cycle_mean,1),1);z.sample_current=[];acc=zeros(3,1);cycle=0;raw=[2048;2048];
for k=0:N-1
 t=k*dt;i=E*uv;
 if mod(k,M)==0,raw=sample_adc(uv,s(3:5),t,c);z.sample_current(end+1,:)=i';end
 next=control_tick(s,[raw;0],c);
 if s(6)
  if switched,carrier=abs(2*mod(k/W,1)-1);pole=p.Vbus*double(s(3:5)>=carrier);
  else,pole=p.Vbus*s(3:5);end
 else,pole=zeros(3,1);end
 uv=Ad*uv+Bd*pole;s=next;acc=acc+i;
 if mod(k+1,W)==0,cycle=cycle+1;z.cycle_mean(cycle,:)=(acc/W)';z.t(cycle)=(k+1)*dt;acc(:)=0;end
end
z.trip=s(7);z.dt=dt;
end
