function run_model_b
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));p=parameters;
assert(exist('firmware_core_mex','file')==3,'Build unchanged C core bridge first');
% independent double-precision controller oracle, including old-integrator order.
rng(420);worst=0;for j=1:1000
 s=.2*randn(2,1);x=[.08*randn(2,1);2*pi*rand; .06*randn(2,1);12;.001];
 if any(abs(x(1:2))>.2),continue;end
 y=firmware_core_mex(s,x);z=oracle(s,x,p);worst=max(worst,max(abs(y-z)));assert(y(1)==1);
end
assert(worst<2e-6);
load(fullfile(root,'results','data','identification.mat'),'fits_all');
methods=["raw" "zero_only" "fixed50" "offline_M1" "offline_M2" "offline_M3"];
scenarios=["bias_M1" "bias_M2" "bias_M3" "no_bias" "zero_vector_diagnostic"];
rows={};traces=struct;
for scenario=scenarios
 for method=methods
  z=simulate(p,scenario,method,fits_all,5e-6);key=char(scenario+"_"+method);traces.(key)=z;
  active=z.active; if any(active),er=z.meas(:,active)-z.truth(:,active);rmse=sqrt(mean(sum(er.^2,1)));else,rmse=NaN;end
  rows(end+1,:)={scenario,method,z.trip,z.tripTime,rmse,max(vecnorm(z.truth)),max(abs(z.integral),[],'all'),nnz(active)};
 end
end
result=cell2table(rows,'VariableNames',{'scenario','method','trip_reason','trip_time_s','active_dq_error_RMSE_A','peak_true_A','peak_integral_V','active_samples'});
% Main finding: raw W protection precedes correction in current firmware.
assert(all(result.trip_reason(result.scenario=="bias_M1")=="raw_W"));
assert(all(result.trip_reason(result.scenario=="zero_vector_diagnostic")=="duration"));
a=simulate(p,"no_bias","zero_only",fits_all,5e-6);b=simulate(p,"no_bias","zero_only",fits_all,2.5e-6);
refine=max(vecnorm(a.truth-b.truth(:,1:2:end)));assert(refine<.001);
writetable(result,fullfile(root,'results','data','model_b_protected.csv'));
writetable(table(worst,refine),fullfile(root,'results','data','model_b_checks.csv'));
save(fullfile(root,'results','data','model_b.mat'),'result','traces','worst','refine');
f=figure('Visible','off');tiledlayout(2,1);nexttile;z=traces.no_bias_zero_only;plot(z.t,z.truth'*1000);ylabel('True dq (mA)');grid on;nexttile;z=traces.bias_M1_offline_M1;plot(z.t,z.meas'*1000);ylabel('Feedback dq (mA)');xlabel('s');title('Existing raw-W protection stops before compensation');grid on;exportgraphics(f,fullfile(root,'results','figures','model_b_protected.png'));close(f);
disp(result);fprintf('MODEL_B_CORE_AND_PROTECTION_PASS oracle=%g refine=%g\n',worst,refine);
end
function z=simulate(p,scenario,method,fits,dt)
N=round(.25/dt)+1;M=round(p.Ts/dt);delay=round(.000775/dt);x=zeros(2,1);integ=x;pending=x;v=x;due=inf;duty=[.5;.5;.5];theta=0;
z.t=(0:N-1)*dt;z.truth=zeros(2,N);z.meas=z.truth;z.integral=z.truth;z.active=false(1,N);z.trip="none";z.tripTime=NaN;live=true;meas=x;
for j=1:N
 t=z.t(j);
 if j==due && live,integ=pending(3:4);duty=pending(5:7);ph=p.Vbus*(duty-mean(duty));v=[ph(1);(ph(1)+2*ph(2))/sqrt(3)];end
 if mod(j-1,M)==0 && live
  uv=[x(1);(-x(1)+sqrt(3)*x(2))/2];
  d.duty=duty';d.bus=12;d.state=[0 0];d.age=[12.5e-6 12.5e-6];d.edge_sign=[-1 -1];
  switch scenario
  case {"bias_M1","zero_vector_diagnostic"},bias=p.b0'+p.k'.*duty(1:2);
  case "bias_M2",bias=p.b0'+p.k'*mean(duty);
  case "bias_M3",bias=zeros(2,1);for ch=1:2,bias(ch)=bias_features(d,ch,'M3')*[p.b0(ch);p.k(ch)*.65;.02;.035];end
  otherwise,bias=p.b0';
  end
  raw=round((p.ref+uv+bias)/p.Vref*4096);baseline=(p.ref+p.b0')/p.Vref*4096;counts=raw-baseline;
  measured=(raw/4096*p.Vref-p.ref);corrected=measured;
  switch method
  case "zero_only",corrected=measured-p.b0';
  case "fixed50",corrected=measured-p.b0'-.0615;
  case {"offline_M1","offline_M2","offline_M3"},f=fits.(char(extractAfter(method,'offline_')));corrected=measured-predict_bias(f,d)';
  end
  meas=clarke_park(corrected',theta)';reason="none";
  if any(raw<=0|raw>=4095),reason="ADC_saturation";
  elseif any(abs(counts)>128),reason="raw_UV";
  elseif scenario~="zero_vector_diagnostic" && abs(sum(counts))>128,reason="raw_W";
  elseif t>=.25 || (scenario=="zero_vector_diagnostic" && t>=.02),reason="duration";
  end
  if reason~="none",live=false;z.trip=reason;z.tripTime=t;v(:)=0;
  elseif scenario~="zero_vector_diagnostic"
   ramp=min([t/.025,1,(.25-t)/.025]);ref=.04*max(0,ramp);y=firmware_core_mex(integ,[corrected;theta;0;ref;12;.001]);
   if ~y(1),reason="core_rejected";elseif any(abs(y(4:5))>.08),reason="dq_absolute";elseif t>.1 && abs(y(5)-ref)>.04,reason="tracking";end
   if reason~="none",live=false;z.trip=reason;z.tripTime=t;v(:)=0;
   else,pending=[y(6:7);y(2:3);round(y(8:10)*1800)/1800];due=j+delay;
   end
  end
 end
 z.truth(:,j)=x;z.meas(:,j)=meas;z.integral(:,j)=integ;z.active(j)=live;
 % Locked rotor dq: exact ZOH RL step, separate Ld/Lq. No oracle to controller.
 L=[p.Ld;p.Lq];a=exp(-p.Rs*dt./L);x=a.*x+(1-a).*v/p.Rs;
end
end
function z=oracle(s,x,p)
dq=clarke_park(x(1:2)',x(3))';ref=min(.04,max(-.04,x(4:5)));err=ref-dq;request=.5*err+s;
limit=p.mod_limit*x(6)/sqrt(3);limited=norm(request)>limit;v=request*min(1,limit/max(norm(request),eps));
ns=s+150*x(7)*err+200*x(7)*(v-request);th=x(3);ab=[cos(th) -sin(th);sin(th) cos(th)]*v;
phase=[ab(1);-.5*ab(1)+sqrt(3)/2*ab(2);-.5*ab(1)-sqrt(3)/2*ab(2)];duty=.5+(phase-(max(phase)+min(phase))/2)/x(6);
z=[1;ns;dq;v;duty;limited;any(ref~=x(4:5));request];
end
