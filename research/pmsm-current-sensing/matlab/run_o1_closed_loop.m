function run_o1_closed_loop
% 可信独立标定台架先递推学习；运行电机时没有独立参考，门控冻结。
% 不向控制器传入仿真真值。与已有273组使用同一工况/噪声种子。
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));
p=parameters;d=make_design(1200,'train',2801);rng(18);
y=p.b0+d.duty(:,1:2).*p.k+.0008*randn(1200,2);rows={};calibration=struct;
methods=["RLS_frozen" "gradient_frozen"];
scenarios=["matched" "low_bus" "high_bus" "high_noise" "slow_apply" "R_high" "L_low" "flux_high" "temperature_shift" "bias_M2" "bias_M3" "no_bias" "dynamic_rotor"];
for method=methods
 fit.model="M1";history=zeros(1200,4);updates=zeros(1200,2);
 for ch=1:2
  theta=zeros(2,1);P=1000*eye(2);
  for j=1:1200
   x=[1;d.duty(j,ch)];first=max(1,j-31);X=[ones(j-first+1,1),d.duty(first:j,ch)];
   pe=rank(X)==2 && min(eig(X'*X))>.001;
   if pe
    error=y(j,ch)-x'*theta;
    if method=="RLS_frozen",g=P*x/(.995+x'*P*x);theta=theta+g*error;P=(P-g*x'*P)/.995;
    else,theta=theta+.15*x*error/(x'*x);end
    updates(j,ch)=1;
   end
   history(j,(ch-1)*2+(1:2))=theta';
  end
  fit.coef{ch}=theta;
 end
 calibration.(char(method))=struct('fit',fit,'history',history,'updates',updates,'known_reference','synthetic independent zero-current calibration');
 for sc=scenarios
 for seed=1:3
  c=closed_loop_config;c.guard="interval";c.certified=true;c.method="offline_"+method;c.fit=fit;c.seed=2800+seed;
  switch sc
   case "low_bus",c.bus=10;
   case "high_bus",c.bus=14;
   case "high_noise",c.sensor_noise=.003;
   case "slow_apply",c.delay=.00095;
   case "R_high",c.Rfactor=1.3;
   case "L_low",c.Lfactor=.7;
   case "flux_high",c.psifactor=2;c.motion="prescribed";c.omega=1;
   case "temperature_shift",c.temp_shift=.01;
   case "bias_M2",c.bias_model="M2";
   case "bias_M3",c.bias_model="M3";
   case "no_bias",c.bias_model="none";
   case "dynamic_rotor",c.motion="dynamic";
  end
  rng(c.seed);c.noise=c.sensor_noise*randn(ceil(c.duration/c.p.Ts)+2,2);z=simulate_foc(c);s=z.controller;x=z.plant;keep=s(:,6)>0&s(:,20)>1;
  rmse=NaN;tracking=NaN;if any(keep),rmse=sqrt(mean(sum((s(keep,8:9)-x(keep,4:5)).^2,2)));tracking=sqrt(mean((x(keep,5)-s(keep,18)).^2));end
  sample=find(mod(round(z.t/c.dt),round(c.p.Ts/c.dt))==1 & keep);breach=0;
  for j=sample',breach=breach+any(abs(s(j,16:17)-x(j-1,1:2))>c.error_bound'+c.p.lsb/2);end
  assert(isequal(z.config.fit.coef,fit.coef)); % 运行段参数确实冻结
  rows(end+1,:)={sc,method,seed,max(s(:,7)),nnz(keep)*c.dt,rmse,tracking,max(vecnorm(x(:,4:5),2,2)),max(abs(s(:,1:2)),[],'all'),mean(s(:,19)),breach,c.sensor_noise,c.bus};
 end
 end
 fprintf('O1 closed-loop completed %s\n',method);
end
old=readtable(fullfile(root,'results','data','foc_robustness.csv'),'TextType','string');
T=cell2table(rows,'VariableNames',old.Properties.VariableNames);writetable(T,fullfile(root,'results','data','o1_closed_loop.csv'));
save(fullfile(root,'results','data','o1_closed_loop.mat'),'T','calibration');
allCases=[old;T];writetable(allCases,fullfile(root,'results','data','unified_closed_loop.csv'));
assert(height(T)==78 && height(allCases)==351);
fprintf('O1_CLOSED_LOOP_PASS 78 new cases; unified 351 cases\n');
end
