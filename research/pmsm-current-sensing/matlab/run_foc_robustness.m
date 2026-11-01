function run_foc_robustness
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));f=load(fullfile(root,'results','data','identification.mat'),'fits_all');
scenarios=["matched" "low_bus" "high_bus" "high_noise" "slow_apply" "R_high" "L_low" "flux_high" "temperature_shift" "bias_M2" "bias_M3" "no_bias" "dynamic_rotor"];
methods=["raw" "zero_only" "fixed50" "offline_M1" "offline_M2" "offline_M3" "bias_ESO"];rows={}; evidence=struct;
for sc=scenarios
 for method=methods
 for seed=1:3
 c=closed_loop_config;c.guard="interval";c.certified=true;c.method=method;c.seed=2800+seed;
 if startsWith(method,'offline'),c.fit=f.fits_all.(char(extractAfter(method,'offline_')));end
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
 validTime=nnz(keep)*c.dt;rmse=NaN;tracking=NaN;if any(keep),e=s(keep,8:9)-x(keep,4:5);rmse=sqrt(mean(sum(e.^2,2)));tracking=sqrt(mean((x(keep,5)-s(keep,18)).^2));end
 % This is deliberately a certificate-stress test. Flag invalid envelopes
 % afterwards; NEVER feed truth back to permit/deny controller operation.
 sample=find(mod(round(z.t/c.dt),round(c.p.Ts/c.dt))==1 & keep);breach=0;residuals=zeros(numel(sample),2);si=0;
 for j=sample'
  % corrected UV is from previous sample; compare to previous true UV.
  si=si+1;residual=s(j,16:17)-x(j-1,1:2);residuals(si,:)=residual;breach=breach+any(abs(residual)>c.error_bound'+c.p.lsb/2);
 end
 evidence.(char(sc+"_"+method+"_"+string(seed)))=struct('residual_uv',residuals,'bound',c.error_bound+c.p.lsb/2,'t',z.t(sample));
 rows(end+1,:)={sc,method,seed,max(s(:,7)),validTime,rmse,tracking,max(vecnorm(x(:,4:5),2,2)),max(abs(s(:,1:2)),[],'all'),mean(s(:,19)),breach,c.sensor_noise,c.bus};
 end
 end
 fprintf('Robustness completed scenario %s\n',sc);
end
T=cell2table(rows,'VariableNames',{'scenario','algorithm','seed','trip_code','active_duration_s','dq_error_RMSE_A','iq_tracking_RMSE_A','peak_true_A','peak_integral_V','saturation_fraction','diagnostic_envelope_breaches','noise_A','Vbus_V'});
writetable(T,fullfile(root,'results','data','foc_robustness.csv'));save(fullfile(root,'results','data','foc_robustness.mat'),'T','evidence');
fprintf('FOC_ROBUSTNESS_COMPLETE %d cases; not all cases expected to pass\n',height(T));
end
