function run_phase_features
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));f=load(fullfile(root,'results','data','identification.mat'),'fits_all');rows={};
for phase=[225 250]*1e-6
 c=closed_loop_config;c.adc_phase_s=phase;c.bias_model="M3";c.method="offline_M3";c.fit=f.fits_all.M3;c.guard="interval";c.certified=true;
 % 独立TIM1中心对齐计数公式：0起计，上数到1800再下数，PWM1高有效。
 counter_phase=mod(round(phase*72e6),3600);counter=1800-abs(counter_phase-1800);expected=double(counter<900);
 normalized=mod(.5+mod(phase*c.p.fpwm,1),1);state=double(normalized>=.25 && normalized<.75);assert(state==expected);
 rng(c.seed);c.noise=c.sensor_noise*randn(ceil(c.duration/c.p.Ts)+2,2);z=simulate_foc(c);live=z.controller(:,6)>0 & z.controller(:,20)>1;
 e=z.controller(live,8:9)-z.plant(live,4:5);r=sqrt(mean(sum(e.^2,2)));trip=max(z.controller(:,7));assert(trip==8 && r<.003);
 rows(end+1,:)={phase,expected,trip,r};
end
T=cell2table(rows,'VariableNames',{'phase_s','equal50_PWM_high','trip','dq_RMSE_A'});writetable(T,fullfile(root,'results','data','phase_features.csv'));
assert(T.equal50_PWM_high(1)==0 && T.equal50_PWM_high(2)==1);
fprintf('PHASE_FEATURES_PASS low225/high250; synthetic M3 only\n');
end
