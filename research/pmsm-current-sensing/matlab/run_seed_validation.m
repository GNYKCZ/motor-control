function run_seed_validation
% 20独立噪声种子复核名义匹配案例；不借此声称参数空间全域鲁棒。
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));rows={};
for seed=1:20
 c=closed_loop_config;c.guard="interval";c.certified=true;c.seed=9300+seed;
 rng(c.seed);c.noise=c.sensor_noise*randn(ceil(c.duration/c.p.Ts)+2,2);z=simulate_foc(c);keep=z.controller(:,6)>0&z.controller(:,20)>1;
 e=z.controller(keep,8:9)-z.plant(keep,4:5);r=sqrt(mean(sum(e.^2,2)));trip=max(z.controller(:,7));
 rows(end+1,:)={c.seed,r,trip,max(abs(z.controller(:,1:2)),[],'all')};assert(trip==8 && isfinite(r));
end
T=cell2table(rows,'VariableNames',{'seed','dq_RMSE_A','trip','peak_integral_V'});writetable(T,fullfile(root,'results','data','seed_validation.csv'));
mu=mean(T.dq_RMSE_A);sd=std(T.dq_RMSE_A);ci=mu+[-1 1]*2.093*sd/sqrt(20);
save(fullfile(root,'results','data','seed_validation.mat'),'T','mu','sd','ci');
fprintf('SEED_VALIDATION_PASS n=20 mean=%g sd=%g CI_mean_only=[%g,%g]\n',mu,sd,ci);
end
