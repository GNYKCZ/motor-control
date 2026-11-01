function run_o3_mismatch
root=fileparts(fileparts(mfilename('fullpath')));p=parameters;rows={};
for tau=[5e-6 20e-6 60e-6]
 for nonlin=[0 .02]
 for seed=1:20
  train=make_design(500,'train',seed+100);val=make_design(300,'holdout',seed+200);test=make_design(300,'mixed',seed+300);
  % Plant differs from candidate M3 fixed20us basis when tau differs.
  bt=truth(train,p,tau,nonlin);bv=truth(val,p,tau,nonlin);be=truth(test,p,tau,nonlin);
  rng(seed+400);observed=bt+.0008*randn(size(bt));scores=zeros(1,3);fitset=cell(1,3);models=["M1" "M2" "M3"];
  for k=1:3,fitset{k}=fit_bias_models(train,observed,models(k));ev=bv-predict_bias(fitset{k},val);scores(k)=sqrt(mean(ev.^2,'all'));end
  eligible=find(scores<=min(scores)+.0002);idx=eligible(1);fit=fitset{idx};e=be-predict_bias(fit,test);
  % Predeclared 3mA screening target, not a proven worst-case certificate.
  accept=max(abs(bv-predict_bias(fit,val)),[],'all')<=.003;
  rows(end+1,:)={tau,nonlin,seed,models(idx),scores(idx),sqrt(mean(e.^2,'all')),max(abs(e),[],'all'),accept,max(abs(e),[],'all')<=.003};
 end
 end
end
T=cell2table(rows,'VariableNames',{'plant_tau_s','nonlinear_coefficient_A','seed','selected','validation_RMSE_A','test_RMSE_A','test_peak_A','validation_accept','test_within_3mA'});
writetable(T,fullfile(root,'results','data','o3_mismatch.csv'));save(fullfile(root,'results','data','o3_mismatch.mat'),'T');
matched=T.plant_tau_s==20e-6&T.nonlinear_coefficient_A==0;assert(all(T.validation_accept(matched)));assert(all(T.test_within_3mA(matched)));
fprintf('O3_MODEL_MISMATCH_COMPLETE %d cases; accepted=%d rejected=%d\n',height(T),nnz(T.validation_accept),nnz(~T.validation_accept));
end
function b=truth(d,p,tau,nonlin)
b=zeros(numel(d.t),2);for ch=1:2,u=d.duty(:,ch);b(:,ch)=p.b0(ch)+p.k(ch)*.65*u.*d.bus/12+.02*d.state(:,ch).*d.bus/12+.035*d.edge_sign(:,ch).*exp(-d.age(:,ch)/tau).*d.bus/12+nonlin*(u-.5).^2;end
end
