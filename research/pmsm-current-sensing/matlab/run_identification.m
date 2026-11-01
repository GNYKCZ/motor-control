function run_identification(root,p)
out=fullfile(root,'results','data');models=["M1" "M2" "M3"];
% Identifiability: distinguish joint b0,k_phase,k_mean, not just one slope.
equal=make_design(300,'equal',1);constant=make_design(300,'constant',1);mixed=make_design(300,'mixed',1);
% Zero-sequence only at one fixed voltage vector: du=dmean+constant,
% so intercept absorbs the fixed difference. Varying vectors adds information.
z=linspace(-.15,.15,300)';zeroSeq=constant;zeroSeq.duty=[.45 .5 .55]+z;
D={equal,constant,zeroSeq,mixed};names=["equal_duty" "constant_duty" "zero_sequence_one_vector" "vary_vector_and_zero_sequence"];
ranks={};for j=1:4,d=D{j};X=[ones(300,1),d.duty(:,1),mean(d.duty,2)];sv=svd(X);ranks(end+1,:)={names(j),rank(X),cond(X),sv(end)};end
assert(ranks{1,2}==2 && ranks{2,2}==1 && ranks{3,2}==2 && ranks{4,2}==3);
writetable(cell2table(ranks,'VariableNames',{'design','rank','condition','smallest_singular_value'}),fullfile(out,'identifiability.csv'));
% Fixing known b0 changes the question: one nonzero duty identifies k if
% true current is independently known; it cannot validate slope generality.
assert(rank(.5*ones(300,1))==1);
train=make_design(1200,'train',2801);tune=make_design(400,'holdout',2802);test=make_design(600,'holdout',2803);
% Synthetic known-current calibration: reference is separate from estimator.
% Non-equal duty zero current here requires a signal-chain bench, not an
% energized motor assumed to have zero current.
rows={};coefrows={};algorithmrows={};online_records=struct;fits_all=struct;
for truthModel=models
 btrain=generator(train,truthModel,p);btune=generator(tune,truthModel,p);btest=generator(test,truthModel,p);
 rng(18);ytrain=btrain+.0008*randn(size(btrain));ytune=btune+.0008*randn(size(btune));
 fitset=cell(1,3);tuneScore=zeros(1,3);
 for j=1:3
  fit=fit_bias_models(train,ytrain,models(j));fitset{j}=fit;pred=predict_bias(fit,tune);e=ytune-pred;
  tuneScore(j)=sqrt(mean(e.^2,'all'));
  for ch=1:2,for k=1:numel(fit.coef{ch}),coefrows(end+1,:)={truthModel,models(j),ch,k,fit.coef{ch}(k),fit.ci95{ch}(k,1),fit.ci95{ch}(k,2),fit.rank(ch),fit.condition(ch)};end;end
  testError=btest-predict_bias(fit,test);rows(end+1,:)={truthModel,models(j),tuneScore(j),sqrt(mean(testError.^2,'all')),max(abs(testError),[],'all')};
 end
 % Select only by tuning set, prefer simpler model within 0.2mA of best.
 eligible=find(tuneScore<=min(tuneScore)+.0002);chosen=eligible(1);fit=fitset{chosen};fits_all.(char(truthModel))=fit;
 for noiseSD=[0 .0008 .003]
 for delaySamples=[0 1 2]
  % Real signal present; first100 independent known-zero frames are certified
  % in this synthetic bench only. Tuning/fit uses no samples from test.
  n=600;truth=[.03*sin(2*pi*4*test.t),.025*cos(2*pi*3*test.t)];truth(1:100,:)=0;
  rng(913);noise=noiseSD*randn(n,2);sensor=model_sensor_bias(test.t,truth,btest,p,noise,delaySamples*p.Ts);
  ix=max(1,(1:n)'-delaySamples);ds=test;fields={'duty','bus','state','age','edge_sign'};for ff=fields,ds.(ff{1})=test.(ff{1})(ix,:);end
  % Estimate fixed zero offset from independent 512-frame baseline.
  rng(912);b0est=mean(repmat(p.b0,512,1)+.0008*randn(512,2));
  methods=["raw" "zero_only" "fixed50" "offline_selected" "gated_RLS" "gated_gradient" "unsafe_RLS"];
  knownZero=(1:n)'<=100;theta=2*pi*test.t;history=struct;
  for method=methods
   switch method
   case "raw",bh=zeros(n,2);
   case "zero_only",bh=repmat(b0est,n,1);
   case "fixed50",bh=repmat(b0est+[.0615 .0615],n,1);
   case "offline_selected",bh=predict_bias(fit,ds);
   otherwise
    gate=knownZero;if method=="unsafe_RLS",gate=true(n,1);end
    [bh,hist,updates]=online_estimate(sensor.measured,ds.duty(:,1:2),gate,method,b0est);
    history.(char(method))=struct('parameters',hist,'updates',updates);
   end
   corrected=sensor.measured-bh;err=corrected-sensor.truth_at_sample;dqerr=clarke_park(err,theta);
   dynerr=corrected-sensor.truth;
   algorithmrows(end+1,:)={truthModel,fit.model,noiseSD,delaySamples,method,sqrt(mean(err.^2,'all')),sqrt(mean(sum(dqerr.^2,2))),max(vecnorm(dqerr,2,2)),sqrt(mean(dynerr.^2,'all')),sqrt(mean(err(101:end,:).^2,'all'))};
  end
  if noiseSD==.0008 && delaySamples==0,online_records.(char(truthModel))=history;end
 end
 end
end
modelscores=cell2table(rows,'VariableNames',{'truth_model','candidate','tuning_RMSE_A','heldout_bias_RMSE_A','heldout_peak_A'});
coefs=cell2table(coefrows,'VariableNames',{'truth_model','candidate','channel','parameter','estimate','ci95_low','ci95_high','rank','condition'});
alg=cell2table(algorithmrows,'VariableNames',{'truth_model','selected_model','noise_A','delay_samples','algorithm','sample_current_RMSE_A','dq_vector_RMSE_A','dq_peak_A','now_current_RMSE_A','nonzero_current_RMSE_A'});
writetable(modelscores,fullfile(out,'model_selection.csv'));writetable(coefs,fullfile(out,'fit_parameters.csv'));writetable(alg,fullfile(out,'algorithm_comparison_A.csv'));
% Exact ambiguity and gate checks: no updates outside trusted window.
for tm=models,h=online_records.(char(tm));assert(~any(h.gated_RLS.updates(101:end)));assert(max(abs(diff(h.gated_RLS.parameters(101:end,:),1,1)),[],'all')==0);end
% Demonstrate underexcited regressors: b0+k*.5=.0615 has many solutions.
assert(abs([1 .5]*[0;.123]-[1 .5]*[.02;.083])<1e-12);
% Dedicated poisoning test: constant real +40mA offset becomes estimated bias
% if naive adaptation assumes measurement is bias. It is not a convergence win.
d=.5*ones(600,2);measurement=.1015*ones(600,2);
[bad,~,~]=online_estimate(measurement,d,true(600,1),'unsafe_RLS',[0 0]);
assert(abs(bad(end,1)-.1015)<.001);
poison=table(.04,.0615,bad(end,1),measurement(end,1)-bad(end,1),'VariableNames',{'true_current_A','true_bias_A','estimated_bias_A','corrected_current_A'});writetable(poison,fullfile(out,'unsafe_adaptation_counterexample.csv'));
save(fullfile(out,'identification.mat'),'fits_all','modelscores','coefs','alg','online_records','train','tune','test');
f=figure('Visible','off');tiledlayout(1,3);for k=1:3,nexttile;q=alg(alg.truth_model==models(k)&alg.noise_A==.0008&alg.delay_samples==0,:);bar(categorical(q.algorithm),q.dq_vector_RMSE_A*1000);ylabel('dq error RMS (mA)');title(models(k));grid on;end
exportgraphics(f,fullfile(root,'results','figures','algorithm_comparison_A.png'));close(f);
fprintf('Identification: %d model comparisons, %d algorithm cases; rank/gate/poisoning assertions passed\n',height(modelscores),height(alg));
end
function b=generator(d,model,p)
b=zeros(size(d.duty,1),2);for ch=1:2
 X=bias_features(d,ch,model);
 if model=="M3",coef=[p.b0(ch);p.k(ch)*.65;.02;.035];else,coef=[p.b0(ch);p.k(ch)];end
 b(:,ch)=X*coef;
end
end
function [bh,hist,updates]=online_estimate(y,duty,gate,method,b0)
n=size(y,1);theta=[b0;zeros(1,2)];P=repmat(1000*eye(2),1,1,2);bh=zeros(n,2);hist=zeros(n,4);updates=false(n,1);
for j=1:n
 % Predictions use previous parameters; updates never rewrite earlier samples.
 for ch=1:2
  x=[1;duty(j,ch)];bh(j,ch)=x'*theta(:,ch);
  begin=max(1,j-31);X=[ones(j-begin+1,1),duty(begin:j,ch)];pe=rank(X)==2 && min(eig(X'*X))>.001;
  allowed=gate(j)&&(pe||string(method)=="unsafe_RLS");
  if allowed
   err=y(j,ch)-x'*theta(:,ch);
   if string(method)=="gated_gradient",theta(:,ch)=theta(:,ch)+.15*x*err/(x'*x);
   else,lambda=.995;gain=P(:,:,ch)*x/(lambda+x'*P(:,:,ch)*x);theta(:,ch)=theta(:,ch)+gain*err;P(:,:,ch)=(P(:,:,ch)-gain*x'*P(:,:,ch))/lambda;end
   updates(j)=true;
  end
 end
 hist(j,:)=theta(:)';
end
end
