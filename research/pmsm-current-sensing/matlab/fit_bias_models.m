function fit=fit_bias_models(d,y,model)
% 仅接收已知参考条件下取得的偏移目标y；不接收生成器参数。
fit.model=string(model);fit.coef=cell(1,2);fit.ci95=cell(1,2);fit.rank=zeros(1,2);fit.condition=zeros(1,2);
for ch=1:2
 X=bias_features(d,ch,model);k=size(X,2);n=size(X,1);fit.rank(ch)=rank(X);fit.condition(ch)=cond(X);
 if rank(X)<k,fit.coef{ch}=nan(k,1);fit.ci95{ch}=nan(k,2);continue;end
 beta=X\y(:,ch);res=y(:,ch)-X*beta;s2=sum(res.^2)/(n-k);[~,RR]=qr(X,0);covar=s2*(RR\eye(k))*(RR\eye(k))';
 % 正态近似区间；非硬件置信保证。
 half=1.96*sqrt(diag(covar));fit.coef{ch}=beta;fit.ci95{ch}=[beta-half beta+half];
end
end
