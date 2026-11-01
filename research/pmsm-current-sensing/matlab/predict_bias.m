function y=predict_bias(fit,d)
y=zeros(size(d.duty,1),2);for ch=1:2,y(:,ch)=bias_features(d,ch,fit.model)*fit.coef{ch};end
end
