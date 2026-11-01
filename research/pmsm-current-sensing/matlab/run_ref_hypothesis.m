function run_ref_hypothesis
% 原理图REF分压网络的独立数量级检查，不将直流简图当成PWM动态模型。
root=fileparts(fileparts(mfilename('fullpath')));p=parameters;
Rth=1/(1/1e4+1/1e4);Rpath=1e6+20e3;Vsource=1.65;
duty=[0;.5;.7];observed=[0;.0615;.086];
% 每个输入至REF的约1M电阻路径；两只器件共享参考节点。
vcm=p.Vbus*duty;ref=(Vsource/Rth+2*vcm/Rpath)/(1/Rth+2/Rpath);
baseline=(Vsource/Rth)/(1/Rth+2/Rpath);prediction=(ref-baseline)/(p.shunt*p.gain);
slope=2*p.Vbus*Rth/(Rpath+2*Rth)/(p.shunt*p.gain);
observedSlope=duty\observed;equivalentPath=2*p.Vbus*Rth/observedSlope/(p.shunt*p.gain)-2*Rth;
assert(abs(slope-.116504854368932)<1e-12);
assert(max(abs(prediction-[0;.058252427184466;.081553398058252]))<1e-12);
% 未用实测调参的名义预测；阻值容差仅敏感性范围，非统计置信区间。
ranges=Rpath*[.7;1;1.3];slopes=2*p.Vbus*Rth./(ranges+2*Rth)/(p.shunt*p.gain);
buffered=2*p.Vbus*1/(Rpath+2)/(p.shunt*p.gain); % 假设1Ω低阻参考作对照
assert(buffered<slope/1000);
T=table(duty,observed,prediction,ref,observed-prediction,'VariableNames',{'equal_duty','historical_40kHz_reading_A','DC_average_REF_prediction_A','REF_node_V','difference_A'});
S=table(ranges,slopes,'VariableNames',{'assumed_internal_path_ohm','predicted_slope_A_per_duty'});
writetable(T,fullfile(root,'results','data','ref_hypothesis.csv'));writetable(S,fullfile(root,'results','data','ref_sensitivity.csv'));
save(fullfile(root,'results','data','ref_hypothesis.mat'),'T','S','Rth','Rpath','slope','observedSlope','equivalentPath','buffered');
f=figure('Visible','off');plot(duty,observed*1000,'o',duty,prediction*1000,'-');xlabel('Equal duty');ylabel('Offset mA');legend('Historical approximate readings','Unfitted DC REF hypothesis','Location','northwest');grid on;exportgraphics(f,fullfile(root,'results','figures','ref_hypothesis.png'));close(f);
fprintf('REF_HYPOTHESIS_PASS nominal=%g A/duty observed=%g inferred_path=%g ohm buffered_assumed=%g A/duty\n',slope,observedSlope,equivalentPath,buffered);
end
