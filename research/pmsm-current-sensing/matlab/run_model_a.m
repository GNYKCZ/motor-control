function run_model_a(root,p)
out=fullfile(root,'results','data');checks={};
uv=[.0615 .0615];ab=clarke_park(uv,0);
assert(abs(norm(ab)-.123)<1e-12);assert(abs(atan2d(ab(2),ab(1))-60)<1e-10);checks(end+1,:)={'analytic_123mA_60deg',true};
t=(0:.001:.2)';i=[.02*sin(2*pi*8*t),.015*cos(2*pi*5*t)];b=repmat(p.b0,numel(t),1);noise=zeros(size(i));delay=.002;
s=model_sensor_bias(t,i,b,p,noise,delay);
assert(max(abs(s.measured-s.truth_at_sample-b),[],'all')<=p.lsb/2+1e-14);checks(end+1,:)={'ADC_half_LSB',true};
assert(max(abs(s.truth_at_sample(3:end,:)-i(1:end-2,:)),[],'all')<1e-12);checks(end+1,:)={'two_sample_delay',true};
high=model_sensor_bias(t,4*ones(size(i)),b,p,noise,0);low=model_sensor_bias(t,-4*ones(size(i)),b,p,noise,0);
assert(all(high.raw==4095,'all') && all(low.raw==0,'all'));checks(end+1,:)={'ADC_clipping',true};
rng(p.seed);raw0=model_sensor_bias((0:511)'*p.Ts,zeros(512,2),repmat(p.b0,512,1),p,.0008*randn(512,2),0);
b0hat=mean(raw0.measured);assert(max(abs(b0hat-p.b0))<.0003);checks(end+1,:)={'512_frame_zero_offset',true};
% Check invertibility/power of orthonormal Park, independent of theta.
th=linspace(-pi,pi,numel(t))';dq=clarke_park(i,th);aball=clarke_park(i,zeros(size(th)));assert(max(abs(sum(dq.^2,2)-sum(aball.^2,2)))<1e-15);checks(end+1,:)={'Park_norm_invariant',true};
% Real block-based Simulink signal chain, with stored workspace inputs.
name='sensor_bias_model';if bdIsLoaded(name),close_system(name,0);end;new_system(name);
mw=get_param(name,'ModelWorkspace');assignin(mw,'sensor_input',timeseries(i,t));assignin(mw,'bias_input',timeseries(b,t));
add_block('simulink/Sources/From Workspace',[name '/True_UV'],'VariableName','sensor_input','Interpolate','on','Position',[25 40 140 70]);
add_block('simulink/Sources/From Workspace',[name '/Input_referred_bias'],'VariableName','bias_input','Interpolate','on','Position',[25 110 140 140]);
add_block('simulink/Math Operations/Sum',[name '/Current_plus_bias'],'Inputs','++','Position',[175 45 195 95]);
add_block('simulink/Math Operations/Gain',[name '/Shunt_20mOhm'],'Gain',num2str(p.shunt),'Position',[230 45 310 85]);
add_block('simulink/Math Operations/Gain',[name '/INA199_gain50'],'Gain',num2str(p.gain),'Position',[345 45 425 85]);
add_block('simulink/Sources/Constant',[name '/REF_assumed'],'Value',num2str(p.ref),'Position',[340 125 405 155]);
add_block('simulink/Math Operations/Sum',[name '/Analog_output'],'Inputs','++','Position',[465 50 485 100]);
initial=p.ref+(i(1,:)+p.b0)*p.shunt*p.gain;
add_block('simulink/Discrete/Delay',[name '/Sampling_delay'],'DelayLength','2','InitialCondition',mat2str(repmat(initial',1,2),17),'SampleTime','.001','Position',[520 45 610 85]);
add_block('simulink/Math Operations/Gain',[name '/ADC_counts'],'Gain',num2str(4096/p.Vref,17),'Position',[650 45 725 85]);
add_block('simulink/Discontinuities/Quantizer',[name '/ADC_quantizer'],'QuantizationInterval','1','Position',[765 45 835 85]);
add_block('simulink/Discontinuities/Saturation',[name '/ADC_range'],'UpperLimit','4095','LowerLimit','0','Position',[875 45 950 85]);
add_block('simulink/Math Operations/Bias',[name '/Subtract_REF_count'],'Bias',num2str(-p.ref*4096/p.Vref,17),'Position',[880 175 975 215]);
add_block('simulink/Math Operations/Gain',[name '/Counts_to_A'],'Gain',num2str(p.lsb,17),'Position',[755 175 835 215]);
T=[1 0;1/sqrt(3) 2/sqrt(3)];theta=.37;P=[cos(theta) sin(theta);-sin(theta) cos(theta)];
add_block('simulink/Math Operations/Gain',[name '/Clarke_Park'],'Gain',mat2str(P*T,17),'Multiplication','Matrix(K*u)','Position',[590 175 710 215]);
add_block('simulink/Sinks/To Workspace',[name '/Measured_UV'],'VariableName','measured_uv','SaveFormat','Timeseries','Position',[755 265 870 295]);
add_block('simulink/Sinks/To Workspace',[name '/Measured_dq'],'VariableName','measured_dq','SaveFormat','Timeseries','Position',[420 175 540 215]);
links={'True_UV/1','Current_plus_bias/1';'Input_referred_bias/1','Current_plus_bias/2';'Current_plus_bias/1','Shunt_20mOhm/1';'Shunt_20mOhm/1','INA199_gain50/1';'INA199_gain50/1','Analog_output/1';'REF_assumed/1','Analog_output/2';'Analog_output/1','Sampling_delay/1';'Sampling_delay/1','ADC_counts/1';'ADC_counts/1','ADC_quantizer/1';'ADC_quantizer/1','ADC_range/1';'ADC_range/1','Subtract_REF_count/1';'Subtract_REF_count/1','Counts_to_A/1';'Counts_to_A/1','Clarke_Park/1';'Counts_to_A/1','Measured_UV/1';'Clarke_Park/1','Measured_dq/1'};
for j=1:size(links,1),add_line(name,links{j,1},links{j,2},'autorouting','on');end
set_param(name,'StopTime','.2','Solver','FixedStepDiscrete','FixedStep','.001');save_system(name,fullfile(root,'models',[name '.slx']));so=sim(name);
measured=reshape(so.measured_uv.Data,[],2);dqs=reshape(so.measured_dq.Data,[],2);
err=max(abs(measured-s.measured),[],'all');assert(err<1e-12);assert(max(abs(dqs-clarke_park(s.measured,theta)),[],'all')<1e-12);
checks(end+1,:)={'Simulink_vs_independent_matlab',true};close_system(name,0);
writetable(cell2table(checks,'VariableNames',{'check','passed'}),fullfile(out,'model_a_checks.csv'));
save(fullfile(out,'model_a.mat'),'p','s','b0hat','err');
f=figure('Visible','off');tiledlayout(2,1);nexttile;plot(t,[i(:,1) s.measured(:,1)]*1000);legend('True now','ADC measurement');ylabel('U current (mA)');grid on;nexttile;plot(t,(s.measured-s.truth_at_sample-b)*1000);ylabel('Quantization residual (mA)');xlabel('Time (s)');grid on;exportgraphics(f,fullfile(root,'results','figures','model_a.png'));close(f);
fprintf('Model A: %d independent checks passed; Simulink error %.3g A\n',size(checks,1),err);
end
