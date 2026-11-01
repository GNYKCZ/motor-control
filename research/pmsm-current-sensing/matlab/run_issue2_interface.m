function run_issue2_interface
root=fileparts(fileparts(mfilename('fullpath')));c=issue2_drive_config;name='issue2_drive_demo';
if bdIsLoaded(name),close_system(name,0);end;new_system(name);cleanup=onCleanup(@()close_system(name,0));
mw=get_param(name,'ModelWorkspace');assignin(mw,'drive_cfg',c);
add_block('simulink/Sources/Step',[name '/iq_reference_A'],'Time','.02','Before','0','After','.02','Position',[30 30 70 60]);
add_block('simulink/Sources/Constant',[name '/motor_speed_rad_s'],'Value','0','Position',[30 100 70 130]);
add_block('simulink/Sources/Constant',[name '/residual_A'],'Value','.002','Position',[30 170 70 200]);
add_block('simulink/Signal Routing/Mux',[name '/Input'],'Inputs','3','Position',[150 35 155 200]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Electrical_drive'],'FunctionName','issue2_drive_sfun','Parameters','drive_cfg','Position',[250 90 470 150]);
add_block('simulink/Sinks/To Workspace',[name '/Results'],'VariableName','drive_signals','SaveFormat','Timeseries','Position',[550 100 650 130]);
add_line(name,'iq_reference_A/1','Input/1');add_line(name,'motor_speed_rad_s/1','Input/2');add_line(name,'residual_A/1','Input/3');add_line(name,'Input/1','Electrical_drive/1');add_line(name,'Electrical_drive/1','Results/1');
set_param(name,'Solver','FixedStepDiscrete','FixedStep','.001','StopTime','1');
set_param(name,'InitFcn',"addpath(fullfile(fileparts(get_param(bdroot,'FileName')),'..','matlab'));");
save_system(name,fullfile(root,'models',[name '.slx']));so=sim(name);t=so.drive_signals.Time;y=so.drive_signals.Data;
s=zeros(3,1);reference=zeros(size(y));for k=1:numel(t),[s,reference(k,:)]=issue2_drive_step(s,[.02*double(t(k)>=.02);0;.002],c);end
cross=max(abs(y-reference),[],'all');assert(cross<1e-12);assert(abs(y(end,1)-.018)<c.lsb);assert(abs(y(end,3)-c.Kt*y(end,1))<1e-12);
% 去掉量化/残差后独立线性状态空间解析校验，包括775us分段保持。
c0=c;c0.lsb=0;a=exp(-c.R*c.Ts/c.L);b=exp(-c.R*(c.Ts-c.delay)/c.L);g1=(b-a)/c.R;g2=(1-b)/c.R;
A=[a-g2*c.kp,g2,g1;-c.ki*c.Ts,1,0;-c.kp,1,0];B=[g2*c.kp;c.ki*c.Ts;c.kp];s=zeros(3,1);linear=s;err=0;
for k=1:1000,[s,~]=issue2_drive_step(s,[.02;0;0],c0);linear=A*linear+B*.02;err=max(err,max(abs(s-linear)));end
assert(err<1e-12 && abs(s(1)-.02)<1e-10);
save(fullfile(root,'results','data','issue2_interface.mat'),'c','t','y','cross','err');
writetable(table(c.Kt,c.Ts,c.delay,c.lsb,c.reference_limit,c.voltage_limit,cross,err,'VariableNames',{'Kt_Nm_per_A','Ts_s','delay_s','scalar_q_quantum_A','reference_limit_A','voltage_limit_V','simulink_error','linear_error'}),fullfile(root,'results','data','issue2_interface.csv'));
f=figure('Visible','off');plot(t,y(:,1:2)*1000);legend('True iq','Feedback iq');xlabel('s');ylabel('mA');grid on;exportgraphics(f,fullfile(root,'results','figures','issue2_interface.png'));close(f);
fprintf('ISSUE2_INTERFACE_PASS cross=%g linear=%g Kt=%g\n',cross,err,c.Kt);
end
