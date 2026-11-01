function build_foc_model
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));
c=closed_loop_config;rng(c.seed);c.noise=c.sensor_noise*randn(ceil(c.duration/c.p.Ts)+2,2);
name='pmsm_foc_model';if bdIsLoaded(name),close_system(name,0);end;new_system(name);
mw=get_param(name,'ModelWorkspace');assignin(mw,'cfg',c);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/SPMSM_dq_and_mechanics'],'FunctionName','pmsm_plant_sfun','Parameters','cfg','Position',[500 90 740 160]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Two_phase_sensor_and_ADC'],'FunctionName','sensor_adc_sfun','Parameters','cfg','Position',[490 270 740 340]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[name '/Firmware_PI_SVPWM_and_guard'],'FunctionName','firmware_controller_sfun','Parameters','cfg','Position',[75 150 345 225]);
add_block('simulink/Sinks/To Workspace',[name '/Plant_signals'],'VariableName','plant_signals','SaveFormat','Timeseries','Position',[820 95 950 125]);
add_block('simulink/Sinks/To Workspace',[name '/Controller_signals'],'VariableName','controller_signals','SaveFormat','Timeseries','Position',[80 320 260 350]);
add_line(name,'Firmware_PI_SVPWM_and_guard/1','SPMSM_dq_and_mechanics/1','autorouting','on');
add_line(name,'Firmware_PI_SVPWM_and_guard/1','Two_phase_sensor_and_ADC/2','autorouting','on');
add_line(name,'SPMSM_dq_and_mechanics/1','Two_phase_sensor_and_ADC/1','autorouting','on');
add_line(name,'Two_phase_sensor_and_ADC/1','Firmware_PI_SVPWM_and_guard/1','autorouting','on');
add_line(name,'SPMSM_dq_and_mechanics/1','Plant_signals/1');add_line(name,'Firmware_PI_SVPWM_and_guard/2','Controller_signals/1','autorouting','on');
set_param(name,'Solver','ode4','FixedStep','5e-6','StopTime','.25001');
set_param(name,'InitFcn',"model_dir=fileparts(get_param(bdroot,'FileName')); addpath(fullfile(model_dir,'..','matlab')); addpath(fullfile(model_dir,'..','..','..','build','research_host'));");
Simulink.Annotation(name,'Research model: actual production C PI/SVPWM; original guard by default. Conditional interval guard is simulation-only.');
save_system(name,fullfile(root,'models',[name '.slx']));close_system(name,0);
fprintf('FOC_MODEL_BUILT\n');
end
