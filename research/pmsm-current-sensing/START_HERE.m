% 在MATLAB中打开本文件并按“运行”：重跑7工况，显示真值/反馈/参考。
% 只运行软件仿真，不访问板卡。默认保存SLX保留原保护，原始W停止是预期。
research_root=fileparts(mfilename('fullpath'));repo_root=fileparts(fileparts(research_root));
addpath(fullfile(research_root,'matlab'));addpath(fullfile(repo_root,'build','research_host'));
if ~isfile(fullfile(research_root,'results','data','identification.mat')),run_all('A');end
if exist('firmware_core_mex','file')~=3
 [build_status,build_output]=system(sprintf('python "%s"',fullfile(research_root,'c','build_bridge.py')));
 disp(build_output);assert(build_status==0,'C bridge build failed; see README compiler prerequisites');rehash;
end
run_foc_simulink;
demo=load(fullfile(research_root,'results','data','foc_simulink.mat'));disp(demo.T);
figure('Name','Conditional compensation: synthetic model, not hardware accuracy');tiledlayout(2,1);
trace=demo.traces.certificate_bias;nexttile;plot(trace.t,[trace.plant(:,5),trace.controller(:,9),trace.controller(:,18)]*1000);
legend('True iq','Feedback iq','Reference');ylabel('mA');grid on;title('Matched model + assumed calibration validity');
nexttile;plot(trace.t,(trace.controller(:,9)-trace.plant(:,5))*1000);xlabel('s');ylabel('Feedback minus true iq (mA)');grid on;
open_system(fullfile(research_root,'models','pmsm_foc_model.slx'));
