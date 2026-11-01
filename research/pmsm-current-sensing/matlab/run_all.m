function run_all(stage)
% Full reproducible research entry. stage: all/A/B/C. Unknown real parameters
% remain assumptions; no hardware or network actions are performed.
if nargin<1,stage="all";end
assert(isscalar(string(stage)) && ismember(string(stage),["all","A","B","C"]),'Unknown research stage');
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(root,'matlab'));
log=fullfile(root,'results','logs',['run_' char(stage) '_' char(datetime('now','Format','yyyyMMdd_HHmmss')) '.txt']);diary(log);cleanup=onCleanup(@()diary('off'));
fprintf('Environment: %s\n',version);p=parameters;save(fullfile(root,'results','data','parameters.mat'),'p');
toolboxes=ver;save(fullfile(root,'results','data','environment.mat'),'toolboxes');
cache=fullfile(repo,'build','research_simulink');if ~exist(cache,'dir'),mkdir(cache);end
Simulink.fileGenControl('set','CacheFolder',cache,'CodeGenFolder',cache,'createDir',true);
if ismember(string(stage),["all","A"])
 run_model_a(root,p);run_identification(root,p);run_adaptation_stress;run_o3_mismatch;run_eso;run_guard_validation;run_ref_hypothesis;
end
if ismember(string(stage),["all","B","C"])
 addpath(fullfile(repo,'build','research_host'));
 if exist('firmware_core_mex','file')~=3
  command=sprintf('python "%s"',fullfile(root,'c','build_bridge.py'));[status,output]=system(command);fprintf('%s',output);assert(status==0,'Host bridge compilation failed');rehash;
 end
end
if ismember(string(stage),["all","B"])
 run_model_b;run_linear_analysis;run_foc_simulink;run_foc_robustness;run_control_boundaries;run_o1_closed_loop;run_issue2_interface;compare_compensation;run_seed_validation;run_phase_features;
end
if ismember(string(stage),["all","C"]),run_model_c;run_switching_crosscheck;end
if string(stage)=="all"
 command=sprintf('python "%s"',fullfile(root,'c','test_bias_compensation.py'));[status,output]=system(command);fprintf('%s',output);assert(status==0,'Portable candidate C tests failed');
end
fprintf('REQUESTED_RESEARCH_STAGE_COMPLETE: %s\n',string(stage));
end
