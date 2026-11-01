function run_foc_simulink
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));
build_foc_model;name='pmsm_foc_model';load_system(fullfile(root,'models',[name '.slx']));
cleanup=onCleanup(@()close_system(name,0));rows={};traces=struct;
cases={"original_bias", "original", "M1", "offline_M1",false,0,.04;...
"certificate_bias","interval","M1","offline_M1",true,0,.04;...
"unknown_certificate","interval","M1","offline_M1",false,0,.04;...
"no_bias","original","none","zero_only",false,0,.04;...
"zero_reference","interval","M1","offline_M1",true,0,0;...
"negative_reference","interval","M1","offline_M1",true,0,-.04;...
"rotating","interval","M1","offline_M1",true,1,.04};
for k=1:size(cases,1)
 c=closed_loop_config;c.guard=cases{k,2};c.bias_model=cases{k,3};c.method=cases{k,4};c.certified=cases{k,5};c.omega=cases{k,6};c.reference=cases{k,7};if c.omega~=0,c.motion="prescribed";end
 rng(c.seed);c.noise=c.sensor_noise*randn(ceil(c.duration/c.p.Ts)+2,2);
 mw=get_param(name,'ModelWorkspace');assignin(mw,'cfg',c);set_param(name,'StopTime',num2str(c.duration+2*c.dt,17));so=sim(name);
 t=so.plant_signals.Time;x=so.plant_signals.Data;s=so.controller_signals.Data;live=s(:,6)>0 & s(:,20)>1;trip=max(s(:,7));
 if any(live),err=s(live,8:9)-x(live,4:5);rmse=sqrt(mean(sum(err.^2,2)));else,rmse=NaN;end
 rows(end+1,:)={cases{k,1},trip,rmse,max(vecnorm(x(:,4:5),2,2)),max(abs(s(:,1:2)),[],'all'),mean(s(:,19)>0)};
 traces.(char(cases{k,1}))=struct('t',t,'plant',x,'controller',s,'config',c);
 fprintf('FOC case %s trip=%g RMSE=%g\n',cases{k,1},trip,rmse);
end
T=cell2table(rows,'VariableNames',{'scenario','trip_code','dq_error_RMSE_A','peak_true_A','peak_integral_V','saturation_fraction'});
assert(T.trip_code(1)==3);assert(T.trip_code(3)==4);assert(T.trip_code(2)==8);assert(T.trip_code(5)==8);
% Electromechanical power identity evaluated independently at random states.
p=parameters;rng(44);energyErr=0;for k=1:100
 c=closed_loop_config;c.motion="dynamic";c.load=.001;c.omega=0;x=[.03*randn(2,1);2*pi*rand;randn];d=.5+.02*randn(3,1);dx=pmsm_rhs(x,d,true,c);
 phase=p.Vbus*(d-mean(d));ab=[phase(1);(phase(1)+2*phase(2))/sqrt(3)];v=[cos(x(3)) sin(x(3));-sin(x(3)) cos(x(3))]*ab;
 stored=1.5*(p.Ld*x(1)*dx(1)+p.Lq*x(2)*dx(2))+c.J*x(4)*dx(4);
 balance=1.5*(v'*x(1:2)-p.Rs*sum(x(1:2).^2))-c.B*x(4)^2-c.load*x(4);
 energyErr=max(energyErr,abs(stored-balance));
end
assert(energyErr<1e-12);
writetable(T,fullfile(root,'results','data','foc_simulink.csv'));save(fullfile(root,'results','data','foc_simulink.mat'),'T','traces','energyErr');
f=figure('Visible','off');tiledlayout(2,1);z=traces.certificate_bias;nexttile;plot(z.t,[z.plant(:,5),z.controller(:,9),z.controller(:,18)]*1000);legend('True iq','Feedback iq','Reference');ylabel('mA');grid on;nexttile;plot(z.t,z.controller(:,1:2));ylabel('PI integral V');xlabel('s');grid on;exportgraphics(f,fullfile(root,'results','figures','foc_simulink.png'));close(f);
fprintf('FOC_SIMULINK_PASS energy_error=%g\n',energyErr);
end
