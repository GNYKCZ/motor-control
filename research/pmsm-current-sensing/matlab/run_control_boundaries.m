function run_control_boundaries
% 直接注入ADC/配置进行异常验收，不使用对象真值做控制决策。
root=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(fileparts(root));addpath(fullfile(repo,'build','research_host'));c=closed_loop_config;c.guard="interval";c.certified=true;c.method="zero_only";
p=c.p;raw=round((p.ref+p.b0')/p.Vref*4096);rows={};s=control_initial;
a=control_tick(s,[raw;0],c);assert(a(6)==1);rows(end+1,:)={"valid_zero",a(7)};
cases={"saturated_low",[0;raw(2);0],c,1;"saturated_high",[4095;raw(2);0],c,1};
cc=c;cc.certified=false;cases(end+1,:)={"unknown_calibration",[raw;0],cc,4};
cc=c;cc.bus=15;cases(end+1,:)={"bus_out_of_domain",[raw;0],cc,4};
cc=c;cc.error_bound=[NaN;.003];cases(end+1,:)={"invalid_bound",[raw;0],cc,4};
cases(end+1,:)={"nonfinite_ADC",[NaN;raw(2);0],c,4};
for j=1:size(cases,1)
 a=control_tick(s,cases{j,2},cases{j,3});assert(a(6)==0 && a(7)==cases{j,4});assert(all(a(1:2)==0));rows(end+1,:)={cases{j,1},a(7)};
end
% 长时间受限指令下检查实际C核心积分有界，而不是调小测试输入。
integral=zeros(2,1);peak=0;saturated=0;
for j=1:10000
 y=firmware_core_mex(integral,[0;-.1;0;0;.04;12;p.Ts]);integral=y(2:3);peak=max(peak,max(abs(integral)));saturated=saturated+y(11);
 assert(y(1)==1 && all(isfinite(y)));
end
assert(saturated>9000 && peak<1);
% 移除强制反馈，接入简单电阻对象、零参考；检查积分退出饱和。
iq=0;
for j=1:1000,y=firmware_core_mex(integral,[0;sqrt(3)/2*iq;0;0;0;12;p.Ts]);integral=y(2:3);iq=y(7)/p.Rs;end
assert(all(isfinite(integral)) && max(abs(integral))<.001 && y(11)==0);
T=cell2table(rows,'VariableNames',{'case_name','trip_code'});writetable(T,fullfile(root,'results','data','control_boundaries.csv'));
save(fullfile(root,'results','data','control_boundaries.mat'),'T','peak','saturated','integral');
fprintf('CONTROL_BOUNDARIES_PASS %d failure/valid cases; saturation %d; peak integral %g V\n',height(T),saturated,peak);
end
