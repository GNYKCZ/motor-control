function X=bias_features(d,channel,model)
% d字段均为估计器可获得的采样时刻量；真电流/生成参数不在此接口。
n=size(d.duty,1);u=d.duty(:,channel);meanDuty=mean(d.duty,2);
switch string(model)
case "M1",X=[ones(n,1),u];
case "M2",X=[ones(n,1),meanDuty];
case "M3"
 X=[ones(n,1),u.*d.bus/12,d.state(:,channel).*d.bus/12,...
 d.edge_sign(:,channel).*exp(-d.age(:,channel)/20e-6).*d.bus/12];
otherwise,error('Unknown model');
end
end
