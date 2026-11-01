function d=make_design(n,kind,seed)
% 合成信号链设计：需要独立参考才能对应实物标定。
rng(seed);d.t=(0:n-1)'*.001;kind=string(kind);
if kind=="equal",a=.3+.4*rand(n,1);d.duty=repmat(a,1,3);
elseif kind=="constant",d.duty=.5*ones(n,3);
elseif kind=="train",levels=[.3 .5 .7];d.duty=levels(randi(3,n,3));
elseif kind=="holdout",levels=[.4 .6];d.duty=levels(randi(2,n,3));
else,d.duty=.3+.4*rand(n,3);end
if kind=="train",levels=[10 12 14];d.bus=levels(randi(3,n,1))';d.bus=d.bus(:);
elseif kind=="holdout",d.bus=11+2*rand(n,1);else,d.bus=12*ones(n,1);end
phase=rand(n,1);d.state=zeros(n,2);d.age=zeros(n,2);d.edge_sign=zeros(n,2);
for ch=1:2
 rise=(1-d.duty(:,ch))/2;fall=(1+d.duty(:,ch))/2;
 d.state(:,ch)=phase>=rise & phase<fall;
 ar=mod(phase-rise,1);af=mod(phase-fall,1);d.age(:,ch)=min(ar,af)/20000;
 d.edge_sign(:,ch)=2*(ar<af)-1;
end
end
