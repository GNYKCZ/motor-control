function run_eso
% Discrete extended-state observer: sensor bias and physical voltage
% disturbance are explicitly DIFFERENT state equations.
root=fileparts(fileparts(mfilename('fullpath')));p=parameters;Ts=p.Ts;t=(0:Ts:1.5)';n=numel(t);R=p.Rs;L=mean([p.Ld p.Lq]);a=exp(-R*Ts/L);g=(1-a)/R;
% Measurement bias model [i;b], x+ = diag(a,1)x+[g;0]v, y=[1 1]x.
Ab=diag([a 1]);Bb=[g;0];Cb=[1 1];
% Physical disturbance model [i;dv], x+=[a g;0 1]x+[g;0]v,y=[1 0]x.
Ad=[a g;0 1];Bd=[g;0];Cd=[1 0];
% Simultaneous unknown physical disturbance and bias cannot be separated
% in steady parameters: unobservable vector [c;R*c;-c].
Aa=[a g 0;0 1 0;0 0 1];Ca=[1 0 1];assert(rank(obsv(Aa,Ca))==2);
nullvec=[1;R;-1];assert(norm((Aa-eye(3))*nullvec)<1e-12 && norm(Ca*nullvec)<1e-12);
rows={};trace=struct;
for scenario=["bias_slow" "bias_fast" "physical_voltage" "both" "R_mismatch"]
 for sigma=[0 .0008 .003]
 for bandwidth=[5 20 80 200]
  v=.2*sin(2*pi*3*t);bias=zeros(n,1);dist=zeros(n,1);Rp=R;
  if ismember(scenario,["bias_slow","both","R_mismatch"]),bias=.06+.01*sin(2*pi*2*t);end
  if scenario=="bias_fast",bias=.06+.025*sin(2*pi*80*t);end
  if ismember(scenario,["physical_voltage","both"]),dist=.1*(t>.4);end
  if scenario=="R_mismatch",Rp=1.3*R;end
  ap=exp(-Rp*Ts/L);gp=(1-ap)/Rp;truei=zeros(n,1);for j=2:n,truei(j)=ap*truei(j-1)+gp*(v(j-1)+dist(j-1));end
  rng(2811);y=truei+bias+sigma*randn(n,1);
  for mode=["bias_ESO" "disturbance_ESO"]
   if mode=="bias_ESO",A=Ab;B=Bb;C=Cb;else,A=Ad;B=Bd;C=Cd;end
   poles=exp(-2*pi*bandwidth*Ts*[1 1.2]);K=place(A',C',poles)';
   % Predictor update; xhat(k) estimates x(k), no oracle injection.
   x=zeros(2,1);hist=zeros(n,2);for j=1:n,hist(j,:)=x';x=A*x+B*v(j)+K*(y(j)-C*x);end
   keep=t>.2;err=hist(:,1)-truei;
   if mode=="bias_ESO",bhat=hist(:,2);target=bias;else,bhat=hist(:,2);target=dist;end
   rows(end+1,:)={scenario,sigma,bandwidth,mode,sqrt(mean(err(keep).^2)),max(abs(err(keep))),sqrt(mean((bhat(keep)-target(keep)).^2)),max(abs(eig(A-K*C)))};
   if sigma==.0008 && bandwidth==20,trace.(char(scenario+"_"+mode))=struct('t',t,'true',truei,'estimate',hist,'bias',bias,'physical_voltage',dist,'y',y);end
  end
 end
 end
end
T=cell2table(rows,'VariableNames',{'scenario','noise_A','bandwidth_Hz','observer','current_RMSE_A','current_peak_A','extended_state_RMSE_A_or_V','spectral_radius'});
assert(all(T.spectral_radius<1));writetable(T,fullfile(root,'results','data','eso_comparison.csv'));save(fullfile(root,'results','data','eso.mat'),'T','trace','Aa','Ca','nullvec');
f=figure('Visible','off');tiledlayout(2,1);nexttile;z=trace.bias_slow_bias_ESO;plot(t,[z.bias z.estimate(:,2)]*1000);legend('Actual sensor bias','Bias ESO');ylabel('mA');grid on;nexttile;z=trace.physical_voltage_bias_ESO;plot(t,[z.true z.estimate(:,1)]*1000);legend('True current','Bias ESO current');ylabel('mA');xlabel('s');title('Physical voltage disturbance mistaken for sensor bias');grid on;exportgraphics(f,fullfile(root,'results','figures','eso_confusion.png'));close(f);
fprintf('ESO: %d comparisons; combined disturbance/bias rank 2 of 3\n',height(T));
end
