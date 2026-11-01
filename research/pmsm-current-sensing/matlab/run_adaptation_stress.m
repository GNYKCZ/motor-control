function run_adaptation_stress
root=fileparts(fileparts(mfilename('fullpath')));rng(2812);t=(0:.001:2)';n=numel(t);rows={};traces=struct;
for design=["constant" "excited"]
 if design=="constant",d=.5*ones(n,1);else,d=.3+.4*mod(floor(t/.025),2);end
 for drift=[0 .02]
  trueb0=.004;truek=.123+drift*t/2;i=.04*(t>.5);y=i+trueb0+truek.*d+.0008*randn(n,1);
  for method=["RLS_gated" "gradient_gated" "RLS_ungated" "scalar_RLS_known_b0"]
   theta=[trueb0;0];P=1000*eye(2);hist=zeros(n,2);bh=zeros(n,1);updated=false(n,1);predicted=zeros(n,1);
   for j=1:n
    xx=[1;d(j)];bh(j)=xx'*theta;predicted(j)=y(j)-bh(j);
    X=[ones(min(j,100),1),d(max(1,j-99):j)];pe=rank(X)==2 && min(eig(X'*X))>.01;
    gate=t(j)<=.5;
    if method=="RLS_ungated",gate=true;pe=true;end
    if method=="scalar_RLS_known_b0",pe=d(j)>.01;end
    if gate&&pe
     e=y(j)-xx'*theta;
     if method=="gradient_gated",theta=theta+.15*xx*e/(xx'*xx);
     elseif method=="scalar_RLS_known_b0",g=P(2,2)*d(j)/(.995+d(j)^2*P(2,2));theta(2)=theta(2)+g*e;P(2,2)=(P(2,2)-g*d(j)*P(2,2))/.995;
     else,g=P*xx/(.995+xx'*P*xx);theta=theta+g*e;P=(P-g*xx'*P)/.995;end
     updated(j)=true;
    end
    hist(j,:)=theta';
   end
   settled=abs(hist(:,2)-truek)<.005;convergence=NaN;
   for j=1:n-49,if all(settled(j:j+49)),convergence=t(j);break;end;end
   keep=t>1;rows(end+1,:)={design,drift,method,convergence,nnz(updated),hist(end,2),sqrt(mean((predicted(keep)-i(keep)).^2))};
   if drift==.02,traces.(char(design+"_"+method))=struct('t',t,'theta',hist,'truek',truek,'corrected',predicted,'truei',i,'updates',updated);end
   if method~="RLS_ungated",assert(~any(updated(t>.5)));end
  end
 end
end
T=cell2table(rows,'VariableNames',{'design','k_drift_A_per_duty','algorithm','convergence_50ms_within005_s','updates','final_k','current_RMSE_A'});
writetable(T,fullfile(root,'results','data','adaptation_stress.csv'));save(fullfile(root,'results','data','adaptation_stress.mat'),'T','traces');
assert(all(T.updates(T.design=="constant" & ismember(T.algorithm,["RLS_gated","gradient_gated"]))==0));
f=figure('Visible','off');hold on;for method=["RLS_gated" "RLS_ungated" "scalar_RLS_known_b0"],z=traces.(char("excited_"+method));plot(z.t,z.theta(:,2));end;plot(t,z.truek,'k--');legend('Gated RLS','Unsafe RLS','Known-b0 RLS','Actual k');ylabel('A/duty');xlabel('s');grid on;exportgraphics(f,fullfile(root,'results','figures','adaptation_drift.png'));close(f);
disp(T);fprintf('ADAPTATION_STRESS_PASS\n');
end
