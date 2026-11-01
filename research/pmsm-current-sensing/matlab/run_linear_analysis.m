function run_linear_analysis
root=fileparts(fileparts(mfilename('fullpath')));p=parameters;rows={};
for axis=["d" "q"]
 if axis=="d",L0=p.Ld;else,L0=p.Lq;end
 for rf=[.7 1 1.3]
 for lf=[.7 1 1.3]
 for latency=[.0005 .000775 .00095]
  R=p.Rs*rf;L=L0*lf;T=p.Ts;a=exp(-R*T/L);b=exp(-R*(T-latency)/L);g1=(b-a)/R;g2=(1-b)/R;
  % state [sampled current; integral voltage; old applied voltage]
  A=[a-g2*p.kp,g2,g1;-p.ki*T,1,0;-p.kp,1,0];B=[g2*p.kp;p.ki*T;p.kp];C=[1 0 0];
  eigmax=max(abs(eig(A)));f=logspace(-2,log10(499),1500);h=zeros(size(f));
  for j=1:numel(f),h(j)=C*((exp(1i*2*pi*f(j)*T)*eye(3)-A)\B);end
  ix=find(abs(h)<=1/sqrt(2),1);bw=NaN;if ~isempty(ix),bw=f(ix);end
  rows(end+1,:)={axis,rf,lf,latency,eigmax,bw,abs(C*((eye(3)-A)\B))};
 end
 end
 end
end
T=cell2table(rows,'VariableNames',{'axis','R_factor','L_factor','delay_s','spectral_radius','bandwidth_Hz','dc_gain'});
assert(all(T.spectral_radius<1));writetable(T,fullfile(root,'results','data','linear_current_loop.csv'));
nominal=T(T.R_factor==1&T.L_factor==1&T.delay_s==.000775,:);disp(nominal);
% Valid only local unsaturated linear electrical model, no protective trips.
f=figure('Visible','off');plot(T.delay_s*1e6,T.spectral_radius,'o');xlabel('Apply delay us');ylabel('Spectral radius');grid on;exportgraphics(f,fullfile(root,'results','figures','local_stability.png'));close(f);
fprintf('LOCAL_LINEAR_SCAN_PASS %d cases\n',height(T));
end
