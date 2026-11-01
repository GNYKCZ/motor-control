function z=simulate_foc(c)
% Independent time scheduler / classical RK4, used to cross-check Simulink.
N=round((c.duration+2*c.dt)/c.dt)+1;z.t=(0:N-1)'*c.dt;x=[0;0;0;c.omega];s=control_initial;
z.plant=zeros(N,7);z.controller=zeros(N,28);
for j=1:N
 z.plant(j,:)=pmsm_outputs(x,c)';z.controller(j,:)=s';
 raw=sample_adc(z.plant(j,1:2)',s(3:5),z.t(j),c);next=control_tick(s,[raw;z.plant(j,3)],c);
 h=c.dt;d=s(3:5);enable=s(6);
 k1=pmsm_rhs(x,d,enable,c);k2=pmsm_rhs(x+h*k1/2,d,enable,c);k3=pmsm_rhs(x+h*k2/2,d,enable,c);k4=pmsm_rhs(x+h*k3,d,enable,c);
 x=x+h/6*(k1+2*k2+2*k3+k4);s=next;
end
z.config=c;
end
