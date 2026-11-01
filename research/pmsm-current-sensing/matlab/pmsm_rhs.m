function dx=pmsm_rhs(x,duty,enable,c)
% x=[id iq theta_e omega_m]. Positive electrical/mechanical power consistent.
p=c.p;id=x(1);iq=x(2);theta=x(3);omega=x(4);we=p.poles*omega;
phase=c.bus*(duty(:)-mean(duty));if ~enable,phase(:)=0;end
ab=[phase(1);(phase(1)+2*phase(2))/sqrt(3)];Q=[cos(theta) sin(theta);-sin(theta) cos(theta)];v=Q*ab;
Ld=p.Ld*c.Lfactor;Lq=p.Lq*c.Lfactor;R=p.Rs*c.Rfactor;psi=p.psi*c.psifactor;
did=(v(1)-R*id+we*Lq*iq)/Ld;diq=(v(2)-R*iq-we*(Ld*id+psi))/Lq;
torque=1.5*p.poles*(psi*iq+(Ld-Lq)*id*iq);
if c.motion=="locked",dtheta=0;dw=0;elseif c.motion=="prescribed",dtheta=p.poles*c.omega;dw=0;else,dtheta=we;dw=(torque-c.B*omega-c.load)/c.J;end
dx=[did;diq;dtheta;dw];
end
