function [next,y]=issue2_drive_step(state,input,c)
% state=[真实iq A;积分V;上一拍施加电压V]，input=[iq_ref A;omega_m rad/s;反馈残差A]。
% 真实iq只给电气植物/转矩，控制使用独立的带残差/量化反馈。
assert(all(isfinite([state(:);input(:)])) && c.delay>=0 && c.delay<c.Ts);
feedback=state(1)+input(3);if c.lsb>0,feedback=round(feedback/c.lsb)*c.lsb;end
reference=max(-c.reference_limit,min(c.reference_limit,input(1)));error=reference-feedback;
requested=c.kp*error+state(2);voltage=max(-c.voltage_limit,min(c.voltage_limit,requested));
a=exp(-c.R*c.Ts/c.L);b=exp(-c.R*(c.Ts-c.delay)/c.L);g1=(b-a)/c.R;g2=(1-b)/c.R;
next=[a*state(1)+g1*state(3)+g2*voltage-(1-a)/c.R*c.poles*c.psi*input(2);...
 state(2)+c.ki*c.Ts*error+c.tracking*c.Ts*(voltage-requested);voltage];
y=[state(1);feedback;c.Kt*state(1);c.Kt*feedback;double(voltage~=requested)];
end
