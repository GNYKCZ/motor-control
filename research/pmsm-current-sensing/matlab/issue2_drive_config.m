function c=issue2_drive_config
% 名义低速q轴电气内环，供Issue2使用；不包含关节机械惯量或传动比。
p=parameters;c.R=p.Rs;c.L=p.Lq;c.psi=p.psi;c.poles=p.poles;c.Kt=1.5*p.poles*p.psi;
c.Ts=p.Ts;c.delay=.000775;c.kp=p.kp;c.ki=p.ki;c.tracking=p.tracking;
c.reference_limit=p.ref_limit;c.voltage_limit=.4;c.lsb=p.lsb;
end
