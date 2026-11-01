function out=pmsm_outputs(x,c)
p=c.p;ab=[cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))]*x(1:2);
uv=[ab(1);(-ab(1)+sqrt(3)*ab(2))/2];torque=1.5*p.poles*(p.psi*c.psifactor*x(2)+(p.Ld-p.Lq)*c.Lfactor*x(1)*x(2));
out=[uv;mod(x(3),2*pi);x(1:2);x(4);torque];
end
