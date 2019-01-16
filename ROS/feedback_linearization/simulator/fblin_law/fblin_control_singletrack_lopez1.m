function out = fblin_control_singletrack_lopez1(vxp,vyp,psi,r,beta,v,P,car_param)

% Vehicle parameters
a  = car_param(1);
b  = car_param(2);
Cf = car_param(3);
Cr = car_param(4);
Iz = car_param(5);
m  = car_param(6);

% Feedback linearization
dv     = (vxp*cos(psi+beta)+vyp*sin(psi+beta)-v)/P;
if (abs(v)<0.001)
    delta = 0;
else
    delta = m/Cf*(vyp*cos(psi+beta)-vxp*sin(psi+beta))/P+(Cr/Cf+1)*beta-(Cr/Cf*b-a)*r/v;
end

out = [dv; delta];
