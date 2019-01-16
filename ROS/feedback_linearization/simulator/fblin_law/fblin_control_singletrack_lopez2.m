function out = fblin_control_singletrack_lopez2(vxp,vyp,psi,r,beta,delta,P,car_param)

% Vehicle parameters
a  = car_param(1);
b  = car_param(2);
Cf = car_param(3);
Cr = car_param(4);
Iz = car_param(5);
m  = car_param(6);

% Feedback linearization
v      = (vxp*cos(delta+psi)+vyp*sin(delta+psi)-r*a*sin(delta))/cos(beta-delta);
ddelta = (vyp*cos(psi+beta)-vxp*sin(psi+beta)-r*a*cos(beta))/(P*cos(beta-delta))-r;

out = [v; ddelta];
