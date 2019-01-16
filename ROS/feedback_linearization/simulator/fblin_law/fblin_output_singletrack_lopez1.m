function out = fblin_output_singletrack_lopez2(x,y,psi,beta,v,P,car_param)

% Vehicle parameters
a  = car_param(1);
b  = car_param(2);
Cf = car_param(3);
Cr = car_param(4);
Iz = car_param(5);
m  = car_param(6);

% Coordinate transformation
xp = x+v*P*cos(psi+beta);
yp = y+v*P*sin(psi+beta);

out = [xp; yp];
