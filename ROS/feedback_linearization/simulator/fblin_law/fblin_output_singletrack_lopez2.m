function out = fblin_output_singletrack_lopez2(x,y,psi,delta,P,car_param)

% Vehicle parameters
a  = car_param(1);
b  = car_param(2);
Cf = car_param(3);
Cr = car_param(4);
Iz = car_param(5);
m  = car_param(6);

% Coordinate transformation
xp = x+a*cos(psi)+P*cos(psi+delta);
yp = y+a*sin(psi)+P*sin(psi+delta);

out = [xp; yp];
