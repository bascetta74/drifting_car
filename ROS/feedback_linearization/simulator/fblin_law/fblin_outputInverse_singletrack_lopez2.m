function out = fblin_outputInverse_singletrack_lopez2(xp,yp,psi,delta,P,car_param)

% Vehicle parameters
a  = car_param(1);
b  = car_param(2);
Cf = car_param(3);
Cr = car_param(4);
Iz = car_param(5);
m  = car_param(6);

% Coordinate transformation
x = xp -a*cos(psi)-P*cos(psi+delta);
y = yp -a*sin(psi)-P*sin(psi+delta);

out = [x; y];
