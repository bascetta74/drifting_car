function out = fblin_outputInverse_singletrack_lopez1(xp,yp,psi,beta,v,P,car_param)

% Vehicle parameters
a  = car_param(1);
b  = car_param(2);
Cf = car_param(3);
Cr = car_param(4);
Iz = car_param(5);
m  = car_param(6);

% Coordinate transformation
x = xp -v*P*cos(psi+beta);
y = yp -v*P*sin(psi+beta);

out = [x; y];
