function out = fblin_control_singletrack_spaliviero(vxp,vyp,psi,r,beta,P,car_param)

% Vehicle parameters
a  = car_param(1);
b  = car_param(2);
Cf = car_param(3);
Cr = car_param(4);
Iz = car_param(5);
m  = car_param(6);

% Feedback linearization
w     = 1/P*(vyp*cos(beta+psi)-vxp*sin(beta+psi));
v     = vxp*cos(beta+psi)+vyp*sin(beta+psi);
if (abs(v)<0.001)
    delta = 0;
else
    delta = v*m/Cf*w+(Cr/Cf+1)*beta-(Cr/Cf*b-a)*r/v;
end

out = [v; delta];
