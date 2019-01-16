function dydt = singleTrack_model(in,state,param)

% States
psi  = state(1);
r    = state(2);
beta = state(3);
x    = state(4);
y    = state(5);

% Parameters
a  = param(1);
b  = param(2);
Cf = param(3);
Cr = param(4);
Iz = param(5);
m  = param(6);

% Inputs
v     = in(1);
delta = in(2);

dydt = zeros(5,1);
if (v>0.001)
    dydt(1) = r;
    dydt(2) =   (b*Cr-a*Cf)/Iz*beta -(a^2*Cf+b^2*Cr)/(v*Iz)*r  +a*Cf/Iz*delta;
    dydt(3) =  -(Cf+Cr)/(m*v)*beta+((Cr*b-Cf*a)/(m*v^2)-1)*r+Cf/(m*v)*delta;
    dydt(4) = v*cos(beta+psi);
    dydt(5) = v*sin(beta+psi);
else
    dydt(1) = 0;
    dydt(2) =  -1000*r;
    dydt(3) =  -1000*beta;
    dydt(4) = 0;
    dydt(5) = 0;
end
