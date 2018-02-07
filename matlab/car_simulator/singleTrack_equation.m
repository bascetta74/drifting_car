function [dstate] = singleTrack_equation(state,v,delta,param)

v = max(0.1,v);

% States
yaw      = state(1);
yaw_rate = state(2);
beta     = state(3);

% Parameters
m   = param(1);
Izz = param(2);
lf  = param(3);
lr  = param(4);
Cf  = param(5);
Cr  = param(6);

% Slips
alpha_f = beta+lf*yaw_rate/v-delta;
alpha_r = beta-lr*yaw_rate/v;

% State derivatives
dstate(1) = yaw_rate;
dstate(2) = (lr*Cr*alpha_r-lf*Cf*alpha_f)/Izz;
dstate(3) = -(Cf*alpha_f+Cr*alpha_r)/(v*m)-yaw_rate;
dstate(4) = v*cos(beta+yaw);
dstate(5) = v*sin(beta+yaw);