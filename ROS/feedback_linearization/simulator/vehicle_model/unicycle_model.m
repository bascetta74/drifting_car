function [dstate] = unicycle_model(in,state)

% States
x     = state(1);
y     = state(2);
theta = state(3);

% Inputs
v     = in(1);
omega = in(2);

% Equations
dx     = v*cos(theta);
dy     = v*sin(theta);
dtheta = omega;

dstate = [dx; dy; dtheta];
