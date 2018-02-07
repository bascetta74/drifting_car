clear all
close all

% Vehicle parameters
m   = 348.5;    %kg, massa
Izz = 40.45;    %kgm2 momento d'inerzia
lf  = 0.7768;   %distanza asse anteriore baricentro
lr  = 0.4732;   %distanza asse posteriore baricentro
Cf  = 5e4;      %cornering stiffness equivalente anteriore
Cr  = 5e4;      %cornering stiffness equivalente posteriore
param = [m, Izz, lf, lr, Cf, Cr];

% Simulation parameters
measure_publishing_period = 0.01;
clock_publishing_period = 0.001;