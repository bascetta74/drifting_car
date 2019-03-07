clear all
close all

addpath ./fblin_law ./vehicle_model

% Simulator parameters
meas_Ts = 0.001;

ROS_statePublishingPeriod = 0.01;
ROS_clockPublishingPeriod = 0.0005;

% Vehicle initial state
x0    = 0;
y0    = 0;
psi0  = 0;
beta0 = 0;
r0    = 0;

car_state0 = [psi0, r0, beta0, x0, y0];

% Vehicle parameters
m  = 2.040;
Iz = 0.030;
a  = 0.1513;
b  = 0.1087;
Cf = 47.3;
Cr = 369;

car_param = [a, b, Cf, Cr, Iz, m];

actuation_delay = 0.09;
xy_noiseVar   = 1e-8;
psi_noiseVar  = 9e-7;
r_noiseVar    = 1e-10;

% Feedback linearization parameters
P = 0.3;
fblin_Ts = 0.01;
fblin_car_param = car_param;

% Beta estimator parameters
vel_fir = [0.0144, 0.0295, 0.0763, 0.0823, 0.0355, -0.0355, -0.0823, -0.0763, -0.0295, -0.0144]; % 5 Hz lowpass
psi_fir = [0.0144, 0.0439, 0.1202, 0.2025, 0.2380, 0.2025, 0.1202, 0.0439, 0.0144]; % 5 Hz lowpass
beta_velocity_threshold = 0.05;

