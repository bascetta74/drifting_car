close all; 
clear; 
clc

% Inputs
Ts = 0.001; t = 0:Ts:25;
Fxr_ref   = 5.0*ones(size(t));
steer_ref = 0.1*ones(size(t));

% Car parameters
a  = 0.1368;
b  = 0.1232;
m  = 1.9;
mu = 0.385;
Cf = 50.13;
Cr = 122.05;
Iz = 0.029;
car_param = [a b m mu Cf Cr Iz];

% Actuator parameters
mu_steer  = 1;
wn_steer  = 87.62;
csi_steer = 0.75;
tau_steer = 0.055;
mu_force  = 1;
wn_force  = 650;
csi_force = 0.9;
tau_force = 1e-15;
actuator_param = [mu_steer wn_steer csi_steer tau_steer mu_force wn_force csi_force tau_force];

% Initial values of the state variables
v0    = 1;       % initial longitudinal velocity
beta0 = 0;       % initial sideslip
r0    = 0;       % initial yaw rate
psi0  = 0;       % initial orientation
x0    = 0;       % initial position x
y0    = 0;       % initial position y
initial_state = [v0 beta0 r0 psi0 x0 y0];

% Generate the inputs
Fxr_ref   = timeseries(Fxr_ref,t);    % input Fxr timeseries
steer_ref = timeseries(steer_ref,t);  % input delta timeseries

% Set the flag that will choose the model describing the behavior of lateral forces
tyre_model = 0; % 0='Linear Model'  1='Fiala model'  2='Fiala without saturation'

% Set the flag that will choose the model describing the behavior of actuators
actuator_model = 0; % 0='Ideal actuators'  1='Second order actuator model'

% Run the system simulation
if (actuator_model==0)
    load_system('carsim_beta_force');
    set_param('carsim_beta_force/Actuator models','Commented','through');
else
    load_system('carsim_beta_force');
    set_param('carsim_beta_force/Actuator models','Commented','off');
end

sim_res = sim('carsim_beta_force.slx','StartTime','steer_ref.Time(1)','StopTime',...
    'steer_ref.Time(end)','SrcWorkspace' ,'current');

% Plot pose
figure,
subplot(3,1,1),plot(sim_res.time,sim_res.x),ylabel('x [m]'),xlabel('Time (s)'),grid
subplot(3,1,2),plot(sim_res.time,sim_res.y),ylabel('y [m]'),xlabel('Time (s)'),grid
subplot(3,1,3),plot(sim_res.time,sim_res.psi),ylabel('\psi [m]'),xlabel('Time (s)'),grid

% Plot the input signals
figure,
subplot(2,1,1),plot(sim_res.time,sim_res.Fxr_ref),ylabel('Fxr ref [N]'),xlabel('Time (s)'),grid
subplot(2,1,2),plot(sim_res.time,sim_res.steer_ref),ylabel('Delta [rad]'),xlabel('Time (s)'),grid

% Plot longitudinal dynamics measured outputs
figure,plot(sim_res.time,sim_res.V.*cos(sim_res.beta)),ylabel('Vx [m/s]'),xlabel('Time (s)'),grid

% Plot lateral dynamics measured outputs
figure,plot(sim_res.time,sim_res.V.*sin(sim_res.beta)),ylabel('Vy [m/s]'),xlabel('Time (s)'),grid
figure,plot(sim_res.time,sim_res.r),ylabel('r [rad/s]'),xlabel('Time (s)'),grid
figure,plot(sim_res.time,sim_res.ay),ylabel('ay [m/s^2]'),xlabel('Time (s)'),grid
figure,plot(sim_res.time,sim_res.beta),ylabel('sideslip [rad]'),xlabel('Time (s)'),grid

return

% Plot tyre measured outputs
figure,plot(sim_res.time,sim_res.alphaf),ylabel('Front slip [rad]'),xlabel('Time (s)'),grid
figure,plot(sim_res.time,sim_res.alphar),ylabel('Rear slip [rad]'),xlabel('Time (s)'),grid

figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.Fyf, ode_res.t,ode_res.Fyf,'--'),ylabel('Front lateral force [N]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.Fyf-ode_res.Fyf)),ylabel('Front lateral force error [N]'),xlabel('Time (s)'),...
    grid

figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.Fyr, ode_res.t,ode_res.Fyr,'--'),ylabel('Rear lateral force [N]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.Fyr-ode_res.Fyr)),ylabel('Rear lateral force error [N]'),xlabel('Time (s)'),...
    grid
