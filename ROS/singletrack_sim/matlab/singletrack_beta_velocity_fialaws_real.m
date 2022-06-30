close all; 
clear; 
clc

% Run cpp simulation
system('../test/test_singletrack_beta_velocity_ode fiala_w_sat real');

% Load cpp simulation results
res = load('test_singletrack_beta_velocity_fiala_w_sat_real.txt');
ode_res.t         = res(:,1);
ode_res.speed_ref = res(:,2);
ode_res.steer_ref = res(:,3);
ode_res.speed_cmd = res(:,4);
ode_res.steer_cmd = res(:,5);
ode_res.x         = res(:,6);
ode_res.y         = res(:,7);
ode_res.psi       = res(:,8);
ode_res.ay        = res(:,9);
ode_res.r         = res(:,10);
ode_res.vy        = res(:,11);
ode_res.beta      = res(:,12);
ode_res.alphaf    = res(:,13);
ode_res.alphar    = res(:,14);
ode_res.Fyf       = res(:,15);
ode_res.Fyr       = res(:,16);
clear res

delete('test_singletrack_beta_velocity_fiala_w_sat_real.txt');

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
mu_speed  = 1;
wn_speed  = 650;
csi_speed = 0.9;
tau_speed = 0.036;
actuator_param = [mu_steer wn_steer csi_steer tau_steer mu_speed wn_speed csi_speed tau_speed];

% Initial values of the state variables
beta0 = 0;       % initial sideslip
r0    = 0;       % initial yaw rate
psi0  = 0;       % initial orientation
x0    = 0;       % initial position x
y0    = 0;       % initial position y
initial_state = [beta0 r0 psi0 x0 y0];

% Generate the inputs
speed_ref = timeseries(ode_res.speed_ref,ode_res.t);  % input Vx timeseries
steer_ref = timeseries(ode_res.steer_ref,ode_res.t);  % input delta timeseries

% Set the flag that will choose the model describing the behavior of lateral forces
tyre_model = 1; % 0='Linear Model'  1='Fiala model'  2='Fiala without saturation'

% Set the flag that will choose the model describing the behavior of actuators
actuator_model = 1; % 0='Ideal actuators'  1='Second order actuator model'

% Run the system simulation
Ts = ode_res.t(2)-ode_res.t(1);
if (actuator_model==0)
    load_system('carsim_beta_velocity');
    set_param('carsim_beta_velocity/Actuator models','Commented','through');
else
    load_system('carsim_beta_velocity');
    set_param('carsim_beta_velocity/Actuator models','Commented','off');
end

sim_res = sim('carsim_beta_velocity.slx','StartTime','steer_ref.Time(1)','StopTime',...
    'steer_ref.Time(end)+Ts','SrcWorkspace' ,'current');

% Plot the input signals
figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.speed_cmd, ode_res.t,ode_res.speed_cmd,'--'),ylabel('Vx [m/s]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.speed_cmd-ode_res.speed_cmd)),ylabel('Vx abs error [m/s]'),xlabel('Time (s)'),grid

figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.steer_cmd, ode_res.t,ode_res.steer_cmd,'--'),ylabel('Delta [rad]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.steer_cmd-ode_res.steer_cmd)),ylabel('Delta abs error [rad]'),xlabel('Time (s)'),grid

% Plot lateral dynamics measured outputs
figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.vy, ode_res.t,ode_res.vy,'--'),ylabel('Vy [m/s]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.vy-ode_res.vy)),ylabel('Vy abs error [m/s]'),xlabel('Time (s)'),...
    grid

figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.r, ode_res.t,ode_res.r,'--'),ylabel('r [rad/s]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.r-ode_res.r)),ylabel('r abs error [rad/s]'),xlabel('Time (s)'),...
    grid

figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.ay, ode_res.t,ode_res.ay,'--'),ylabel('ay [m/s^2]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.ay-ode_res.ay)),ylabel('ay abs error [m/s^2]'),xlabel('Time (s)'),...
    grid

figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.beta, ode_res.t,ode_res.beta,'--'),ylabel('sideslip [rad]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.beta-ode_res.beta)),ylabel('sideslip abs error [rad]'),xlabel('Time (s)'),...
    grid

% Plot tyre measured outputs
figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.alphaf, ode_res.t,ode_res.alphaf,'--'),ylabel('Front slip [rad]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.alphaf-ode_res.alphaf)),ylabel('Front slip error [rad]'),xlabel('Time (s)'),...
    grid

figure,
subplot(2,1,1),...
    plot(sim_res.time,sim_res.alphar, ode_res.t,ode_res.alphar,'--'),ylabel('Rear slip [rad]'),xlabel('Time (s)'),...
    grid,legend('Simulink','ODEINT')
subplot(2,1,2),...
    plot(ode_res.t,abs(sim_res.alphar-ode_res.alphar)),ylabel('Rear slip error [rad]'),xlabel('Time (s)'),...
    grid

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
