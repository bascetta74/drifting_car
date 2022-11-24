close all; 
clear all; 
clc

% Run cpp simulation
system('../test/test_beta_estimator_velocity');

% Load cpp simulation results
res = load('test_beta_estimator_velocity.txt');
vel_cpp_res.t            = res(:,1);
vel_cpp_res.velocity_ref = res(:,2);
vel_cpp_res.steer_ref    = res(:,3);
vel_cpp_res.velocity_cmd = res(:,4);
vel_cpp_res.steer_cmd    = res(:,5);
vel_cpp_res.x            = res(:,6);
vel_cpp_res.y            = res(:,7);
vel_cpp_res.psi          = res(:,8);
vel_cpp_res.ay           = res(:,9);
vel_cpp_res.r            = res(:,10);
vel_cpp_res.vy           = res(:,11);
vel_cpp_res.beta         = res(:,12);
vel_cpp_res.alphaf       = res(:,13);
vel_cpp_res.alphar       = res(:,14);
vel_cpp_res.Fyf          = res(:,15);
vel_cpp_res.Fyr          = res(:,16);
vel_cpp_res.beta_est     = res(:,17);
vel_cpp_res.vPx          = res(:,18);
vel_cpp_res.vPy          = res(:,19);
vel_cpp_res.xun          = res(:,20);
vel_cpp_res.yun          = res(:,21);
vel_cpp_res.gammaun      = res(:,22);
clear res

delete('test_beta_estimator_velocity.txt');

% Simulate beta estimator
Kp = 100;
p  = 0.1;
Ts = 0.001;
x0_vel = [0,0,0];
sim_res = sim('beta_estimator_velocity.slx', 'StartTime',mat2str(vel_cpp_res.t(1)),'StopTime',mat2str(vel_cpp_res.t(end)));

% Plot trajectory
figure,plot(vel_cpp_res.x,vel_cpp_res.y),ylabel('y [m]'),xlabel('x [m]'),grid

% Plot the input signals
figure,
subplot(2,1,1),plot(vel_cpp_res.t,vel_cpp_res.velocity_cmd, vel_cpp_res.t,vel_cpp_res.velocity_ref,'--'),ylabel('Speed [m/s]'),xlabel('Time (s)'),grid
subplot(2,1,2),plot(vel_cpp_res.t,vel_cpp_res.steer_cmd, vel_cpp_res.t,vel_cpp_res.steer_ref,'--'),ylabel('Steer [rad]'),xlabel('Time (s)'),grid

% Plot lateral dynamics measured outputs
% figure,plot(ode_res.t,ode_res.vy),ylabel('Vy [m/s]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.r),ylabel('ode_res.r [rad/s]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.ay),ylabel('ode_res.ay [m/s^2]'),xlabel('Time [s]'),grid
figure,
subplot(2,1,1),plot(vel_cpp_res.t,vel_cpp_res.beta, vel_cpp_res.t,vel_cpp_res.beta_est,'--'),ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('cpp beta sim','cpp beta est')
subplot(2,1,2),plot(sim_res.t,sim_res.beta_est, vel_cpp_res.t,vel_cpp_res.beta_est,'--'),ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('simulink beta est','cpp beta est')

% Plot tyre measured outputs
% figure,plot(ode_res.t,ode_res.alphaf),ylabel('Front slip [rad]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.alphar),ylabel('Rear slip [rad]'),xlabel('Time [s]'),grid
figure,plot(vel_cpp_res.t,vel_cpp_res.Fyf),ylabel('Front lateral force [N]'),xlabel('Time [s]'),grid
figure,plot(vel_cpp_res.t,vel_cpp_res.Fyr, sim_res.t,sim_ref.Fy_ref,'--'),ylabel('Rear lateral force [N]'),xlabel('Time [s]'),grid

% figure,
% subplot(3,1,1),plot(ode_res.t,ode_res.xun),ylabel('Unicycle ode_res.x [m]'),xlabel('Time [s]'),grid
% subplot(3,1,2),plot(ode_res.t,ode_res.yun),ylabel('Unicycle ode_res.y [m]'),xlabel('Time [s]'),grid
% subplot(3,1,3),plot(ode_res.t,ode_res.gammaun),ylabel('Unicycle gamma [rad]'),xlabel('Time [s]'),grid
