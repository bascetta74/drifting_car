close all; 
clear all; 
clc

% Run cpp simulation
system('../test/test_beta_estimator_velocity');

% Load cpp simulation results
res = load('test_beta_estimator_velocity.txt');
ode_res.t            = res(:,1);
ode_res.velocity_ref = res(:,2);
ode_res.steer_ref    = res(:,3);
ode_res.velocity_cmd = res(:,4);
ode_res.steer_cmd    = res(:,5);
ode_res.x            = res(:,6);
ode_res.y            = res(:,7);
ode_res.psi          = res(:,8);
ode_res.ay           = res(:,9);
ode_res.r            = res(:,10);
ode_res.vy           = res(:,11);
ode_res.beta         = res(:,12);
ode_res.alphaf       = res(:,13);
ode_res.alphar       = res(:,14);
ode_res.Fyf          = res(:,15);
ode_res.Fyr          = res(:,16);
ode_res.beta_est     = res(:,17);
ode_res.vPx          = res(:,18);
ode_res.vPy          = res(:,19);
ode_res.v            = res(:,20);
ode_res.w            = res(:,21);
ode_res.xun          = res(:,22);
ode_res.yun          = res(:,23);
ode_res.gammaun      = res(:,24);
ode_res.xerr         = res(:,25);
ode_res.yerr         = res(:,26);
clear res

delete('test_beta_estimator_velocity.txt');

% Simulate ode_res.beta estimator
Kp = 100;
p  = 0.1;
Ts = 0.001;
x0_vel = [0,0,0];
sim_res = sim('beta_estimator_velocity.slx', 'StartTime',mat2str(ode_res.t(1)),'StopTime',mat2str(ode_res.t(end)));

% Plot trajectory
figure,plot(ode_res.x,ode_res.y),ylabel('ode_res.y [m]'),xlabel('ode_res.x [m]'),grid

% Plot the input signals
figure,
subplot(2,1,1),plot(ode_res.t,ode_res.velocity_cmd, ode_res.t,ode_res.velocity_ref,'--'),ylabel('Speed [m/s]'),xlabel('Time (s)'),grid
subplot(2,1,2),plot(ode_res.t,ode_res.steer_cmd, ode_res.t,ode_res.steer_ref,'--'),ylabel('Steer [rad]'),xlabel('Time (s)'),grid

% Plot lateral dynamics measured outputs
% figure,plot(ode_res.t,ode_res.vy),ylabel('Vy [m/s]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.r),ylabel('ode_res.r [rad/s]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.ay),ylabel('ode_res.ay [m/s^2]'),xlabel('Time [s]'),grid
figure,
subplot(2,1,1),plot(ode_res.t,ode_res.beta, ode_res.t,ode_res.beta_est,'--'),ylabel('sideslip [rad]'),xlabel('Time [s]'),grid,legend('beta sim','beta est odeint')
subplot(2,1,2),plot(sim_res.t,sim_res.beta_est, ode_res.t,ode_res.beta_est,'--'),ylabel('sideslip [rad]'),xlabel('Time [s]'),grid,legend('beta est sim','beta est odeint')

% Plot tyre measured outputs
% figure,plot(ode_res.t,ode_res.alphaf),ylabel('Front slip [rad]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.alphar),ylabel('Rear slip [rad]'),xlabel('Time [s]'),grid
figure,plot(ode_res.t,ode_res.Fyf),ylabel('Front lateral force [N]'),xlabel('Time [s]'),grid
figure,plot(ode_res.t,ode_res.Fyr),ylabel('Rear lateral force [N]'),xlabel('Time [s]'),grid

% figure,
% subplot(2,1,1),plot(ode_res.t,ode_res.xerr),ylabel('ode_res.x tracking error [m]'),xlabel('Time [s]'),grid
% subplot(2,1,2),plot(ode_res.t,ode_res.yerr),ylabel('ode_res.y tracking error [m]'),xlabel('Time [s]'),grid

% figure,
% subplot(2,1,1),plot(ode_res.t,ode_res.v),ylabel('Unicycle ode_res.v [m/s]'),xlabel('Time [s]'),grid
% subplot(2,1,2),plot(ode_res.t,ode_res.w),ylabel('Unicycle ode_res.w [rad/s]'),xlabel('Time [s]'),grid

% figure,
% subplot(3,1,1),plot(ode_res.t,ode_res.xun),ylabel('Unicycle ode_res.x [m]'),xlabel('Time [s]'),grid
% subplot(3,1,2),plot(ode_res.t,ode_res.yun),ylabel('Unicycle ode_res.y [m]'),xlabel('Time [s]'),grid
% subplot(3,1,3),plot(ode_res.t,ode_res.gammaun),ylabel('Unicycle gamma [rad]'),xlabel('Time [s]'),grid
