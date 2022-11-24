close all; 
clear all; 
clc

% Run cpp simulation
system('../test/test_beta_estimator_acceleration');

% Load cpp simulation results
res = load('test_beta_estimator_acceleration.txt');
acc_cpp_res.t            = res(:,1);
acc_cpp_res.velocity_ref = res(:,2);
acc_cpp_res.steer_ref    = res(:,3);
acc_cpp_res.velocity_cmd = res(:,4);
acc_cpp_res.steer_cmd    = res(:,5);
acc_cpp_res.x            = res(:,6);
acc_cpp_res.y            = res(:,7);
acc_cpp_res.psi          = res(:,8);
acc_cpp_res.ay           = res(:,9);
acc_cpp_res.r            = res(:,10);
acc_cpp_res.vy           = res(:,11);
acc_cpp_res.beta         = res(:,12);
acc_cpp_res.alphaf       = res(:,13);
acc_cpp_res.alphar       = res(:,14);
acc_cpp_res.Fyf          = res(:,15);
acc_cpp_res.Fyr          = res(:,16);
acc_cpp_res.beta_est     = res(:,17);
acc_cpp_res.vPx          = res(:,18);
acc_cpp_res.vPy          = res(:,19);
acc_cpp_res.xun          = res(:,20);
acc_cpp_res.yun          = res(:,21);
acc_cpp_res.gammaun      = res(:,22);
clear res

delete('test_beta_estimator_acceleration.txt');

% Simulate beta estimator
Kpa = 1000;
Kda = 100;
Ts = 0.001;
Ta = 0.001;
Num_PDa=[(Kpa*(2+Ts/Ta)+2*Kda/Ta)/(2+Ts/Ta),(-2*Kpa+Ts*Kpa/Ta-2*Kda/Ta)/(2+Ts/Ta)];
Den_PDa=[1,(Ts/Ta-2)/(Ts/Ta+2)];
x0_acc = [0,0,0,0.1];
sim_res = sim('beta_estimator_acceleration.slx', 'StartTime',mat2str(acc_cpp_res.t(1)),'StopTime',mat2str(acc_cpp_res.t(end)));

% Plot trajectory
figure,plot(acc_cpp_res.x,acc_cpp_res.y),ylabel('y [m]'),xlabel('x [m]'),grid

% Plot the input signals
figure,
subplot(2,1,1),plot(acc_cpp_res.t,acc_cpp_res.velocity_cmd, acc_cpp_res.t,acc_cpp_res.velocity_ref,'--'),ylabel('Speed [m/s]'),xlabel('Time (s)'),grid
subplot(2,1,2),plot(acc_cpp_res.t,acc_cpp_res.steer_cmd, acc_cpp_res.t,acc_cpp_res.steer_ref,'--'),ylabel('Steer [rad]'),xlabel('Time (s)'),grid

% Plot lateral dynamics measured outputs
% figure,plot(ode_res.t,ode_res.vy),ylabel('Vy [m/s]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.r),ylabel('ode_res.r [rad/s]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.ay),ylabel('ode_res.ay [m/s^2]'),xlabel('Time [s]'),grid
figure,
subplot(2,1,1),plot(acc_cpp_res.t,acc_cpp_res.beta, acc_cpp_res.t,acc_cpp_res.beta_est,'--'),ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('cpp beta sim','cpp beta est')
subplot(2,1,2),plot(sim_res.t,sim_res.beta_est, acc_cpp_res.t,acc_cpp_res.beta_est,'--'),ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('simulink beta est','cpp beta est')

% Plot tyre measured outputs
% figure,plot(ode_res.t,ode_res.alphaf),ylabel('Front slip [rad]'),xlabel('Time [s]'),grid
% figure,plot(ode_res.t,ode_res.alphar),ylabel('Rear slip [rad]'),xlabel('Time [s]'),grid
figure,plot(acc_cpp_res.t,acc_cpp_res.Fyf),ylabel('Front lateral force [N]'),xlabel('Time [s]'),grid
figure,plot(acc_cpp_res.t,acc_cpp_res.Fyr),ylabel('Rear lateral force [N]'),xlabel('Time [s]'),grid

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
