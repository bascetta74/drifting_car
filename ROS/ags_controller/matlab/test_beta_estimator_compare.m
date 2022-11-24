close all; 
clear all; 
clc

%% Velocity estimator
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
vel_sim_res = sim('beta_estimator_velocity.slx', 'StartTime',mat2str(vel_cpp_res.t(1)),'StopTime',mat2str(vel_cpp_res.t(end)));

%% Acceleration estimator
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
acc_sim_res = sim('beta_estimator_acceleration.slx', 'StartTime',mat2str(acc_cpp_res.t(1)),'StopTime',mat2str(acc_cpp_res.t(end)));

%% Plot results

% Plot trajectory
figure,
plot(vel_cpp_res.x,vel_cpp_res.y, acc_cpp_res.x,acc_cpp_res.y,'r--'),ylabel('y [m]'),xlabel('x [m]'),legend('cpp vel','cpp acc'),grid

% Plot the input signals
figure,title('velocity')
subplot(2,1,1),plot(vel_cpp_res.t,vel_cpp_res.velocity_cmd, vel_cpp_res.t,vel_cpp_res.velocity_ref,'--'),...
    ylabel('Speed [m/s]'),xlabel('Time (s)'),legend('cpp vel','cpp vel ref'),grid
subplot(2,1,2),plot(vel_cpp_res.t,vel_cpp_res.steer_cmd, vel_cpp_res.t,vel_cpp_res.steer_ref,'--'),...
    ylabel('Steer [rad]'),xlabel('Time (s)'),legend('cpp vel','cpp vel ref'),grid
figure,title('acceleration')
subplot(2,1,1),plot(acc_cpp_res.t,acc_cpp_res.velocity_cmd, acc_cpp_res.t,acc_cpp_res.velocity_ref,'--'),...
    ylabel('Speed [m/s]'),xlabel('Time (s)'),legend('cpp acc','cpp acc ref'),grid
subplot(2,1,2),plot(acc_cpp_res.t,acc_cpp_res.steer_cmd, acc_cpp_res.t,acc_cpp_res.steer_ref,'--'),...
    ylabel('Steer [rad]'),xlabel('Time (s)'),legend('cpp acc','cpp acc ref'),grid

% Plot lateral dynamics measured outputs
figure,title('velocity')
subplot(2,1,1),plot(vel_cpp_res.t,vel_cpp_res.beta, vel_cpp_res.t,vel_cpp_res.beta_est,'--'),...
    ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('cpp vel beta sim','cpp vel beta est')
subplot(2,1,2),plot(vel_sim_res.t,vel_sim_res.beta_est, vel_cpp_res.t,vel_cpp_res.beta_est,'--'),...
    ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('simulink vel beta est','cpp vel beta est')
figure,title('acceleration')
subplot(2,1,1),plot(acc_cpp_res.t,acc_cpp_res.beta, acc_cpp_res.t,acc_cpp_res.beta_est,'--'),...
    ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('cpp acc beta sim','cpp acc beta est')
subplot(2,1,2),plot(acc_sim_res.t,acc_sim_res.beta_est, acc_cpp_res.t,acc_cpp_res.beta_est,'--'),...
    ylabel('Sideslip [rad]'),xlabel('Time [s]'),grid,legend('simulink acc beta est','cpp acc beta est')

% Plot tyre measured outputs
figure,plot(vel_cpp_res.t,vel_cpp_res.Fyf, acc_cpp_res.t,acc_cpp_res.Fyf,'r--'),...
    ylabel('Front lateral force [N]'),xlabel('Time [s]'),grid,legend('cpp vel','cpp acc')
figure,plot(vel_cpp_res.t,vel_cpp_res.Fyr, acc_cpp_res.t,acc_cpp_res.Fyr,'r--'),...
    ylabel('Rear lateral force [N]'),xlabel('Time [s]'),grid,legend('cpp vel','cpp acc')
