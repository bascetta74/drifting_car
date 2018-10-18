
clear all;close all;clc;

%% File to load
filename='linear_motion_5March2018';

%% parameters
LineWidth=1.5;
MarkerSize=1;


%% Load mat file
filepath=fullfile(['~/WorkingDirectory2/Mat_Files/',filename,'.mat']);
load(filepath);

%% Opt yaw tuning (only on linear motion bags) 


% trajectory plot
fig_XY=figure('name','opt XY');
plot(car_pose.Data(:,1),car_pose.Data(:,2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('X[m]');ylabel('Y[m]');
axis square
% Vehicle opt Velocity plot
dX=diff(car_pose.Data(:,1));
dY=diff(car_pose.Data(:,2));
dt=diff(car_pose.Time);
Vx_opt=dX./dt; %[m/s]
Vy_opt=dY./dt; %[m/s]
V_opt=timeseries([Vx_opt,Vy_opt],car_pose.Time(2:end));
fig_V=figure('name','V');
g(1)=subplot(211);
plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[m/s]');title('V opt');
axis([V_opt.Time(1),V_opt.Time(end),0,5]);
g(3)=subplot(212);
plot(V_opt.Time,dt*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
axis([car_pose.Time(2),car_pose.Time(end),0 20]);
linkaxes(g,'x');clear g;

%  gamma opt spline
x_opt_spline=spline(car_pose.Time,car_pose.Data(:,1)); %[m]
y_opt_spline=spline(car_pose.Time,car_pose.Data(:,2)); %[m]
dx_opt_spline=fnder(x_opt_spline);
dy_opt_spline=fnder(y_opt_spline);
gamma_opt_spline=timeseries(wrapTo2Pi(unwrap(atan2(ppval(car_pose.Time,dy_opt_spline),ppval(car_pose.Time,dx_opt_spline)))),car_pose.Time); %[rad] from 0 to 2pi

%%%%%%%%%%%%%%%%%%
% difference and delta_theta computation
t_begin=4; %[s]
t_end=6; %[s]

car_pose_orientation_euler=timeseries(quat2eul(car_pose_orientation.Data),car_pose_orientation.Time);
dt_opt=1/100; %[s]
index_begin=find(car_pose_orientation_euler.Time>=t_begin-dt_opt & car_pose_orientation_euler.Time<=t_begin+dt_opt,1);
index_end=find(car_pose_orientation_euler.Time>=t_end-dt_opt & car_pose_orientation_euler.Time<=t_end+dt_opt,1);


difference=timeseries(unwrap(gamma_opt_spline.Data(index_begin:index_end))-unwrap(car_pose_orientation_euler.Data(index_begin:index_end,1)),...
    gamma_opt_spline.Time(index_begin:index_end)); %[rad]

delta_theta=mean(difference.Data); %[rad]

% plot of the difference
figure('name','difference yaw opt');
g(1)=subplot(211);
plot(difference.Time,difference.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
plot([difference.Time(1),difference.Time(end)],delta_theta*180/pi*[1,1],'r --','linewidth',1.5);
xlabel('t[s]');ylabel('[deg]');title('\gamma - \theta');
g(2)=subplot(212);
plot(difference.Time(2:end),diff(difference.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('dt[ms]');ylabel('msgs freq');
linkaxes(g,'x');

figure('name','difference yaw histogram');
histogram((difference.Data-delta_theta)*180/pi,20,'Normalization','probability');
xlabel('error [deg]');ylabel('freq');

theta_offsetted=timeseries(wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta),car_pose_orientation_euler.Time);

figure('name','correct yaw opt method');
g(1)=subplot(311); %V
plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[m/s]');title('V opt');
g(2)=subplot(312); %gamma,theta
plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
plot(theta_offsetted.Time,theta_offsetted.Data*180/pi,'rd-','linewidth',LineWidth,'MarkerSize',MarkerSize);
plot(car_pose_orientation_euler.Time,car_pose_orientation_euler.Data(:,1)*180/pi,'go-','linewidth',1.5,'MarkerSize',MarkerSize);
xlabel('t[s]');ylabel('[deg]');
legend('\gamma','\theta offsetted','\theta');
g(3)=subplot(313);
plot(gamma_opt_spline.Time(2:end),diff(gamma_opt_spline.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[ms]');title('messages freq');
axis([car_pose.Time(2),car_pose.Time(end),0 20]);
linkaxes(g,'x');clear g;

fprintf('\n opt yaw offset = %g rad = %g deg \n',delta_theta,delta_theta*180/pi);


%% IMU pitch,roll determination of the steady-state value

% trajectory plot
fig_XY=figure('name','opt XY');
plot(car_pose.Data(:,1),car_pose.Data(:,2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('X[m]');ylabel('Y[m]');
axis square

% Vehicle opt Velocity plot
dX=diff(car_pose.Data(:,1));
dY=diff(car_pose.Data(:,2));
dt=diff(car_pose.Time);
Vx_opt=dX./dt; %[m/s]
Vy_opt=dY./dt; %[m/s]
V_opt=timeseries([Vx_opt,Vy_opt],car_pose.Time(2:end));
fig_V=figure('name','V');
g(1)=subplot(211);
plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[m/s]');title('V opt');
axis([V_opt.Time(1),V_opt.Time(end),0,5]);
g(3)=subplot(212);
plot(V_opt.Time,dt*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
axis([car_pose.Time(2),car_pose.Time(end),0 20]);
linkaxes(g,'x');clear g;

% roll,pitch angles
imu_data_orientation_euler=timeseries(quat2eul(imu_data_orientation.Data),imu_data_orientation.Time); %[yaw,pitch,roll] rad
fig_imu_orient_euler=figure('name','imu vehicle angles');
g(1)=subplot(311); %roll
plot(imu_data_orientation_euler.Time,imu_data_orientation_euler.Data(:,3)*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[deg]');title('roll');
g(2)=subplot(312); %delta pitch
plot(imu_data_orientation_euler.Time,imu_data_orientation_euler.Data(:,2)*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[deg]');title('pitch');
g(3)=subplot(313); %delta yaw
plot(imu_data_orientation_euler.Time,imu_data_orientation_euler.Data(:,1)*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[deg]');title('Yaw');
linkaxes(g,'x');clear g;

% ruoto le accelerazioni secondo l'opposto degli angoli forniti dal
% giroscopio
% NB solo angoli di pitch e roll, non di yaw
eul=quat2eul(imu_data_orientation.Data); %[yaw,pitch,roll]
q_imu=eul2quat([zeros(length(eul),1),eul(:,[2,3])]);
imu_data_linear_acc_rot=timeseries(quatrotate(quatinv(q_imu),imu_data_linear_acc.Data),imu_data_linear_acc.Time);


figure('name','imu rotated accelerations');
g(1)=subplot(311); %ax
plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,1),'b');grid on;hold on;
plot(imu_data_linear_acc.Time,imu_data_linear_acc.Data(:,1),'k');
xlabel('t[s]');ylabel('[m/s^2]');legend('rot','orig');
title('a_x');
g(2)=subplot(312); %ay
plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,2),'b');grid on;hold on;
plot(imu_data_linear_acc.Time,imu_data_linear_acc.Data(:,2),'k');
xlabel('t[s]');ylabel('[m/s^2]');legend('rot','orig');
title('a_y');
g(3)=subplot(313); %az
plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,3),'b');grid on;hold on;
plot(imu_data_linear_acc.Time,imu_data_linear_acc.Data(:,3),'k');
xlabel('t[s]');ylabel('[m/s^2]');legend('rot','orig');
title('a_z');

% % mi porto alla convenzione europea (z verso alto, x verso avanti, y verso
% % sx)
% convention_rotation=eul2quat([pi,0,pi]);
% imu_data_linear_acc_rot_conv=timeseries(quatrotate(convention_rotation,imu_data_linear_acc_rot.Data),imu_data_linear_acc_rot.Time);
% figure('name','imu rotated accelerations convention');
% g(1)=subplot(311); %ax
% plot(imu_data_linear_acc_rot_conv.Time,imu_data_linear_acc_rot_conv.Data(:,1),'b');grid on;hold on;
% plot(imu_data_linear_acc.Time,imu_data_linear_acc.Data(:,1),'k');
% xlabel('t[s]');ylabel('[m/s^2]');legend('rot','orig');
% title('a_x');
% g(2)=subplot(312); %ay
% plot(imu_data_linear_acc_rot_conv.Time,imu_data_linear_acc_rot_conv.Data(:,2),'b');grid on;hold on;
% plot(imu_data_linear_acc.Time,imu_data_linear_acc.Data(:,2),'k');
% xlabel('t[s]');ylabel('[m/s^2]');legend('rot','orig');
% title('a_y');
% g(3)=subplot(313); %az
% plot(imu_data_linear_acc_rot_conv.Time,imu_data_linear_acc_rot_conv.Data(:,3),'b');grid on;hold on;
% plot(imu_data_linear_acc.Time,imu_data_linear_acc.Data(:,3),'k');
% xlabel('t[s]');ylabel('[m/s^2]');legend('rot','orig');
% title('a_z');

%%%%%%%%%%%%%%%%
t_begin=0.5; %[s]
t_end=3.5;%[s]

dt_imu=1/100; %[s]
%%%%%%%%%%%%%%%%%%%%%%%%%

index_begin_imu=find(imu_data_orientation_euler.Time>=t_begin-dt_imu & imu_data_orientation_euler.Time<=t_begin+dt_imu,1);
index_end_imu=find(imu_data_orientation_euler.Time>=t_end-dt_imu & imu_data_orientation_euler.Time<=t_end+dt_imu,1);
imu_data_pitchroll0_tuning=timeseries(imu_data_orientation_euler.Data(index_begin_imu:index_end_imu,:),imu_data_orientation_euler.Time(index_begin_imu:index_end_imu));

pitch0=mean(imu_data_pitchroll0_tuning.Data(:,2)); %[rad]
roll0=mean(imu_data_pitchroll0_tuning.Data(:,3)); %[rad]

fprintf('\n imu pitch 0=%g rad = %g deg \n',pitch0,pitch0*180/pi);
fprintf('\n imu roll 0=%g rad = %g deg \n',roll0,roll0*180/pi);

figure('name','pitch roll used data');
g(1)=subplot(211); %pitch
plot(imu_data_pitchroll0_tuning.Time,imu_data_pitchroll0_tuning.Data(:,2)*180/pi,'b','linewidth',LineWidth);grid on;hold on;
plot([imu_data_pitchroll0_tuning.Time(1),imu_data_pitchroll0_tuning.Time(end)],pitch0*[1,1]*180/pi,'r--','linewidth',LineWidth);
xlabel('t[s]');ylabel('[deg]');title('pitch');
axis([imu_data_pitchroll0_tuning.Time(1),imu_data_pitchroll0_tuning.Time(end),-10,+10]);
g(2)=subplot(212); %roll
plot(imu_data_pitchroll0_tuning.Time,imu_data_pitchroll0_tuning.Data(:,3)*180/pi,'b','linewidth',LineWidth);grid on;hold on;
plot([imu_data_pitchroll0_tuning.Time(1),imu_data_pitchroll0_tuning.Time(end)],roll0*[1,1]*180/pi,'r--','linewidth',LineWidth);
xlabel('t[s]');ylabel('[deg]');title('roll');
linkaxes(g,'x');clear g;

figure('name','IMU delta pitch,delta roll');
delta_pitch=timeseries(wrapToPi(unwrap(imu_data_orientation_euler.Data(:,2)-pitch0)),imu_data_orientation_euler.Time);
delta_roll=timeseries(wrapToPi(unwrap(imu_data_orientation_euler.Data(:,3)-roll0)),imu_data_orientation_euler.Time);
plot(delta_pitch.Time,delta_pitch.Data*180/pi,'k','linewidth',LineWidth);grid on;hold on;
plot(delta_roll.Time,delta_roll.Data*180/pi,'m','linewidth',LineWidth);grid on;hold on;
xlabel('t[s]');ylabel('[deg]');
legend('\Delta pitch','\Delta roll');

figure('name','imu pitch,roll offset error histogram');
subplot(121); %pitch
histogram((imu_data_pitchroll0_tuning.Data(:,2)-pitch0)*180/pi,20,'Normalization','probability');
xlabel('error [deg]');ylabel('freq');title('pitch');
subplot(122); %roll
histogram((imu_data_pitchroll0_tuning.Data(:,3)-roll0)*180/pi,20,'Normalization','probability');
xlabel('error [deg]');ylabel('freq');title('roll');

%% IMU yaw tuning (only on linear motion bags)
% 
% % trajectory plot
% fig_XY=figure('name','opt XY');
% plot(car_pose.Data(:,1),car_pose.Data(:,2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
% xlabel('X[m]');ylabel('Y[m]');
% axis square
% % Vehicle opt Velocity plot
% dX=diff(car_pose.Data(:,1));
% dY=diff(car_pose.Data(:,2));
% dt=diff(car_pose.Time);
% Vx_opt=dX./dt; %[m/s]
% Vy_opt=dY./dt; %[m/s]
% V_opt=timeseries([Vx_opt,Vy_opt],car_pose.Time(2:end));
% fig_V=figure('name','V');
% g(1)=subplot(211);
% plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
% xlabel('t[s]');ylabel('[m/s]');title('V opt');
% axis([V_opt.Time(1),V_opt.Time(end),0,5]);
% g(3)=subplot(212);
% plot(V_opt.Time,dt*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
% xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
% axis([car_pose.Time(2),car_pose.Time(end),0 20]);
% linkaxes(g,'x');clear g;
% 
% pitch0=0.0684383; %[rad]
% roll0=-0.0191842; %[rad]
% 
% t_begin=1; %[s]
% t_end=2; %[s]
% 
% q_rot_imu=eul2quat([0,pitch0,roll0]);
% imu_data_linear_acc_rot=timeseries(quatrotate(q_rot_imu,imu_data_linear_acc.Data),imu_data_orientation.Time);
% dt=1/100; %[s]
% index_begin_imu=find(imu_data_linear_acc_rot.Time>=t_begin-dt & imu_data_linear_acc_rot.Time<=t_begin+dt,1);
% index_end_imu=find(imu_data_linear_acc_rot.Time>=t_end-dt & imu_data_linear_acc_rot.Time<=t_end+dt,1);
% imu_acc_yaw_tuning=timeseries(imu_data_linear_acc_rot.Data(index_begin_imu:index_end_imu,:),imu_data_linear_acc_rot.Time(index_begin_imu:index_end_imu));
% 
% % filter definition
% s=tf('s');
% wf=10; %[rad/s]
% F=1/(1+s/wf);
% Fd=c2d(F,1e-2,'tustin');
% 
% % data filtering
% ax_imu=timeseries(imu_acc_yaw_tuning.Data(:,1),imu_acc_yaw_tuning.Time); %[m/s^2]
% ay_imu=timeseries(imu_acc_yaw_tuning.Data(:,2),imu_acc_yaw_tuning.Time); %[m/s^2]
% 
% ax_imu_filt=timeseries(filter(Fd.Num{1},Fd.Den{1},ax_imu.Data),ax_imu.Time);
% ay_imu_filt=timeseries(filter(Fd.Num{1},Fd.Den{1},ay_imu.Data),ax_imu.Time);
% 
% %%%%%%%%%%
% figure('name','ax ay filter');
% g(1)=subplot(211); %ax
% plot(ax_imu.Time,ax_imu.Data,'r');grid on;hold on;
% plot(ax_imu_filt.Time,ax_imu_filt.Data,'b');
% xlabel('t[s]');ylabel('[m/s^2]');title('a_x');
% legend('data','filtered');
% g(2)=subplot(212); %ay
% plot(ay_imu.Time,ay_imu.Data,'r');grid on;hold on;
% plot(ay_imu_filt.Time,ay_imu_filt.Data,'b');
% xlabel('t[s]');ylabel('[m/s^2]');title('a_y');
% 
% %calcolo acc optitrack
% dVx_opt=diff(V_opt.Data(:,1));
% dVy_opt=diff(V_opt.Data(:,2));
% dt2=diff(V_opt.Time);
% acc_opt=timeseries(sqrt((dVx_opt./dt2).^2+(dVy_opt./dt2).^2),V_opt.Time(2:end)); %[m/s^2]
% 
% figure('name','acc imu opt');
% g(1)=subplot(311); %ax
% plot(ax_imu.Time,ax_imu.Data,'r');grid on;hold on;
% plot(ax_imu_filt.Time,ax_imu_filt.Data,'b');
% plot(acc_opt.Time,acc_opt.Data,'g');
% axis([ax_imu.Time(1),ax_imu.Time(end),0,5]);
% xlabel('t[s]');ylabel('[m/s^2]');title('a_x');
% legend('data','filtered');
% g(2)=subplot(312); %ay
% plot(ay_imu.Time,ay_imu.Data,'r');grid on;hold on;
% plot(ay_imu_filt.Time,ay_imu_filt.Data,'b');
% xlabel('t[s]');ylabel('[m/s^2]');title('a_y');
% axis([ax_imu.Time(1),ax_imu.Time(end),-1,1])
% g(3)=subplot(313); %V opt
% plot(V_opt.Time,V_opt.Data,'g');grid on;
% xlabel('t[s]');ylabel('[m/s]');title('V_{opt}');
% axis([ax_imu.Time(1),ax_imu.Time(end),0,3]);
% linkaxes(g,'x');
% 
% %pitch, roll angle time evolution
% imu_data_orientation_euler=timeseries(quat2eul(imu_data_orientation.Data),imu_data_orientation.Time);
% pitch=timeseries(wrapToPi(imu_data_orientation_euler.Data(index_begin_imu:index_end_imu,2)+pitch0),imu_data_orientation_euler.Time(index_begin_imu:index_end_imu));
% roll=timeseries(wrapToPi(imu_data_orientation_euler.Data(index_begin_imu:index_end_imu,3)+roll0),imu_data_orientation_euler.Time(index_begin_imu:index_end_imu));
% 
% figure('name','pitch, roll');
% g(1)=subplot(211); %angles
% plot(pitch.Time,pitch.Data*180/pi,'b');grid on; hold on;
% plot(roll.Time,roll.Data*180/pi,'r');
% xlabel('t[s]');ylabel('[deg]');
% legend('pitch','roll');
% axis([ax_imu.Time(1),ax_imu.Time(end),-3,3]);
% g(2)=subplot(212); %V opt,ax,ay
% plot(V_opt.Time,V_opt.Data,'g');grid on;hold on;
% plot(ax_imu_filt.Time,ax_imu_filt.Data,'m');
% plot(ay_imu_filt.Time,ay_imu_filt.Data,'c');
% xlabel('t[s]');ylabel('[m/s]');
% legend('V_{opt}','a_x filt','a_y filt');
% axis([ax_imu.Time(1),ax_imu.Time(end),-1,3]);
% linkaxes(g,'x');


%% imu yaw offset with respect to opt yaw tuning

% fig_XY=figure('name','opt XY');
% plot(car_pose.Data(:,1),car_pose.Data(:,2),'b');grid on;
% xlabel('X[m]');ylabel('Y[m]');
% axis square
% 
% figure('name','opt comet XY');
% axis square;
% comet(car_pose.Data(:,1),car_pose.Data(:,2));
% xlabel('X[m]');ylabel('Y[m]');
% 
% %    Vx,Vy using the opt pose topic
% dX=diff(car_pose.Data(:,1));
% dY=diff(car_pose.Data(:,2));
% dt=diff(car_pose.Time);
% Vx_opt=dX./dt; %[m/s]
% Vy_opt=dY./dt; %[m/s]
% V_opt=timeseries([Vx_opt,Vy_opt],car_pose.Time(2:end));
% fig_V=figure('name','V opt');
% g(1)=subplot(211);
% plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b');grid on;
% xlabel('t[s]');ylabel('[m/s]');title('V opt');
% axis([V_opt.Time(1),V_opt.Time(end),0,5]);
% g(3)=subplot(212);
% plot(V_opt.Time,dt*1000,'b');grid on;
% xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
% axis([car_pose.Time(2),car_pose.Time(end),0 20]);
% linkaxes(g,'x');clear g;
% 
% %%%%%%%%%%
% t_begin=10; %[s]
% t_end=12.5;%[s]
% 
% dt_imu=1/160; %[s]
% dt_opt=1/100; %[s]
% 
% imu_data_orientation_euler =timeseries(quat2eul(imu_data_orientation.Data),imu_data_orientation.Time);
% car_pose_orientation_euler =timeseries(quat2eul(car_pose_orientation.Data),car_pose_orientation.Time);
% 
% % imu sampling time is different from opt sampling time, therefore I
% % resample opt data to fit with imu data samples
% 
% index_begin_imu=min(find(imu_data_orientation_euler.Time>=t_begin-dt_imu & imu_data_orientation_euler.Time<=t_begin+dt_imu));
% index_end_imu=min(find(imu_data_orientation_euler.Time>=t_end-dt_imu & imu_data_orientation_euler.Time<=t_end+dt_imu));
% imu_data_yawtuning=timeseries(imu_data_orientation_euler.Data(index_begin_imu:index_end_imu,3),imu_data_orientation_euler.Time(index_begin_imu:index_end_imu));
% 
% index_begin_opt=min(find(car_pose_orientation_euler.Time>=t_begin-dt_opt & car_pose_orientation_euler.Time<=t_begin+dt_opt));
% index_end_opt=min(find(car_pose_orientation_euler.Time>=t_end-dt_opt & car_pose_orientation_euler.Time<=t_end+dt_opt));
% interpolated_yawopt=interp1(car_pose_orientation_euler.Time(index_begin_opt:index_end_opt),...
%     unwrap(car_pose_orientation_euler.Data(index_begin_opt:index_end_opt,3)),...
%     imu_data_orientation_euler.Time(index_begin_imu:index_end_imu),...
%     'spline'); %[rad] unwrapped
% car_pose_yawtuning=timeseries(wrapToPi(interpolated_yawopt),imu_data_orientation_euler.Time(index_begin_imu:index_end_imu));
% 
% 
% difference_yaw=timeseries(-unwrap(car_pose_yawtuning.Data)+unwrap(imu_data_orientation_euler.Data(index_begin_imu:index_end_imu,3)),car_pose_yawtuning.Time);
% delta_yaw_imu=mean(difference_yaw.Data); %[rad]
% 
% 
% figure('name','IMU yaw offset');
% g(1)=subplot(311); %delta theta
% plot(difference_yaw.Time,difference_yaw.Data*180/pi,'b');grid on;hold on;
% plot([difference_yaw.Time(1),difference_yaw.Time(end)],delta_yaw_imu*180/pi*[1,1],'r --');
% xlabel('t[s]');ylabel('[deg]');title('\theta_{imu}-\theta_{opt}');
% g(2)=subplot(312); %imu msgs dt
% plot(difference_yaw.Time(2:end),diff(difference_yaw.Time)*1000,'b');grid on;
% xlabel('t[s]');ylabel('dt[ms]');ylabel('[ms]');title('imu msgs dt');
% axis([difference_yaw.Time(1),difference_yaw.Time(end),0,15]);
% g(3)=subplot(313); % opt and imu data
% plot(car_pose_orientation_euler.Time(index_begin_opt:index_end_opt),...
%     wrapToPi(unwrap(car_pose_orientation_euler.Data(index_begin_opt:index_end_opt,3)))*180/pi,'k');grid on;hold on;
% plot(car_pose_yawtuning.Time,car_pose_yawtuning.Data*180/pi,'m');
% plot(imu_data_orientation_euler.Time(index_begin_imu:index_end_imu),...
%     wrapToPi(unwrap(imu_data_orientation_euler.Data(index_begin_imu:index_end_imu,3))-delta_yaw_imu)*180/pi,'g');
% xlabel('t[s]');ylabel('[deg]');title('yaw angle \theta');
% legend('opt','opt interp','imu with offset');
% linkaxes(g,'x');clear g;
% 
% 
% figure('name','imu yaw offset error histogram');
% histogram((difference_yaw.Data-delta_yaw_imu)*180/pi,20,'Normalization','probability');
% xlabel('error [deg]');ylabel('freq');
% 
% fprintf('\n imu yaw offset (imu_yaw-opt_yaw)=%g rad = %g deg \n',delta_yaw_imu,delta_yaw_imu*180/pi);


