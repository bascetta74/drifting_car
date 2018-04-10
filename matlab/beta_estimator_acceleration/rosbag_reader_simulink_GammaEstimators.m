clear all;close all;clc;

%% PARAMETERS

filename='casual_drifting_motion_15February2018_filtered';

% imu offset angles for data 15 february
delta_yaw_imu=0; %[rad]
pitch0=0.0677409; %[rad]
roll0=-0.0178287; %[rad]

% % imu offset angles for data 26 february
% delta_yaw_imu=0; %[rad]
% pitch0=0.0684383; %[rad]
% roll0=-0.0191842; %[rad]

% % DATA 5 MARCH 2018
% delta_yaw_imu=0; %[rad]
% pitch0=-0.0877552; %[rad]
% roll0=0.0233003; %[rad]

% Casual filter
wf=250; %[rad/s]
n_filter=1;%lowpass filter order
Ts_filter=1e-3; %[s]

% FIR moving average filter
Nsmooth=10;

%Tolgo buchi
Vmin=0.1; %[m/s]
Vmax=3.5; %[m/s]

%%%%%%%%%%%%%%%%
sim_active=1;

Kp=10*2*pi; %wc vel estimator
p=0.10; %[m]

wca=50; %[rad/s]

% % remove data from opt (GPS)
% t_start_remotion=[0]; %[s]
% t_end_remotion=[0];%[s]

%%%%%%%%%%
mu=0.35; %friction coeff (per figura con ay_max)

%%%%%%%
% graph parameters
LineWidth=1.5;
MarkerSize=5;

comet_XY_active=1;

%% Load mat file
filepath=fullfile(['~/WorkingDirectory2/Mat_Files/',filename,'.mat']);
load(filepath);

%% Causal filter building
    s=tf('s');
    FilterCTF=1/(1+s/wf)^n_filter;
    FilterDTF=c2d(FilterCTF,Ts_filter,'tustin');

%% imu orientation (vehicle roll,pitch,yaw)
if imu_data_active
    
    % IMU ORIENTATION QUATERNION COMPONENTS (TO VERIFIY W BELONGS TO [0,1])
    fig_imu_orient_quat=figure('name','imu orientation quaternion');
    g(1)=subplot(211); 
    plot(imu_data_orientation,'linewidth',LineWidth);grid on;
    legend('W','X','Y','Z');
    xlabel('t[s]');title('imu orientation quaternion');
    g(2)=subplot(212); %messages frequency
    plot(imu_data_orientation.Time(2:end),diff(imu_data_orientation.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('dt [ms]');title('messages freq');
    linkaxes(g,'x');clear g;
  
    
    % VEHICLE ROLL, PITCH, YAW (difference with respect to standing still values)
    imu_data_orientation_euler=timeseries(quat2eul(imu_data_orientation.Data),imu_data_orientation.Time); %[yaw,pitch,roll] rad
    imu_deltaroll=timeseries(wrapToPi(unwrap(imu_data_orientation_euler.Data(:,3))-roll0),imu_data_orientation_euler.Time);
    imu_deltapitch=timeseries(wrapToPi(unwrap(imu_data_orientation_euler.Data(:,2))-pitch0),imu_data_orientation_euler.Time);
    imu_yaw=timeseries(wrapTo2Pi(unwrap(imu_data_orientation_euler.Data(:,1))+delta_yaw_imu),imu_data_orientation_euler.Time);
    
    fig_imu_orient_euler=figure('name','imu vehicle angles');
    g(1)=subplot(311); %delta roll
    plot(imu_deltaroll.Time,imu_deltaroll.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[deg]');title('\Delta roll');
    g(2)=subplot(312); %delta pitch 
    plot(imu_deltapitch.Time,imu_deltapitch.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[deg]');title('\Delta pitch');
    g(3)=subplot(313); %delta yaw 
    plot(imu_yaw.Time,imu_yaw.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[deg]');title('Yaw');
    linkaxes(g,'x');clear g;
    

    %     Causale
    imu_deltaroll_filt=timeseries(wrapToPi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltaroll.Data))),imu_deltaroll.Time);
    imu_deltapitch_filt=timeseries(wrapToPi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltapitch.Data))),imu_deltapitch.Time);
    % Acausale
    imu_deltaroll_filtfilt=timeseries(wrapToPi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltaroll.Data))),imu_deltaroll.Time);
    imu_deltapitch_filtfilt=timeseries(wrapToPi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltapitch.Data))),imu_deltapitch.Time);
    
    %     Figura paragone dati originali con dati filtrati
    figure('name','imu filtered roll,pitch');
    g(1)=subplot(211); %Delta Roll
    plot(imu_deltaroll.Time,imu_deltaroll.Data*180/pi,'b');grid on;hold on;
    plot(imu_deltaroll_filt.Time,imu_deltaroll_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(imu_deltaroll_filtfilt.Time,imu_deltaroll_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    title('\Delta roll');
    axis([imu_deltaroll.Time(1),imu_deltaroll.Time(end),-5,+5])
    g(2)=subplot(212); %ay
    plot(imu_deltapitch.Time,imu_deltapitch.Data*180/pi,'b');grid on;hold on;
    plot(imu_deltapitch_filt.Time,imu_deltapitch_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(imu_deltapitch_filtfilt.Time,imu_deltapitch_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    title('\Delta pitch');
    axis([imu_deltapitch.Time(1),imu_deltapitch.Time(end),-5,+5]);
    legend('data','causal','acausal');
    linkaxes(g,'x');clear g;
end

%% imu angular velocity

if imu_data_active
    %IMU ANGULAR VELOCITY: ruoto,come per le acc, dell'opposto degli angoli
    %forniti dal giroscopio (X,Y come convenzione europea).
    eul=quat2eul(imu_data_orientation.Data); %[yaw,pitch,roll] in rad
    q_imu=eul2quat([zeros(length(eul),1),eul(:,[2,3])]);
    imu_data_angular_vel_rot=timeseries(quatrotate(quatinv(q_imu),imu_data_angular_vel.Data),imu_data_angular_vel.Time);
    
    fig_imu_ang_vel=figure('name','imu angular velocity rot');
    g(1)=subplot(311); %wx
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,1),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[rad/s]');title('\omega_x');
    g(2)=subplot(312); %wy
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[rad/s]');title('\omega_y');
    g(3)=subplot(313); %wz
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,3),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[rad/s]');title('\omega_z');
    linkaxes(g,'x');clear g;  
    
    
     %     Causale
    wx_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,1)),imu_data_angular_vel_rot.Time);
    wy_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,2)),imu_data_angular_vel_rot.Time);
    wz_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,3)),imu_data_angular_vel_rot.Time);
    % Acausale
    wx_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,1)),imu_data_angular_vel_rot.Time);
    wy_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,2)),imu_data_angular_vel_rot.Time);
    wz_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,3)),imu_data_angular_vel_rot.Time);
    
    %     Figura paragone dati originali con dati filtrati
    figure('name','imu filtered ang vel');
    g(1)=subplot(311); %wx
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,1),'b');grid on;hold on;
    plot(wx_filt.Time,wx_filt.Data,'g','linewidth',LineWidth);
    plot(wx_filtfilt.Time,wx_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[rad/s]');
    title('w_x');
    axis([wx_filt.Time(1),wx_filt.Time(end),-1,+1])
    g(2)=subplot(312); %wy
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,2),'b');grid on;hold on;
    plot(wy_filt.Time,wy_filt.Data,'g','linewidth',LineWidth);
    plot(wy_filtfilt.Time,wy_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[rad/s]');
    title('w_y');
    axis([wx_filt.Time(1),wx_filt.Time(end),-1,+1]);
    g(3)=subplot(313);%wz
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,3),'b');grid on;hold on;
    plot(wz_filt.Time,wz_filt.Data,'g','linewidth',LineWidth);
    plot(wz_filtfilt.Time,wz_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[rad/s]');
    title('w_z');
    axis([wx_filt.Time(1),wx_filt.Time(end),-3,+3]);
    legend('data','causal','acausal');
    linkaxes(g,'x');clear g;
end

%% imu linear acc
if imu_data_active
    
    % ruoto le accelerazioni secondo l'opposto degli angoli di pitch e roll forniti dal
    % giroscopio
    eul=quat2eul(imu_data_orientation.Data); %[yaw,pitch,roll]
    q_imu=eul2quat([zeros(length(eul),1),eul(:,[2,3])]);
    imu_data_linear_acc_rot=timeseries(quatrotate(quatinv(q_imu),imu_data_linear_acc.Data),imu_data_linear_acc.Time);
    
    fig_imu_lin_acc=figure('name','imu rotated accelerations');
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
    
%     Filtraggio causale:
    ax_rot_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,1)),imu_data_linear_acc_rot.Time); %[m/s^2]
    ay_rot_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,2)),imu_data_linear_acc_rot.Time); %[m/s^2]
    az_rot_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,3)),imu_data_linear_acc_rot.Time); %[m/s^2]
%         Filtraggio acausale:
    ax_rot_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,1)),imu_data_linear_acc_rot.Time); %[m/s^2]
    ay_rot_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,2)),imu_data_linear_acc_rot.Time); %[m/s^2]
    az_rot_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,3)),imu_data_linear_acc_rot.Time); %[m/s^2]
    
%     smoothing
    ax_rot_smooth=timeseries(smooth(imu_data_linear_acc_rot.Data(:,1),Nsmooth),imu_data_linear_acc_rot.Time);
    ay_rot_smooth=timeseries(smooth(imu_data_linear_acc_rot.Data(:,2),Nsmooth),imu_data_linear_acc_rot.Time);
    az_rot_smooth=timeseries(smooth(imu_data_linear_acc_rot.Data(:,3),Nsmooth),imu_data_linear_acc_rot.Time);

    
%     Figura paragone dati originali con dati filtrati
    fig_imu_lin_acc_filt=figure('name','imu rotated filtered accelerations');
    g(1)=subplot(311); %ax
    plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,1),'b');grid on;hold on;
    plot(ax_rot_filt.Time,ax_rot_filt.Data,'g','linewidth',LineWidth);
    plot(ax_rot_filtfilt.Time,ax_rot_filtfilt.Data,'r','linewidth',LineWidth);
    plot(ax_rot_smooth.Time,ax_rot_smooth.Data,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[m/s^2]');
    title('a_x');
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),-5,+5])
    g(2)=subplot(312); %ay
    plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,2),'b');grid on;hold on;
    plot(ay_rot_filt.Time,ay_rot_filt.Data,'g','linewidth',LineWidth);
    plot(ay_rot_filtfilt.Time,ay_rot_filtfilt.Data,'r','linewidth',LineWidth);
    plot(ay_rot_smooth.Time,ay_rot_smooth.Data,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[m/s^2]');
    title('a_y');
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),-5,+5])
    g(3)=subplot(313); %az
    plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,3),'b');grid on;hold on;
    plot(az_rot_filt.Time,az_rot_filt.Data,'g','linewidth',LineWidth);
    plot(az_rot_filtfilt.Time,az_rot_filtfilt.Data,'r','linewidth',LineWidth);
    plot(az_rot_smooth.Time,az_rot_smooth.Data,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[m/s^2]');legend('rot data','causal','acausal','smooth');
    title('a_z');    
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),8,+11])
    linkaxes(g,'x');clear g;

end
    
%% figure composte imu
 if imu_data_active   
    
    %     figura deltapitch+ax,deltaroll+ay
    figure('name','ax, delta pitch');
    plot(ax_rot_filtfilt.Time,ax_rot_filtfilt.Data,'r','linewidth',LineWidth);grid on;hold on;
    plot(imu_deltapitch_filtfilt.Time,imu_deltapitch_filtfilt.Data*180/pi,'k','linewidth',LineWidth);
    xlabel('t[s]');legend('a_x [m/s^2]','\Delta pitch [deg]');
    axis([imu_deltapitch.Time(1),imu_deltapitch.Time(end),-5,+5]);
    
    
    %     yaw rate, roll, ay
    figure('name','r,ay,roll');
    plot(ay_rot_filtfilt.Time,ay_rot_filtfilt.Data,'r','linewidth',LineWidth);grid on;hold on;
    plot(imu_deltaroll_filtfilt.Time,imu_deltaroll_filtfilt.Data*180/pi,'k','linewidth',LineWidth);
    plot(wz_filtfilt.Time,wz_filtfilt.Data,'b','linewidth',LineWidth);
    xlabel('t[s]');
    legend('a_y [m/s^2]','\Delta roll [deg]','r [rad/s]');
    axis([ay_rot_filtfilt.Time(1),ay_rot_filtfilt.Time(end),-4,+4]);
    
end

%% imu mag

if imu_mag_active
    %IMU MAG
    fig_imu_mag=figure('name','imu mag');
    g(1)=subplot(411); %mag x
    plot(imu_mag.Time,imu_mag.Data(:,1),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');title('x');
    g(2)=subplot(412); %mag y
    plot(imu_mag.Time,imu_mag.Data(:,2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');title('y');
    g(3)=subplot(413); %mag z
    plot(imu_mag.Time,imu_mag.Data(:,3),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');title('z');
    g(4)=subplot(414); %messages frequency
    plot(imu_mag.Time(2:end),diff(imu_mag.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
    linkaxes(g,'x');clear g;
end
%% imu temperature
if imu_temperature_active
    %IMU TEMPERATURE
    fig_imu_temp=figure('name','imu temperature');
    g(1)=subplot(211);
    plot(imu_temperature.Time,imu_temperature.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    title('t[s]');ylabel('[°]');title('temperature')
    g(2)=subplot(212);
    plot(imu_temperature.Time(2:end),diff(imu_temperature.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
    linkaxes(g,'x');clear g;
    
end

%% radio topics
if radio_cmd_active
    %RADIO CMDS
    fig_radio_cmd=figure('name','radio cmd');
    g(1)=subplot(411); %speed ref
    plot(radio_cmd_speed_ref.Time,radio_cmd_speed_ref.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');title('speed ref');
    g(2)=subplot(412); %steer ref
    plot(radio_cmd_steer_ref.Time,radio_cmd_steer_ref.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[deg]');title('steer ref');
    g(3)=subplot(413); %control state
    plot(radio_cmd_control_state.Time,radio_cmd_control_state.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');title('radio cmd control state');
    g(4)=subplot(414); %messages frequencies
    plot(radio_cmd_control_state.Time(2:end),diff(radio_cmd_control_state.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
    linkaxes(g,'x');clear g;
end

%% optitrack pose,yaw

if car_pose_active
    %CAR POSE
    fig_car_pose=figure('name','car pose original');
    g(1)=subplot(211); 
    plot(car_pose,'linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[m]');title('opt car pose');
    legend('X','Y','Z');
    axis([car_pose.Time(1),car_pose.Time(end),-4,+4]);
    g(2)=subplot(212);
    plot(car_pose.Time(2:end),diff(car_pose.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[ms]');title('messages freq');
    axis([car_pose.Time(2),car_pose.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    %CAR POSE YAW
    car_pose_orientation_euler =timeseries(quat2eul(car_pose_orientation.Data),car_pose_orientation.Time);
    
    fig_car_pose_orient_euler=figure('name','car pose yaw');
    g(1)=subplot(211); %Z yaw
    plot(car_pose_orientation_euler.Time,wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1)))*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    title('t[s]');ylabel('[deg]');
    axis([car_pose.Time(1),car_pose.Time(end),0 360]);
    g(2)=subplot(212);
    plot(car_pose_orientation_euler.Time(2:end),diff(car_pose_orientation_euler.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[ms]');title('messages freq');
    axis([car_pose.Time(2),car_pose.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
end


%% opt pose trajectory
if car_pose_active
    %     TRAJECTORY XY car pose
    fig_XY=figure('name','opt XY original');
    plot(car_pose.Data(:,1),car_pose.Data(:,2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('X[m]');ylabel('Y[m]');
    axis square
    
%     Trajectory in 3D (time z axis)
    figure('name','3D traj');
    plot3(car_pose.Data(:,1),car_pose.Data(:,2),car_pose.Time);
    xlabel('X[m]');ylabel('Y[m]');zlabel('t[s]');
    
    if comet_XY_active
        %        comet(X,Y,p) uses a comet of length p*length(Y).  Default is p = 0.10.
        figure('name','opt comet XY');
        comet(car_pose.Data(:,1),car_pose.Data(:,2),0.1);
        xlabel('X[m]');ylabel('Y[m]');
        
        figure('name','3D traj');
        comet3(car_pose.Data(:,1),car_pose.Data(:,2),car_pose.Time);
        xlabel('X[m]');ylabel('Y[m]');zlabel('t[s]');
    end
end    
    
%% V,gamma,beta computation from optitrack (original data)

if car_pose_active
    car_pose_orientation_euler =timeseries(quat2eul(car_pose_orientation.Data),car_pose_orientation.Time);

%    Vx,Vy using the opt pose topic
    dX=diff(car_pose.Data(:,1));
    dY=diff(car_pose.Data(:,2));
    dt=diff(car_pose.Time);
    Vx_opt=dX./dt; %[m/s]
    Vy_opt=dY./dt; %[m/s]
    V_opt=timeseries([Vx_opt,Vy_opt],car_pose.Time(2:end));
    
    
    fig_V=figure('name','V original opt data');
    g(1)=subplot(211);
    plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m/s]');title('V opt');
    axis([V_opt.Time(1),V_opt.Time(end),0,5]);
    g(2)=subplot(212);
    plot(V_opt.Time,dt*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
    axis([car_pose.Time(2),car_pose.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    % gamma, theta,beta optitrack
    gamma_opt=timeseries(wrapTo2Pi(unwrap(atan2(dY,dX))),car_pose.Time(2:end)); %[rad] from 0 to 2pi
    theta_opt=timeseries(wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta),car_pose_orientation_euler.Time); %[rad] from 0 to 2pi
    beta_opt=timeseries(wrapToPi(unwrap(gamma_opt.Data)-unwrap(theta_opt.Data(2:end))),gamma_opt.Time);%[rad] from -pi to +pi
    
    figure('name','opt angles original');
    g(1)=subplot(311); %gamma,theta
    plot(gamma_opt.Time,gamma_opt.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
    plot(theta_opt.Time,theta_opt.Data*180/pi,'g','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[deg]');title('\gamma,\theta');
    legend('\gamma opt','\theta opt');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0,360]);
    g(2)=subplot(312); %beta opt
    plot(beta_opt.Time,beta_opt.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[deg]');title('\beta');
    axis([beta_opt.Time(1),beta_opt.Time(end),-50,+50]);
    g(3)=subplot(313); %dt
    plot(gamma_opt.Time(2:end),diff(gamma_opt.Time)*1000,'b','linewidth',1.5);grid on;
    xlabel('t[s]');ylabel('[ms]');title('messages freq');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    figure('name','V,beta original');
    g(1)=subplot(211);
    plot(beta_opt.Time,beta_opt.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[deg]');title('\beta opt');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),-50,+50]);
    g(2)=subplot(212);
    plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m/s]');title('V opt');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0,5]);
    linkaxes(g,'x');clear g;
    
    
end

%% tolgo dati a V=0 e V troppo grande dai dati optitrack (x,y,theta)

if car_pose_active
    
    V=sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2); %[m/s]
    indexes_below_Vmin=find(V < Vmin);
    indexes_above_Vmax=find(V > Vmax);
    indexes_removed=sort([1;indexes_below_Vmin+1;indexes_above_Vmax+1]);
    
    car_pose_buchi=delsample(car_pose,'Index',indexes_removed);
    car_pose_orientation_euler_buchi=delsample(car_pose_orientation_euler,'Index',indexes_removed);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %     interpolo dati gps bucati con spline
    x_opt_spline=spline(car_pose_buchi.Time,car_pose_buchi.Data(:,1)); %[m]
    y_opt_spline=spline(car_pose_buchi.Time,car_pose_buchi.Data(:,2)); %[m]
    dx_opt_spline=fnder(x_opt_spline);
    dy_opt_spline=fnder(y_opt_spline);
    gamma_opt_spline=timeseries(wrapTo2Pi(unwrap(atan2(ppval(car_pose_buchi.Time,dy_opt_spline),ppval(car_pose_buchi.Time,dx_opt_spline)))),car_pose_buchi.Time); %[rad] from 0 to 2pi
    beta_opt_spline=timeseries(wrapToPi(unwrap(gamma_opt_spline.Data)-(unwrap(car_pose_orientation_euler_buchi.Data(:,1))+delta_theta)),gamma_opt_spline.Time);
    
    %     ricalcolo V_opt
    dX=diff(car_pose_buchi.Data(:,1));
    dY=diff(car_pose_buchi.Data(:,2));
    dt=diff(car_pose_buchi.Time);
    Vx_opt=dX./dt; %[m/s]
    Vy_opt=dY./dt; %[m/s]
    V_opt=timeseries(sqrt(Vx_opt.^2+Vy_opt.^2),car_pose_buchi.Time(2:end));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % buchi gps car pose
    fig_buchiGPS=figure('name','buchi GPS xy');
    g(1)=subplot(311); %Xopt
    plot(car_pose_buchi.Time,car_pose_buchi.Data(:,1),'bo-','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m]');title('X GPS');
    g(2)=subplot(312); %Yopt
    plot(car_pose_buchi.Time,car_pose_buchi.Data(:,2),'bo-','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m]');title('Y GPS');
    g(3)=subplot(313); %dt msgs opt
    plot(car_pose_buchi.Time(2:end),diff(car_pose_buchi.Time)*1000,'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[ms]');title('msgs dt');
    axis([car_pose_buchi.Time(1),car_pose_buchi.Time(end),5,100]);
    linkaxes(g,'x');clear g;
    
    % velocità
    figure('name','V no buchi');
    g(1)=subplot(211);
    plot(V_opt.Time,V_opt.Data,'bo-','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m/s]');title('V_{opt}');
    g(2)=subplot(212);
    plot(V_opt.Time(2:end),diff(V_opt.Time)*1000,'bo-','linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[ms]');title('msgs dt');
    linkaxes(g,'x');clear g;
    
    
    
    % traiettoria con buchi
    fig_XY=figure('name','opt XY con buchi');
    plot(car_pose_buchi.Data(:,1),car_pose_buchi.Data(:,2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('X[m]');ylabel('Y[m]');
    axis square
    
    figure('name','3D traj buchi');
    plot3(car_pose_buchi.Data(:,1),car_pose_buchi.Data(:,2),car_pose_buchi.Time);
    xlabel('X[m]');ylabel('Y[m]');zlabel('t[s]');
    
    
    if comet_XY_active
        %        comet(X,Y,p) uses a comet of length p*length(Y).  Default is p = 0.10.
        figure('name','opt comet XY con buchi');
        comet(car_pose_buchi.Data(:,1),car_pose_buchi.Data(:,2),0.1);
        xlabel('X[m]');ylabel('Y[m]');
        
        figure('name','3D comet buchi');
        comet3(car_pose_buchi.Data(:,1),car_pose_buchi.Data(:,2),car_pose_buchi.Time);
        xlabel('X[m]');ylabel('Y[m]');zlabel('t[s]');
    end
end
    
    
%%     Gamma,theta,beta,Vopt filtrato
if car_pose_active

%     Causale
    gamma_opt_filt=timeseries(wrapTo2Pi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(gamma_opt_spline.Data))),gamma_opt_spline.Time);
    theta_opt_filt=timeseries(wrapTo2Pi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(theta_opt.Data))),theta_opt.Time);
    beta_opt_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},beta_opt_spline.Data),beta_opt_spline.Time);
    V_opt_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},V_opt.Data),V_opt.Time);
    %     Acausale
    gamma_opt_filtfilt=timeseries(wrapTo2Pi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(gamma_opt_spline.Data))),gamma_opt_spline.Time);
    theta_opt_filtfilt=timeseries(wrapTo2Pi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(theta_opt.Data))),theta_opt.Time);
    beta_opt_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},beta_opt_spline.Data),beta_opt_spline.Time);
    V_opt_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},V_opt.Data),V_opt.Time);
    
%     Gamma filtrato
    figure('name','Gamma filt no buchi');
    plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'b');grid on;hold on;
    plot(gamma_opt_filt.Time,gamma_opt_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(gamma_opt_filtfilt.Time,gamma_opt_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([gamma_opt_spline.Time(1),gamma_opt_spline.Time(end),0,360]);
    
%     Gamma smooth (moving average)
    gamma_opt_smooth=timeseries(wrapTo2Pi(smooth(unwrap(gamma_opt_spline.Data),Nsmooth)),gamma_opt_spline.Time);
    figure('name','Gamma filt,smooth no buchi');
    plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'b');grid on;hold on;
    plot(gamma_opt_filt.Time,gamma_opt_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(gamma_opt_filtfilt.Time,gamma_opt_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    plot(gamma_opt_smooth.Time,gamma_opt_smooth.Data*180/pi,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal','smooth');
    axis([gamma_opt_spline.Time(1),gamma_opt_spline.Time(end),0,360]);
    
    %     theta filtrato
    figure('name','Theta filt no buchi');
    plot(theta_opt.Time,theta_opt.Data*180/pi,'b');grid on;hold on;
    plot(theta_opt_filt.Time,theta_opt_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(theta_opt_filtfilt.Time,theta_opt_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([theta_opt.Time(1),theta_opt.Time(end),0,360]);
    
%     gamma e theta filtrato
    figure('name','gamma,Theta filt no buchi');
    plot(theta_opt.Time,theta_opt.Data*180/pi,'b-.','linewidth',LineWidth);grid on;hold on;
    plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'b-','linewidth',LineWidth);
    plot(theta_opt_filt.Time,theta_opt_filt.Data*180/pi,'g-.','linewidth',LineWidth);
    plot(gamma_opt_filt.Time,gamma_opt_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(theta_opt_filtfilt.Time,theta_opt_filtfilt.Data*180/pi,'r-.','linewidth',LineWidth);
    plot(gamma_opt_filtfilt.Time,gamma_opt_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('\theta data','\gamma data','\theta causal','\gamma causal','\theta acausal','\gamma acausal');
    axis([theta_opt.Time(1),theta_opt.Time(end),0,360]);
    
    %     beta filtrato
    figure('name','beta filt no buchi');
    plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'b');grid on;hold on;
    plot(beta_opt_filt.Time,beta_opt_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(beta_opt_filtfilt.Time,beta_opt_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([beta_opt_spline.Time(1),beta_opt_spline.Time(end),-45,45]);
    
        %     V_opt filtrato
    figure('name','V_opt filt no buchi');
    plot(V_opt.Time,V_opt.Data,'b');grid on;hold on;
    plot(V_opt_filt.Time,V_opt_filt.Data,'g','linewidth',LineWidth);
    plot(V_opt_filtfilt.Time,V_opt_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([V_opt.Time(1),V_opt.Time(end),0,3]);
    
end

%% ay,r,beta,V
if (car_pose_active && imu_data_active)
%     original (noisy) data
    figure('name','ay,r,beta,V causal');
    plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,2),'r');grid on;hold on;
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,3),'b');
    plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'g');
    plot(V_opt.Time,V_opt.Data,'k');
    xlabel('t[s]');
    legend('a_y [m/s]','r [rad/s]','\beta [deg]','V [m/s]');
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),-10,+10]);
    
%     causal filter
    figure('name','ay,r,beta,V causal');
    plot(ay_rot_filt.Time,ay_rot_filt.Data,'r','linewidth',LineWidth);grid on;hold on;
    plot(wz_filt.Time,wz_filt.Data,'b','linewidth',LineWidth);
    plot(beta_opt_filt.Time,beta_opt_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(V_opt_filt.Time,V_opt_filt.Data,'k','linewidth',LineWidth);
    xlabel('t[s]');
    legend('a_y [m/s]','r [rad/s]','\beta [deg]','V [m/s]');
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),-10,+10]);
    
    %     acausal filter
    figure('name','ay,r,beta,V acausal');
    plot(ay_rot_filtfilt.Time,ay_rot_filtfilt.Data,'r','linewidth',LineWidth);grid on;hold on;
    plot(wz_filtfilt.Time,wz_filtfilt.Data,'b','linewidth',LineWidth);
    plot(beta_opt_filtfilt.Time,beta_opt_filtfilt.Data*180/pi,'g','linewidth',LineWidth);
    plot(V_opt_filtfilt.Time,V_opt_filtfilt.Data,'k','linewidth',LineWidth);
    xlabel('t[s]');
    legend('a_y [m/s]','r [rad/s]','\beta [deg]','V [m/s]');
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),-10,+10]);
    
    
end

%% simulations of the two estimators
if sim_active
    
    %PDa 
    Tsa=1e-3; %[s]
    Kda=wca;
    Kpa=0.1*wca*Kda;
    Ta=1/(wca*10); %[s] filter on the derivative action one decade after wca
    Na=1/Ta;
    
    Num_PDa=[(Kpa*(2+Na*Tsa)+2*Kda*Na)/(2+Na*Tsa),(-2*Kpa+Na*Tsa*Kpa-2*Kda*Na)/(2+Na*Tsa)];
    Den_PDa=[1,(Na*Tsa-2)/(Na*Tsa+2)];
    
   X_opt=timeseries(car_pose.Data(:,1),car_pose.Time);
   Y_opt=timeseries(car_pose.Data(:,2),car_pose.Time);
   
   theta_opt_buchi=timeseries(car_pose_orientation_euler_buchi.Data(:,3),car_pose_orientation_euler_buchi.Time);


% setto a zero il tempo iniziale dei dati gps, altrimenti la simulazione,
% partendo da t=0, farebbe si che all'inizio la macchina fosse ferma
   T_zero=X_opt.Time(1);
   set(X_opt,'Time',X_opt.Time-T_zero); 
   set(Y_opt,'Time',Y_opt.Time-T_zero); 
   T_stop=Y_opt.Time(end);
   
%    condizioni iniziali integratori
    x0_vel=[X_opt.Data(1),Y_opt.Data(1),gamma_opt_spline.Data(1)];
    x0_acc=[X_opt.Data(1),Y_opt.Data(1),gamma_opt_spline.Data(1),V_opt.Data(1)];
   
   load_system('beta_estimators_simulink.slx');
   sim('beta_estimators_simulink.slx',T_stop);
%    close_system('beta_estimators_simulink.slx');
    
% ri-aggiungo il tempo iniziale dei dati gps al tempo del gamma stimato per poter confrontare
% direttamente dati stimatore con dati imu (il riferimento temporale
% rimane lo stesso)
    set(gamma_estimator_acceleration,'Time',gamma_estimator_acceleration.Time+T_zero);
    set(gamma_estimator_velocity,'Time',gamma_estimator_velocity.Time+T_zero);
    set(v,'Time',v.Time+T_zero);
    
end

%% Two estimators figures

if sim_active
       
    %%%%%%%%%%%%%%%figure spline fitting dei dati gps (bucati)%%%%%%%%%%%

%     fig_Xspline=figure('name','X spline');
%     g(1)=subplot(211);
%     plot(car_pose.Time,car_pose.Data(:,1),'bo-','linewidth',LineWidth);grid on;hold on;
%     plot(car_pose.Time,ppval(car_pose.Time,x_opt_spline),'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');title('X opt');
%     legend('data','spline');
%     g(2)=subplot(212);
%     plot(car_pose.Time(2:end),diff(car_pose.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
%     xlabel('t[s]');ylabel('[ms]');title('msgs dt');
%     axis([car_pose.Time(1) car_pose.Time(end) 0 20]);
%     linkaxes(g,'x');clear g;
%     
%     fig_Yspline=figure('name','Y spline');
%     g(1)=subplot(211);
%     plot(car_pose.Time,car_pose.Data(:,2),'bo-','linewidth',LineWidth);grid on;hold on;
%     plot(car_pose.Time,ppval(car_pose.Time,y_opt_spline),'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');title('Y opt');
%     legend('data','spline');
%     g(2)=subplot(212);
%     plot(car_pose.Time(2:end),diff(car_pose.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
%     xlabel('t[s]');ylabel('[ms]');title('msgs dt');
%     axis([car_pose.Time(1) car_pose.Time(end) 0 20]);
%     linkaxes(g,'x');clear g;
%     
%     fig_dXspline=figure('name','dX spline');
%     g(1)=subplot(211);
%     plot(car_pose.Time(2:end),diff(car_pose.Data(:,1))./diff(car_pose.Time),'bo-','linewidth',LineWidth);grid on;hold on;
%     plot(car_pose.Time,ppval(car_pose.Time,dx_opt_spline),'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');title('dX opt');
%     legend('data','spline');
%     g(2)=subplot(212);
%     plot(car_pose.Time(2:end),diff(car_pose.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
%     xlabel('t[s]');ylabel('[ms]');title('msgs dt');
%     axis([car_pose.Time(1) car_pose.Time(end) 0 20]);
%     linkaxes(g,'x');clear g;
%     
%     fig_dYspline=figure('name','dY spline');
%     g(1)=subplot(211);
%     plot(car_pose.Time(2:end),diff(car_pose.Data(:,2))./diff(car_pose.Time),'bo-','linewidth',LineWidth);grid on;hold on;
%     plot(car_pose.Time,ppval(car_pose.Time,dy_opt_spline),'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');title('dY opt');
%     legend('data','spline');
%     g(2)=subplot(212);
%     plot(car_pose.Time(2:end),diff(car_pose.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
%     xlabel('t[s]');ylabel('[ms]');title('msgs dt');
%     axis([car_pose.Time(1) car_pose.Time(end) 0 20]);
%     linkaxes(g,'x');clear g;
%     
%     fig_gamma_spline=figure('name','gamma opt spline');
%     plot(gamma_opt.Time,gamma_opt.Data*180/pi,'b','linewidth',LineWidth);grid on;hold on;
%     plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[deg]');title('\gamma opt');
%     legend('finite difference','spline');
%     
%     fig_debug_gamma_spline=figure('name','gamma X Y opt spline');
%     g(1)=subplot(411); %gamma
%     plot(gamma_opt.Time,gamma_opt.Data*180/pi,'b','linewidth',LineWidth);grid on;hold on;
%     plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[deg]');title('\gamma opt');
%     legend('finite difference','spline');
%     g(2)=subplot(412); %X
%     plot(car_pose.Time,car_pose.Data(:,1),'b','linewidth',LineWidth);grid on;hold on;
%     plot(car_pose.Time,ppval(car_pose.Time,x_opt_spline),'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');title('X opt');
%     g(3)=subplot(413); % Y
%     plot(car_pose.Time,car_pose.Data(:,2),'b','linewidth',LineWidth);grid on;hold on;
%     plot(car_pose.Time,ppval(car_pose.Time,y_opt_spline),'r','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');title('Y opt');
%     g(4)=subplot(414); %msgs dt
%     plot(car_pose.Time(2:end),diff(car_pose.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
%     xlabel('t[s]');ylabel('[ms]');title('msgs dt');
%     axis([car_pose.Time(1) car_pose.Time(end) 0 20]);
%     linkaxes(g,'x');clear g;
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
% %   v (stato stimatore acc) vs V_opt
%     figure('name','v,Vopt');
%     g(1)=subplot(211);
%     plot(V_opt.Time,V_opt.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
%     plot(v.time,v.data,'r--','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m/s]');
%     legend('V_{opt}','v');
%     axis([v.time(1),v.time(end),0,3]);
%     g(2)=subplot(212);
%     plot(V_opt.Time(2:end),diff(V_opt.Time)*1000,'b','linewidth',LineWidth);grid on;
%     xlabel('t[s]');ylabel('[ms]');title('dt msgs V_{opt}');
%     axis([v.time(1),v.time(end),8,15]);
%     linkaxes(g,'x');clear g;
    
    
%     GAMMA ESTIMATES WITH DT
    fig_gamma_estimators=figure('name','gamma theta estimators');
    g(1)=subplot(211); %gamma,theta
    plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'b','linewidth',LineWidth);grid on;hold on;
    plot(gamma_estimator_velocity.Time,gamma_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth);
    plot(gamma_estimator_acceleration.Time,gamma_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth);
    plot(gamma_estimator_opt.Time,gamma_estimator_opt.Data*180/pi,'c','linewidth',LineWidth);
    plot(car_pose_orientation_euler.Time,wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta)*180/pi,'g','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[deg]');title('\gamma,\theta');
    axis([gamma_opt_spline.Time(1),gamma_opt_spline.Time(end),0 360]);
    legend('\gamma opt','\gamma vel','\gamma acc','\gamma est opt','\theta opt');
    g(2)=subplot(313); %dt msgs
    plot(gamma_opt_spline.Time(2:end),diff(gamma_opt_spline.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);hold on;grid on;
    plot(gamma_estimator_velocity.Time(2:end),diff(gamma_estimator_velocity.Time)*1000,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[ms]');title('messages freq');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    %BETA ESTIMATORS
%     devo interpolare i dati di theta opt nei tempi delle stime di gamma
%     per poter calcolare beta: Vq = interp1(X,V,Xq,'spline')
%     velocity estimator
    theta_opt_interp_vel=interp1(car_pose_orientation_euler.Time,unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta,gamma_estimator_velocity.Time,'spline'); %[rad] unwrapped
    beta_estimator_velocity=timeseries(wrapToPi(unwrap(gamma_estimator_velocity.Data)-theta_opt_interp_vel),gamma_estimator_velocity.Time); %[rad] from -pi to pi
%     acceleration estimator
    theta_opt_interp_acc=interp1(car_pose_orientation_euler.Time,unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta,gamma_estimator_acceleration.Time,'spline'); %[rad] unwrapped
    beta_estimator_acceleration=timeseries(wrapToPi(unwrap(gamma_estimator_acceleration.Data)-theta_opt_interp_acc),gamma_estimator_acceleration.Time); %[rad] from -pi to pi
       
    
%     BETA ESTIMATES
    fig_beta_estimators=figure('name','beta');
    g(1)=subplot(211);
    plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
    plot(beta_estimator_velocity.Time,beta_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    plot(beta_estimator_acceleration.Time,beta_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[deg]');title('\beta');
    legend('\beta opt spline','\beta vel','\beta acc');
    axis([beta_opt_spline.Time(1),beta_opt_spline.Time(end),-35,+35]);
    g(2)=subplot(212); %dt msgs
    plot(beta_opt_spline.Time(2:end),diff(beta_opt_spline.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);hold on;grid on;
    plot(gamma_estimator_velocity.Time(2:end),diff(gamma_estimator_velocity.Time)*1000,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[ms]');title('messages freq');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    
    %     graph without subplot dt window
    fig_gamma_estimators_nodt=figure('name','est gamma no dt');
    plot(gamma_opt_spline.Time,gamma_opt_spline.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
    plot(gamma_estimator_velocity.Time,gamma_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    plot(gamma_estimator_acceleration.Time,gamma_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
    plot(car_pose_orientation_euler.Time,wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta)*180/pi,'g','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[deg]');title('\gamma,\theta');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0 360]);
    legend('\gamma opt','\gamma vel','\gamma acc','\theta opt');
    
    fig_beta_estimators_nodt=figure('name','vel beta no dt');
    plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
    plot(beta_estimator_velocity.Time,beta_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    plot(beta_estimator_acceleration.Time,beta_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[deg]');title('\beta');
    legend('\beta opt','\beta vel','\beta acc');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),-35,+35]);
    
    %     beta with V, no dt
    fig_Vbeta_estimators_nodt=figure('name','beta,V no dt');
    g(1)=subplot(211); %beta
    plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
    plot(beta_estimator_velocity.Time,beta_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    plot(beta_estimator_acceleration.Time,beta_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[deg]');title('\beta');
    legend('\beta opt','\beta vel','\beta acc');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),-35,+35]);
    g(1)=subplot(212); %V
    plot(V_opt.Time,V_opt.Data,'m.-','linewidth',LineWidth,'MarkerSize',MarkerSize*5);grid on;
    xlabel('t[s]');ylabel('[m/s]');title('V opt');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0,5]);
    linkaxes(g,'x');clear g;
    
    fig_rVbeta=figure('name','ay,V,beta');
    g(1)=subplot(211); %imu ay,V_opt
    plot(ay_imu_filt.Time,ay_imu_filt.Data,'r','linewidth',LineWidth);grid on;hold on;
    plot(V_opt.Time,V_opt.Data(:,1),'b','linewidth',LineWidth);grid on;
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,3),'k','linewidth',LineWidth);
    xlabel('t[s]');legend('a_y [m/s^2]','V opt [m/s]','r [rad/s]');
    axis([ay_imu_filt.Time(1),ay_imu_filt.Time(end),-5,+5])
    g(2)=subplot(212); %beta
    plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'b','linewidth',LineWidth);grid on;hold on;
    plot(beta_estimator_velocity.Time,beta_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth);
    plot(beta_estimator_acceleration.Time,beta_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\beta');
    legend('\beta opt','\beta vel','\beta acc');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),-35,+35]);
    linkaxes(g,'x');clear g;
    
end

%% estimators figures with causal filter
if sim_active

%    Beta
    %BETA ESTIMATORS
    %     devo interpolare i dati di theta opt nei tempi delle stime di gamma
    %     per poter calcolare beta: Vq = interp1(X,V,Xq,'spline')
    
    %     velocity estimator
    theta_opt_interp_vel=interp1(car_pose_orientation_euler.Time,unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta,gamma_estimator_velocity.Time,'previous'); %[rad] unwrapped
    beta_estimator_velocity=timeseries(wrapToPi(unwrap(gamma_estimator_velocity.Data)-theta_opt_interp_vel),gamma_estimator_velocity.Time); %[rad] from -pi to pi
    %     acceleration estimator
    theta_opt_interp_acc=interp1(car_pose_orientation_euler.Time,unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta,gamma_estimator_acceleration.Time,'previous'); %[rad] unwrapped
    beta_estimator_acceleration=timeseries(wrapToPi(unwrap(gamma_estimator_acceleration.Data)-theta_opt_interp_acc),gamma_estimator_acceleration.Time); %[rad] from -pi to pi
    
    
    %     BETA ESTIMATES
    fig_beta_estimators=figure('name','beta');
    g(1)=subplot(211);
    plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
    plot(beta_estimator_velocity.Time,filter(FilterDTF.Num{1},FilterDTF.Den{1},beta_estimator_velocity.Data)*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    plot(beta_estimator_acceleration.Time,filter(FilterDTF.Num{1},FilterDTF.Den{1},beta_estimator_acceleration.Data)*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[deg]');title('\beta');
    legend('\beta opt spline','\beta vel','\beta acc');
    axis([beta_opt_spline.Time(1),beta_opt_spline.Time(end),-35,+35]);
    g(2)=subplot(212); %dt msgs
    plot(beta_opt_spline.Time(2:end),diff(beta_opt_spline.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);hold on;grid on;
    plot(gamma_estimator_velocity.Time(2:end),diff(gamma_estimator_velocity.Time)*1000,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
    xlabel('t[s]');ylabel('[ms]');title('messages freq');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    %     graph without subplot dt window
    
%     fig_gamma_estimators_nodt=figure('name','est gamma no dt');
%     plot(gamma_opt_spline.Time,wrapTo2Pi(filtfilt(Fd.Num{1},Fd.Den{1},unwrap(gamma_opt_spline.Data)))*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
%     plot(gamma_estimator_velocity.Time,gamma_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
%     plot(gamma_estimator_acceleration.Time,gamma_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
%     plot(car_pose_orientation_euler.Time,wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta)*180/pi,'g','linewidth',LineWidth,'MarkerSize',MarkerSize);
%     xlabel('t[s]');ylabel('[deg]');title('\gamma,\theta');
%     axis([gamma_opt.Time(1),gamma_opt.Time(end),0 360]);
%     legend('\gamma opt','\gamma vel','\gamma acc','\theta opt');
    
%     fig_beta_estimators_nodt=figure('name','vel beta no dt');
%     plot(beta_opt_spline.Time,beta_opt_spline.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
%     plot(beta_estimator_velocity.Time,beta_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
%     plot(beta_estimator_acceleration.Time,beta_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
%     xlabel('t[s]');ylabel('[deg]');title('\beta');
%     legend('\beta opt','\beta vel','\beta acc');
%     axis([gamma_opt.Time(1),gamma_opt.Time(end),-35,+35]);
    
%     %     beta with V, no dt
%     fig_Vbeta_estimators_nodt=figure('name','beta,V no dt');
%     g(1)=subplot(211); %beta
%     plot(beta_opt_spline.Time,wrapToPi(filtfilt(Fd.Num{1},Fd.Den{1},unwrap(beta_opt_spline.Data)))*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
%     plot(beta_estimator_velocity.Time,beta_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth,'MarkerSize',MarkerSize);
%     plot(beta_estimator_acceleration.Time,beta_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth,'MarkerSize',MarkerSize);
%     xlabel('t[s]');ylabel('[deg]');title('\beta');
%     legend('\beta opt','\beta vel','\beta acc');
%     axis([gamma_opt.Time(1),gamma_opt.Time(end),-35,+35]);
%     g(1)=subplot(212); %V
%     plot(V_opt.Time,V_opt.Data,'m.-','linewidth',LineWidth,'MarkerSize',MarkerSize*5);grid on;
%     xlabel('t[s]');ylabel('[m/s]');title('V opt');
%     axis([gamma_opt.Time(1),gamma_opt.Time(end),0,5]);
%     linkaxes(g,'x');clear g;
%     
%     fig_rVbeta=figure('name','ay,V,beta');
%     g(1)=subplot(211); %imu ay,V_opt
%     plot(ay_imu_filt.Time,ay_imu_filt.Data,'r','linewidth',LineWidth);grid on;hold on;
%     plot(V_opt.Time,V_opt.Data(:,1),'b','linewidth',LineWidth);grid on;
%     plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,3),'k','linewidth',LineWidth);
%     xlabel('t[s]');legend('a_y [m/s^2]','V opt [m/s]','r [rad/s]');
%     axis([ay_imu_filt.Time(1),ay_imu_filt.Time(end),-5,+5])
%     g(2)=subplot(212); %beta
%     plot(beta_opt_spline.Time,wrapToPi(filtfilt(Fd.Num{1},Fd.Den{1},unwrap(beta_opt_spline.Data)))*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;hold on;
%     plot(beta_estimator_velocity.Time,beta_estimator_velocity.Data*180/pi,'r','linewidth',LineWidth);
%     plot(beta_estimator_acceleration.Time,beta_estimator_acceleration.Data*180/pi,'k','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[deg]');title('\beta');
%     legend('\beta opt','\beta vel','\beta acc');
%     axis([gamma_opt.Time(1),gamma_opt.Time(end),-35,+35]);
%     linkaxes(g,'x');clear g;
end




