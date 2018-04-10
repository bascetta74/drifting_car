clear all;close all;clc;

%% PARAMETERS

filename='recorded_data';

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

%     Filtro dati imu
wf=10*2*pi; %[rad/s]
n=1; %filter order
Ts=0.001; %[s]

Nsmooth=10;

% Limiti rimozione buchi optitrack:
Vmin=0.1; %[m/s]
Vmax=4; %[m/s]

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
FilterCTF=1/(1+s/wf)^n;
FilterDTF=c2d(FilterCTF,Ts,'tustin');


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

%% Confronto Dati Opt calcolati qui con quelli dello stimatore in ROS

if state_estimator_opt_active
    
    figure('name','opt beta est');
    g(1)=subplot(211); %beta opt
    plot(beta_opt.Time,beta_opt.Data*180/pi,'r--','linewidth',LineWidth);grid on;hold on;
    plot(state_estimator_opt_beta.Time,state_estimator_opt_beta.Data*180/pi,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\beta');
    axis([beta_opt.Time(1),beta_opt.Time(end),-50,+50]);
    legend('matlab','ROS node');
    g(2)=subplot(212); %dt
    plot(gamma_opt.Time(2:end),diff(gamma_opt.Time)*1000,'b','linewidth',1.5);grid on;
    xlabel('t[s]');ylabel('[ms]');title('messages freq');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    figure('name','V,beta opt');
    g(1)=subplot(211);
    plot(beta_opt.Time,beta_opt.Data*180/pi,'r--','linewidth',LineWidth);grid on;hold on;
    plot(state_estimator_opt_beta.Time,state_estimator_opt_beta.Data*180/pi,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\beta opt');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),-50,+50]);
    legend('matlab','ROS node');
    g(2)=subplot(212);
    plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m/s]');title('V opt');
    axis([gamma_opt.Time(1),gamma_opt.Time(end),0,5]);
    linkaxes(g,'x');clear g;
    
%     figure('name','dX dY opt');
%     epsilon=1e-3; %[m]
%     g(1)=subplot(311);%dX
%     plot(car_pose.Time(2:end),diff(car_pose.Data(:,1)),'b','linewidth',LineWidth);grid on;hold on;
%     plot([car_pose.Time(2),car_pose.Time(end)],[1,1]*epsilon,'m--','linewidth',LineWidth);
%     plot([car_pose.Time(2),car_pose.Time(end)],-[1,1]*epsilon,'m--','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');
%     title('dX opt');
%     axis([car_pose.Time(2),car_pose.Time(end),-0.025,+0.025])
%     g(2)=subplot(312);%dY
%     plot(car_pose.Time(2:end),diff(car_pose.Data(:,2)),'b','linewidth',LineWidth);grid on;hold on;
%     plot([car_pose.Time(2),car_pose.Time(end)],[1,1]*epsilon,'m--','linewidth',LineWidth);
%     plot([car_pose.Time(2),car_pose.Time(end)],-[1,1]*epsilon,'m--','linewidth',LineWidth);
%     xlabel('t[s]');ylabel('[m]');
%     title('dY opt');
%     axis([car_pose.Time(2),car_pose.Time(end),-0.025,+0.025])
%     g(3)=subplot(313);%dt
%     plot(car_pose.Time(2:end),diff(car_pose.Time)*1000,'r','linewidth',LineWidth);grid on;
%     xlabel('t[s]');ylabel('[ms]');title('dt');
%     axis([car_pose.Time(2),car_pose.Time(end),5,12]);
%     linkaxes(g,'x');clear g;
%     
%     figure('name','d(dY) opt');
%     plot(car_pose.Time(3:end),diff(diff(car_pose.Data(:,2))),'b-d','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
%     xlabel('t[s]');ylabel('[m]');title('d (dY) opt');
%     axis([car_pose.Time(3),car_pose.Time(end),-0.01,0.01]);
    
    figure('name','V');
    plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'r','linewidth',LineWidth);grid on;hold on;
    plot(state_estimator_opt_V.Time,state_estimator_opt_V.Data,'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[m/s]');title('V');
    legend('Matlab','ROS node');
    axis([V_opt.Time(1),V_opt.Time(end),0,4]);
    
    figure('name','dS opt');
    ds=timeseries(sqrt((diff(car_pose.Data(:,1))).^2+(diff(car_pose.Data(:,2))).^2),car_pose.Time(2:end));
    plot(ds,'linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[m]');title('ds');
    axis([ds.Time(1),ds.Time(end),0,0.05]);
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

%% Fitting Circonferenza
% Parametri
tstart=6; %[s]
tend=10.2; %[s]
% Estraggo dati traiettoria
index_start=find(car_pose_buchi.Time>tstart,1);
index_end=find(car_pose_buchi.Time>tend,1);
circonferenza=getsamples(car_pose_buchi,index_start:index_end);
Vcirc=getsamples(V_opt,index_start-1:index_end-1);
beta_opt_filtfilt_circ=getsamples(beta_opt_filtfilt,index_start-1:index_end-1);

index_start_imu=find(ay_rot_filtfilt.Time>tstart,1);
index_end_imu=find(ay_rot_filtfilt.Time>tend,1);
ay_rot_filtfilt_circ=getsamples(ay_rot_filtfilt,index_start_imu:index_end_imu);
wz_filtfilt_circ=getsamples(wz_filtfilt,index_start_imu:index_end_imu);

index_start_cmd=find(radio_cmd_speed_ref.Time>tstart,1);
index_end_cmd=find(radio_cmd_speed_ref.Time>tend,1);
radio_cmd_speed_ref_circ=getsamples(radio_cmd_speed_ref,index_start_cmd:index_end_cmd);
radio_cmd_steer_ref_circ=getsamples(radio_cmd_steer_ref,index_start_cmd:index_end_cmd);

% Visualizzo la porzione di traiettoria seguita
figure('name','Traj selezionata circ fit');
comet(circonferenza.Data(:,1),circonferenza.Data(:,2))
xlabel('X[m]');ylabel('Y[m]');

% Fitto circonferenza a dati traiettoria
XY=circonferenza.Data(:,[1,2]);
Par = CircleFitByPratt(XY); %Par=[a b R] is the fitting circle:
%                           center (a,b) and radius R

% Grafico confronto dati con circonferenza
theta_circ=0:0.01:2*pi; %[rad]
Xcirc=Par(1)+Par(3)*cos(theta_circ);%[m]
Ycirc=Par(2)+Par(3)*sin(theta_circ);%[m]
figure('name','circ fit');
plot(circonferenza.Data(:,1),circonferenza.Data(:,2),'b','linewidth',LineWidth);grid on;hold on;axis square;
plot(Xcirc,Ycirc,'r','linewidth',LineWidth);
xlabel('X[m]');ylabel('Y[m]');
legend('data','Fitted Circ');
titolo=['Fitted Circ: C [',num2str(Par(1)),',',num2str(Par(2)),'],R=',num2str(Par(3)),' m'];
title(titolo);

% grafico V,ay,r,delta_roll
ayref= (mean(Vcirc.Data))^2/Par(3); %[m/s^2]
ay_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc.Data(:,2)),imu_data_linear_acc.Time); % acc non ruotata

figure('name','V,ay,r circ fit');
g(1)=subplot(211);
plot(Vcirc.Time,Vcirc.Data,'b','linewidth',LineWidth);grid on;hold on;
plot(ay_rot_filtfilt_circ.Time,abs(ay_rot_filtfilt_circ.Data),'r','linewidth',LineWidth);
plot([ay_rot_filtfilt_circ.Time(1),ay_rot_filtfilt_circ.Time(end)],[1,1]*abs(mean(ay_rot_filtfilt_circ.Data)),'c--','linewidth',LineWidth);
plot([ay_rot_filtfilt_circ.Time(1),ay_rot_filtfilt_circ.Time(end)],[1,1]*ayref,'g--','linewidth',LineWidth);
plot(ay_filtfilt.Time,ay_filtfilt.Data,'m','linewidth',LineWidth);
plot(wz_filtfilt_circ.Time,abs(wz_filtfilt_circ.Data),'k','linewidth',LineWidth);
xlabel('t[s]');ylabel('[m/s]');
legend('V [m/s]','a_y [m/s^2]','a_y rot mean','a_y ref','a_y unrot','r [rad/s]');
axis([Vcirc.Time(1),Vcirc.Time(end),0,5]);
g(2)=subplot(212); %roll,beta
plot(imu_deltaroll.Time,imu_deltaroll.Data*180,'y','linewidth',LineWidth);grid on;hold on;
plot(beta_opt_filtfilt_circ.Time,beta_opt_filtfilt_circ.Data*180/pi,'m','linewidth',LineWidth);
xlabel('t[s]');ylabel('[deg]');
legend('\Delta roll','\beta opt');
axis([Vcirc.Time(1),Vcirc.Time(end),-25,25]);
linkaxes(g,'x');clear g;

fprintf('\n Circ fit: V mean = %g m/s \n',mean(Vcirc.Data));
fprintf('\n Circ fit: ay ref = %g m/s^2 \n',ayref);
fprintf('\n Circ fit: ay measured mean = %g m/s^2 \n',mean(ay_rot_filtfilt_circ.Data));

% Histogram di ay(measured)-ay mean
figure('name','circ fit ay histogram');
histogram(ay_rot_filtfilt_circ.Data-mean(ay_rot_filtfilt_circ.Data),20,'Normalization','probability');
xlabel('error [m/s^2]');ylabel('freq');title('a_y rot filtfilt');

% figura comandi impartiti
figure('name','circ fit cmd');
g(1)=subplot(211); %speed ref
plot(radio_cmd_speed_ref_circ.Time,radio_cmd_speed_ref_circ.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');title('speed ref');
axis([radio_cmd_speed_ref_circ.Time(1),radio_cmd_speed_ref_circ.Time(end),-1,+1]);
g(2)=subplot(212); %steer ref
plot(radio_cmd_steer_ref_circ.Time,radio_cmd_steer_ref_circ.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[deg]');title('steer ref');
axis([radio_cmd_steer_ref_circ.Time(1),radio_cmd_steer_ref_circ.Time(end),-25,+25])
linkaxes(g,'x');clear g;

%% stima coppia resistente
% NOTA: assumo che la forza di drag aerodinamico sia trascurabile
% basta vedere che Fd=1/2*rho(aria)*V^2*Sf*Cx è piccolissima anche per
% Cx=0.5 e Sf=0.26^2 m^2. Risulta un ax di circa 0.1 m/s^2

tstart=2.7; %[s]
tend=3.6;%[s]

% Estraggo dati traiettoria
index_start=find(car_pose_buchi.Time>tstart,1);
index_end=find(car_pose_buchi.Time>tend,1);
trajectory=getsamples(car_pose_buchi,index_start:index_end);
Vtraj=getsamples(V_opt,index_start-1:index_end-1);
beta_opt_filtfilt_traj=getsamples(beta_opt_filtfilt,index_start-1:index_end-1);

index_start_imu=find(ay_rot_filtfilt.Time>tstart,1);
index_end_imu=find(ay_rot_filtfilt.Time>tend,1);
ay_rot_filtfilt_traj=getsamples(ay_rot_filtfilt,index_start_imu:index_end_imu);
% ay_traj=getsamples(imu_data_linear_acc,index_start_imu:index_end_imu);
wz_filtfilt_traj=getsamples(wz_filtfilt,index_start_imu:index_end_imu);
ax_rot_filtfilt_traj=getsamples(ax_rot_filtfilt,index_start_imu:index_end_imu);


index_start_cmd=find(radio_cmd_speed_ref.Time>tstart,1);
index_end_cmd=find(radio_cmd_speed_ref.Time>tend,1);
radio_cmd_speed_ref_traj=getsamples(radio_cmd_speed_ref,index_start_cmd:index_end_cmd);
radio_cmd_steer_ref_traj=getsamples(radio_cmd_steer_ref,index_start_cmd:index_end_cmd);

% figura comandi impartiti
figure('name','cmd traj');
g(1)=subplot(211); %speed ref
plot(radio_cmd_speed_ref_traj.Time,radio_cmd_speed_ref_traj.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');title('speed ref');
axis([radio_cmd_speed_ref_traj.Time(1),radio_cmd_speed_ref_traj.Time(end),-1,+1]);
g(2)=subplot(212); %steer ref
plot(radio_cmd_steer_ref_traj.Time,radio_cmd_steer_ref_traj.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[deg]');title('steer ref');
axis([radio_cmd_steer_ref_traj.Time(1),radio_cmd_steer_ref_traj.Time(end),-25,+25])
linkaxes(g,'x');clear g;

% figura ax,V
ax_mean=mean(ax_rot_filtfilt_traj.Data);%[m/s^2]
P=polyfit(Vtraj.Time,Vtraj.Data,1); %least square line
fprintf('\n Acc media da dati optitrack = %g m/s^2 \n',P(1));

figure('name','ax,V traj');
plot(ax_rot_filtfilt_traj.Time,ax_rot_filtfilt_traj.Data,'r','linewidth',LineWidth);grid on;hold on;
plot([ax_rot_filtfilt_traj.Time(1),ax_rot_filtfilt_traj.Time(end)],[1,1]*ax_mean,'m--','linewidth',LineWidth);
plot(Vtraj.Time,Vtraj.Data,'b','linewidth',LineWidth);
plot(Vtraj.Time,polyval(P,Vtraj.Time),'c--','linewidth',LineWidth);
xlabel('t[s]');
legend('a_x [m/s^2]','a_x mean','V [/s]','V least square');
axis([Vtraj.Time(1),Vtraj.Time(end),-3,+3]);


% Stimo forza resistente:
m=2; %[Kg] massa macchinina
Rw=0.049; %[m] wheel outer radius
Kt=1/340.34; %[Nm/A]
Imax=10; %[A] max motor current (set in VESC GUI)
Cm=mean(radio_cmd_speed_ref_traj.Data)*Imax*Kt; %[Nm]
tau_trasm=0.1159; %transmission gear ratio
Cml=Cm/tau_trasm; %[Nm] coppia motrice alle ruote
Fx_drag=m*P(1)-Cml/Rw; %[N]
% Cr_drag=Fx_drag*Rw-; %[Nm]
fprintf('\n ax imu mean = %g m/s^2 \n',ax_mean);
fprintf('\n Fx drag estimated = %g N \n',Fx_drag);
% fprintf('\n Cr drag estimated = %g Nm \n',Cr_drag);


% histogram ax-ax mean
figure('name','ax-ax mean traj');
histogram(ax_rot_filtfilt_traj.Data-ax_mean,20,'Normalization','probability');
xlabel('error [m/s^2]');ylabel('freq');title('a_x rot filtfilt');


% figura parametri moto laterale
figure('name','V,ay,r traj');
g(1)=subplot(211);
plot(Vtraj.Time,Vtraj.Data,'b','linewidth',LineWidth);grid on;hold on;
plot(ay_rot_filtfilt_traj.Time,ay_rot_filtfilt_traj.Data,'r','linewidth',LineWidth);
plot(wz_filtfilt_traj.Time,wz_filtfilt_traj.Data,'k','linewidth',LineWidth);
xlabel('t[s]');ylabel('[m/s]');
legend('V[m/s]','a_y [m/s^2]','r [rad/s]');
axis([Vtraj.Time(1),Vtraj.Time(end),-3,3]);
g(2)=subplot(212); %roll,beta
plot(imu_deltaroll.Time,imu_deltaroll.Data*180,'y','linewidth',LineWidth);grid on;hold on;
plot(beta_opt_filtfilt_traj.Time,beta_opt_filtfilt_traj.Data*180/pi,'m','linewidth',LineWidth);
xlabel('t[s]');ylabel('[deg]');
legend('\Delta roll','\beta opt');
axis([Vtraj.Time(1),Vtraj.Time(end),-5,5]);
linkaxes(g,'x');clear g;


%% savefig
risp=input('\n Would you like to save the figures?[y]\n');
if risp=='y'
    fprintf('\n figures are being saved...\n');
    
    if imu_data_active
%             savefig(figV,'V');
        savefig(fig_imu_orient_quat,'imu_orient_quat');
        savefig(fig_imu_orient_euler,'imu_orient_euler');
        savefig(fig_imu_ang_vel,'imu_ang_vel');
        savefig(fig_imu_lin_acc,'imu_lin_acc');
    end
    
    if imu_mag_active
       savefig(fig_imu_mag,'imu_mag'); 
    end
    
    if imu_temperature_active
       savefig(fig_imu_temp,'imu_temp'); 
    end
    
    if radio_cmd_active
        savefig(fig_radio_cmd,'radio_cmd');
    end
    
    if car_pose_active
       savefig(fig_car_pose,'car_pose'); 
       savefig(fig_car_pose_orient_euler,'car_pose_orient_euler');
       savefig(fig_XY,'XY');
       savefig(fig_V,'V');
       
       savefig(fig_Xspline,'Xspline');
       savefig(fig_Yspline,'Yspline');
       savefig(fig_dXspline,'dXspline');
       savefig(fig_dYspline,'dYspline');
       savefig(fig_gamma_spline,'gamma_spline');
       savefig(fig_debug_gamma_spline,'debug_gamma_spline');
    end
    
    if sim_active
       savefig(fig_gamma_estimators,'gamma_estimators');
       savefig(fig_beta_estimators,'beta_estimators');
       savefig(fig_gamma_estimators_nodt,'gamma_estimators_nodt');
       savefig(fig_beta_estimators_nodt,'beta_estimators_nodt');
       savefig(fig_Vbeta_estimators_nodt,'Vbeta_estimators_nodt');
       savefig(fig_rVbeta,'ayVbeta');
       savefig(fig_buchiGPS,'buchiGPS');
    end
    
    fprintf('\n completed!\n');
    close all;
end


