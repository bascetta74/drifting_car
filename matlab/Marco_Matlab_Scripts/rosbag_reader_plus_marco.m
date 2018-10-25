clear all;close all;clc;

%% PARAMETERS

filename='drifting_stabilization_differential_11_filter250rads_parquet_eq';

delta_yaw_imu=0; %[rad]
pitch0=0; %[rad]
roll0=0; %[rad]

% data filtering: filter parameters:
wf=250; %[rad/s]
n=1; %filter order
Ts=0.01; %[s]

% Optitrack gaps remotion:
ds_min=1e-3; %[m] threshold below which samples are removed

% Vmin=0.5; %[m/s]
% Vmax=2.5; %[m/s]

%%%%%%%
% graph parameters
LineWidth=1.5;
MarkerSize=5;

comet_XY_active=1;


%%%%%%%%%% for old bags:
circular_drifting_lqr_e_active=0;
circular_drifting_lqr_delta_psi_active=0;

%% Load mat file
filepath=fullfile(['~/WorkingDirectory2/Mat_Files/',filename,'.mat']);
% filepath=fullfile(['/home/marco/Dropbox/CDC2018/matlab_CEP/RC_Car_Experimental_Data/',filename,'.mat']);
load(filepath);

%% Causal filter building

s=tf('s');
FilterCTF=1/(1+s/wf)^n;
FilterDTF=c2d(FilterCTF,Ts,'tustin');

%% Computation of needed quantities

% IMU TOPICS:
if imu_data_active
    imu_data_orientation_euler=timeseries(quat2eul(imu_data_orientation.Data),imu_data_orientation.Time); %[yaw,pitch,roll] rad
    imu_deltaroll=timeseries(wrapToPi(unwrap(imu_data_orientation_euler.Data(:,3))-roll0),imu_data_orientation_euler.Time);
    imu_deltapitch=timeseries(wrapToPi(unwrap(imu_data_orientation_euler.Data(:,2))-pitch0),imu_data_orientation_euler.Time);
    imu_yaw=timeseries(wrapTo2Pi(unwrap(imu_data_orientation_euler.Data(:,1))+delta_yaw_imu),imu_data_orientation_euler.Time);
    
    %     Causale
    imu_deltaroll_filt=timeseries(wrapToPi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltaroll.Data))),imu_deltaroll.Time);
    imu_deltapitch_filt=timeseries(wrapToPi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltapitch.Data))),imu_deltapitch.Time);
    % Acausale
    imu_deltaroll_filtfilt=timeseries(wrapToPi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltaroll.Data))),imu_deltaroll.Time);
    imu_deltapitch_filtfilt=timeseries(wrapToPi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(imu_deltapitch.Data))),imu_deltapitch.Time);
    
%     angular velocity
    eul=quat2eul(imu_data_orientation.Data); %[yaw,pitch,roll] in rad
    q_imu=eul2quat([zeros(length(eul),1),eul(:,[2,3])]);
    imu_data_angular_vel_rot=timeseries(quatrotate(quatinv(q_imu),imu_data_angular_vel.Data),imu_data_angular_vel.Time);
    
     %     Causale
    wx_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,1)),imu_data_angular_vel_rot.Time);
    wy_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,2)),imu_data_angular_vel_rot.Time);
    wz_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,3)),imu_data_angular_vel_rot.Time);
    % Acausale
    wx_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,1)),imu_data_angular_vel_rot.Time);
    wy_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,2)),imu_data_angular_vel_rot.Time);
    wz_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,3)),imu_data_angular_vel_rot.Time);
    
%     acceleration
    imu_data_linear_acc_rot=timeseries(quatrotate(quatinv(q_imu),imu_data_linear_acc.Data),imu_data_linear_acc.Time);
    
    %     Filtraggio causale:
    ax_rot_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,1)),imu_data_linear_acc_rot.Time); %[m/s^2]
    ay_rot_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,2)),imu_data_linear_acc_rot.Time); %[m/s^2]
    az_rot_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,3)),imu_data_linear_acc_rot.Time); %[m/s^2]
%         Filtraggio acausale:
    ax_rot_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,1)),imu_data_linear_acc_rot.Time); %[m/s^2]
    ay_rot_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,2)),imu_data_linear_acc_rot.Time); %[m/s^2]
    az_rot_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_linear_acc_rot.Data(:,3)),imu_data_linear_acc_rot.Time); %[m/s^2]

end

%OPT TOPICS:
if car_pose_active
    car_pose_orientation_euler=timeseries(quat2eul(car_pose_orientation.Data),car_pose_orientation.Time); %[yaw,pitch,roll] [rad]
    
    % velocity computation with gaps:
    dX=diff(car_pose.Data(:,1));
    dY=diff(car_pose.Data(:,2));
    dt=diff(car_pose.Time);
    Vx_opt=dX./dt; %[m/s]
    Vy_opt=dY./dt; %[m/s]
    V_opt=timeseries([Vx_opt,Vy_opt],car_pose.Time(2:end)); %[m/s]
    
    % gamma, theta,beta optitrack with gaps:
    gamma_opt=timeseries(wrapTo2Pi(unwrap(atan2(dY,dX))),car_pose.Time(2:end)); %[rad] from 0 to 2pi
    theta_opt=timeseries(wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1))+delta_theta),car_pose_orientation_euler.Time); %[rad] from 0 to 2pi
    beta_opt=timeseries(wrapToPi(unwrap(gamma_opt.Data)-unwrap(theta_opt.Data(2:end))),gamma_opt.Time);%[rad] from -pi to +pi
    
    %velocity computation with gaps remotion:
    % remotion based on ds:
    X_opt=timeseries(car_pose.Data(:,1),car_pose.Time);%[m]
    Y_opt=timeseries(car_pose.Data(:,1),car_pose.Time);%[m]
    ds_opt=timeseries(sqrt(dX.^2+dY.^2),car_pose.Time(2:end)); %[m]
    indexes_below_ds_min=find(ds_opt.Data<=ds_min);
    indexes_remotion=indexes_below_ds_min+1;
    X_opt_rem=delsample(X_opt,'Index',indexes_remotion); %[m]
    Y_opt_rem=delsample(Y_opt,'Index',indexes_remotion); %[m]
    dX_opt_rem=diff(X_opt_rem.Data); %[m]
    dY_opt_rem=diff(Y_opt_rem.Data); %[m]
    ds_opt_rem=timeseries(sqrt(dX_opt_rem.^2+dY_opt_rem.^2),Y_opt_rem.Time(2:end)); %[m]
    dt=diff(X_opt_rem.Time); %[s]
    V_opt_rem=timeseries((ds_opt_rem.Data)./dt,ds_opt_rem.Time); %[m/s]
    
    theta_opt_rem=delsample(theta_opt,'Index',indexes_remotion); %[rad] from 0 to 2Pi
    
    gamma_opt_rem=wrapTo2Pi(unwrap(atan2(dY_opt_rem,dX_opt_rem))); %[rad] from 0 to 2pi
    beta_tmp=wrapToPi(unwrap(gamma_opt_rem)-unwrap(theta_opt_rem.Data(2:end))); %[rad] from [-pi,pi]
    beta_opt_rem=timeseries(beta_tmp,ds_opt_rem.Time);%[rad] from -pi to +pi
    
    %     Causale
    gamma_opt_rem_filt=timeseries(wrapTo2Pi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(gamma_opt_rem))),ds_opt_rem.Time);
    theta_opt_rem_filt=timeseries(wrapTo2Pi(filter(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(theta_opt_rem.Data))),theta_opt_rem.Time);
    beta_opt_rem_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},beta_opt_rem.Data),beta_opt_rem.Time);
    V_opt_rem_filt=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},V_opt_rem.Data),V_opt_rem.Time);
    %     Acausale
    gamma_opt_rem_filtfilt=timeseries(wrapTo2Pi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(gamma_opt_rem))),ds_opt_rem.Time);
    theta_opt_rem_filtfilt=timeseries(wrapTo2Pi(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},unwrap(theta_opt_rem.Data))),theta_opt_rem.Time);
    beta_opt_rem_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},beta_opt_rem.Data),beta_opt_rem.Time);
    V_opt_rem_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},V_opt_rem.Data),V_opt_rem.Time);
    

    
end

%STATE ESTIMATOR TOPICS:
if state_estimator_opt_active
    %causal filter:
    state_estimator_opt_V_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_V.Data),state_estimator_opt_V.Time);
    state_estimator_opt_beta_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta.Data),state_estimator_opt_beta.Time);
    % acausal filter:
    state_estimator_opt_V_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_V.Data),state_estimator_opt_V.Time);
    state_estimator_opt_beta_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta.Data),state_estimator_opt_beta.Time);
end

% state_estimator_opt_driftingcar_multibeta TOPICS:
if state_estimator_opt_driftingcar_multibeta_active
%     causal filter:
    state_estimator_opt_V_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_V.Data),state_estimator_opt_V.Time);
    state_estimator_opt_beta_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta.Data),state_estimator_opt_beta.Time);
    state_estimator_opt_beta_vel_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta_vel.Data),state_estimator_opt_beta_vel.Time);
    state_estimator_opt_beta_acc_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta_acc.Data),state_estimator_opt_beta_acc.Time);
    state_estimator_opt_gamma_vel_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_gamma_vel.Data),state_estimator_opt_gamma_vel.Time);
    state_estimator_opt_gamma_acc_filter=timeseries(filter(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_gamma_acc.Data),state_estimator_opt_gamma_acc.Time);
    %     acausal filter:
    state_estimator_opt_V_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_V.Data),state_estimator_opt_V.Time);
    state_estimator_opt_beta_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta.Data),state_estimator_opt_beta.Time);
    state_estimator_opt_beta_vel_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta_vel.Data),state_estimator_opt_beta_vel.Time);
    state_estimator_opt_beta_acc_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_beta_acc.Data),state_estimator_opt_beta_acc.Time);
    state_estimator_opt_gamma_vel_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_gamma_vel.Data),state_estimator_opt_gamma_vel.Time);
    state_estimator_opt_gamma_acc_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},state_estimator_opt_gamma_acc.Data),state_estimator_opt_gamma_acc.Time);
    

end

%% Debug serial_comm

if test_comm_controller_cmd_active
    figure('name','debug serial_comm');
    g(1)=subplot(311); %speed ref
    stairs(controller_cmd.Time,controller_cmd.Data(:,1),'b','linewidth',LineWidth);hold on;grid on;
    stairs(test_comm_controller_cmd.Time,test_comm_controller_cmd.Data(:,1),'r--','linewidth',LineWidth);
    xlabel('t[s]');title('speed ref');
    g(2)=subplot(312); %steer ref
    stairs(controller_cmd.Time,controller_cmd.Data(:,2)*180/pi,'b','linewidth',LineWidth);hold on; grid on;
    stairs(test_comm_controller_cmd.Time,test_comm_controller_cmd.Data(:,2)*180/pi,'r--','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('steer ref');
    g(3)=subplot(313); %control state
    stairs(controller_cmd.Time,controller_cmd.Data(:,3),'b','linewidth',LineWidth);hold on;grid on;
    stairs(test_comm_controller_cmd.Time,test_comm_controller_cmd.Data(:,3),'r--','linewidth',LineWidth);
    xlabel('t[s]');title('radio cmd control state');
    linkaxes(g,'x');clear g;
    legend('controller cmd','serial comm');
    
end
   
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
    
    fig_imu_ang_vel=figure('name','imu angular velocity');
    g(1)=subplot(311); %wx
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,1),'b','linewidth',LineWidth);hold on;grid on;
    plot(imu_data_angular_vel.Time,imu_data_angular_vel.Data(:,1),'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[rad/s]');title('\omega_x');
    g(2)=subplot(312); %wy
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,2),'b','linewidth',LineWidth);hold on;grid on;
    plot(imu_data_angular_vel.Time,imu_data_angular_vel.Data(:,2),'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[rad/s]');title('\omega_y');
    g(3)=subplot(313); %wz
    plot(imu_data_angular_vel_rot.Time,imu_data_angular_vel_rot.Data(:,3),'b','linewidth',LineWidth);hold on;grid on;
    plot(imu_data_angular_vel.Time,imu_data_angular_vel.Data(:,3),'k','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[rad/s]');title('\omega_z');
    legend('rot','orig');
    linkaxes(g,'x');clear g;  

    
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

    
%     Figura paragone dati originali con dati filtrati
    fig_imu_lin_acc_filt=figure('name','imu rotated filtered accelerations');
    g(1)=subplot(311); %ax
    plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,1),'b');grid on;hold on;
    plot(ax_rot_filt.Time,ax_rot_filt.Data,'g','linewidth',LineWidth);
    plot(ax_rot_filtfilt.Time,ax_rot_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[m/s^2]');
    title('a_x');
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),-5,+5])
    g(2)=subplot(312); %ay
    plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,2),'b');grid on;hold on;
    plot(ay_rot_filt.Time,ay_rot_filt.Data,'g','linewidth',LineWidth);
    plot(ay_rot_filtfilt.Time,ay_rot_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[m/s^2]');
    title('a_y');
    axis([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end),-5,+5])
    g(3)=subplot(313); %az
    plot(imu_data_linear_acc_rot.Time,imu_data_linear_acc_rot.Data(:,3),'b');grid on;hold on;
    plot(az_rot_filt.Time,az_rot_filt.Data,'g','linewidth',LineWidth);
    plot(az_rot_filtfilt.Time,az_rot_filtfilt.Data,'r','linewidth',LineWidth);
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

%% Controller Cmd
if controller_cmd_active
    figure('name','controller cmd');
    g(1)=subplot(311); %speed ref
    stairs(controller_cmd.Time,controller_cmd.Data(:,1),'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');title('speed ref');
    g(2)=subplot(312); %steer ref
    stairs(controller_cmd.Time,controller_cmd.Data(:,2)*180/pi,'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[deg]');title('steer ref');
    g(3)=subplot(313); %control state
    stairs(controller_cmd.Time,controller_cmd.Data(:,3),'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');title('radio cmd control state');
    linkaxes(g,'x');clear g;
    
    figure('name','controller cmd dt');
    plot(controller_cmd.Time(2:end),diff(controller_cmd.Time)*1000,'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('dt [ms]');
    title('controller cmd dt msgs');
    
end

if t0_active
    figure('name','t0');
    plot(t0.Time,t0.Data,'linewidth',LineWidth);hold on;grid on;
    xlabel('t[s]');ylabel('t[s]');title('OLCL t0');
end

if OLCL_Switch_Signal_active
    figure('name','OLCL switch signal');
    plot(OLCL_Switch_Signal.Time,OLCL_Switch_Signal.Data,'linewidth',LineWidth);hold on;grid on;
    xlabel('t[s]');title('OLCL Switch Signal (-1 for OL, +1 for CL)');
end

if OLCL_Switch_Signal_active==1 && t0_active==1
    figure('name','debug OLCL');
    g(1)=subplot(311); %radio state
    plot(radio_cmd_control_state.Time,radio_cmd_control_state.Data,'linewidth',LineWidth);hold on;grid on;
    xlabel('t[s]');title('radio cmd state');
    g(2)=subplot(312); %t0
    plot(t0.Time,t0.Data,'linewidth',LineWidth);hold on;grid on;
    xlabel('t[s]');ylabel('t[s]');title('OLCL t0');
    g(3)=subplot(313); %OLCL_Switch_Signal
    plot(OLCL_Switch_Signal.Time,OLCL_Switch_Signal.Data,'linewidth',LineWidth);hold on;grid on;
    xlabel('t[s]');title('OLCL Switch Signal (-1 for OL, +1 for CL)');
    linkaxes(g,'x');clear g;
    
    figure('name','debug 2 OLCL');
    g(1)=subplot(311); %radio state + OLCL_Switch_Signal
    plot(radio_cmd_control_state.Time,radio_cmd_control_state.Data,'b-','linewidth',LineWidth);hold on;grid on;
    plot(OLCL_Switch_Signal.Time,OLCL_Switch_Signal.Data,'r--','linewidth',LineWidth);hold on;grid on;
    xlabel('t[s]');legend('radio cmd state','OLCL switch signal');
    g(2)=subplot(312); %throttle ref
    stairs(controller_cmd.Time,controller_cmd.Data(:,1),'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');title('throttle ref');
    g(3)=subplot(313); %steer ref
    stairs(controller_cmd.Time,controller_cmd.Data(:,2)*180/pi,'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[deg]');title('steer ref');
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
    fig_car_pose_orient_euler=figure('name','car pose yaw');
    g(1)=subplot(211); %Z yaw
    plot(car_pose_orientation_euler.Time,wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1)))*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[deg]');title('yaw angle');
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
%    Vx,Vy using the opt pose topic
    
    
    
    fig_V=figure('name','V original opt data');
    g(1)=subplot(211);
    plot(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m/s]');title('V opt');
    axis([V_opt.Time(1),V_opt.Time(end),0,5]);
    g(2)=subplot(212);
    plot(V_opt.Time(2:end),diff(V_opt.Time)*1000,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('dt[ms]');title('messages freq');
    axis([car_pose.Time(2),car_pose.Time(end),0 20]);
    linkaxes(g,'x');clear g;
    
    
    
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
    
    
%     Vedo l'effetto di un filtro passabasso sulle stime di V,beta 
    figure('name','effetto filtro passabasso');
    g(1)=subplot(211); %V
    plot(state_estimator_opt_V.Time,state_estimator_opt_V.Data,'b','linewidth',LineWidth);hold on;grid on;
    plot(state_estimator_opt_V_filter.Time,state_estimator_opt_V_filter.Data,'g','linewidth',LineWidth);
    plot(state_estimator_opt_V_filtfilt.Time,state_estimator_opt_V_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[m/s]');title('V ROS node state estimator');
    g(2)=subplot(212); %beta
    plot(state_estimator_opt_beta.Time,state_estimator_opt_beta.Data*180/pi,'b','linewidth',LineWidth);hold on;grid on;
    plot(state_estimator_opt_beta_filter.Time,state_estimator_opt_beta_filter.Data*180/pi,'g','linewidth',LineWidth);
    plot(state_estimator_opt_beta_filtfilt.Time,state_estimator_opt_beta_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\beta ROS node state estimator');
    legend('orig','causal filter','acausal filter');
    axis([state_estimator_opt_beta_filter.Time(1),state_estimator_opt_beta_filter.Time(end),-60,+60]);
    linkaxes(g,'x');clear g;
end


%% V,beta,gamma,theta with gaps remotion

if car_pose_active
    
    
    % X,Y 
    fig_buchiGPS=figure('name','buchi GPS xy');
    g(1)=subplot(311); %Xopt
    plot(X_opt_rem.Time,X_opt_rem.Data,'bo-','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m]');title('X GPS');
    g(2)=subplot(312); %Yopt
    plot(Y_opt_rem.Time,Y_opt_rem.Data,'bo-','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m]');title('Y GPS');
    g(3)=subplot(313); %dt msgs opt
    plot(X_opt_rem.Time(2:end),diff(X_opt_rem.Time)*1000,'b','linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[ms]');title('msgs dt');
    axis([X_opt_rem.Time(1),X_opt_rem.Time(end),5,100]);
    linkaxes(g,'x');clear g;
    
    %  V
    figure('name','V no gaps');
    g(1)=subplot(211);
    plot(V_opt_rem.Time,V_opt_rem.Data,'bo-','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('t[s]');ylabel('[m/s]');title('V_{opt}');
    g(2)=subplot(212);
    plot(V_opt_rem.Time(2:end),diff(V_opt_rem.Time)*1000,'bo-','linewidth',LineWidth);grid on;
    xlabel('t[s]');ylabel('[ms]');title('msgs dt');
    linkaxes(g,'x');clear g;
    
    
    
    % traiettoria con buchi
    fig_XY=figure('name','opt XY gaps rem');
    plot(X_opt_rem.Data,Y_opt_rem.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
    xlabel('X[m]');ylabel('Y[m]');
    axis square
    
    figure('name','3D traj buchi');
    plot3(X_opt_rem.Data,Y_opt_rem.Data,X_opt_rem.Time);
    xlabel('X[m]');ylabel('Y[m]');zlabel('t[s]');
    
    
    if comet_XY_active
        %        comet(X,Y,p) uses a comet of length p*length(Y).  Default is p = 0.10.
        figure('name','opt comet XY con buchi');
        comet(X_opt_rem.Data,Y_opt_rem.Data,0.1);
        xlabel('X[m]');ylabel('Y[m]');
        
        figure('name','3D comet buchi');
        comet3(X_opt_rem.Data,Y_opt_rem.Data,X_opt_rem.Time);
        xlabel('X[m]');ylabel('Y[m]');zlabel('t[s]');
    end
end
    
    
%%     Gamma,theta,beta,Vopt filtrato
if car_pose_active
    
%     Gamma filtrato
    figure('name','Gamma filt no buchi');
    plot(gamma_opt_rem.Time,gamma_opt_rem.Data*180/pi,'b');grid on;hold on;
    plot(gamma_opt_rem_filt.Time,gamma_opt_rem_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(gamma_opt_rem_filtfilt.Time,gamma_opt_rem_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([gamma_opt_rem.Time(1),gamma_opt_rem.Time(end),0,360]);
    
    %     theta filtrato
    figure('name','Theta filt no buchi');
    plot(theta_opt_rem.Time,theta_opt_rem.Data*180/pi,'b');grid on;hold on;
    plot(theta_opt_rem_filt.Time,theta_opt_rem_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(theta_opt_rem_filtfilt.Time,theta_opt_rem_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([theta_opt_rem.Time(1),theta_opt_rem.Time(end),0,360]);
    
%     gamma e theta filtrato
    figure('name','gamma,Theta filt no buchi');
    plot(theta_opt_rem.Time,theta_opt_rem.Data*180/pi,'b-.','linewidth',LineWidth);grid on;hold on;
    plot(gamma_opt_rem.Time,gamma_opt_rem.Data*180/pi,'b-','linewidth',LineWidth);
    plot(theta_opt_rem_filt.Time,theta_opt_rem_filt.Data*180/pi,'g-.','linewidth',LineWidth);
    plot(gamma_opt_rem_filt.Time,gamma_opt_rem_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(theta_opt_rem_filtfilt.Time,theta_opt_rem_filtfilt.Data*180/pi,'r-.','linewidth',LineWidth);
    plot(gamma_opt_rem_filtfilt.Time,gamma_opt_rem_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('\theta data','\gamma data','\theta causal','\gamma causal','\theta acausal','\gamma acausal');
    axis([theta_opt_rem.Time(1),theta_opt_rem.Time(end),0,360]);
    
    %     beta filtrato
    figure('name','beta filt no buchi');
    plot(beta_opt_rem.Time,beta_opt_rem.Data*180/pi,'b');grid on;hold on;
    plot(beta_opt_rem_filt.Time,beta_opt_rem_filt.Data*180/pi,'g','linewidth',LineWidth);
    plot(beta_opt_rem_filtfilt.Time,beta_opt_rem_filtfilt.Data*180/pi,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([beta_opt_rem.Time(1),beta_opt_rem.Time(end),-45,45]);
    
        %     V_opt filtrato
    figure('name','V_opt filt no buchi');
    plot(V_opt_rem.Time,V_opt_rem.Data,'b');grid on;hold on;
    plot(V_opt_rem_filt.Time,V_opt_rem_filt.Data,'g','linewidth',LineWidth);
    plot(V_opt_rem_filtfilt.Time,V_opt_rem_filtfilt.Data,'r','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');
    legend('spline','causal','acausal');
    axis([V_opt_rem.Time(1),V_opt_rem.Time(end),0,3]);
    
end

%% multibeta (beta estimators); 

if state_estimator_opt_driftingcar_multibeta_active
    
%     WITHOUT FILTER:
%     beta
    figure('name','beta est');
    plot(state_estimator_opt_beta.Time,state_estimator_opt_beta.Data*180/pi,'r','linewidth',LineWidth);hold on;grid on;
    plot(state_estimator_opt_beta_vel.Time,state_estimator_opt_beta_vel.Data*180/pi,'b','linewidth',LineWidth);
    plot(state_estimator_opt_beta_acc.Time,state_estimator_opt_beta_acc.Data*180/pi,'g','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\beta estimate');
    legend('geom','vel','acc');
    
%     gamma
       figure('name','gamma est');
    plot(ds_opt_rem.Time,wrapTo2Pi(unwrap(gamma_opt_rem))*180/pi,'r','linewidth',LineWidth);hold on;grid on;
    plot(state_estimator_opt_gamma_vel.Time,wrapTo2Pi(unwrap(state_estimator_opt_gamma_vel_filtfilt.Data))*180/pi,'b','linewidth',LineWidth);
    plot(state_estimator_opt_gamma_acc.Time,wrapTo2Pi(unwrap(state_estimator_opt_gamma_acc_filtfilt.Data))*180/pi,'g','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\gamma estimate');
    legend('geom','vel','acc');
    
%     WITH CAUSAL FILTER:
    figure('name','beta est filter');
    plot(state_estimator_opt_beta_filter.Time,state_estimator_opt_beta_filter.Data*180/pi,'r','linewidth',LineWidth);hold on;grid on;
    plot(state_estimator_opt_beta_vel_filter.Time,state_estimator_opt_beta_vel_filter.Data*180/pi,'b','linewidth',LineWidth);
    plot(state_estimator_opt_beta_acc_filter.Time,state_estimator_opt_beta_acc_filter.Data*180/pi,'g','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\beta estimate with causal filter');
    legend('geom','vel','acc');
    
%     WITH ACAUSAL FILTER:
    figure('name','beta est filtfilt');
    plot(state_estimator_opt_beta_filtfilt.Time,state_estimator_opt_beta_filtfilt.Data*180/pi,'r','linewidth',LineWidth);hold on;grid on;
    plot(state_estimator_opt_beta_vel_filtfilt.Time,state_estimator_opt_beta_vel_filtfilt.Data*180/pi,'b','linewidth',LineWidth);
    plot(state_estimator_opt_beta_acc_filtfilt.Time,state_estimator_opt_beta_acc_filtfilt.Data*180/pi,'g','linewidth',LineWidth);
    xlabel('t[s]');ylabel('[deg]');title('\beta estimate with acausal filter');
    legend('geom','vel','acc');
    
end

%% Identificazione friction coefficient

% file SingleTrackId_GreyMoq_01: good results for 
% [3.5,5.25] mu=0.26, steer=-15
% [13,18] mu=0.27,  steer=-23°
% [42,52] mu=0.22, steer=-30°

% file SingleTrackId_Parq_01: good results for
% [35,42] mu=0.16, steer=-19°
% [9,16] mu=0.18 steer=26°

% file single_track_id_moquette_differential_01: good results for
% [11,15] mu=0.2583 with ay, 0.2772 with V,r
% [30,41] mu=2897 with ay, 0.2799 with V,r

xlim_start=30; %[s]
xlim_end=41; %[s]

% I need to re-sample V_opt to match wz hits
V_opt_interp=interp1(V_opt.Time,sqrt(V_opt.Data(:,1).^2+V_opt.Data(:,2).^2),imu_data_angular_vel_rot.Time,'pchip'); %[m/s]

% data filtering
wf=1*2*pi; %[rad/s]
Tsmu=0.01; %[s]
FilterCTFmu=tf(1,[1/wf,1]);
FilterDTFmu=c2d(FilterCTFmu,Tsmu,'tustin');
wz_filtfilt_mu=filtfilt(FilterDTFmu.Num{1},FilterDTFmu.Den{1},imu_data_angular_vel_rot.Data(:,3));%[rad/s]
V_opt_filtfilt_mu=filtfilt(FilterDTFmu.Num{1},FilterDTFmu.Den{1},V_opt_interp);%[m/s]
ay_rot_filtfilt_mu=filtfilt(FilterDTFmu.Num{1},FilterDTFmu.Den{1},imu_data_linear_acc_rot.Data(:,2));%[m/s^2]

% lateral acceleration
figure('name','mu lat acc');
plot(imu_data_linear_acc_rot.Time,ay_rot_filtfilt_mu,'r','linewidth',LineWidth);grid on;hold on;
plot(imu_data_angular_vel_rot.Time,wz_filtfilt_mu.*V_opt_filtfilt_mu,'b','linewidth',LineWidth);
xlabel('t[s]');ylabel('[m/s^2]');title('lateral acceleration');
legend('a_y imu','Vr');
axis([xlim_start,xlim_end,-5,+5]);

% roll,pitch angles read by imu
figure('name','roll,pitch angles');
plot(imu_deltaroll.Time,imu_deltaroll.Data*180/pi,'r','linewidth',LineWidth);grid on;hold on;
plot(imu_deltapitch.Time,imu_deltapitch.Data*180/pi,'g','linewidth',LineWidth);
xlabel('t[s]');ylabel('[deg]');title('roll,pitch angles');
legend('\Delta roll','\Delta pitch');
axis([xlim_start,xlim_end,-10,10]);

% roll,pitch angular velocity by imu
wx_filtfilt_mu=timeseries(filtfilt(FilterDTFmu.Num{1},FilterDTFmu.Den{1},imu_data_angular_vel_rot.Data(:,1)),imu_data_angular_vel_rot.Time);%[rad/s]
wy_filtfilt_mu=timeseries(filtfilt(FilterDTFmu.Num{1},FilterDTFmu.Den{1},imu_data_angular_vel_rot.Data(:,2)),imu_data_angular_vel_rot.Time);%[rad/s]
figure('name','roll pitch w');
plot(wx_filtfilt_mu.Time,wx_filtfilt_mu.Data,'r','linewidth',LineWidth);grid on;hold on;
plot(wy_filtfilt_mu.Time,wy_filtfilt_mu.Data,'g','linewidth',LineWidth);grid on;hold on;
xlabel('t[s]');ylabel('[rad/s]');title('roll, pitch ang vel');
legend('w_x (roll)','w_y (pitch)');
axis([xlim_start,xlim_end,-0.5,+0.5]);

% V,r,ay
figure('name','mu V,r');
plot(imu_data_angular_vel_rot.Time,V_opt_filtfilt_mu,'b','linewidth',LineWidth);grid on;hold on;
plot(imu_data_angular_vel_rot.Time,wz_filtfilt_mu,'g','linewidth',LineWidth);
plot(imu_data_linear_acc_rot.Time,ay_rot_filtfilt_mu,'r','linewidth',LineWidth);
xlabel('t[s]');
axis([xlim_start,xlim_end,-5,5]);
legend('V [m/s]','r [rad/s]','a_y [m/s^2]');

% THROTTLE, STEERING COMMANDS:
figure('name','radio cmd');
g(1)=subplot(211); %speed ref
plot(radio_cmd_speed_ref.Time,radio_cmd_speed_ref.Data,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');title('speed ref');
axis([xlim_start,xlim_end,-1,+1]);
g(2)=subplot(212); %steer ref
plot(radio_cmd_steer_ref.Time,radio_cmd_steer_ref.Data*180/pi,'b','linewidth',LineWidth,'MarkerSize',MarkerSize);grid on;
xlabel('t[s]');ylabel('[deg]');title('steer ref');
axis([xlim_start,xlim_end,-45,+45]);
linkaxes(g,'x');clear g;

% mu estimation
t=imu_data_linear_acc_rot.Time;%[s]
dt_find=0.01; %[s]
index_start=find(t>xlim_start-dt_find & t<xlim_start+dt_find,1);
index_end=find(t>xlim_end-dt_find & t<xlim_end+dt_find,1);

mu_ay_mean=mean(abs(ay_rot_filtfilt_mu(index_start:index_end)/9.81));
mu_rV_mean=mean(abs(wz_filtfilt_mu(index_start:index_end).*V_opt_filtfilt_mu(index_start:index_end)/9.81));

figure('name','mu est');
plot(imu_data_linear_acc_rot.Time,abs(ay_rot_filtfilt_mu/9.81),'r','linewidth',LineWidth);grid on;hold on;
plot([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end)],mu_ay_mean*[1,1],'m--','linewidth',LineWidth);
plot(imu_data_angular_vel_rot.Time,abs(wz_filtfilt_mu.*V_opt_filtfilt_mu/9.81),'b','linewidth',LineWidth);
plot([imu_data_linear_acc_rot.Time(1),imu_data_linear_acc_rot.Time(end)],mu_rV_mean*[1,1],'c--','linewidth',LineWidth);
xlabel('t[s]');title('\mu estimation');
legend('from a_y','from a_y mean','from rV','from rV mean');
axis([xlim_start,xlim_end,0,1]);

% XY
index_start=find(car_pose.Time>xlim_start-dt_find & car_pose.Time<xlim_start+dt_find,1);
index_end=find(car_pose.Time>xlim_end-dt_find & car_pose.Time<xlim_end+dt_find,1);
figure('name','mu XY');
plot(car_pose.Data(index_start:index_end,1),car_pose.Data(index_start:index_end,2),'b','linewidth',LineWidth);grid on;
xlabel('X[m]');ylabel('Y[m]');
axis square
% comet
figure('name','mu XY comet');
comet(car_pose.Data(index_start:index_end,1),car_pose.Data(index_start:index_end,2));
xlabel('X[m]');ylabel('Y[m]');
axis square

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

tstart=3.15; %[s]
tend=3.22;%[s]

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

%% Figura identificazione dinamica servo sterzo

% First-order lowpass filter model:
wf_ss=50; %[rad/s] taken from servo datasheet
s=tf('s');
Gf_ss=1/(s/wf_ss+1);


%With servo command coming from the radio controller, opt+ay:
figure('name','man servo dyn');
plot(radio_cmd_steer_ref.Time,radio_cmd_steer_ref.Data/max(radio_cmd_steer_ref.Data),'r','linewidth',LineWidth);hold on;grid on;
plot(car_pose_orientation_euler.Time,(-(wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1)))*180/pi)+300)/40,'b','linewidth',LineWidth);
plot(ay_rot_filtfilt.Time,ay_rot_filtfilt.Data,'m','linewidth',LineWidth);
xlabel('t[s]');legend('Manual Ref','opt measure','a_y IMU');

% With servo command coming from the radio controller,only r:
figure('name','aut servo dyn r');
stairs(controller_cmd.Time,controller_cmd.Data(:,2)/max(controller_cmd.Data(:,2)),'r','linewidth',LineWidth);grid on;hold on;
plot(imu_data_angular_vel_rot.Time,-imu_data_angular_vel_rot.Data(:,3)/max(imu_data_angular_vel_rot.Data(:,3)),'g','linewidth',LineWidth);
xlabel('t[s]');legend('Automatic Ref','imu angular velocity');


%     wz_filtfilt=timeseries(filtfilt(FilterDTF.Num{1},FilterDTF.Den{1},imu_data_angular_vel_rot.Data(:,3)),imu_data_angular_vel_rot.Time);


%With servo command coming from an automatic controller:
figure('name','aut servo dyn');
stairs(controller_cmd.Time,controller_cmd.Data(:,2)*180/pi/30,'r','linewidth',LineWidth);grid on;hold on;
plot(car_pose_orientation_euler.Time,(((wrapTo2Pi(unwrap(car_pose_orientation_euler.Data(:,1)))*180/pi))-270)/45,'b','linewidth',LineWidth); %for ServoDynamics_fm0tom30deg.bag
% I add the lateral acceleration signal:
plot(ay_rot_filtfilt.Time,ay_rot_filtfilt.Data,'m','linewidth',LineWidth);
xlabel('t[s]');legend('Automatic Ref','opt measure','a_y IMU');


% t=radio_cmd_steer_ref.Time(1):1e-3:radio_cmd_steer_ref.Time(end);
% u=interp1(radio_cmd_steer_ref.Time,radio_cmd_steer_ref.Data/max(radio_cmd_steer_ref.Data),t,'spline');
% y=lsim(Gf_ss,u,t);
% plot(t,y,'g','linewidth',LineWidth);

% controller_cmd.Data(:,2)*180/pi
% t=controller_cmd.Time(1):1e-3:controller_cmd.Time(end);
% u=interp1(radio_cmd_steer_ref.Time,radio_cmd_steer_ref.Data/max(radio_cmd_steer_ref.Data),t,'spline');
% y=lsim(Gf_ss,u,t);

%% Test clock

figure('name','clock test');
g(1)=subplot(311); %clock time vs bag time
plot(controller_cmd.Time,controller_cmd.Data(:,2),'b','linewidth',LineWidth);grid on;
xlabel('t[s]');ylabel('[s]');title('clock time');
g(2)=subplot(312); %dt time clock
plot(controller_cmd.Time(2:end),diff(controller_cmd.Data(:,2))*1000,'b','linewidth',LineWidth);grid on;
xlabel('t[s]');ylabel('[ms]');title('dt msgs clock time');
g(3)=subplot(313); %dt msgs bag time
plot(controller_cmd.Time(2:end),diff(controller_cmd.Time)*1000,'b','linewidth',LineWidth);grid on;
xlabel('t[s]');ylabel('[ms]');
title('dt msgs bag time ');
linkaxes(g,'x');clear g;


%% Simulo Modello Single-Track e confronto con dati sperimentali

% seleziono finestra temporale di interesse
t_start_window=4; %[s]
t_end_window=8; %[s]


% Calcolo Vx dai dati sperimentali e ne estraggo window:
Vx_data=timeseries(V_opt_filtfilt.data.*cos(beta_opt_filtfilt.Data(2:end)),V_opt_filtfilt.Time); 
index_start=find(Vx_data.Time>t_start_window,1);
index_end=find(Vx_data.Time>t_end_window,1);
Vx_data_window=getsamples(Vx_data,index_start:index_end);
T_zero_Vx_data_window=Vx_data_window.Time(1); %[s]
set(Vx_data_window,'Time',Vx_data_window.Time-T_zero_Vx_data_window);

% Estraggo V_opt_filtfilt:
V_opt_filtfilt_window=getsamples(V_opt_filtfilt,index_start:index_end);
T_zero_V_opt_window=V_opt_filtfilt_window.Time(1); %[s]
set(V_opt_filtfilt_window,'Time',V_opt_filtfilt_window.Time-T_zero_V_opt_window);

% Estraggo beta_opt_filtfilt
beta_opt_filtfilt_window=getsamples(beta_opt_filtfilt,index_start:index_end);
T_zero_beta_opt_window=beta_opt_filtfilt_window.Time(1); %[s]
set(beta_opt_filtfilt_window,'Time',beta_opt_filtfilt_window.Time-T_zero_beta_opt_window);

% Estraggo wz (non filtrato)
index_start=find(imu_data_angular_vel_rot.Time>t_start_window,1);
index_end=find(imu_data_angular_vel_rot.Time>t_end_window,1);
imu_data_window=getsamples(imu_data_angular_vel_rot,index_start:index_end);
r_window=timeseries(imu_data_window.Data(:,3),imu_data_window.Time);
T_zero_r_window=r_window.Time(1); %[s]
set(r_window,'Time',r_window.Time-T_zero_r_window);

% Estraggo ay (filtrato)
ay_window=getsamples(ay_rot_filtfilt,index_start:index_end);
T_zero_ay_window=ay_window.Time(1); %[s]
set(ay_window,'Time',ay_window.Time-T_zero_ay_window);

% Estraggo steer ref:
index_start=find(radio_cmd_steer_ref.Time>t_start_window,1);
index_end=find(radio_cmd_steer_ref.Time>t_end_window,1);
radio_cmd_steer_ref_window=getsamples(radio_cmd_steer_ref,index_start:index_end);
T_zero_steer_ref_window=radio_cmd_steer_ref_window.Time(1);
set(radio_cmd_steer_ref_window,'Time',radio_cmd_steer_ref_window.Time-T_zero_steer_ref_window);

% Estraggo theta:
index_start=find(car_pose_orientation_euler_buchi.Time>t_start_window,1);
index_end=find(car_pose_orientation_euler_buchi.Time>t_end_window,1);
car_pose_orientation_euler_buchi_window=getsamples(car_pose_orientation_euler_buchi,index_start:index_end);
theta_window=timeseries(wrapTo2Pi(unwrap(car_pose_orientation_euler_buchi_window.Data(:,1)+delta_theta)),car_pose_orientation_euler_buchi_window.Time);
T_zero_theta_window=theta_window.Time(1);
set(theta_window,'Time',theta_window.Time-T_zero_theta_window);

% Estraggo car_pose
car_pose_window=getsamples(car_pose_buchi,index_start:index_end);
T_zero_car_pose_window=car_pose_window.Time(1);
set(car_pose_window,'Time',car_pose_window.Time-T_zero_car_pose_window);

% Condizioni iniziali
V0=V_opt_filtfilt_window.Data(1); %[m/s]
beta0=beta_opt_filtfilt_window.Data(1); %[rad]
Vx0=V0*cos(beta0); %[m/s]
Vy0=V0*sin(beta0);%[m/s]
r0=r_window.Data(1); %[rad/s]
theta0=theta_window.Data(1); %[rad]
X0=car_pose_window.Data(1,1);%[m]
Y0=car_pose_window.Data(1,2);%[m]


T_stop=t_end_window-t_start_window;
load_system('SingleTrackNonlinFiala_SimComp.slx');
sim('SingleTrackNonlinFiala_SimComp.slx',T_stop);
close_system('SingleTrackNonlinFiala_SimComp.slx');

t_end_grafici=T_stop; %[s]

% paragone V
figure('name','V comparison');
plot(V_opt_filtfilt_window.Time,V_opt_filtfilt_window.Data,'r','linewidth',LineWidth);hold on;grid on;
plot(V_st.Time,V_st.Data,'b','linewidth',LineWidth);
xlabel('t[s]');ylabel('[m/s]');title('V');
legend('exp data','sim');
axis([0,t_end_grafici,0,3]);

% paragone r
figure('name','r comparison');
plot(r_window.Time,r_window.Data,'r','linewidth',LineWidth);hold on;grid on;
plot(r_st.Time,r_st.Data,'b','linewidth',LineWidth);
xlabel('t[s]');ylabel('[rad/s]');title('r');
legend('exp data','sim');
axis([0,t_end_grafici,-2,2]);

% paragone ay
figure('name','V comparison');
plot(ay_window.Time,ay_window.Data,'r','linewidth',LineWidth);hold on;grid on;
plot(ay_st.Time,ay_st.Data,'b','linewidth',LineWidth);
xlabel('t[s]');ylabel('[m/s^2]');title('a_y');
legend('exp data','sim');
axis([0,t_end_grafici,-5,+5]);

% paragone beta
figure('name','Beta comparison');
plot(beta_opt_filtfilt_window.Time,beta_opt_filtfilt_window.Data*180/pi,'r','linewidth',LineWidth);hold on;grid on;
plot(beta_st.Time,beta_st.Data*180/pi,'b','linewidth',LineWidth);
xlabel('t[s]');ylabel('[deg]');title('\beta');
legend('exp data','sim');
axis([0,t_end_grafici,-15,15]);

% figura Fy
figure('name','Fy');
plot(Fy_st.Time,Fy_st.Data(:,1),'k','linewidth',LineWidth);hold on; grid on;
plot(Fy_st.Time,Fy_st.Data(:,2),'m','linewidth',LineWidth);
plot([Fy_st.Time(1),Fy_st.Time(end)],[1,1]*muf*Fzf,'k--','linewidth',LineWidth);
plot([Fy_st.Time(1),Fy_st.Time(end)],-[1,1]*muf*Fzf,'k--','linewidth',LineWidth);
plot([Fy_st.Time(1),Fy_st.Time(end)],[1,1]*mur*Fzr,'m--','linewidth',LineWidth);
plot([Fy_st.Time(1),Fy_st.Time(end)],-[1,1]*mur*Fzr,'m--','linewidth',LineWidth);
xlabel('t[s]');ylabel('[N]');
legend('F_y^f','F_y^r');
axis([0,t_end_grafici,-4.5,4.5]);

% Figura traiettoria
fig_XY=figure('name','XY');
plot(car_pose_window.Data(:,1),car_pose_window.Data(:,2),'r','linewidth',LineWidth,'MarkerSize',MarkerSize);hold on;grid on;
plot(X_st.Data,Y_st.Data,'b','LineWidth',LineWidth);
xlabel('X[m]');ylabel('Y[m]');
legend('exp data','sim');
axis square

% figura radio_cmd_steer_ref_selection
figure('name','radio_cmd_steer_ref_window');
plot(radio_cmd_steer_ref_window.Time,radio_cmd_steer_ref_window.Data*180/pi,'r','LineWidth',LineWidth);grid on;hold on;
plot(delta_filtered_st.Time,delta_filtered_st.Data*180/pi,'b','linewidth',LineWidth);
xlabel('t[s]');ylabel('[deg]');title('\delta');
legend('commanded','filtered');
axis([0,t_end_grafici,-30,+30])


%% Figure Debug Drifting LQR Controller

% Eqpoint Parameters:
delta_eqpoint=-20*pi/180; %[rad]
Fxr_eqpoint=3.01; %[N] with rear differential,moquette
% Fxr_eqpoint=2.09; %[N] with solid rear axle, moquette


% % Grey Moquette, rear differential:
% Vx_eqpoint=1.0; %[m/s]
% Vy_eqpoint=-0.7880; %[rad]
% r_eqpoint=2.31807; %[rad/s]

% % Grey Moquette, solid rear axle:
% Vx_eqpoint=1.0; %[m/s]
% Vy_eqpoint=-0.7055; %[rad]
% r_eqpoint=1.848; %[rad/s]

% parquet, rear differential:
Vx_eqpoint=1.0; %[m/s]
Vy_eqpoint=-0.62 %[rad]
r_eqpoint=1.68; %[rad/s]

beta_eqpoint=atan(Vy_eqpoint/Vx_eqpoint); %[rad]
delta_psi_eqpoint=-beta_eqpoint; %[rad]

% Controller state feedback matrix (A-BK) with
% d_x=[d_Vx,d_Vy,d_r],d_u=d_delta,d_Fxr]
K =[-0.8304   -0.9889    0.6146 0 0
    1.2987   -6.6663    0.8143 0 0];

% Transmission and motor parameters:
Fdrag=1.75; %[N] 
Rw=0.049; %[m]
Kt=1/340.34; %[Nm/A]
Imax=13; %[A] max motor current (set in VESC GUI)
tau=0.09799; %transmission gear ratio with rear differential
% tau=0.11; %transmission gear ratio with solid rear axle

% Control Inputs Saturation Limits
delta_max=45*pi/180; %[rad]
delta_min=-delta_max; %[rad]

% Car Data
auto='macchinina';
m=2.040; %[Kg] vehicle mass (w/o Odroid, Arduino,IMU)
a=0.147; %[m] CG-front axle distance (w/o Odroid, Arduino,IMU)
b=0.113; %[m] CG-rear axle distance (w/o Odroid, Arduino,IMU)
l=0.260; %[m] wheelbase
Jz=0.030; %[Kgm^2] measured yaw moment of inertia (w/o Odroid, Arduino,IMU)
hg=0.02; %[m] stimato a naso...
cf=0.180; %[m] DA VERIFICARE
cr=cf; %[m]
Cf=47.86; %[N/rad]muf=0.35;
Cr=127.77; %[N/rad]
% muf=0.35; %moquette with rear differential
muf=0.22; %parquet with rear differential
mur=muf;

Fzf=m*9.81*b/l; %[N]
Fzr=m*9.81*a/l; %[N]

servo_delay=0.04; %[s]
wf_ac=50; %[rad/s] servo cut-off freq (I order lowpass filter model)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

seq=1;
for k=2:radio_cmd_control_state.Length
    
    % prima di tutto devo capire quando sono in "Automatic".
    % Devo ragionare nel tempo, non negli indici, perchè ogni topic
    % pubblica nei suoi tempi:
    
    state_km1=radio_cmd_control_state.Data(k-1);
    state_k=radio_cmd_control_state.Data(k);
    
    if (state_k==2 && state_km1==1) %fronte di salita
       t_begin(seq)=radio_cmd_control_state.Time(k); 
       index_begin_aut(seq)=k;
    end
    
    if (state_k==1 && state_km1==2) %fronte di discesa
        t_end(seq)=radio_cmd_control_state.Time(k-1);
        index_end_aut(seq)=k-1;
        
        
        % costruisco il vettore degli indici da usare per estrarre le porzioni di
        % interesse dei vari segnali:
        indexes_controller_cmd=[];
        indexes_opt_state_estimator=[];
        indexes_imu_yaw_rate=[];
        indexes_car_pose=[];
        indexes_e=[];
        
        %  topic controller_cmd
        index_start=find(controller_cmd.Time>t_begin(seq),1);
        index_end=find(controller_cmd.Time>t_end(seq),1);
        indexes_controller_cmd=[indexes_controller_cmd,index_start:index_end];
        
%         beta,V (state estimator)
        if state_estimator_opt_driftingcar_multibeta_active
            index_start=find(state_estimator_opt_beta_vel.Time>t_begin(seq),1);
            index_end=find(state_estimator_opt_beta_vel.Time>t_end(seq),1);
            indexes_opt_state_estimator=[indexes_opt_state_estimator,index_start:index_end];
        else
            index_start=find(state_estimator_opt_beta.Time>t_begin(seq),1);
            index_end=find(state_estimator_opt_beta.Time>t_end(seq),1);
            indexes_opt_state_estimator=[indexes_opt_state_estimator,index_start:index_end];
        end
        
        
        %     topics imu
        index_start=find(imu_data_angular_vel_rot.Time>t_begin(seq),1);
        index_end=find(imu_data_angular_vel_rot.Time>t_end(seq),1);
        indexes_imu_yaw_rate=[indexes_imu_yaw_rate,index_start:index_end];
        
        % topic car_pose
        index_start=find(car_pose.Time>t_begin(seq),1);
        index_end=find(car_pose.Time>t_end(seq),1);
        indexes_car_pose=[indexes_car_pose,index_start:index_end];
        
        %topic delta_psi,e
        if circular_drifting_lqr_e_active
            index_start=find(circular_drifting_lqr_e.Time>t_begin(seq),1);
            index_end=find(circular_drifting_lqr_e.Time>t_end(seq),1);
            indexes_e=[indexes_e,index_start:index_end];
        end
        
        %  estraggo le corrispondenti porzioni dai segnali che mi interessano:
        controller_cmd_automatic=getsamples(controller_cmd,indexes_controller_cmd);
        
        if state_estimator_opt_driftingcar_multibeta_active
            state_estimator_opt_beta_automatic=getsamples(state_estimator_opt_beta_vel_filter,indexes_opt_state_estimator);
        else
            state_estimator_opt_beta_automatic=getsamples(state_estimator_opt_beta_filter,indexes_opt_state_estimator);
        end
        
        state_estimator_opt_V_automatic=getsamples(state_estimator_opt_V_filter,indexes_opt_state_estimator);
        imu_data_angular_vel_rot_automatic=getsamples(imu_data_angular_vel_rot,indexes_imu_yaw_rate);
        wz_automatic=timeseries(imu_data_angular_vel_rot_automatic.Data(:,3),imu_data_angular_vel_rot_automatic.Time);
        ay_rot_filtfilt_automatic=getsamples(ay_rot_filtfilt,indexes_imu_yaw_rate);
        
        car_pose_automatic=getsamples(car_pose,indexes_car_pose);
        
        if circular_drifting_lqr_e_active
            circular_drifting_lqr_e_automatic=getsamples(circular_drifting_lqr_e,indexes_e);
            circular_drifting_lqr_delta_psi_automatic=getsamples(circular_drifting_lqr_delta_psi,indexes_e);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % COMMANDS TO PLOT THE Fyf GRAPH:
        % resampling parameters:
        res_method='spline';
        res_offset=0.25; %[s] to trim the borders of the data record to avoid fake spikes in the resampled data
        Ts=0.01; %[s] resampling dt
        
        %EXPERIMENTAL DATA RESAMPLING WITH CONSTANT Ts%%%%%%%%%%
        t_aut_res=(car_pose_automatic.Time(1)+res_offset):Ts:(car_pose_automatic.Time(end)-res_offset);
        % Vq = interp1(X,V,Xq,METHOD)
        wz_aut_res=interp1(wz_automatic.Time,wz_automatic.Data,t_aut_res);%[rad/s]
        V_aut_res=interp1(state_estimator_opt_V_automatic.Time,state_estimator_opt_V_automatic.Data,t_aut_res); %[m/s]
        delta_aut_res=interp1(controller_cmd_automatic.Time,controller_cmd_automatic.Data(:,2),t_aut_res);%[rad]
        beta_aut_res=interp1(state_estimator_opt_beta_automatic.Time,state_estimator_opt_beta_automatic.Data,t_aut_res); %[rad]
        
        % ACTUATOR MODEL BUILDING:
        ActuatorCTF=tf(1,[1/wf_ac,1]);%,'InputDelay',tau_servo);
        ActuatorDTF=c2d(ActuatorCTF,Ts,'tustin');
        delta_aut_res_act=filtfilt(ActuatorDTF.Num{1},ActuatorDTF.Den{1},delta_aut_res); %[rad] NB: filtfilt command does not consider delay!
        
        % SERVO DELAY
        % shifto avanti dati servo (taglio coda)
        % taglio inizio dati altre misure (yaw rate etc)
        Ndelay=round(servo_delay/Ts);
        
        t_aut_res_delay=t_aut_res(Ndelay:end);
        wz_aut_res_delay=wz_aut_res(Ndelay:end);
        beta_aut_res_delay=beta_aut_res(Ndelay:end);
        V_aut_res_delay=V_aut_res(Ndelay:end);
      
        delta_aut_res_delay=delta_aut_res(1:end-Ndelay+1);
        
        %ALPHA COMPUTATION:
        Vx_aut_res_delay=V_aut_res_delay.*cos(beta_aut_res_delay);%[m/s]
        alphaf_aut_res_delay=atan(beta_aut_res_delay+a*wz_aut_res_delay./Vx_aut_res_delay)-delta_aut_res_delay;%[rad]
        
        %Fyf COMPUTATION:
        Fyf=zeros(1,length(t_aut_res_delay));
        Np=length(Fyf);
        alphasl=atan(3*muf*Fzf/Cf);%[rad]
        for k=1:Np
            alpha=alphaf_aut_res_delay(k);%[rad]
            z=tan(alpha);
            if abs(z)>=tan(alphasl)
                Fyf(k)=-muf*Fzf*sign(alpha);%[N]
            else
                Fyf(k)=-Cf*z+Cf^2/(3*muf*Fzf)*abs(z)*z-Cf^3/(27*muf^2*Fzf^2)*z^3;%[N]
            end
        end
        
        %GRAPH:
        figure('name','Fyf aut');
        plot(t_aut_res_delay,Fyf,'b','linewidth',LineWidth);hold on;grid on;
        plot([t_aut_res_delay(1),t_aut_res_delay(end)],[1,1]*muf*Fzf,'r--','linewidth',LineWidth);
        plot([t_aut_res_delay(1),t_aut_res_delay(end)],-[1,1]*muf*Fzf,'r--','linewidth',LineWidth);
        xlabel('t[s]');ylabel('[N]');title('F_y^f');

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % GRAFICI
        
        % grafico ingressi di controllo
        figure('name','ingressi di ctrl');
        g(1)=subplot(311); %throttle ref
        plot(controller_cmd_automatic.Time,controller_cmd_automatic.Data(:,1),'bo ','linewidth',LineWidth); hold on; grid on;
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],[1,1],'k--','linewidth',LineWidth);
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],-[1,1],'k--','linewidth',LineWidth);
        xlabel('t[s]');title('throttle ref');
        axis([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end),-1.1,1.1]);
        g(2)=subplot(312);%steer ref
        plot(controller_cmd_automatic.Time,controller_cmd_automatic.Data(:,2)*180/pi,'bo ','linewidth',LineWidth);hold on;grid on;
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],[1,1]*delta_eqpoint*180/pi,'y--','linewidth',LineWidth);
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],delta_max*[1,1]*180/pi,'k--','linewidth',LineWidth);
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],-delta_max*[1,1]*180/pi,'k--','linewidth',LineWidth);
        axis([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end),(-delta_max-0.1)*180/pi,(delta_max+0.1)*180/pi]);
        xlabel('t[s]');title('Steer Ref');ylabel('[deg]');
        g(3)=subplot(313);%msgs dt
        plot(controller_cmd_automatic.Time(2:end),diff(controller_cmd_automatic.Time)*1000,'b','linewidth',LineWidth);grid on;
        xlabel('t[s]');ylabel('[ms]');title('msgs dt');
        
        % grafico stati
        V=state_estimator_opt_V_automatic.Data; %[m/s]
        beta=state_estimator_opt_beta_automatic.Data; %[rad]
        
        Vx=V.*cos(beta); %[m/s]
        Vy=V.*sin(beta); %[m/s]
        Vx_opt_state_est_automatic=timeseries(Vx,state_estimator_opt_beta_automatic.Time);
        Vy_opt_state_est_automatic=timeseries(Vy,state_estimator_opt_beta_automatic.Time);
        
        
        figure('name','stati');
        g(4)=subplot(311); %Vx
        plot(Vx_opt_state_est_automatic.Time,Vx_opt_state_est_automatic.Data,'bo ','linewidth',LineWidth);hold on;grid on;
        plot([Vx_opt_state_est_automatic.Time(1),Vx_opt_state_est_automatic.Time(end)],Vx_eqpoint*[1,1],'y--','linewidth',LineWidth);
        xlabel('t[s]');ylabel('[m/s]');title('Vx');
        g(5)=subplot(312); %Vy
        plot(Vy_opt_state_est_automatic.Time,Vy_opt_state_est_automatic.Data,'bo ','linewidth',LineWidth);hold on;grid on;
        plot([Vy_opt_state_est_automatic.Time(1),Vy_opt_state_est_automatic.Time(end)],Vy_eqpoint*[1,1],'y--','linewidth',LineWidth);
        xlabel('t[s]');ylabel('[m/s]');title('Vy');
        g(6)=subplot(313); %r
        plot(wz_automatic.Time,wz_automatic.Data,'bo ','linewidth',LineWidth);hold on;grid on;
        plot([wz_automatic.Time(1),wz_automatic.Time(end)],r_eqpoint*[1,1],'y--','linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad/s]');title('r');
        
        % path tracking states
        if circular_drifting_lqr_e_active
          figure('name','e,psi automatic');
          g(7)=subplot(211);%delta psi
          plot(circular_drifting_lqr_delta_psi_automatic.Time,circular_drifting_lqr_delta_psi_automatic.Data*180/pi,'b','linewidth',LineWidth);hold on;grid on;
          plot([circular_drifting_lqr_delta_psi_automatic.Time(1),circular_drifting_lqr_delta_psi_automatic.Time(end)],delta_psi_eqpoint*[1,1]*180/pi,'r--','linewidth',LineWidth);
          xlabel('t[s]');ylabel('[deg]');title('\Delta \psi');
          g(7)=subplot(212);%2
          plot(circular_drifting_lqr_e_automatic.Time,circular_drifting_lqr_e_automatic.Data,'b','linewidth',LineWidth);hold on;grid on;
          plot([circular_drifting_lqr_e_automatic.Time(1),circular_drifting_lqr_e_automatic.Time(end)],0*[1,1],'r--','linewidth',LineWidth);
          xlabel('t[s]');ylabel('[m]');title('e');
          linkaxes(g,'x');clear g;
        end
        
%        % XY trajectory
        figure('name','XY automatic');
        plot(car_pose_automatic.Data(:,1),car_pose_automatic.Data(:,2),'b','linewidth',LineWidth);grid on;
        xlabel('X[m]');ylabel('Y[m]');
        axis square;
        
        % grafico beta
        figure('name','beta automatic');
        plot(state_estimator_opt_beta_automatic.Time,(state_estimator_opt_beta_automatic.Data)*180/pi,'bo ');hold on;grid on;
        plot([state_estimator_opt_beta_automatic.Time(1),state_estimator_opt_beta_automatic.Time(end)],beta_eqpoint*[1,1]*180/pi,'r--','linewidth',LineWidth);
        xlabel('t[s]');ylabel('[deg]');title('\beta');
        linkaxes(g,'x');clear g;
        
%         grafico ay
        figure('name','ay automatic')
        plot(ay_rot_filtfilt_automatic.Time,ay_rot_filtfilt_automatic.Data,'b','linewidth',LineWidth);hold on;grid on;
        xlabel('t[s]');ylabel('[m/s^2]');title('a_y');
        
        
        % Figura contributi vari termini su azioni di controllo
        K_Vx_delta=-K(1,1).*(Vx_opt_state_est_automatic.Data-Vx_eqpoint);
        K_Vy_delta=-K(1,2).*(Vy_opt_state_est_automatic.Data-Vy_eqpoint);
        K_r_delta=-K(1,3).*(wz_automatic.Data-r_eqpoint);
        if circular_drifting_lqr_e_active
            K_delta_psi_delta=-K(1,4).*(circular_drifting_lqr_delta_psi_automatic-delta_psi_eqpoint);
            K_e_delta=-K(1,5).*(circular_drifting_lqr_e_automatic);
        end
        
        K_Vx_Fxr=-K(2,1).*(Vx_opt_state_est_automatic.Data-Vx_eqpoint);
        K_Vy_Fxr=-K(2,2).*(Vy_opt_state_est_automatic.Data-Vy_eqpoint);
        K_r_Fxr=-K(2,3).*(wz_automatic.Data-r_eqpoint);
        if circular_drifting_lqr_e_active
            K_delta_psi_Fxr=-K(2,4).*(circular_drifting_lqr_delta_psi_automatic-delta_psi_eqpoint);
            K_e_Fxr=-K(2,5).*(circular_drifting_lqr_e_automatic);
        end
        
        figure('name','contributi delta');
        g(1)=subplot(211);
        plot(Vx_opt_state_est_automatic.Time,K_Vx_delta,'g','linewidth',LineWidth);hold on; grid on;
        plot(Vy_opt_state_est_automatic.Time,K_Vy_delta,'b','linewidth',LineWidth);
        plot(wz_automatic.Time,K_r_delta,'r','linewidth',LineWidth);
        plot([wz_automatic.Time(1),wz_automatic.Time(end)],delta_eqpoint*[1,1],'y--','linewidth',LineWidth); %eq 
        plot(controller_cmd_automatic.Time,controller_cmd_automatic.Data(:,2),'k','linewidth',LineWidth'); %controller cmd data
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],delta_max*[1,1],'k--','linewidth',LineWidth);%sat +
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],-delta_max*[1,1],'k--','linewidth',LineWidth);%sat -
        
        if circular_drifting_lqr_e_active
           plot(circular_drifting_lqr_delta_psi_automatic.Time,K_delta_psi_delta,'c','linewidth',LineWidth);
           plot(circular_drifting_lqr_e_automatic.Time,K_e_delta,'m','linewidth',LineWidth);
        end
        
        axis([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end),(-delta_max-0.1),(delta_max+0.1)]);
        xlabel('t[s]');ylabel('[rad]');
        titolo=['contributi \delta, seq ',num2str(seq)];title(titolo);
        legend('-K \Delta V_x','-K \Delta V_y','-K \Delta r','\delta eq. pt.','\delta','sat','sat','-K \Delta \psi','-K e');
        g(2)=subplot(212); %Vy,r
        yyaxis left
        plot(Vy_opt_state_est_automatic.Time,Vy_opt_state_est_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([Vy_opt_state_est_automatic.Time(1),Vy_opt_state_est_automatic.Time(end)],Vy_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad]');
        yyaxis right
        plot(wz_automatic.Time,wz_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([wz_automatic.Time(1),wz_automatic.Time(end)],r_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad/s]');
        legend('V_y','V_y eq pt','r','r eq pt');
        linkaxes(g,'x');clear g;
        
        figure('name','contributi Fxr');
        g(1)=subplot(211);
        plot(Vx_opt_state_est_automatic.Time,K_Vx_Fxr,'g','linewidth',LineWidth);hold on; grid on;
        plot(Vy_opt_state_est_automatic.Time,K_Vy_Fxr,'b','linewidth',LineWidth);
        plot(wz_automatic.Time,K_r_Fxr,'r','linewidth',LineWidth);
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],Fxr_eqpoint*[1,1],'y--','linewidth',LineWidth); %eq pt
        plot(controller_cmd_automatic.Time,controller_cmd_automatic.Data(:,1)*Imax*Kt/tau/Rw-Fdrag,'k','linewidth',LineWidth'); %throttle cmd
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],mur*Fzr*[1,1],'k-.','linewidth',LineWidth); %sat limit +
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],-mur*Fzr*[1,1],'k-.','linewidth',LineWidth); %sat limit -
        plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],(Imax*Kt/tau/Rw-Fdrag)*[1,1],'k--','linewidth',LineWidth); %current limit
        
        if circular_drifting_lqr_e_active
           plot(circular_drifting_lqr_delta_psi_automatic.Time,K_delta_psi_Fxr,'c','linewidth',LineWidth);
           plot(circular_drifting_lqr_e_automatic.Time,K_e_Fxr,'m','linewidth',LineWidth);
        end
        
        xlabel('t[s]');
        titolo=['contributi F_x^r, seq ',num2str(seq)];title(titolo);
        legend('-K \Delta V_x','-K \Delta V_y','-K \Delta r','F_x^R eq. pt.','F_x^R','sat','sat','I lim +','-K \Delta \psi','-K e');
        g(2)=subplot(212); %Vy,r
        yyaxis left
        plot(Vy_opt_state_est_automatic.Time,Vy_opt_state_est_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([Vy_opt_state_est_automatic.Time(1),Vy_opt_state_est_automatic.Time(end)],Vy_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad]');
        yyaxis right
        plot(wz_automatic.Time,wz_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([wz_automatic.Time(1),wz_automatic.Time(end)],r_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad/s]');
        legend('V_y','V_y eq pt','r','r eq pt');
        linkaxes(g,'x');clear g;  
        
        
        %         Figura Vy,-r
        figure('name','Vy,-r');
        yyaxis left
        plot(Vy_opt_state_est_automatic.Time,Vy_opt_state_est_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([Vy_opt_state_est_automatic.Time(1),Vy_opt_state_est_automatic.Time(end)],Vy_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad]');
        yyaxis right
        plot(wz_automatic.Time,-wz_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([wz_automatic.Time(1),wz_automatic.Time(end)],-r_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad/s]');
        legend('V_y','V_y eq pt','-r','-r eq pt');

%         Figura contributi termini dVy/dt
%         devo calcolare prima il termine -rVx quindi devo ricampionare r
%         nei punti in cui ho Vx da optitrack
        wz_automatic_interp=timeseries(interp1(wz_automatic.Time,wz_automatic.Data,Vx_opt_state_est_automatic.Time,'spline'),Vx_opt_state_est_automatic.Time);
%       adesso interpolo beta (che ho ri-filtrato) con spline e ne calcolo la derivata
        wf_Vy=2*pi*3; %[rad/s]
        Filter_Vy_CTF=tf(1,[1/wf_Vy,1]);
        Filter_Vy_DTF=c2d(Filter_Vy_CTF,0.01,'tustin');
        state_estimator_opt_beta_automatic_filtfilt=filtfilt(Filter_Vy_DTF.Num{1},Filter_Vy_DTF.Den{1},Vy_opt_state_est_automatic.Data);
        Vy_opt_state_est_automatic_spline=spline(state_estimator_opt_beta_automatic.Time,state_estimator_opt_beta_automatic_filtfilt); %[rad]
        d_Vy_opt_state_est_automatic_spline=fnder(Vy_opt_state_est_automatic_spline);        
        figure('name','contributi dVy/dt');
        g(1)=subplot(211);
        plot(state_estimator_opt_beta_automatic.Time,ppval(Vy_opt_state_est_automatic.Time,d_Vy_opt_state_est_automatic_spline),'b','linewidth',LineWidth);hold on;grid on;
        plot(ay_rot_filtfilt_automatic.Time,ay_rot_filtfilt_automatic.Data,'r','linewidth',LineWidth);
        plot(wz_automatic_interp.Time,-wz_automatic_interp.Data.*Vx_opt_state_est_automatic.Data,'g','linewidth',LineWidth);
        xlabel('t[s]');title('d V_y /dt contributions');
        legend('d \beta opt spline','a_y','-r*V_x');
        g(2)=subplot(212);
        yyaxis left
        plot(Vy_opt_state_est_automatic.Time,Vy_opt_state_est_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([Vy_opt_state_est_automatic.Time(1),Vy_opt_state_est_automatic.Time(end)],Vy_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad]');
        yyaxis right
        plot(wz_automatic.Time,wz_automatic.Data,'linewidth',LineWidth);hold on;grid on;
        plot([wz_automatic.Time(1),wz_automatic.Time(end)],r_eqpoint*[1,1],'linewidth',LineWidth);
        xlabel('t[s]');ylabel('[rad/s]');
        legend('V_y','V_y eq pt','r','r eq pt');
        linkaxes(g,'x');clear g;
        
%         %         figura debug contributi Fxr        
%         figure('name','debug contributi Fxr');
%         plot(Vx_opt_state_est_automatic.Time,K_Vx_Fxr,'g','linewidth',LineWidth);hold on; grid on;
%         plot(Vy_opt_state_est_automatic.Time,K_Vy_Fxr,'b','linewidth',LineWidth);
%         plot(wz_automatic.Time,K_r_Fxr,'r','linewidth',LineWidth);
%         plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],Fxr_eqpoint*[1,1],'y--','linewidth',LineWidth);
%         plot(controller_cmd_automatic.Time,controller_cmd_automatic.Data(:,1)*Imax*Kt/tau/Rw-Fdrag,'k','linewidth',LineWidth');
%         sum=K_Vx_Fxr+K_Vy_Fxr-K(2,3)*(wz_automatic_interp.Data-r_eqpoint)+Fxr_eqpoint;
%         plot(Vx_opt_state_est_automatic.Time,sum,'m','linewidth',LineWidth);
%         plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],mur*Fzr*[1,1],'k-.','linewidth',LineWidth);
%         plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],-mur*Fzr*[1,1],'k-.','linewidth',LineWidth);
%         plot([controller_cmd_automatic.Time(1),controller_cmd_automatic.Time(end)],(Imax*Kt/tau/Rw-Fdrag)*[1,1],'k--','linewidth',LineWidth);
%         xlabel('t[s]');title('debug contributi F_x^R');
%         legend('-K \Delta V_x','-K \Delta V_y','-K \Delta r','F_x^r eq. pt.','F_x^r','sum');

        
        fprintf('\n seq = %g\n',seq);
        pause;
        close all;
        seq=seq+1;
    end
    
end


