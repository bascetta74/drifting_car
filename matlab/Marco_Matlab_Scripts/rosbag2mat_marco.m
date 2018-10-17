
close all;
clear all;
clc;

%% Note
% quando fai rosbag record di una rosbag played, nei messaggi PoseStamped
% (/car/pose) rimane il tempo vecchio, cioè il tempo in cui la bag era
% stata registrata. Il tempo nuovo, sincronizzato per tutti i topic, è solo
% quello assegnato da rosbag record

%% PARAMETERS
filename='drifting_stabilization_multibeta_parquet_03'; %rosbag file name
data='2018_10_16';

%%%%%%%%%%%%
% opt yaw tuning
% delta_theta=4.99424; %[rad] for data of the 15 February 2018
delta_theta=5; %[rad] for data collected on 26 February 2018 4.99271
% delta_theta=0;

% imu orientation tuning:
delta_yaw_imu=0; %[rad]

delta_pitch_imu=0; %[rad]

delta_roll_imu=0; %[rad]

%%%%%%%%%%%%%%%%
% active topics
imu_data_active=1;
imu_mag_active=0;
imu_temperature_active=0;
car_pose_active=1;
car_groundpose_active=0;
radio_cmd_active=1;
wheel_speed_active=0;
%%%%%%%%%%%%%%%
beta_estimator_velocity_active=0;
beta_estimator_acceleration_active=0;

state_estimator_opt_active=0;

state_estimator_opt_driftingcar_multibeta_active=1;
%%%%%%%%%%%%%%
controller_cmd_active=1;

OLCL_Switch_Signal_active=0;
t0_active=0;


%%%%%%%%%%%%%%5
opt_time_type= 'Header'; % 'rosbag_record' or 'Header' time for the /car/pose PoseStamped topic

imu_time_type='Header';%%'rosbag_record' or 'Header' time 

T_zero_param='radio_cmd_steer_ref.Time(1)';%'controller_cmd.Time(1)';%'radio_cmd_steer_ref.Time(1)';%'car_pose.Time(1)';


%% Load rosbag

filepath=fullfile(['~/WorkingDirectory2/Rosbag_Files/',data,'/',filename,'.bag']);
robodata = rosbag(filepath);

%% See all topics stored in the rosbag
robodata.AvailableTopics

%% Select all messages on a given topic

% imu topics

if imu_data_active
    imu_data_topic = select(robodata,'Topic','/imu/data');
    fprintf('\n imu sensor_msgs/Imu \n');
    imu_data_msgs=readMessages(imu_data_topic,1);
    imu_data_msgs{1}.showdetails
end

if imu_mag_active
    imu_mag_topic=select(robodata,'Topic','/imu/mag');
    fprintf('\n imu sensor_msgs/MagneticField \n');
    imu_mag_msgs=readMessages(imu_mag_topic,1);
    imu_mag_msgs{1}.showdetails
end

if imu_temperature_active
    imu_temperature_topic=select(robodata,'Topic','/imu/temperature');
end

% Radio commands topic
if radio_cmd_active
    radio_cmd_topic = select(robodata, 'Topic','/radio_cmd');
    fprintf('\n /radio_cmd topic: car_msgs/car_cmd \n');
    radio_cmd_msgs=readMessages(radio_cmd_topic,1);
    radio_cmd_msgs{1}.showdetails
end

% Optitrack topics
if car_pose_active
    optitrack_pose_topic = select(robodata, 'Topic','/car/pose');
end
if car_groundpose_active
    optitrack_groundpose_topic = select(robodata, 'Topic','/car/ground_pose');
    fprintf('\n car/ground_pose: geometry_msgs/Pose2D \n');
    optitrack_groundpose_msgs=readMessages(optitrack_groundpose_topic,1);
    optitrack_groundpose_msgs{1}.showdetails
end

% wheel encoder topic
if wheel_speed_active
    wheel_speed_topic = select(robodata, 'Topic','/wheel_speed');
    fprintf('\n /wheel_speed topic: std_msgs/Float64 \n');
    wheel_speed_msgs = readMessages(wheel_speed_topic, 1);
    wheel_speed_msgs{1}.showdetails
end

% Controller node cmd topic
if controller_cmd_active
   controller_cmd_topic=select(robodata,'Topic','/controller_cmd');
    fprintf('\n /controller cmd topic: car_msgs/car_cmd \n');
    controller_cmd_msgs=readMessages(controller_cmd_topic,1);
end

if OLCL_Switch_Signal_active
   OLCL_Switch_Signal_topic=select(robodata,'Topic','/OLCL_Switch_Signal'); 
end

if t0_active
   t0_topic=select(robodata,'Topic','/t0'); 
end

% beta estimator velocity /gamma_estimator_velocity topic
if beta_estimator_velocity_active;
    gamma_estimator_velocity_topic=select(robodata,'Topic','/gamma_estimator_velocity');
    fprintf('\n /gamma_estimator_velocity topic: std_msgs/Float64 \n');
    gamma_estimator_velocity_msgs=readMessages(gamma_estimator_velocity_topic,1);
end

if beta_estimator_acceleration_active;
    gamma_estimator_acceleration_topic=select(robodata,'Topic','/gamma_estimator_acceleration');
    fprintf('\n /gamma_estimator_acceleration topic: std_msgs/Float64 \n');
    gamma_estimator_acceleration_msgs=readMessages(gamma_estimator_acceleration_topic,1);
end


% State estimator opt node
if state_estimator_opt_active
    state_estimator_opt_beta_topic=select(robodata,'Topic','/state_estimator_opt_beta');
    fprintf('\n /state_estimator_opt_beta topic: std_msgs/Float64 \n');
    beta_estimator_opt_msgs=readMessages(state_estimator_opt_beta_topic,1);
    
    state_estimator_opt_theta_topic=select(robodata,'Topic','/state_estimator_opt_theta'); %[rad] from 0 to 2Pi
    state_estimator_opt_V_topic=select(robodata,'Topic','/state_estimator_opt_V');
    
end

% state_estimator_opt_driftingcar_multibeta node
if state_estimator_opt_driftingcar_multibeta_active
%     geometric beta:
    state_estimator_opt_beta_topic=select(robodata,'Topic','/state_estimator_opt_beta');
%     state_estimator_opt_beta_msgs=readMessages(state_estimator_opt_beta_topic,1);
    
    % velocity beta
    state_estimator_opt_beta_vel_topic=select(robodata,'Topic','/state_estimator_opt_beta_vel');
%     state_estimator_opt_beta_vel_msgs=readMessages(state_estimator_opt_beta_vel_topic,1);
    
    %     acceleration beta
    state_estimator_opt_beta_acc_topic=select(robodata,'Topic','/state_estimator_opt_beta_acc');
%     state_estimator_opt_beta_acc_msgs=readMessages(state_estimator_opt_beta_acc_topic,1);
    
%     gamma vel
    state_estimator_opt_gamma_vel_topic=select(robodata,'Topic','/state_estimator_opt_gamma_vel');
%     state_estimator_opt_gamma_vel_msgs=readMessages(state_estimator_opt_gamma_vel_topic,1);
    
    %     gamma acc
    state_estimator_opt_gamma_acc_topic=select(robodata,'Topic','/state_estimator_opt_gamma_acc');
%     state_estimator_opt_gamma_acc_msgs=readMessages(state_estimator_opt_gamma_acc_topic,1);
    
    %     theta,V
    state_estimator_opt_theta_topic=select(robodata,'Topic','/state_estimator_opt_theta'); %[rad] from 0 to 2Pi
    state_estimator_opt_V_topic=select(robodata,'Topic','/state_estimator_opt_V');

end

%% Extract messages as a time series
% imu_dataraw_orientation.Time(1) is the reference t=0

fprintf('\n timeseries objects creation started...\n');

% imu messages

if imu_data_active
    switch imu_time_type
        case 'rosbag_record'
            imu_data_orientation = timeseries(imu_data_topic,'Orientation.W', 'Orientation.X', 'Orientation.Y', 'Orientation.Z')
            imu_data_angular_vel = timeseries(imu_data_topic, 'AngularVelocity.X', 'AngularVelocity.Y', 'AngularVelocity.Z')
            imu_data_linear_acc = timeseries(imu_data_topic, 'LinearAcceleration.X', 'LinearAcceleration.Y', 'LinearAcceleration.Z')
        case 'Header'
            imu_data_timestamp=timeseries(imu_data_topic, 'Header.Stamp.Sec', 'Header.Stamp.Nsec');
            imu_data_timevector=imu_data_timestamp.Data(:,1)+imu_data_timestamp.Data(:,2)*1e-9;
            
            imu_data_orientation_tmp=timeseries(imu_data_topic,'Orientation.W','Orientation.X', 'Orientation.Y', 'Orientation.Z');
            imu_data_orientation=timeseries(imu_data_orientation_tmp.Data,imu_data_timevector)
            
            imu_data_angular_vel_tmp=timeseries(imu_data_topic, 'AngularVelocity.X', 'AngularVelocity.Y', 'AngularVelocity.Z');
            imu_data_angular_vel=timeseries(imu_data_angular_vel_tmp.Data,imu_data_timevector);
            
            imu_data_linear_acc_tmp=timeseries(imu_data_topic, 'LinearAcceleration.X', 'LinearAcceleration.Y', 'LinearAcceleration.Z');
            imu_data_linear_acc=timeseries(imu_data_linear_acc_tmp.Data,imu_data_timevector);
            
        otherwise
            frpintf('\n error in the imu_time_type value \n');
    end
end

if imu_mag_active
    switch imu_time_type
        case 'rosbag_record'
            imu_mag=timeseries(imu_mag_topic,'MagneticField_.X','MagneticField_.Y','MagneticField_.Z')
        case 'Header'
            
            imu_mag_timestamp=timeseries(imu_mag_topic, 'Header.Stamp.Sec', 'Header.Stamp.Nsec');
            imu_mag_timevector=imu_mag_timestamp.Data(:,1)+imu_mag_timestamp.Data(:,2)*1e-9;
            
            imu_mag_tmp=timeseries(imu_mag_topic,'MagneticField_.X','MagneticField_.Y','MagneticField_.Z');
            imu_mag=timeseries(imu_mag_tmp.Data,imu_mag_timevector)
            
        otherwise
            fprintf('\n error in the imu_time_type parameter value \n');
    end
end

if imu_temperature_active
    % TODO: imu_temperature.Time=imu_dataraw.Time (verificalo!!)
    imu_temperature=timeseries(imu_temperature_topic)
end

% radio cmd messages
if radio_cmd_active
    radio_cmd_speed_ref = timeseries(radio_cmd_topic, 'SpeedRef')
    radio_cmd_steer_ref = timeseries(radio_cmd_topic, 'SteerRef')
    radio_cmd_control_state = timeseries(radio_cmd_topic, 'State')
end

% optitrack messages

%     NB: the time is that of the heading of the pose topic, which is
%     different from the time given by the rosbag record command (time
%     instant at which the data is recorded and saved)
if car_pose_active
    switch opt_time_type
        case 'rosbag_record'
            car_pose=timeseries(optitrack_pose_topic, 'Pose.Position.X', 'Pose.Position.Y', 'Pose.Position.Z'); %time is that of the rosbag record command
            car_pose_orientation=timeseries(optitrack_pose_topic, 'Pose.Orientation.W','Pose.Orientation.X', 'Pose.Orientation.Y', 'Pose.Orientation.Z');
        case 'Header'
            car_pose_tmp = timeseries(optitrack_pose_topic, 'Pose.Position.X', 'Pose.Position.Y', 'Pose.Position.Z'); %time is that of the rosbag record command
            car_pose_timestamp=timeseries(optitrack_pose_topic,'Header.Stamp.Sec','Header.Stamp.Nsec');
            car_pose_timevector=car_pose_timestamp.Data(:,1)+car_pose_timestamp.Data(:,2)*1e-9;
            car_pose=timeseries(car_pose_tmp.Data,car_pose_timevector);
            
            car_pose_orientation_tmp = timeseries(optitrack_pose_topic,'Pose.Orientation.W', 'Pose.Orientation.X', 'Pose.Orientation.Y', 'Pose.Orientation.Z');
            car_pose_orientation=timeseries(car_pose_orientation_tmp.Data,car_pose_timevector);
        otherwise
            fprintf('\n error: opt_time_type value not correct \n');
    end
end

if car_groundpose_active
    car_groundpose=timeseries(optitrack_groundpose_topic,'X','Y','Theta');
end


%wheel encoder msgs
if wheel_speed_active
    wheel_speed= timeseries(wheel_speed_topic)
end 

% Controller cmd msgs
if controller_cmd_active
    controller_cmd= timeseries(controller_cmd_topic)
end

if OLCL_Switch_Signal_active
   OLCL_Switch_Signal=timeseries(OLCL_Switch_Signal_topic); 
end

if t0_active
    t0=timeseries(t0_topic);
end

% gamma estimator velocity msgs
if beta_estimator_velocity_active
    gamma_estimator_velocity=timeseries(gamma_estimator_velocity_topic);
end

if beta_estimator_acceleration_active
    gamma_estimator_acceleration=timeseries(gamma_estimator_acceleration_topic);
end

if state_estimator_opt_active
    state_estimator_opt_beta=timeseries(state_estimator_opt_beta_topic);
    state_estimator_opt_theta=timeseries(state_estimator_opt_theta_topic);
    state_estimator_opt_V=timeseries(state_estimator_opt_V_topic);

end

if state_estimator_opt_driftingcar_multibeta_active
    state_estimator_opt_beta=timeseries(state_estimator_opt_beta_topic);
    state_estimator_opt_theta=timeseries(state_estimator_opt_theta_topic);
    state_estimator_opt_V=timeseries(state_estimator_opt_V_topic);
    state_estimator_opt_beta_vel=timeseries(state_estimator_opt_beta_vel_topic);
    state_estimator_opt_beta_acc=timeseries(state_estimator_opt_beta_acc_topic);
    state_estimator_opt_gamma_vel=timeseries(state_estimator_opt_gamma_vel_topic);
    state_estimator_opt_gamma_acc=timeseries(state_estimator_opt_gamma_acc_topic);
end

fprintf('\n timeseries objects creation ended!\n');


%% time-zero 

T_zero=eval([T_zero_param])

if imu_data_active
    set(imu_data_orientation,'Time',imu_data_orientation.Time-T_zero);
    set(imu_data_angular_vel,'Time',imu_data_angular_vel.Time-T_zero);
    set(imu_data_linear_acc,'Time',imu_data_linear_acc.Time-T_zero); 
end

if imu_mag_active
    set(imu_mag,'Time',imu_mag.Time-T_zero);
end

if imu_temperature_active
    set(imu_temperature,'Time',imu_temperature.Time-T_zero);
end

if radio_cmd_active
    set(radio_cmd_speed_ref,'Time',radio_cmd_speed_ref.Time-T_zero);
    set(radio_cmd_steer_ref,'Time',radio_cmd_steer_ref.Time-T_zero);
    set(radio_cmd_control_state,'Time',radio_cmd_control_state.Time-T_zero);
end

if car_pose_active
    set(car_pose,'Time',car_pose.Time-T_zero);
    set(car_pose_orientation,'Time',car_pose_orientation.Time-T_zero);
end

if car_groundpose_active
    set(car_groundpose,'Time',car_groundpose.Time-T_zero);
end

if wheel_speed_active
    set(wheel_speed,'Time',wheel_speed.Time-T_zero);
end

if controller_cmd_active
    set(controller_cmd,'Time',controller_cmd.Time-T_zero);
end

if OLCL_Switch_Signal_active
    set(OLCL_Switch_Signal,'Time',OLCL_Switch_Signal.Time-T_zero);
end

if t0_active
   set(t0,'Time',t0.Time-T_zero); 
end

if beta_estimator_velocity_active
    set(gamma_estimator_velocity,'Time',gamma_estimator_velocity.Time-T_zero);
end

if beta_estimator_acceleration_active
    set(gamma_estimator_acceleration,'Time',gamma_estimator_acceleration.Time-T_zero);
end

if state_estimator_opt_active
    set(state_estimator_opt_beta,'Time',state_estimator_opt_beta.Time-T_zero);
    set(state_estimator_opt_theta,'Time',state_estimator_opt_theta.Time-T_zero);
    set(state_estimator_opt_V,'Time',state_estimator_opt_V.Time-T_zero);

end

if state_estimator_opt_driftingcar_multibeta_active
    set(state_estimator_opt_beta,'Time',state_estimator_opt_beta.Time-T_zero);
    set(state_estimator_opt_theta,'Time',state_estimator_opt_theta.Time-T_zero);
    set(state_estimator_opt_V,'Time',state_estimator_opt_V.Time-T_zero);
    set(state_estimator_opt_beta_vel,'Time',state_estimator_opt_beta_vel.Time-T_zero);
    set(state_estimator_opt_beta_acc,'Time',state_estimator_opt_beta_acc.Time-T_zero);
    set(state_estimator_opt_gamma_vel,'Time',state_estimator_opt_gamma_vel.Time-T_zero);
    set(state_estimator_opt_gamma_acc,'Time',state_estimator_opt_gamma_acc.Time-T_zero);
end

%% data save as matlab timeseries objects
risp=input('\n would you like to save the dataset into a mat file? [y] \n');
if risp=='y'
    
    fprintf('\n data are being saved in a matlab file...\n');
    
    % first of all: save the parameters
    save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'delta_theta',...
        'opt_time_type','imu_time_type','imu_data_active','imu_mag_active',...
        'imu_temperature_active','car_pose_active','car_groundpose_active',...
        'radio_cmd_active','wheel_speed_active','controller_cmd_active',...
        't0_active','OLCL_Switch_Signal_active','beta_estimator_velocity_active','beta_estimator_acceleration_active',...
        'state_estimator_opt_active','state_estimator_opt_driftingcar_multibeta_active',...
        'T_zero_param',...
        'delta_yaw_imu','delta_pitch_imu','delta_roll_imu');
    
    
    % now save the timeseries matlab objects:
    if imu_data_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'imu_data_orientation','imu_data_angular_vel','imu_data_linear_acc','-append');
    end
    
    if imu_mag_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'imu_mag','-append');
        
    end
    
    if imu_temperature_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'imu_temperature','-append');
    end
    
    if radio_cmd_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'radio_cmd_speed_ref','radio_cmd_steer_ref','radio_cmd_control_state','-append');
    end
    
    if car_pose_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'car_pose','car_pose_orientation','-append');
    end
    
    if car_groundpose_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'car_groundpose','-append');
    end
    
    if wheel_speed_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'wheel_speed','-append');
    end
    
    if controller_cmd_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'controller_cmd','-append');
    end
    
    if t0_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'t0','-append');
    end
    
    if OLCL_Switch_Signal_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'OLCL_Switch_Signal','-append');
    end
    
    if beta_estimator_velocity_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'gamma_estimator_velocity','-append');
    end
    
    if beta_estimator_acceleration_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'gamma_estimator_acceleration','-append');
    end
    
    if state_estimator_opt_active
        save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],'state_estimator_opt_beta','state_estimator_opt_theta','state_estimator_opt_V','-append');
        
    end
    
    if state_estimator_opt_driftingcar_multibeta_active
                save(['~/WorkingDirectory2/Mat_Files/',filename,'.mat'],...
                    'state_estimator_opt_beta','state_estimator_opt_theta',...
                    'state_estimator_opt_V',...
                    'state_estimator_opt_beta_vel',...
                    'state_estimator_opt_beta_acc',...
                    'state_estimator_opt_gamma_vel',...
                'state_estimator_opt_gamma_acc','-append');
    end
    
    fprintf('\n save completed!\n');
end

%% check timestamp opt pose topic

if car_pose_active
    tmp=timeseries(optitrack_pose_topic,'Header.Stamp.Sec','Header.Stamp.Nsec');
    
    opt_pose_timestamp=timeseries(tmp.Data(:,1)+tmp.Data(:,2)*1e-9,tmp.Time);
    
    figure('name','opt timestamp vs rosbag timestamp');
    plot(opt_pose_timestamp.Time-opt_pose_timestamp.Time(1),(opt_pose_timestamp.Data-opt_pose_timestamp.Time),'b','linewidth',1.5);
    xlabel('t[s]');ylabel('\Delta t[s]');title('opt timestamp - rosbag timestamp');
end

