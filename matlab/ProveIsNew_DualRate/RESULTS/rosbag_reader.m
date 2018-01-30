% Created by Marco Baur for the IsNew tests

close all;
clear all;
clc;

% addpath ./bags/Static

%% Parameters
filename = '03'; %selects the rosbag file to analyze
bagsdirectory='/home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/RESULTS';

%% Load bag file
filepath=fullfile(bagsdirectory, ['IsNew_bag_', filename,'.bag']);
bagdata = rosbag(filepath)

%% Show available topics
AvTopics=bagdata.AvailableTopics

%% Select all messages on a given topic (still a bag selection obj)

chatter_topic=select(bagdata,'Topic','/chatter');
Chatter_WithIsNew_topic=select(bagdata,'Topic','/Chatter_WithIsNew');
Chatter_WithoutIsNew_topic=select(bagdata,'Topic','/Chatter_WithoutIsNew');


%% Extract messages as a time series
% Note that this method of extracting data is only supported if the current selection contains a single topic with a single message type.
% timeseries(topic_bagselectionobj); only gives the non-null fields of the
% topic

chatter=timeseries(chatter_topic,'Header.Seq');

Chatter_WithIsNew=timeseries(Chatter_WithIsNew_topic,'Point.X','Point.Y','Point.Z');
Chatter_WithoutIsNew=timeseries(Chatter_WithoutIsNew_topic,'Point.X','Point.Y','Point.Z');

% X is the sequence of the input /chatter topic message which has been
% published
% Y is the sequene message time in s
% Z is the sequence message time in ns



%% Data processing


figure();
plot(chatter.Time-chatter.Time(1),chatter.Data,'rd ');grid on;hold on;
plot(Chatter_WithIsNew.Time-chatter.Time(1),Chatter_WithIsNew.Data(:,1),'bo ');
plot(Chatter_WithoutIsNew.Time-chatter.Time(1),Chatter_WithoutIsNew.Data(:,1),'kx ');
xlabel('t[s]');
ylabel('Msg number');
legend('chatter','WithIsNew','WithoutIsNew');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Atomic Dual rate
close all;
clear all;
clc;

bagsdirectory='/home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/RESULTS';
filepath=fullfile(bagsdirectory, ['AtomicDualRate.bag']);
bagdata = rosbag(filepath)

AvTopics=bagdata.AvailableTopics

chatter_topic=select(bagdata,'Topic','/chatter');
Chatter_Fast_topic=select(bagdata,'Topic','/Chatter_Fast');
Chatter_Slow_topic=select(bagdata,'Topic','/Chatter_Slow');

chatter=timeseries(chatter_topic,'Header.Seq');
Chatter_Fast=timeseries(Chatter_Fast_topic,'Point.X','Point.Y','Point.Z');
Chatter_Slow=timeseries(Chatter_Slow_topic,'Point.X','Point.Y','Point.Z');

figure('name','Atomic Dual Rate');
plot(chatter.Time-chatter.Time(1),chatter.Data,'rd ');grid on;hold on;
plot(Chatter_Fast.Time-chatter.Time(1),Chatter_Fast.Data(:,1),'bo ');
plot(Chatter_Slow.Time-chatter.Time(1),Chatter_Slow.Data(:,1),'kx ');
xlabel('t[s]');
ylabel('Msg number');
legend('chatter','Fast','Slow');
title('Atomic Dual Rate: Fast 100 Hz, Slow 20 Hz');


%%

% %% Create matlab variables
% imu.orientation.data = orientation.Data;
% imu.orientation.time = orientation.Time-imu_topic.StartTime;
% 
% imu.angular_vel.data = angular_vel.Data;
% imu.angular_vel.time = angular_vel.Time-imu_topic.StartTime;
% 
% imu.linear_acc.data = linear_acc.Data;
% imu.linear_acc.time = linear_acc.Time-imu_topic.StartTime;
% 
% if carcontrol_active
%     ref.speed_ref.data = speed_ref.Data;
%     ref.speed_ref.time = speed_ref.Time-radio_cmd_topic.StartTime;
% 
%     ref.steer_ref.data = steer_ref.Data;
%     ref.steer_ref.time = steer_ref.Time-radio_cmd_topic.StartTime;
% 
%     control_state.data = control_state.Data;
%     control_state.time = control_state.Time-radio_cmd_topic.StartTime;
% end
% 
% if optitrack_active
%     optitrack.position.data = optitrack_position.Data;
%     optitrack.position.time = optitrack_position.Time-optitrack_pose_topic.StartTime;
%     
%     optitrack.orientation.data = optitrack_orientation.Data;
%     optitrack.orientation.time = optitrack_orientation.Time-optitrack_pose_topic.StartTime;
% end

% %% Save data
% if carcontrol_active
%     if optitrack_active
%         save(['./dataset/', filename, '.mat'], 'imu', 'ref', 'control_state', 'optitrack');
%     else
%         save(['./dataset/', filename, '.mat'], 'imu', 'ref', 'control_state');
%     end
% else
%     if optitrack_active
%         save(['./dataset/', filename, '.mat'], 'imu', 'optitrack');
%     else
%         save(['./dataset/', filename, '.mat'], 'imu');
%     end
% end


%%%%%%%%%%%%%%%%%% TO DO

%% Extract Topic names

% [Ntopics,~]=size(AvTopics.Properties.RowNames);
% 
% for k=1:Ntopics
%     tmp=AvTopics.Properties.RowNames{k};
%     TopicsName{k}=tmp;
% end
               
%%  Select all messages on each topic (still a bag selection obj), extract messages a time series obj

% for k= 1:Ntopics
% %    selection of all the messages of a single topic:
%     v = genvarname(TopicsName{k}(2:end)); %starting from 2nd char to skip the slash
%     eval([v '=select(bagdata,''Topic'',TopicsName{k});']);
% %     extract messages as a time series obj
%     eval([v,'TSobj=timeseries(',v,',']);
% end
% 
% v = genvarname(TopicsName{1}(2:end))
% 
% eval([v '= [1,2,3];']);
% 
% 
% orientation = timeseries(imu_topic, 'Orientation.X', 'Orientation.Y', 'Orientation.Z');





%%%%%%%%%%%%%%%%%%%%%%%%%%



