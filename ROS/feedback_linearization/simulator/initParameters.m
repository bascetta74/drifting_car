clc
clear all
close all

%% Caricamento dei parametri del veicolo
ROS_clockPublishingPeriod = 1.0e-2;  % [s] Clock message publishing period
ROS_statePublishingPeriod = 2.0e-2;  % [s] Vehicle state message publishing period
ROS_nodePeriod            = 1.0e-3;  % [s] Node run period

% Initial conditions (psi,psip,beta,x,y)
initial_conditions = [0,0,0,0,0];
