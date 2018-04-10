Guide to the files:

-PDa_tuning: script created by Marco Baur to tune the PD controller;
-beta_estimator_acceleration.slx: model created by Luca Bascetta with the original structure of the estimator;
-beta_estimator_acceleration_driftingcar.slx: model created by Marco Baur to generate the ROS node
-beta_estimators_simulink.slx: model created by Marco Baur to be used by the rosbag_reader_simulink_GammaEstimators.m script to compare the performance of the velocity and acceleration based gamma (beta) estimators with the optitrack data, without relying on ROS (all the needed computation is performed within the Matlab/Simulink environment).
-folder "config" and "launch" contains additional ROS files needed to run the node; created by Marco Baur