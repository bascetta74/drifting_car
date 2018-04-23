# README #

## Way of using this repo ##
The ROS subdirectory only contains the catkin workspace's src folder (i.e. nodes packages).

Simulink generated ROS nodes must not be inserted inside the ROS subdirectory on this git repo.
The Similink schemes which generate the ROS nodes must be placed inside the repo's Matlab subdirectory. 
Compiled nodes (i.e. the packages folders which go inside the src subfolder of the catkin workspace) must be kept only on your local machine.

### into the odroid ###
Into the odroid, the following lines have been added to the .git/info/exclude file in order to not upload also the simulink generated ROS nodes:
"# simulink generated ROS nodes
ROS/beta_estimator_velocity_driftingcar/
ROS/beta_estimator_acceleration_driftingcar/"
Note that this list has to be updated for each neewly generated simulink ROS node

This repo is located in ~/Software/driftingcar.
In the catkin_ws workspace folder, the src folder is actually a symbolic link which points to the ROS subdirectory of this repo.

## Installation notes (kinetic) ##

* To use gauges download and recompile rqt_gauges, and then substitute the library rqt_gauges in /opt/ros/kinetic... with the newly compiled version

## Arduino - VESC PPM-app communication ##
The VESC PWM input pin accepts PWM signals with a 62.5 Hz frequency.
Since the dafault frequency of PWM signals generated with the servo Arduino library is 50 Hz, you have to modify the servo.h file of the Arduino Servo library
The servo.h file can be found on your local machine in "eclipse_directory"/arduinoPluginl/libraries/Servo/"your servo library version"/src
The parameter which has to be modified is REFRESH_INTERVAL, which has to be set to 16000 micro-seconds instead of 20000 micro-seconds.

## ROS catkin_make dependencies installation ##
When ROS is missing some libraries and catkin_make cannot compile, run (inside your ROS workspace):
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
(where kinetic is your ROS distribution)

## Telnet server installation
Follow this link for installation:
https://askubuntu.com/questions/668725/how-can-the-telnet-service-on-ubuntu-server-14-04-lts-be-enabled
Follow this link for configuration:
https://askubuntu.com/questions/680550/telnet-is-not-working-in-ubuntu-14-04