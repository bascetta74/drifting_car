# README #

### Installation notes (kinetic) ###

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