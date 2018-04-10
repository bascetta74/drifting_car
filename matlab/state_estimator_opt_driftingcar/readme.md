This Simulink model has been created by Marco Baur (marco.baur@polimi.it)


The generated ROS node online computes vehicle:
-heading angle (theta) in radiands [0,2Pi]
-sideslip angle (beta) in radiands [-pi,+pi]
-Absolute Velocity (V) in m/s
based on the optitrack car/pose topic data.


The parameters which have to be set are contained in the state_estimator_opt_callback.m script, which is executed by Simulink at runtime (InitFcn).

The generated ROS node is able to correctly estimate the aforementioned variables without jumps/gaps in their time-profile due to duplicated optitrack data (i.e. sometimes two consecutive points given by the optitrack car/pose topic are equal and therefore ds=0).

The figures illustrates the working behaviour of the node: data computed online with the ROS node are compared to data computed offline in matlab (starting from the car/pose data).

The node sampling frequency is 150 Hz but the topics are published only when a new data sample has been received.

Queue sizes are 1, both for reading and publishing topics

Be aware that in the Simulink "Model Configuration Parameters" you have to select your catkin workspace in the Hardware Implementation / Device parameters tab
