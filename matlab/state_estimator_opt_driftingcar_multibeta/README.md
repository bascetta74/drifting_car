PUBLISHED TOPICS (all float64)::

state_estimator_opt_theta: vehicle yaw angle in radians [0,2Pi] 

state_estimator_opt_V: vehicle absolute speed in m/s, computed as a finite difference of the X,Y position in time

state_estimator_opt_beta: vehicle sideslip angle in radians [-pi,pi], computed geometrically (atan(Vy/Vx))

state_estimator_opt_beta_vel: vehicle sideslip angle in radians [-pi,pi], computed with the GPS velocity estimator

state_estimator_opt_beta_acc: vehicle sideslip angle in radians [-pi,pi], computed with the GPS acceleration estimator

state_estimator_opt_gamma_vel: velocity vector absolute angle in radians [0,2pi], computed with the GPS velocity estimator

state_estimator_opt_gamma_acc: velocity vector absolute angle in radians [0,2pi], computed with the GPS acceleration estimator

READ TOPICS:
car/pose (coming from optitrack node)


The parameters which have to be set are contained in the state_estimator_opt_driftingcar_multibeta_callback.m script, which is executed by Simulink at runtime (InitFcn).

The generated ROS node is able to correctly estimate the aforementioned variables without jumps/gaps in their time-profile due to duplicated optitrack data (i.e. sometimes two consecutive points given by the optitrack car/pose topic are equal and therefore ds=0).

The node sampling frequency is 1000 Hz but the first three topics are published only when a new data sample has been received.

Queue sizes are 1, both for reading and publishing topics

Be aware that in the Simulink "Model Configuration Parameters" you have to select your catkin workspace in the Hardware Implementation / Device parameters tab


