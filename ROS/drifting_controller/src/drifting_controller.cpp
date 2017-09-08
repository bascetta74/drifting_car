#include "drifting_controller/drifting_controller.h"


void drifting_controller::Prepare(void)
{
    RunPeriod = RUN_PERIOD_DEFAULT;

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
     ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    radioCommand_subscriber = Handle.subscribe("/radio_cmd", 1, &drifting_controller::radioCommand_MessageCallback, this);
    wheelSpeed_subscriber = Handle.subscribe("/wheel_speed", 1, &drifting_controller::wheelSpeed_MessageCallback, this);
    controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);
    imu_subscriber = Handle.subscribe("/imu/data", 1, &drifting_controller::imu_MessageCallback, this); // Why not data_raw?
    
    /* Initialize node state */
    _speed_ref = 0.0;
    _steer_ref = 0.0;
    _state = car_msgs::car_cmd::STATE_SAFE;
    _wheel_speed = 0.0;

    _angular_velocity.x = _angular_velocity.y = _angular_velocity.z = 0.0;
    _linear_acceleration.x = _linear_acceleration.y = _linear_acceleration.z = 0.0;
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void drifting_controller::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
	PeriodicTask();

	ros::spinOnce();

	LoopRate.sleep();
    }
}


void drifting_controller::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void drifting_controller::wheelSpeed_MessageCallback(const std_msgs::Float64::ConstPtr& msg)
{
    _wheel_speed = msg->data;
}


void drifting_controller::radioCommand_MessageCallback(const car_msgs::car_cmd::ConstPtr& msg)
{
    switch (msg->state)
    {
     case car_msgs::car_cmd::STATE_MANUAL:
     case car_msgs::car_cmd::STATE_SAFE:
      _speed_ref = msg->speed_ref;
      _steer_ref = msg->steer_ref;
      _state     = msg->state;
      break;
      
     case car_msgs::car_cmd::STATE_AUTOMATIC:
      _state = msg->state;
      break;
    
     case car_msgs::car_cmd::STATE_HALT:
      ROS_ERROR("Node %s: Arduino in HALT mode.", ros::this_node::getName().c_str());
      drifting_controller::Shutdown();
      break;
    }
}


void drifting_controller::imu_MessageCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    _angular_velocity = msg->angular_velocity;
    _linear_acceleration = msg->linear_acceleration;
}


void drifting_controller::PeriodicTask(void)
{
    /* In automatic mode compute control commands */
    if (_state == car_msgs::car_cmd::STATE_AUTOMATIC)
    {
	/* Compute speed ref */
	_speed_ref = 0.8;

	/* Compute steer ref */
	_steer_ref = 0.25;
    }

    /* Publish control commands */
    car_msgs::car_cmd controller_cmd;

    controller_cmd.speed_ref = _speed_ref;
    controller_cmd.steer_ref = _steer_ref;
    controller_cmd.state     = _state;

    controllerCommand_publisher.publish(controller_cmd);
}
