#include "drifting_controller/drifting_controller.h"

#include <tf/transform_datatypes.h>


void drifting_controller::Prepare(void)
{
    RunPeriod = RUN_PERIOD_DEFAULT;

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/use_optitrack";
    if (false == Handle.getParam(FullParamName, _use_optitrack))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/yawrate_equilibrium";
    if (false == Handle.getParam(FullParamName, _yawrate_equilibrium))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/beta_equilibrium";
    if (false == Handle.getParam(FullParamName, _beta_equilibrium))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/steer_equilibrium";
    if (false == Handle.getParam(FullParamName, _steer_equilibrium))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/speed_equilibrium";
    if (false == Handle.getParam(FullParamName, _speed_equilibrium))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/yawrate_gain";
    if (false == Handle.getParam(FullParamName, _yawrate_gain))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/beta_gain";
    if (false == Handle.getParam(FullParamName, _beta_gain))
       ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    /* ROS topics */
    radioCommand_subscriber = Handle.subscribe("/radio_cmd", 1, &drifting_controller::radioCommand_MessageCallback, this);
    wheelSpeed_subscriber = Handle.subscribe("/wheel_speed", 1, &drifting_controller::wheelSpeed_MessageCallback, this);
    imu_subscriber = Handle.subscribe("/imu/data_cal", 1, &drifting_controller::imu_MessageCallback, this);
    optitrack_subscriber = Handle.subscribe("/Robot_2/pose", 1, &drifting_controller::optitrack_MessageCallback, this);
    controllerCommand_publisher = Handle.advertise<car_msgs::car_cmd>("/controller_cmd", 1);
    
    /* Initialize node state */
    _speed_ref = 0.0;
    _steer_ref = 0.0;
    _state = car_msgs::car_cmd::STATE_SAFE;
    _wheel_speed = 0.0;

    _beta = _yawrate = _lateral_acceleration = _longitudinal_velocity = 0.0;
    
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
    // Update velocity and acceleration
    _lateral_acceleration = msg->linear_acceleration.y;
    _yawrate = msg->angular_velocity.z;
}


void drifting_controller::optitrack_MessageCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Get rpy from quaternion
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // Compute beta
    _beta = yaw - std::atan2(msg->pose.position.y-_car_pose.pose.position.y, msg->pose.position.x-_car_pose.pose.position.x);

    // Compute longitudinal velocity
    _longitudinal_velocity = std::sqrt(std::pow(msg->pose.position.y-_car_pose.pose.position.y, 2)+std::pow(msg->pose.position.x-_car_pose.pose.position.x, 2)) / (msg->header.stamp-_car_pose.header.stamp).toSec();
    
    // Update car pose
    _car_pose.header = msg->header;
    _car_pose.pose = msg->pose;
}


void drifting_controller::PeriodicTask(void)
{
    /* In automatic mode compute control commands */
    if (_state == car_msgs::car_cmd::STATE_AUTOMATIC)
    {
     /* TODO aggiungere parametri relativi ai punti di linearizzazione */
	/* Compute speed ref */
	_speed_ref = _speed_equilibrium;

	/* Compute steer ref */
	_steer_ref = _steer_equilibrium-(_yawrate_gain*(_yawrate-_yawrate_equilibrium)+_beta_gain*(_beta-_beta_equilibrium));
    }

    /* Publish control commands */
    car_msgs::car_cmd controller_cmd;

    controller_cmd.speed_ref = _speed_ref;
    controller_cmd.steer_ref = _steer_ref;
    controller_cmd.state     = _state;

    controllerCommand_publisher.publish(controller_cmd);
}
