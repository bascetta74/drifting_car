#include "beta_estimator/beta_estimator.h"

#include <tf/transform_datatypes.h>


void beta_estimator::Prepare(void)
{
 RunPeriod = RUN_PERIOD_DEFAULT;

 /* Retrieve parameters from ROS parameter server */
 std::string FullParamName;

 // run_period
 FullParamName = ros::this_node::getName()+"/run_period";

 if (false == Handle.getParam(FullParamName, RunPeriod))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // PD proportional gain
 FullParamName = ros::this_node::getName()+"/proportional_gain";

 if (false == Handle.getParam(FullParamName, Kp))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // PD derivative gain
 FullParamName = ros::this_node::getName()+"/derivative_gain";

 if (false == Handle.getParam(FullParamName, Kd))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // PD derivative time constant
 FullParamName = ros::this_node::getName()+"/derivative_time_const";

 if (false == Handle.getParam(FullParamName, T))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 /* ROS topics */
 vehiclePose_subscriber = Handle.subscribe("/vehiclePose", 1, &beta_estimator::vehiclePose_MessageCallback, this);
 vehicleSideslip_publisher = Handle.advertise<std_msgs::Float64>("/vehicleSideslip", 1);


 /* Initialize node state */
 _vehiclePose.push_back(0.0);
 _vehiclePose.push_back(0.0);
 _vehiclePose.push_back(0.0);
	
 _vehicleSideslip = 0.0;
	
 ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void beta_estimator::RunPeriodically(float Period)
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


void beta_estimator::Shutdown(void)
{
 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void beta_estimator::vehiclePose_MessageCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
 /* Vehicle 2D pose */
 _vehiclePose.at(0) = msg->x;
 _vehiclePose.at(1) = msg->y;
 _vehiclePose.at(2) = msg->theta;
}

void beta_estimator::PeriodicTask(void)
{
 /* Compute sideslip estimate */
 _vehicleSideslip = _vehiclePose.at(2);
 // ...put the algorithm here
	
 /* Publish sideslip value */
 std_msgs::Float64 msg;
 msg.data = _vehicleSideslip;
 vehicleSideslip_publisher.publish(msg);
}
