#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "drifting_controller_stabilization";

// For Block drifting_controller_stabilization/Subscriber /Subscribe
SimulinkSubscriber<std_msgs::Float64, SL_Bus_drifting_controller_stabilization_std_msgs_Float64> Sub_drifting_controller_stabilization_129;

// For Block drifting_controller_stabilization/Subscriber /Subscribe1
SimulinkSubscriber<sensor_msgs::Imu, SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu> Sub_drifting_controller_stabilization_164;

// For Block drifting_controller_stabilization/Subscriber /Subscribe2
SimulinkSubscriber<std_msgs::Float64, SL_Bus_drifting_controller_stabilization_std_msgs_Float64> Sub_drifting_controller_stabilization_134;

// For Block drifting_controller_stabilization/Publisher/Publish1
SimulinkPublisher<car_msgs::car_cmd, SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd> Pub_drifting_controller_stabilization_75;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

