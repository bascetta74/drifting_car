#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "WithoutIsNew";

// For Block WithoutIsNew/Subscribe
SimulinkSubscriber<geometry_msgs::PoseArray, SL_Bus_WithoutIsNew_geometry_msgs_PoseArray> Sub_WithoutIsNew_40;

// For Block WithoutIsNew/Publisher/Publish2
SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_WithoutIsNew_geometry_msgs_PointStamped> Pub_WithoutIsNew_66;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

