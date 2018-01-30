#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "WithIsNew";

// For Block WithIsNew/Subscribe
SimulinkSubscriber<geometry_msgs::PoseArray, SL_Bus_WithIsNew_geometry_msgs_PoseArray> Sub_WithIsNew_40;

// For Block WithIsNew/Signals Publisher/Publish2
SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_WithIsNew_geometry_msgs_PointStamped> Pub_WithIsNew_8;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

