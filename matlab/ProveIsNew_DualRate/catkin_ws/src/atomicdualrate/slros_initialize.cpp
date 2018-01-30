#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "AtomicDualRate";

// For Block AtomicDualRate/Subscribe
SimulinkSubscriber<geometry_msgs::PoseArray, SL_Bus_AtomicDualRate_geometry_msgs_PoseArray> Sub_AtomicDualRate_40;

// For Block AtomicDualRate/Publisher_Fast/Publish2
SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_AtomicDualRate_geometry_msgs_PointStamped> Pub_AtomicDualRate_66;

// For Block AtomicDualRate/Publisher_Slow/Publish2
SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_AtomicDualRate_geometry_msgs_PointStamped> Pub_AtomicDualRate_90;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

