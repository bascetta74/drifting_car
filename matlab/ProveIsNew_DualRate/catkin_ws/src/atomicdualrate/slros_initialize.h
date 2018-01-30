#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block AtomicDualRate/Subscribe
extern SimulinkSubscriber<geometry_msgs::PoseArray, SL_Bus_AtomicDualRate_geometry_msgs_PoseArray> Sub_AtomicDualRate_40;

// For Block AtomicDualRate/Publisher_Fast/Publish2
extern SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_AtomicDualRate_geometry_msgs_PointStamped> Pub_AtomicDualRate_66;

// For Block AtomicDualRate/Publisher_Slow/Publish2
extern SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_AtomicDualRate_geometry_msgs_PointStamped> Pub_AtomicDualRate_90;

void slros_node_init(int argc, char** argv);

#endif
