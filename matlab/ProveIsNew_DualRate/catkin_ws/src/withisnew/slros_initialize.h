#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block WithIsNew/Subscribe
extern SimulinkSubscriber<geometry_msgs::PoseArray, SL_Bus_WithIsNew_geometry_msgs_PoseArray> Sub_WithIsNew_40;

// For Block WithIsNew/Signals Publisher/Publish2
extern SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_WithIsNew_geometry_msgs_PointStamped> Pub_WithIsNew_8;

void slros_node_init(int argc, char** argv);

#endif
