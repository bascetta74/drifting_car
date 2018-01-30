#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block WithoutIsNew/Subscribe
extern SimulinkSubscriber<geometry_msgs::PoseArray, SL_Bus_WithoutIsNew_geometry_msgs_PoseArray> Sub_WithoutIsNew_40;

// For Block WithoutIsNew/Publisher/Publish2
extern SimulinkPublisher<geometry_msgs::PointStamped, SL_Bus_WithoutIsNew_geometry_msgs_PointStamped> Pub_WithoutIsNew_66;

void slros_node_init(int argc, char** argv);

#endif
