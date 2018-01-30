#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include "WithIsNew_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_WithIsNew_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_WithIsNew_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::PointStamped* msgPtr, SL_Bus_WithIsNew_geometry_msgs_PointStamped const* busPtr);
void convertToBus(SL_Bus_WithIsNew_geometry_msgs_PointStamped* busPtr, geometry_msgs::PointStamped const* msgPtr);

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_WithIsNew_geometry_msgs_Pose const* busPtr);
void convertToBus(SL_Bus_WithIsNew_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr);

void convertFromBus(geometry_msgs::PoseArray* msgPtr, SL_Bus_WithIsNew_geometry_msgs_PoseArray const* busPtr);
void convertToBus(SL_Bus_WithIsNew_geometry_msgs_PoseArray* busPtr, geometry_msgs::PoseArray const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_WithIsNew_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_WithIsNew_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_WithIsNew_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_WithIsNew_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_WithIsNew_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_WithIsNew_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif
