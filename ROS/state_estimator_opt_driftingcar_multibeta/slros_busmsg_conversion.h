#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include "state_estimator_opt_driftingcar_multibeta_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_state_estimator_opt_drift_Point_ca9vbs const* busPtr);
void convertToBus(SL_Bus_state_estimator_opt_drift_Point_ca9vbs* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_state_estimator_opt_drift_Pose_e9lon const* busPtr);
void convertToBus(SL_Bus_state_estimator_opt_drift_Pose_e9lon* busPtr, geometry_msgs::Pose const* msgPtr);

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb const* busPtr);
void convertToBus(SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb* busPtr, geometry_msgs::PoseStamped const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_state_estimator_opt_drift_Quaternion_ze4f3u const* busPtr);
void convertToBus(SL_Bus_state_estimator_opt_drift_Quaternion_ze4f3u* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_state_estimator_opt_drift_Time_ak69qm const* busPtr);
void convertToBus(SL_Bus_state_estimator_opt_drift_Time_ak69qm* busPtr, ros::Time const* msgPtr);

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_state_estimator_opt_drift_Float64_pfh567 const* busPtr);
void convertToBus(SL_Bus_state_estimator_opt_drift_Float64_pfh567* busPtr, std_msgs::Float64 const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_state_estimator_opt_drift_Header_icd2d2 const* busPtr);
void convertToBus(SL_Bus_state_estimator_opt_drift_Header_icd2d2* busPtr, std_msgs::Header const* msgPtr);


#endif
