#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include "beta_estimator_velocity_driftingcar_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Pose2D* msgPtr, SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk const* busPtr);
void convertToBus(SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk* busPtr, geometry_msgs::Pose2D const* msgPtr);

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64 const* busPtr);
void convertToBus(SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64* busPtr, std_msgs::Float64 const* msgPtr);


#endif
