#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include "beta_estimator_acceleration_driftingcar_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Pose2D* msgPtr, SL_Bus_beta_estimator_accelerati_Pose2D_de4qul const* busPtr);
void convertToBus(SL_Bus_beta_estimator_accelerati_Pose2D_de4qul* busPtr, geometry_msgs::Pose2D const* msgPtr);

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_beta_estimator_accelerati_Float64_woid8t const* busPtr);
void convertToBus(SL_Bus_beta_estimator_accelerati_Float64_woid8t* busPtr, std_msgs::Float64 const* msgPtr);


#endif
