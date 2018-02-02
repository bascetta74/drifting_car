#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk and geometry_msgs::Pose2D

void convertFromBus(geometry_msgs::Pose2D* msgPtr, SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose2D");

  msgPtr->theta =  busPtr->Theta;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
}

void convertToBus(SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk* busPtr, geometry_msgs::Pose2D const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose2D");

  busPtr->Theta =  msgPtr->theta;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
}


// Conversions between SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64 and std_msgs::Float64

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  msgPtr->data =  busPtr->Data;
}

void convertToBus(SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64* busPtr, std_msgs::Float64 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  busPtr->Data =  msgPtr->data;
}

