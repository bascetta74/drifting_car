#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_beta_estimator_accelerati_Pose2D_de4qul and geometry_msgs::Pose2D

void convertFromBus(geometry_msgs::Pose2D* msgPtr, SL_Bus_beta_estimator_accelerati_Pose2D_de4qul const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose2D");

  msgPtr->theta =  busPtr->Theta;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
}

void convertToBus(SL_Bus_beta_estimator_accelerati_Pose2D_de4qul* busPtr, geometry_msgs::Pose2D const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose2D");

  busPtr->Theta =  msgPtr->theta;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
}


// Conversions between SL_Bus_beta_estimator_accelerati_Float64_woid8t and std_msgs::Float64

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_beta_estimator_accelerati_Float64_woid8t const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  msgPtr->data =  busPtr->Data;
}

void convertToBus(SL_Bus_beta_estimator_accelerati_Float64_woid8t* busPtr, std_msgs::Float64 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  busPtr->Data =  msgPtr->data;
}

