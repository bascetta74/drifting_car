#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd and car_msgs::car_cmd

void convertFromBus(car_msgs::car_cmd* msgPtr, SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd const* busPtr)
{
  const std::string rosMessageType("car_msgs/car_cmd");

  msgPtr->speed_ref =  busPtr->SpeedRef;
  msgPtr->state =  busPtr->State;
  msgPtr->steer_ref =  busPtr->SteerRef;
}

void convertToBus(SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd* busPtr, car_msgs::car_cmd const* msgPtr)
{
  const std::string rosMessageType("car_msgs/car_cmd");

  busPtr->SpeedRef =  msgPtr->speed_ref;
  busPtr->State =  msgPtr->state;
  busPtr->SteerRef =  msgPtr->steer_ref;
}


// Conversions between SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4 and geometry_msgs::Quaternion

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  busPtr->W;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4* busPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->W =  msgPtr->w;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_drifting_controller_stabi_Vector3_6j7nya and geometry_msgs::Vector3

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_drifting_controller_stabi_Vector3_6j7nya const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_drifting_controller_stabi_Vector3_6j7nya* busPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_drifting_controller_stabilization_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_drifting_controller_stabilization_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_drifting_controller_stabilization_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu and sensor_msgs::Imu

void convertFromBus(sensor_msgs::Imu* msgPtr, SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/Imu");

  convertFromBus(&msgPtr->angular_velocity, &busPtr->AngularVelocity);
  convertFromBusFixedPrimitiveArray(msgPtr->angular_velocity_covariance, busPtr->AngularVelocityCovariance);
  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->linear_acceleration, &busPtr->LinearAcceleration);
  convertFromBusFixedPrimitiveArray(msgPtr->linear_acceleration_covariance, busPtr->LinearAccelerationCovariance);
  convertFromBus(&msgPtr->orientation, &busPtr->Orientation);
  convertFromBusFixedPrimitiveArray(msgPtr->orientation_covariance, busPtr->OrientationCovariance);
}

void convertToBus(SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu* busPtr, sensor_msgs::Imu const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/Imu");

  convertToBus(&busPtr->AngularVelocity, &msgPtr->angular_velocity);
  convertToBusFixedPrimitiveArray(busPtr->AngularVelocityCovariance, msgPtr->angular_velocity_covariance, slros::NoopWarning());
  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->LinearAcceleration, &msgPtr->linear_acceleration);
  convertToBusFixedPrimitiveArray(busPtr->LinearAccelerationCovariance, msgPtr->linear_acceleration_covariance, slros::NoopWarning());
  convertToBus(&busPtr->Orientation, &msgPtr->orientation);
  convertToBusFixedPrimitiveArray(busPtr->OrientationCovariance, msgPtr->orientation_covariance, slros::NoopWarning());
}


// Conversions between SL_Bus_drifting_controller_stabilization_std_msgs_Float64 and std_msgs::Float64

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_drifting_controller_stabilization_std_msgs_Float64 const* busPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  msgPtr->data =  busPtr->Data;
}

void convertToBus(SL_Bus_drifting_controller_stabilization_std_msgs_Float64* busPtr, std_msgs::Float64 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Float64");

  busPtr->Data =  msgPtr->data;
}


// Conversions between SL_Bus_drifting_controller_stabilization_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_drifting_controller_stabilization_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_drifting_controller_stabilization_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}

