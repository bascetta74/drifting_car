#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_WithIsNew_geometry_msgs_Point and geometry_msgs::Point

void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_WithIsNew_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_WithIsNew_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_WithIsNew_geometry_msgs_PointStamped and geometry_msgs::PointStamped

void convertFromBus(geometry_msgs::PointStamped* msgPtr, SL_Bus_WithIsNew_geometry_msgs_PointStamped const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PointStamped");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->point, &busPtr->Point);
}

void convertToBus(SL_Bus_WithIsNew_geometry_msgs_PointStamped* busPtr, geometry_msgs::PointStamped const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PointStamped");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->Point, &msgPtr->point);
}


// Conversions between SL_Bus_WithIsNew_geometry_msgs_Pose and geometry_msgs::Pose

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_WithIsNew_geometry_msgs_Pose const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertFromBus(&msgPtr->orientation, &busPtr->Orientation);
  convertFromBus(&msgPtr->position, &busPtr->Position);
}

void convertToBus(SL_Bus_WithIsNew_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  convertToBus(&busPtr->Orientation, &msgPtr->orientation);
  convertToBus(&busPtr->Position, &msgPtr->position);
}


// Conversions between SL_Bus_WithIsNew_geometry_msgs_PoseArray and geometry_msgs::PoseArray

void convertFromBus(geometry_msgs::PoseArray* msgPtr, SL_Bus_WithIsNew_geometry_msgs_PoseArray const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseArray");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBusVariableNestedArray(msgPtr->poses, busPtr->Poses, busPtr->Poses_SL_Info);
}

void convertToBus(SL_Bus_WithIsNew_geometry_msgs_PoseArray* busPtr, geometry_msgs::PoseArray const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseArray");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBusVariableNestedArray(busPtr->Poses, busPtr->Poses_SL_Info, msgPtr->poses, slros::EnabledWarning(rosMessageType, "poses"));
}


// Conversions between SL_Bus_WithIsNew_geometry_msgs_Quaternion and geometry_msgs::Quaternion

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_WithIsNew_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  busPtr->W;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_WithIsNew_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->W =  msgPtr->w;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_WithIsNew_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_WithIsNew_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_WithIsNew_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_WithIsNew_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_WithIsNew_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_WithIsNew_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}

