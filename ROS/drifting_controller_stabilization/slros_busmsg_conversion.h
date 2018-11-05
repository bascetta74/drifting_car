#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <car_msgs/car_cmd.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include "drifting_controller_stabilization_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(car_msgs::car_cmd* msgPtr, SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd const* busPtr);
void convertToBus(SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd* busPtr, car_msgs::car_cmd const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4 const* busPtr);
void convertToBus(SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_drifting_controller_stabi_Vector3_6j7nya const* busPtr);
void convertToBus(SL_Bus_drifting_controller_stabi_Vector3_6j7nya* busPtr, geometry_msgs::Vector3 const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_drifting_controller_stabilization_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_drifting_controller_stabilization_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::Imu* msgPtr, SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu const* busPtr);
void convertToBus(SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu* busPtr, sensor_msgs::Imu const* msgPtr);

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_drifting_controller_stabilization_std_msgs_Float64 const* busPtr);
void convertToBus(SL_Bus_drifting_controller_stabilization_std_msgs_Float64* busPtr, std_msgs::Float64 const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_drifting_controller_stabilization_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_drifting_controller_stabilization_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif
