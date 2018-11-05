//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: drifting_controller_stabilization_types.h
//
// Code generated for Simulink model 'drifting_controller_stabilization'.
//
// Model version                  : 1.102
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Fri Oct 26 14:53:10 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objective: Safety precaution
// Validation result: Not run
//
#ifndef RTW_HEADER_drifting_controller_stabilization_types_h_
#define RTW_HEADER_drifting_controller_stabilization_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd_
#define DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd_

// MsgType=car_msgs/car_cmd
typedef struct {
  real_T SpeedRef;
  real_T SteerRef;
  uint8_T State;
} SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_std_msgs_Float64_

// MsgType=std_msgs/Float64
typedef struct {
  real_T Data;
} SL_Bus_drifting_controller_stabilization_std_msgs_Float64;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_drifting_controller_stabilization_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_drifting_controller_stabilization_ros_time_Time Stamp;
} SL_Bus_drifting_controller_stabilization_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4_
#define DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4_

// MsgType=geometry_msgs/Quaternion
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabi_Vector3_6j7nya_
#define DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabi_Vector3_6j7nya_

// MsgType=geometry_msgs/Vector3
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_drifting_controller_stabi_Vector3_6j7nya;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu_
#define DEFINED_TYPEDEF_FOR_SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu_

// MsgType=sensor_msgs/Imu
typedef struct {
  real_T OrientationCovariance[9];
  real_T AngularVelocityCovariance[9];
  real_T LinearAccelerationCovariance[9];

  // MsgType=std_msgs/Header
  SL_Bus_drifting_controller_stabilization_std_msgs_Header Header;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_drifting_controller_stabi_Quaternion_nbuxf4 Orientation;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_drifting_controller_stabi_Vector3_6j7nya AngularVelocity;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_drifting_controller_stabi_Vector3_6j7nya LinearAcceleration;
} SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_a_T
#define typedef_robotics_slros_internal_blo_a_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_blo_a_T;

#endif                                 //typedef_robotics_slros_internal_blo_a_T

#ifndef typedef_struct_T_drifting_controller__T
#define typedef_struct_T_drifting_controller__T

typedef struct {
  real_T f1[2];
} struct_T_drifting_controller__T;

#endif                                 //typedef_struct_T_drifting_controller__T

#ifndef typedef_struct_T_drifting_controlle_a_T
#define typedef_struct_T_drifting_controlle_a_T

typedef struct {
  char_T f1[4];
} struct_T_drifting_controlle_a_T;

#endif                                 //typedef_struct_T_drifting_controlle_a_T

#ifndef typedef_struct_T_drifting_controll_ar_T
#define typedef_struct_T_drifting_controll_ar_T

typedef struct {
  char_T f1[8];
} struct_T_drifting_controll_ar_T;

#endif                                 //typedef_struct_T_drifting_controll_ar_T

#ifndef typedef_struct_T_drifting_control_aru_T
#define typedef_struct_T_drifting_control_aru_T

typedef struct {
  char_T f1[7];
} struct_T_drifting_control_aru_T;

#endif                                 //typedef_struct_T_drifting_control_aru_T

#ifndef typedef_struct_T_drifting_contro_arus_T
#define typedef_struct_T_drifting_contro_arus_T

typedef struct {
  char_T f1[8];
  char_T f2[7];
  char_T f3[6];
} struct_T_drifting_contro_arus_T;

#endif                                 //typedef_struct_T_drifting_contro_arus_T

// Parameters (auto storage)
typedef struct P_drifting_controller_stabili_T_ P_drifting_controller_stabili_T;

// Forward declaration for rtModel
typedef struct tag_RTM_drifting_controller_s_T RT_MODEL_drifting_controller__T;

#endif                                 // RTW_HEADER_drifting_controller_stabilization_types_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
