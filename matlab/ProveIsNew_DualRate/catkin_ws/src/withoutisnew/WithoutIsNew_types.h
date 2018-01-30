//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: WithoutIsNew_types.h
//
// Code generated for Simulink model 'WithoutIsNew'.
//
// Model version                  : 1.29
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Fri Jan 26 16:53:38 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_WithoutIsNew_types_h_
#define RTW_HEADER_WithoutIsNew_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_WithoutIsNew_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_WithoutIsNew_ros_time_Time Stamp;
} SL_Bus_WithoutIsNew_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_WithoutIsNew_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_PointStamped_
#define DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_PointStamped_

// MsgType=geometry_msgs/PointStamped
typedef struct {
  // MsgType=std_msgs/Header
  SL_Bus_WithoutIsNew_std_msgs_Header Header;

  // MsgType=geometry_msgs/Point
  SL_Bus_WithoutIsNew_geometry_msgs_Point Point;
} SL_Bus_WithoutIsNew_geometry_msgs_PointStamped;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_Quaternion_

// MsgType=geometry_msgs/Quaternion
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_WithoutIsNew_geometry_msgs_Quaternion;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_Pose_

// MsgType=geometry_msgs/Pose
typedef struct {
  // MsgType=geometry_msgs/Point
  SL_Bus_WithoutIsNew_geometry_msgs_Point Position;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_WithoutIsNew_geometry_msgs_Quaternion Orientation;
} SL_Bus_WithoutIsNew_geometry_msgs_Pose;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_PoseArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_WithoutIsNew_geometry_msgs_PoseArray_

// MsgType=geometry_msgs/PoseArray
typedef struct {
  // MsgType=std_msgs/Header
  SL_Bus_WithoutIsNew_std_msgs_Header Header;

  // MsgType=geometry_msgs/Pose:IsVarLen=1:VarLenCategory=data:VarLenElem=Poses_SL_Info:TruncateAction=warn 
  SL_Bus_WithoutIsNew_geometry_msgs_Pose Poses[16];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Poses
  SL_Bus_ROSVariableLengthArrayInfo Poses_SL_Info;
} SL_Bus_WithoutIsNew_geometry_msgs_PoseArray;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_n_T
#define typedef_robotics_slros_internal_blo_n_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_blo_n_T;

#endif                                 //typedef_robotics_slros_internal_blo_n_T

#ifndef typedef_struct_T_WithoutIsNew_T
#define typedef_struct_T_WithoutIsNew_T

typedef struct {
  real_T f1[2];
} struct_T_WithoutIsNew_T;

#endif                                 //typedef_struct_T_WithoutIsNew_T

#ifndef typedef_struct_T_WithoutIsNew_n_T
#define typedef_struct_T_WithoutIsNew_n_T

typedef struct {
  char_T f1[4];
} struct_T_WithoutIsNew_n_T;

#endif                                 //typedef_struct_T_WithoutIsNew_n_T

#ifndef typedef_struct_T_WithoutIsNew_n3_T
#define typedef_struct_T_WithoutIsNew_n3_T

typedef struct {
  char_T f1[8];
} struct_T_WithoutIsNew_n3_T;

#endif                                 //typedef_struct_T_WithoutIsNew_n3_T

#ifndef typedef_struct_T_WithoutIsNew_n3q_T
#define typedef_struct_T_WithoutIsNew_n3q_T

typedef struct {
  char_T f1[7];
} struct_T_WithoutIsNew_n3q_T;

#endif                                 //typedef_struct_T_WithoutIsNew_n3q_T

#ifndef typedef_struct_T_WithoutIsNew_n3qt_T
#define typedef_struct_T_WithoutIsNew_n3qt_T

typedef struct {
  char_T f1[8];
  char_T f2[7];
  char_T f3[6];
} struct_T_WithoutIsNew_n3qt_T;

#endif                                 //typedef_struct_T_WithoutIsNew_n3qt_T

// Parameters (auto storage)
typedef struct P_WithoutIsNew_T_ P_WithoutIsNew_T;

// Forward declaration for rtModel
typedef struct tag_RTM_WithoutIsNew_T RT_MODEL_WithoutIsNew_T;

#endif                                 // RTW_HEADER_WithoutIsNew_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
