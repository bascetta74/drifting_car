//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: state_estimator_opt_driftingcar_multibeta_types.h
//
// Code generated for Simulink model 'state_estimator_opt_driftingcar_multibeta'.
//
// Model version                  : 1.111
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Thu Oct 11 08:30:51 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_state_estimator_opt_driftingcar_multibeta_types_h_
#define RTW_HEADER_state_estimator_opt_driftingcar_multibeta_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Float64_pfh567_
#define DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Float64_pfh567_

// MsgType=std_msgs/Float64
typedef struct {
  real_T Data;
} SL_Bus_state_estimator_opt_drift_Float64_pfh567;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Time_ak69qm_
#define DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Time_ak69qm_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_state_estimator_opt_drift_Time_ak69qm;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Header_icd2d2_
#define DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Header_icd2d2_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_state_estimator_opt_drift_Time_ak69qm Stamp;
} SL_Bus_state_estimator_opt_drift_Header_icd2d2;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Point_ca9vbs_
#define DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Point_ca9vbs_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_state_estimator_opt_drift_Point_ca9vbs;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Quaternion_ze4f3u_
#define DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Quaternion_ze4f3u_

// MsgType=geometry_msgs/Quaternion
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_state_estimator_opt_drift_Quaternion_ze4f3u;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Pose_e9lon_
#define DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_Pose_e9lon_

// MsgType=geometry_msgs/Pose
typedef struct {
  // MsgType=geometry_msgs/Point
  SL_Bus_state_estimator_opt_drift_Point_ca9vbs Position;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_state_estimator_opt_drift_Quaternion_ze4f3u Orientation;
} SL_Bus_state_estimator_opt_drift_Pose_e9lon;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb_
#define DEFINED_TYPEDEF_FOR_SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb_

// MsgType=geometry_msgs/PoseStamped
typedef struct {
  // MsgType=std_msgs/Header
  SL_Bus_state_estimator_opt_drift_Header_icd2d2 Header;

  // MsgType=geometry_msgs/Pose
  SL_Bus_state_estimator_opt_drift_Pose_e9lon Pose;
} SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb;

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

#ifndef typedef_struct_T_state_estimator_opt__T
#define typedef_struct_T_state_estimator_opt__T

typedef struct {
  real_T f1[2];
} struct_T_state_estimator_opt__T;

#endif                                 //typedef_struct_T_state_estimator_opt__T

#ifndef typedef_struct_T_state_estimator_op_n_T
#define typedef_struct_T_state_estimator_op_n_T

typedef struct {
  char_T f1[4];
} struct_T_state_estimator_op_n_T;

#endif                                 //typedef_struct_T_state_estimator_op_n_T

#ifndef typedef_struct_T_state_estimator_o_nm_T
#define typedef_struct_T_state_estimator_o_nm_T

typedef struct {
  char_T f1[8];
} struct_T_state_estimator_o_nm_T;

#endif                                 //typedef_struct_T_state_estimator_o_nm_T

#ifndef typedef_struct_T_state_estimator__nmz_T
#define typedef_struct_T_state_estimator__nmz_T

typedef struct {
  char_T f1[7];
} struct_T_state_estimator__nmz_T;

#endif                                 //typedef_struct_T_state_estimator__nmz_T

#ifndef typedef_struct_T_state_estimator_nmzr_T
#define typedef_struct_T_state_estimator_nmzr_T

typedef struct {
  char_T f1[8];
  char_T f2[7];
  char_T f3[6];
} struct_T_state_estimator_nmzr_T;

#endif                                 //typedef_struct_T_state_estimator_nmzr_T

// Parameters (auto storage)
typedef struct P_state_estimator_opt_driftin_T_ P_state_estimator_opt_driftin_T;

// Forward declaration for rtModel
typedef struct tag_RTM_state_estimator_opt_d_T RT_MODEL_state_estimator_opt__T;

#endif                                 // RTW_HEADER_state_estimator_opt_driftingcar_multibeta_types_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
