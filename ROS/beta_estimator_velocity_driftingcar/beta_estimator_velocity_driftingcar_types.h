//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: beta_estimator_velocity_driftingcar_types.h
//
// Code generated for Simulink model 'beta_estimator_velocity_driftingcar'.
//
// Model version                  : 1.38
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Fri Feb  2 11:56:10 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_beta_estimator_velocity_driftingcar_types_h_
#define RTW_HEADER_beta_estimator_velocity_driftingcar_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64_

// MsgType=std_msgs/Float64
typedef struct {
  real_T Data;
} SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk_
#define DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk_

// MsgType=geometry_msgs/Pose2D
typedef struct {
  real_T X;
  real_T Y;
  real_T Theta;
} SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  int32_T isInitialized;
  real_T SampleTime;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_j_T
#define typedef_robotics_slros_internal_blo_j_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_blo_j_T;

#endif                                 //typedef_robotics_slros_internal_blo_j_T

#ifndef typedef_robotics_slros_internal_bl_jx_T
#define typedef_robotics_slros_internal_bl_jx_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_bl_jx_T;

#endif                                 //typedef_robotics_slros_internal_bl_jx_T

#ifndef typedef_struct_T_beta_estimator_veloc_T
#define typedef_struct_T_beta_estimator_veloc_T

typedef struct {
  char_T f1[4];
} struct_T_beta_estimator_veloc_T;

#endif                                 //typedef_struct_T_beta_estimator_veloc_T

#ifndef typedef_struct_T_beta_estimator_vel_j_T
#define typedef_struct_T_beta_estimator_vel_j_T

typedef struct {
  char_T f1[8];
} struct_T_beta_estimator_vel_j_T;

#endif                                 //typedef_struct_T_beta_estimator_vel_j_T

#ifndef typedef_struct_T_beta_estimator_ve_jx_T
#define typedef_struct_T_beta_estimator_ve_jx_T

typedef struct {
  char_T f1[7];
} struct_T_beta_estimator_ve_jx_T;

#endif                                 //typedef_struct_T_beta_estimator_ve_jx_T

#ifndef typedef_struct_T_beta_estimator_v_jxl_T
#define typedef_struct_T_beta_estimator_v_jxl_T

typedef struct {
  char_T f1[8];
  char_T f2[4];
  char_T f3[6];
} struct_T_beta_estimator_v_jxl_T;

#endif                                 //typedef_struct_T_beta_estimator_v_jxl_T

#ifndef typedef_struct_T_beta_estimator__jxlq_T
#define typedef_struct_T_beta_estimator__jxlq_T

typedef struct {
  real_T f1[2];
} struct_T_beta_estimator__jxlq_T;

#endif                                 //typedef_struct_T_beta_estimator__jxlq_T

#ifndef typedef_struct_T_beta_estimator_jxlq1_T
#define typedef_struct_T_beta_estimator_jxlq1_T

typedef struct {
  char_T f1[8];
  char_T f2[7];
  char_T f3[6];
} struct_T_beta_estimator_jxlq1_T;

#endif                                 //typedef_struct_T_beta_estimator_jxlq1_T

// Parameters (auto storage)
typedef struct P_beta_estimator_velocity_dri_T_ P_beta_estimator_velocity_dri_T;

// Forward declaration for rtModel
typedef struct tag_RTM_beta_estimator_veloci_T RT_MODEL_beta_estimator_veloc_T;

#endif                                 // RTW_HEADER_beta_estimator_velocity_driftingcar_types_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
