//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: beta_estimator_acceleration_driftingcar_types.h
//
// Code generated for Simulink model 'beta_estimator_acceleration_driftingcar'.
//
// Model version                  : 1.25
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Fri Feb  2 14:24:58 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_beta_estimator_acceleration_driftingcar_types_h_
#define RTW_HEADER_beta_estimator_acceleration_driftingcar_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_accelerati_Float64_woid8t_
#define DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_accelerati_Float64_woid8t_

// MsgType=std_msgs/Float64
typedef struct {
  real_T Data;
} SL_Bus_beta_estimator_accelerati_Float64_woid8t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_accelerati_Pose2D_de4qul_
#define DEFINED_TYPEDEF_FOR_SL_Bus_beta_estimator_accelerati_Pose2D_de4qul_

// MsgType=geometry_msgs/Pose2D
typedef struct {
  real_T X;
  real_T Y;
  real_T Theta;
} SL_Bus_beta_estimator_accelerati_Pose2D_de4qul;

#endif

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  int32_T isInitialized;
  real_T SampleTime;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef typedef_robotics_slros_internal_blo_c_T
#define typedef_robotics_slros_internal_blo_c_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_blo_c_T;

#endif                                 //typedef_robotics_slros_internal_blo_c_T

#ifndef typedef_robotics_slros_internal_bl_cc_T
#define typedef_robotics_slros_internal_bl_cc_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_bl_cc_T;

#endif                                 //typedef_robotics_slros_internal_bl_cc_T

#ifndef typedef_struct_T_beta_estimator_accel_T
#define typedef_struct_T_beta_estimator_accel_T

typedef struct {
  char_T f1[4];
} struct_T_beta_estimator_accel_T;

#endif                                 //typedef_struct_T_beta_estimator_accel_T

#ifndef typedef_struct_T_beta_estimator_acc_c_T
#define typedef_struct_T_beta_estimator_acc_c_T

typedef struct {
  char_T f1[8];
} struct_T_beta_estimator_acc_c_T;

#endif                                 //typedef_struct_T_beta_estimator_acc_c_T

#ifndef typedef_struct_T_beta_estimator_ac_cc_T
#define typedef_struct_T_beta_estimator_ac_cc_T

typedef struct {
  char_T f1[7];
} struct_T_beta_estimator_ac_cc_T;

#endif                                 //typedef_struct_T_beta_estimator_ac_cc_T

#ifndef typedef_struct_T_beta_estimator_a_cct_T
#define typedef_struct_T_beta_estimator_a_cct_T

typedef struct {
  char_T f1[8];
  char_T f2[4];
  char_T f3[6];
} struct_T_beta_estimator_a_cct_T;

#endif                                 //typedef_struct_T_beta_estimator_a_cct_T

#ifndef typedef_struct_T_beta_estimator__ccti_T
#define typedef_struct_T_beta_estimator__ccti_T

typedef struct {
  real_T f1[2];
} struct_T_beta_estimator__ccti_T;

#endif                                 //typedef_struct_T_beta_estimator__ccti_T

#ifndef typedef_struct_T_beta_estimator_cctik_T
#define typedef_struct_T_beta_estimator_cctik_T

typedef struct {
  char_T f1[8];
  char_T f2[7];
  char_T f3[6];
} struct_T_beta_estimator_cctik_T;

#endif                                 //typedef_struct_T_beta_estimator_cctik_T

// Parameters (auto storage)
typedef struct P_beta_estimator_acceleration_T_ P_beta_estimator_acceleration_T;

// Forward declaration for rtModel
typedef struct tag_RTM_beta_estimator_accele_T RT_MODEL_beta_estimator_accel_T;

#endif                                 // RTW_HEADER_beta_estimator_acceleration_driftingcar_types_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
