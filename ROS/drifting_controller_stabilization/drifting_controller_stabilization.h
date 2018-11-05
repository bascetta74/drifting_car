//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: drifting_controller_stabilization.h
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
#ifndef RTW_HEADER_drifting_controller_stabilization_h_
#define RTW_HEADER_drifting_controller_stabilization_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef drifting_controller_stabilization_COMMON_INCLUDES_
# define drifting_controller_stabilization_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 // drifting_controller_stabilization_COMMON_INCLUDES_ 

#include "drifting_controller_stabilization_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define drifting_controller_stabilization_M (drifting_controller_stabiliz_M)

// Block signals (auto storage)
typedef struct {
  SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu In1;// '<S25>/In1'
  SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu varargout_2;
  char_T cv0[26];
  SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd BusAssignment1;// '<S2>/Bus Assignment1' 
  real_T Product3_o;                   // '<S21>/Product3'
  real_T sqrt_m;                       // '<S19>/sqrt'
  real_T Product3_n;                   // '<S15>/Product3'
  real_T Product1_p;                   // '<S15>/Product1'
  real_T Saturation;                   // '<S1>/Saturation'
  real_T Subtract2;                    // '<S1>/Subtract2'
  real_T Subtract;                     // '<S1>/Subtract'
  real_T DiscreteTransferFcn1_tmp;
  SL_Bus_drifting_controller_stabilization_std_msgs_Float64 In1_m;// '<S26>/In1' 
  SL_Bus_drifting_controller_stabilization_std_msgs_Float64 In1_g;// '<S24>/In1' 
  SL_Bus_drifting_controller_stabilization_std_msgs_Float64 varargout_2_c;
} B_drifting_controller_stabili_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T DiscreteTransferFcn2_states;  // '<S1>/Discrete Transfer Fcn2'
  real_T DiscreteTransferFcn1_states;  // '<S1>/Discrete Transfer Fcn1'
  void *SourceBlock_PWORK;             // '<S12>/SourceBlock'
  void *SourceBlock_PWORK_d;           // '<S11>/SourceBlock'
  void *SourceBlock_PWORK_n;           // '<S10>/SourceBlock'
  void *SinkBlock_PWORK;               // '<S5>/SinkBlock'
  robotics_slros_internal_block_T obj; // '<S5>/SinkBlock'
  robotics_slros_internal_blo_a_T obj_k;// '<S12>/SourceBlock'
  robotics_slros_internal_blo_a_T obj_o;// '<S11>/SourceBlock'
  robotics_slros_internal_blo_a_T obj_b;// '<S10>/SourceBlock'
} DW_drifting_controller_stabil_T;

// Parameters (auto storage)
struct P_drifting_controller_stabili_T_ {
  real_T Den_Filter[2];                // Variable: Den_Filter
                                       //  Referenced by:
                                       //    '<S1>/Discrete Transfer Fcn1'
                                       //    '<S1>/Discrete Transfer Fcn2'

  real_T Fdrag;                        // Variable: Fdrag
                                       //  Referenced by: '<S1>/Constant5'

  real_T Fxr_eqpoint;                  // Variable: Fxr_eqpoint
                                       //  Referenced by: '<S1>/Constant4'

  real_T Imax;                         // Variable: Imax
                                       //  Referenced by: '<S1>/Gain1'

  real_T K_Vx_Fxr;                     // Variable: K_Vx_Fxr
                                       //  Referenced by: '<S1>/Gain4'

  real_T K_Vx_delta;                   // Variable: K_Vx_delta
                                       //  Referenced by: '<S1>/Gain'

  real_T K_Vy_Fxr;                     // Variable: K_Vy_Fxr
                                       //  Referenced by: '<S1>/Gain5'

  real_T K_Vy_delta;                   // Variable: K_Vy_delta
                                       //  Referenced by: '<S1>/Gain2'

  real_T K_r_Fxr;                      // Variable: K_r_Fxr
                                       //  Referenced by: '<S1>/Gain6'

  real_T K_r_delta;                    // Variable: K_r_delta
                                       //  Referenced by: '<S1>/Gain3'

  real_T Kt;                           // Variable: Kt
                                       //  Referenced by: '<S1>/Gain1'

  real_T Num_Filter[2];                // Variable: Num_Filter
                                       //  Referenced by:
                                       //    '<S1>/Discrete Transfer Fcn1'
                                       //    '<S1>/Discrete Transfer Fcn2'

  real_T Rw;                           // Variable: Rw
                                       //  Referenced by: '<S1>/Gain1'

  real_T Vx_eqpoint;                   // Variable: Vx_eqpoint
                                       //  Referenced by: '<S1>/Constant'

  real_T Vy_eqpoint;                   // Variable: Vy_eqpoint
                                       //  Referenced by: '<S1>/Constant1'

  real_T delta_eqpoint;                // Variable: delta_eqpoint
                                       //  Referenced by: '<S1>/Constant3'

  real_T delta_max;                    // Variable: delta_max
                                       //  Referenced by: '<S1>/Saturation'

  real_T delta_min;                    // Variable: delta_min
                                       //  Referenced by: '<S1>/Saturation'

  real_T r_eqpoint;                    // Variable: r_eqpoint
                                       //  Referenced by: '<S1>/Constant2'

  real_T tau;                          // Variable: tau
                                       //  Referenced by: '<S1>/Gain1'

  SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu Out1_Y0;// Computed Parameter: Out1_Y0
                                                                   //  Referenced by: '<S25>/Out1'

  SL_Bus_drifting_controller_stabilization_sensor_msgs_Imu Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S11>/Constant'

  SL_Bus_drifting_controller_stabilization_car_msgs_car_cmd Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                                      //  Referenced by: '<S4>/Constant'

  real_T DiscreteTransferFcn2_InitialSta;// Expression: 0
                                         //  Referenced by: '<S1>/Discrete Transfer Fcn2'

  real_T DiscreteTransferFcn1_InitialSta;// Expression: 0
                                         //  Referenced by: '<S1>/Discrete Transfer Fcn1'

  real_T Constant_Value_m;             // Expression: 0
                                       //  Referenced by: '<S3>/Constant'

  real_T u2_Gain;                      // Expression: 0.5
                                       //  Referenced by: '<S9>/1//2'

  real_T Gain_Gain;                    // Expression: 2
                                       //  Referenced by: '<S18>/Gain'

  real_T Gain1_Gain;                   // Expression: 2
                                       //  Referenced by: '<S18>/Gain1'

  real_T Constant_Value_d;             // Expression: 0.5
                                       //  Referenced by: '<S18>/Constant'

  real_T Gain2_Gain;                   // Expression: 2
                                       //  Referenced by: '<S18>/Gain2'

  real_T Saturation1_UpperSat;         // Expression: 1
                                       //  Referenced by: '<S1>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: -1
                                       //  Referenced by: '<S1>/Saturation1'

  SL_Bus_drifting_controller_stabilization_std_msgs_Float64 Out1_Y0_o;// Computed Parameter: Out1_Y0_o
                                                                      //  Referenced by: '<S24>/Out1'

  SL_Bus_drifting_controller_stabilization_std_msgs_Float64 Constant_Value_f;// Computed Parameter: Constant_Value_f
                                                                      //  Referenced by: '<S10>/Constant'

  SL_Bus_drifting_controller_stabilization_std_msgs_Float64 Out1_Y0_f;// Computed Parameter: Out1_Y0_f
                                                                      //  Referenced by: '<S26>/Out1'

  SL_Bus_drifting_controller_stabilization_std_msgs_Float64 Constant_Value_o;// Computed Parameter: Constant_Value_o
                                                                      //  Referenced by: '<S12>/Constant'

  uint8_T Constant2_Value;             // Computed Parameter: Constant2_Value
                                       //  Referenced by: '<Root>/Constant2'

};

// Real-time Model Data Structure
struct tag_RTM_drifting_controller_s_T {
  const char_T *errorStatus;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_drifting_controller_stabili_T drifting_controller_stabiliza_P;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B_drifting_controller_stabili_T drifting_controller_stabiliza_B;

// Block states (auto storage)
extern DW_drifting_controller_stabil_T drifting_controller_stabiliz_DW;

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void drifting_controller_stabilization_initialize(void);
  extern void drifting_controller_stabilization_step(void);
  extern void drifting_controller_stabilization_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_drifting_controller__T *const drifting_controller_stabiliz_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S16>/Constant' : Unused code path elimination
//  Block '<S16>/Gain' : Unused code path elimination
//  Block '<S16>/Gain1' : Unused code path elimination
//  Block '<S16>/Gain2' : Unused code path elimination
//  Block '<S16>/Product' : Unused code path elimination
//  Block '<S16>/Product1' : Unused code path elimination
//  Block '<S16>/Product2' : Unused code path elimination
//  Block '<S16>/Product3' : Unused code path elimination
//  Block '<S16>/Product4' : Unused code path elimination
//  Block '<S16>/Product5' : Unused code path elimination
//  Block '<S16>/Product6' : Unused code path elimination
//  Block '<S16>/Product7' : Unused code path elimination
//  Block '<S16>/Product8' : Unused code path elimination
//  Block '<S16>/Sum' : Unused code path elimination
//  Block '<S16>/Sum1' : Unused code path elimination
//  Block '<S16>/Sum2' : Unused code path elimination
//  Block '<S16>/Sum3' : Unused code path elimination
//  Block '<S17>/Constant' : Unused code path elimination
//  Block '<S17>/Gain' : Unused code path elimination
//  Block '<S17>/Gain1' : Unused code path elimination
//  Block '<S17>/Gain2' : Unused code path elimination
//  Block '<S17>/Product' : Unused code path elimination
//  Block '<S17>/Product1' : Unused code path elimination
//  Block '<S17>/Product2' : Unused code path elimination
//  Block '<S17>/Product3' : Unused code path elimination
//  Block '<S17>/Product4' : Unused code path elimination
//  Block '<S17>/Product5' : Unused code path elimination
//  Block '<S17>/Product6' : Unused code path elimination
//  Block '<S17>/Product7' : Unused code path elimination
//  Block '<S17>/Product8' : Unused code path elimination
//  Block '<S17>/Sum' : Unused code path elimination
//  Block '<S17>/Sum1' : Unused code path elimination
//  Block '<S17>/Sum2' : Unused code path elimination
//  Block '<S17>/Sum3' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'drifting_controller_stabilization'
//  '<S1>'   : 'drifting_controller_stabilization/Closed Loop Controller'
//  '<S2>'   : 'drifting_controller_stabilization/Publisher'
//  '<S3>'   : 'drifting_controller_stabilization/Subscriber '
//  '<S4>'   : 'drifting_controller_stabilization/Publisher/Blank Message1'
//  '<S5>'   : 'drifting_controller_stabilization/Publisher/Publish1'
//  '<S6>'   : 'drifting_controller_stabilization/Subscriber /Quaternion Inverse'
//  '<S7>'   : 'drifting_controller_stabilization/Subscriber /Quaternion Rotation'
//  '<S8>'   : 'drifting_controller_stabilization/Subscriber /Quaternions to Rotation Angles'
//  '<S9>'   : 'drifting_controller_stabilization/Subscriber /Rotation Angles to Quaternions'
//  '<S10>'  : 'drifting_controller_stabilization/Subscriber /Subscribe'
//  '<S11>'  : 'drifting_controller_stabilization/Subscriber /Subscribe1'
//  '<S12>'  : 'drifting_controller_stabilization/Subscriber /Subscribe2'
//  '<S13>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Inverse/Quaternion Conjugate'
//  '<S14>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Inverse/Quaternion Norm'
//  '<S15>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Rotation/Quaternion Normalize'
//  '<S16>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Rotation/V1'
//  '<S17>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Rotation/V2'
//  '<S18>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Rotation/V3'
//  '<S19>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Rotation/Quaternion Normalize/Quaternion Modulus'
//  '<S20>'  : 'drifting_controller_stabilization/Subscriber /Quaternion Rotation/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S21>'  : 'drifting_controller_stabilization/Subscriber /Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S22>'  : 'drifting_controller_stabilization/Subscriber /Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S23>'  : 'drifting_controller_stabilization/Subscriber /Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S24>'  : 'drifting_controller_stabilization/Subscriber /Subscribe/Enabled Subsystem'
//  '<S25>'  : 'drifting_controller_stabilization/Subscriber /Subscribe1/Enabled Subsystem'
//  '<S26>'  : 'drifting_controller_stabilization/Subscriber /Subscribe2/Enabled Subsystem'

#endif                                 // RTW_HEADER_drifting_controller_stabilization_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
