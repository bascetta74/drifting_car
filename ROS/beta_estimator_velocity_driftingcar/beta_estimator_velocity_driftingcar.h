//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: beta_estimator_velocity_driftingcar.h
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
#ifndef RTW_HEADER_beta_estimator_velocity_driftingcar_h_
#define RTW_HEADER_beta_estimator_velocity_driftingcar_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef beta_estimator_velocity_driftingcar_COMMON_INCLUDES_
# define beta_estimator_velocity_driftingcar_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // beta_estimator_velocity_driftingcar_COMMON_INCLUDES_ 

#include "beta_estimator_velocity_driftingcar_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define beta_estimator_velocity_driftingcar_M (beta_estimator_velocity_drif_M)

// Block signals (auto storage)
typedef struct {
  char_T cv0[26];
  SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk In1;// '<S9>/In1'
  SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk varargout_2;
} B_beta_estimator_velocity_dri_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  robotics_slros_internal_block_T obj; // '<S2>/Get Parameter3'
  real_T DiscreteTimeIntegrator_DSTATE[3];// '<S1>/Discrete-Time Integrator'
  void *SourceBlock_PWORK;             // '<S8>/SourceBlock'
  void *SinkBlock_PWORK;               // '<S7>/SinkBlock'
  void *GetParameter3_PWORK;           // '<S2>/Get Parameter3'
  robotics_slros_internal_blo_j_T obj_a;// '<S7>/SinkBlock'
  robotics_slros_internal_bl_jx_T obj_l;// '<S8>/SourceBlock'
} DW_beta_estimator_velocity_dr_T;

// Parameters (auto storage)
struct P_beta_estimator_velocity_dri_T_ {
  SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk Out1_Y0;// Computed Parameter: Out1_Y0
                                                         //  Referenced by: '<S9>/Out1'

  SL_Bus_beta_estimator_velocity_d_Pose2D_nguzzk Constant_Value;// Computed Parameter: Constant_Value
                                                                //  Referenced by: '<S8>/Constant'

  real_T GetParameter3_SampleTime;     // Expression: SampleTime
                                       //  Referenced by: '<S2>/Get Parameter3'

  real_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                        //  Referenced by: '<S1>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                       //  Referenced by: '<S1>/Discrete-Time Integrator'

  real_T Kp_x_Gain;                    // Expression: 2*pi*100
                                       //  Referenced by: '<S1>/Kp_x'

  real_T Kp_y_Gain;                    // Expression: 2*pi*100
                                       //  Referenced by: '<S1>/Kp_y'

  SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64 Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                                      //  Referenced by: '<S6>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_beta_estimator_veloci_T {
  const char_T *errorStatus;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_beta_estimator_velocity_dri_T beta_estimator_velocity_drift_P;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B_beta_estimator_velocity_dri_T beta_estimator_velocity_drift_B;

// Block states (auto storage)
extern DW_beta_estimator_velocity_dr_T beta_estimator_velocity_drif_DW;

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
  extern void beta_estimator_velocity_driftingcar_initialize(void);
  extern void beta_estimator_velocity_driftingcar_step(void);
  extern void beta_estimator_velocity_driftingcar_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_beta_estimator_veloc_T *const beta_estimator_velocity_drif_M;

#ifdef __cplusplus

}
#endif

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
//  '<Root>' : 'beta_estimator_velocity_driftingcar'
//  '<S1>'   : 'beta_estimator_velocity_driftingcar/Beta estimator velocity'
//  '<S2>'   : 'beta_estimator_velocity_driftingcar/Parameters from parameter server'
//  '<S3>'   : 'beta_estimator_velocity_driftingcar/Publisher'
//  '<S4>'   : 'beta_estimator_velocity_driftingcar/Subscriber_To_Optitrack'
//  '<S5>'   : 'beta_estimator_velocity_driftingcar/Beta estimator velocity/unicycle_feedback_velocity'
//  '<S6>'   : 'beta_estimator_velocity_driftingcar/Publisher/Blank Message1'
//  '<S7>'   : 'beta_estimator_velocity_driftingcar/Publisher/Publish1'
//  '<S8>'   : 'beta_estimator_velocity_driftingcar/Subscriber_To_Optitrack/Subscribe'
//  '<S9>'   : 'beta_estimator_velocity_driftingcar/Subscriber_To_Optitrack/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_beta_estimator_velocity_driftingcar_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
