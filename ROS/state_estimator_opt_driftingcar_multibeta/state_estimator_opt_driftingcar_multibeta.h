//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: state_estimator_opt_driftingcar_multibeta.h
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
#ifndef RTW_HEADER_state_estimator_opt_driftingcar_multibeta_h_
#define RTW_HEADER_state_estimator_opt_driftingcar_multibeta_h_
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef state_estimator_opt_driftingcar_multibeta_COMMON_INCLUDES_
# define state_estimator_opt_driftingcar_multibeta_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "slros_initialize.h"
#endif                                 // state_estimator_opt_driftingcar_multibeta_COMMON_INCLUDES_ 

#include "state_estimator_opt_driftingcar_multibeta_types.h"
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

#define state_estimator_opt_driftingcar_multibeta_M (state_estimator_opt_drifting_M)

// Block signals (auto storage)
typedef struct {
  SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb In1;// '<S31>/In1'
  SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb varargout_2;
  char_T cv0[31];
  char_T cv1[30];
  char_T cv2[27];
  real_T Divide;                       // '<S2>/Divide'
  real_T Betapipi;                     // '<S1>/Switch'
  real_T Switch1;                      // '<S1>/Switch1'
  real_T Product3_m;                   // '<S28>/Product3'
  real_T Product2_o;                   // '<S28>/Product2'
  SL_Bus_state_estimator_opt_drift_Float64_pfh567 BusAssignment7;// '<S3>/Bus Assignment7' 
  SL_Bus_state_estimator_opt_drift_Float64_pfh567 BusAssignment6;// '<S3>/Bus Assignment6' 
  SL_Bus_state_estimator_opt_drift_Float64_pfh567 BusAssignment5;// '<S3>/Bus Assignment5' 
  SL_Bus_state_estimator_opt_drift_Float64_pfh567 BusAssignment4;// '<S3>/Bus Assignment4' 
  SL_Bus_state_estimator_opt_drift_Float64_pfh567 BusAssignment3;// '<S3>/Bus Assignment3' 
  SL_Bus_state_estimator_opt_drift_Float64_pfh567 BusAssignment2;// '<S3>/Bus Assignment2' 
  SL_Bus_state_estimator_opt_drift_Float64_pfh567 BusAssignment1;// '<S3>/Bus Assignment1' 
} B_state_estimator_opt_driftin_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE[3];// '<S35>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator_DSTATE_j[4];// '<S32>/Discrete-Time Integrator'
  real_T PD_x_states;                  // '<S32>/PD_x'
  real_T PD_y_states;                  // '<S32>/PD_y'
  real_T UD_DSTATE;                    // '<S11>/UD'
  real_T UD_DSTATE_m;                  // '<S10>/UD'
  real_T UD_DSTATE_a;                  // '<S9>/UD'
  real_T UD_DSTATE_mn;                 // '<S8>/UD'
  real_T UD_DSTATE_b;                  // '<S7>/UD'
  real_T Unwrap1_Prev;                 // '<S36>/Unwrap1'
  real_T Unwrap1_Cumsum;               // '<S36>/Unwrap1'
  real_T Unwrap1_Prev_o;               // '<S33>/Unwrap1'
  real_T Unwrap1_Cumsum_b;             // '<S33>/Unwrap1'
  real_T Unwrap1_Prev_j;               // '<S1>/Unwrap1'
  real_T Unwrap1_Cumsum_c;             // '<S1>/Unwrap1'
  real_T Memory1_PreviousInput;        // '<S1>/Memory1'
  real_T Memory2_PreviousInput;        // '<S1>/Memory2'
  real_T Unwrap_Prev;                  // '<S1>/Unwrap'
  real_T Unwrap_Cumsum;                // '<S1>/Unwrap'
  void *SourceBlock_PWORK;             // '<S27>/SourceBlock'
  void *SinkBlock_PWORK;               // '<S25>/SinkBlock'
  void *SinkBlock_PWORK_n;             // '<S24>/SinkBlock'
  void *SinkBlock_PWORK_nm;            // '<S23>/SinkBlock'
  void *SinkBlock_PWORK_o;             // '<S22>/SinkBlock'
  void *SinkBlock_PWORK_m;             // '<S21>/SinkBlock'
  void *SinkBlock_PWORK_h;             // '<S20>/SinkBlock'
  void *SinkBlock_PWORK_f;             // '<S19>/SinkBlock'
  robotics_slros_internal_block_T obj; // '<S25>/SinkBlock'
  robotics_slros_internal_block_T obj_c;// '<S24>/SinkBlock'
  robotics_slros_internal_block_T obj_g;// '<S23>/SinkBlock'
  robotics_slros_internal_block_T obj_b;// '<S22>/SinkBlock'
  robotics_slros_internal_block_T obj_d;// '<S21>/SinkBlock'
  robotics_slros_internal_block_T obj_cl;// '<S20>/SinkBlock'
  robotics_slros_internal_block_T obj_dt;// '<S19>/SinkBlock'
  robotics_slros_internal_blo_n_T obj_n;// '<S27>/SourceBlock'
  boolean_T Unwrap1_FirstStep;         // '<S36>/Unwrap1'
  boolean_T Unwrap1_FirstStep_f;       // '<S33>/Unwrap1'
  boolean_T Unwrap1_FirstStep_m;       // '<S1>/Unwrap1'
  boolean_T Unwrap_FirstStep;          // '<S1>/Unwrap'
} DW_state_estimator_opt_drifti_T;

// Parameters (auto storage)
struct P_state_estimator_opt_driftin_T_ {
  real_T Den_PDa[2];                   // Variable: Den_PDa
                                       //  Referenced by:
                                       //    '<S32>/PD_x'
                                       //    '<S32>/PD_y'

  real_T Kp;                           // Variable: Kp
                                       //  Referenced by:
                                       //    '<S35>/Gain'
                                       //    '<S35>/Gain1'

  real_T Num_PDa[2];                   // Variable: Num_PDa
                                       //  Referenced by:
                                       //    '<S32>/PD_x'
                                       //    '<S32>/PD_y'

  real_T acc_integrator_initial_condition[4];// Variable: acc_integrator_initial_condition
                                             //  Referenced by: '<S32>/Discrete-Time Integrator'

  real_T ds_nonzero_threshold;         // Variable: ds_nonzero_threshold
                                       //  Referenced by:
                                       //    '<S1>/Switch1'
                                       //    '<S1>/Switch2'
                                       //    '<S1>/Switch3'

  real_T p;                            // Variable: p
                                       //  Referenced by: '<S35>/Constant'

  real_T Difference1_ICPrevInput;      // Mask Parameter: Difference1_ICPrevInput
                                       //  Referenced by: '<S8>/UD'

  real_T Difference_ICPrevInput;       // Mask Parameter: Difference_ICPrevInput
                                       //  Referenced by: '<S7>/UD'

  real_T Difference2_ICPrevInput;      // Mask Parameter: Difference2_ICPrevInput
                                       //  Referenced by: '<S11>/UD'

  real_T Difference1_ICPrevInput_n;    // Mask Parameter: Difference1_ICPrevInput_n
                                       //  Referenced by: '<S10>/UD'

  real_T Difference_ICPrevInput_d;     // Mask Parameter: Difference_ICPrevInput_d
                                       //  Referenced by: '<S9>/UD'

  SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb Out1_Y0;// Computed Parameter: Out1_Y0
                                                              //  Referenced by: '<S31>/Out1'

  SL_Bus_state_estimator_opt_drift_PoseStamped_8suobb Constant_Value;// Computed Parameter: Constant_Value
                                                                     //  Referenced by: '<S27>/Constant'

  real_T Constant5_Value;              // Expression: 0
                                       //  Referenced by: '<S1>/Constant5'

  real_T Constant4_Value;              // Expression: 1
                                       //  Referenced by: '<S1>/Constant4'

  real_T Constant1_Value;              // Expression: 2*pi
                                       //  Referenced by: '<S1>/Constant1'

  real_T beta_opt_Y0;                  // Computed Parameter: beta_opt_Y0
                                       //  Referenced by: '<S1>/beta_opt'

  real_T theta_opt_2Pi_Y0;             // Computed Parameter: theta_opt_2Pi_Y0
                                       //  Referenced by: '<S1>/theta_opt_2Pi'

  real_T ds_isNonZero_Y0;              // Computed Parameter: ds_isNonZero_Y0
                                       //  Referenced by: '<S1>/ds_isNonZero'

  real_T Memory1_X0;                   // Expression: 0
                                       //  Referenced by: '<S1>/Memory1'

  real_T Theta_Offsetrad_Value;        // Expression: 5
                                       //  Referenced by: '<S1>/Theta_Offset [rad]'

  real_T Constant_Value_j;             // Expression: pi
                                       //  Referenced by: '<S1>/Constant'

  real_T Constant2_Value;              // Expression: 2*pi
                                       //  Referenced by: '<S1>/Constant2'

  real_T Memory2_X0;                   // Expression: 0
                                       //  Referenced by: '<S1>/Memory2'

  real_T Switch_Threshold;             // Expression: 1
                                       //  Referenced by: '<S1>/Switch'

  real_T V_opt_Y0;                     // Computed Parameter: V_opt_Y0
                                       //  Referenced by: '<S2>/V_opt'

  real_T Constant1_Value_h;            // Expression: 2*pi
                                       //  Referenced by: '<S33>/Constant1'

  real_T Constant1_Value_k;            // Expression: 2*pi
                                       //  Referenced by: '<S36>/Constant1'

  real_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                        //  Referenced by: '<S35>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                       //  Referenced by: '<S35>/Discrete-Time Integrator'

  real_T Theta_Offsetrad_Value_m;      // Expression: 5
                                       //  Referenced by: '<S36>/Theta_Offset [rad]'

  real_T Constant2_Value_i;            // Expression: 2*pi
                                       //  Referenced by: '<S36>/Constant2'

  real_T Constant_Value_js;            // Expression: pi
                                       //  Referenced by: '<S36>/Constant'

  real_T Switch_Threshold_a;           // Expression: 1
                                       //  Referenced by: '<S36>/Switch'

  real_T Constant3_Value;              // Expression: 2*pi
                                       //  Referenced by: '<S36>/Constant3'

  real_T Gain_Gain;                    // Expression: 1e-9
                                       //  Referenced by: '<S4>/Gain'

  real_T DiscreteTimeIntegrator_gainva_j;// Computed Parameter: DiscreteTimeIntegrator_gainva_j
                                         //  Referenced by: '<S32>/Discrete-Time Integrator'

  real_T Theta_Offsetrad_Value_d;      // Expression: 5
                                       //  Referenced by: '<S33>/Theta_Offset [rad]'

  real_T Constant2_Value_m;            // Expression: 2*pi
                                       //  Referenced by: '<S33>/Constant2'

  real_T Constant_Value_m;             // Expression: pi
                                       //  Referenced by: '<S33>/Constant'

  real_T Switch_Threshold_j;           // Expression: 1
                                       //  Referenced by: '<S33>/Switch'

  real_T PD_x_InitialStates;           // Expression: 0
                                       //  Referenced by: '<S32>/PD_x'

  real_T PD_y_InitialStates;           // Expression: 0
                                       //  Referenced by: '<S32>/PD_y'

  SL_Bus_state_estimator_opt_drift_Float64_pfh567 Constant_Value_mc;// Computed Parameter: Constant_Value_mc
                                                                    //  Referenced by: '<S12>/Constant'

  SL_Bus_state_estimator_opt_drift_Float64_pfh567 Constant_Value_a;// Computed Parameter: Constant_Value_a
                                                                   //  Referenced by: '<S13>/Constant'

  SL_Bus_state_estimator_opt_drift_Float64_pfh567 Constant_Value_av;// Computed Parameter: Constant_Value_av
                                                                    //  Referenced by: '<S14>/Constant'

  SL_Bus_state_estimator_opt_drift_Float64_pfh567 Constant_Value_jx;// Computed Parameter: Constant_Value_jx
                                                                    //  Referenced by: '<S15>/Constant'

  SL_Bus_state_estimator_opt_drift_Float64_pfh567 Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                   //  Referenced by: '<S16>/Constant'

  SL_Bus_state_estimator_opt_drift_Float64_pfh567 Constant_Value_l;// Computed Parameter: Constant_Value_l
                                                                   //  Referenced by: '<S17>/Constant'

  SL_Bus_state_estimator_opt_drift_Float64_pfh567 Constant_Value_p;// Computed Parameter: Constant_Value_p
                                                                   //  Referenced by: '<S18>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_state_estimator_opt_d_T {
  const char_T *errorStatus;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_state_estimator_opt_driftin_T state_estimator_opt_driftingc_P;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B_state_estimator_opt_driftin_T state_estimator_opt_driftingc_B;

// Block states (auto storage)
extern DW_state_estimator_opt_drifti_T state_estimator_opt_drifting_DW;

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
  extern void state_estimator_opt_driftingcar_multibeta_initialize(void);
  extern void state_estimator_opt_driftingcar_multibeta_step(void);
  extern void state_estimator_opt_driftingcar_multibeta_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_state_estimator_opt__T *const state_estimator_opt_drifting_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Constant3' : Unused code path elimination
//  Block '<S1>/Math Function3' : Unused code path elimination
//  Block '<S1>/ds' : Unused code path elimination
//  Block '<S1>/ds_nonzero' : Unused code path elimination
//  Block '<S33>/Constant3' : Unused code path elimination
//  Block '<S33>/Math Function3' : Unused code path elimination


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
//  '<Root>' : 'state_estimator_opt_driftingcar_multibeta'
//  '<S1>'   : 'state_estimator_opt_driftingcar_multibeta/BetaOptEstimator1'
//  '<S2>'   : 'state_estimator_opt_driftingcar_multibeta/Enabled Subsystem'
//  '<S3>'   : 'state_estimator_opt_driftingcar_multibeta/Publisher'
//  '<S4>'   : 'state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack'
//  '<S5>'   : 'state_estimator_opt_driftingcar_multibeta/beta estimator acceleration'
//  '<S6>'   : 'state_estimator_opt_driftingcar_multibeta/beta estimator velocity'
//  '<S7>'   : 'state_estimator_opt_driftingcar_multibeta/BetaOptEstimator1/Difference'
//  '<S8>'   : 'state_estimator_opt_driftingcar_multibeta/BetaOptEstimator1/Difference1'
//  '<S9>'   : 'state_estimator_opt_driftingcar_multibeta/Enabled Subsystem/Difference'
//  '<S10>'  : 'state_estimator_opt_driftingcar_multibeta/Enabled Subsystem/Difference1'
//  '<S11>'  : 'state_estimator_opt_driftingcar_multibeta/Enabled Subsystem/Difference2'
//  '<S12>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Blank Message1'
//  '<S13>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Blank Message2'
//  '<S14>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Blank Message3'
//  '<S15>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Blank Message4'
//  '<S16>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Blank Message5'
//  '<S17>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Blank Message6'
//  '<S18>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Blank Message7'
//  '<S19>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Publish1'
//  '<S20>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Publish2'
//  '<S21>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Publish3'
//  '<S22>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Publish4'
//  '<S23>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Publish5'
//  '<S24>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Publish6'
//  '<S25>'  : 'state_estimator_opt_driftingcar_multibeta/Publisher/Publish7'
//  '<S26>'  : 'state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack/Quaternions to Rotation Angles'
//  '<S27>'  : 'state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack/Subscribe'
//  '<S28>'  : 'state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S29>'  : 'state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S30>'  : 'state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S31>'  : 'state_estimator_opt_driftingcar_multibeta/Subscriber_To_Optitrack/Subscribe/Enabled Subsystem'
//  '<S32>'  : 'state_estimator_opt_driftingcar_multibeta/beta estimator acceleration/Beta estimator acceleration'
//  '<S33>'  : 'state_estimator_opt_driftingcar_multibeta/beta estimator acceleration/from_gamma_to_beta'
//  '<S34>'  : 'state_estimator_opt_driftingcar_multibeta/beta estimator acceleration/Beta estimator acceleration/unicycle_feedback_acceleration'
//  '<S35>'  : 'state_estimator_opt_driftingcar_multibeta/beta estimator velocity/Beta estimator velocity'
//  '<S36>'  : 'state_estimator_opt_driftingcar_multibeta/beta estimator velocity/from_gamma_to_beta'
//  '<S37>'  : 'state_estimator_opt_driftingcar_multibeta/beta estimator velocity/Beta estimator velocity/unicycle_feedback_velocity'

#endif                                 // RTW_HEADER_state_estimator_opt_driftingcar_multibeta_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
