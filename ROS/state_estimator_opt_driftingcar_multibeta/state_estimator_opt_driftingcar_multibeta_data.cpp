//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: state_estimator_opt_driftingcar_multibeta_data.cpp
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
#include "state_estimator_opt_driftingcar_multibeta.h"
#include "state_estimator_opt_driftingcar_multibeta_private.h"

// Block parameters (auto storage)
P_state_estimator_opt_driftin_T state_estimator_opt_driftingc_P = {
  //  Variable: Den_PDa
  //  Referenced by:
  //    '<S32>/PD_x'
  //    '<S32>/PD_y'

  { 1.0, 0.1111 },
  100.0,                               // Variable: Kp
                                       //  Referenced by:
                                       //    '<S35>/Gain'
                                       //    '<S35>/Gain1'


  //  Variable: Num_PDa
  //  Referenced by:
  //    '<S32>/PD_x'
  //    '<S32>/PD_y'

  { 284030.0, -277080.0 },

  //  Variable: acc_integrator_initial_condition
  //  Referenced by: '<S32>/Discrete-Time Integrator'

  { 0.0, 0.0, 0.0, 0.1 },
  0.001,                               // Variable: ds_nonzero_threshold
                                       //  Referenced by:
                                       //    '<S1>/Switch1'
                                       //    '<S1>/Switch2'
                                       //    '<S1>/Switch3'

  0.1,                                 // Variable: p
                                       //  Referenced by: '<S35>/Constant'

  0.0,                                 // Mask Parameter: Difference1_ICPrevInput
                                       //  Referenced by: '<S8>/UD'

  0.0,                                 // Mask Parameter: Difference_ICPrevInput
                                       //  Referenced by: '<S7>/UD'

  0.0,                                 // Mask Parameter: Difference2_ICPrevInput
                                       //  Referenced by: '<S11>/UD'

  0.0,                                 // Mask Parameter: Difference1_ICPrevInput_n
                                       //  Referenced by: '<S10>/UD'

  0.0,                                 // Mask Parameter: Difference_ICPrevInput_d
                                       //  Referenced by: '<S9>/UD'


  {
    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    },                                 // Header

    {
      {
        0.0,                           // X
        0.0,                           // Y
        0.0                            // Z
      },                               // Position

      {
        0.0,                           // X
        0.0,                           // Y
        0.0,                           // Z
        0.0                            // W
      }                                // Orientation
    }                                  // Pose
  },                                   // Computed Parameter: Out1_Y0
                                       //  Referenced by: '<S31>/Out1'


  {
    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    },                                 // Header

    {
      {
        0.0,                           // X
        0.0,                           // Y
        0.0                            // Z
      },                               // Position

      {
        0.0,                           // X
        0.0,                           // Y
        0.0,                           // Z
        0.0                            // W
      }                                // Orientation
    }                                  // Pose
  },                                   // Computed Parameter: Constant_Value
                                       //  Referenced by: '<S27>/Constant'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S1>/Constant5'

  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S1>/Constant4'

  6.2831853071795862,                  // Expression: 2*pi
                                       //  Referenced by: '<S1>/Constant1'

  0.0,                                 // Computed Parameter: beta_opt_Y0
                                       //  Referenced by: '<S1>/beta_opt'

  0.0,                                 // Computed Parameter: theta_opt_2Pi_Y0
                                       //  Referenced by: '<S1>/theta_opt_2Pi'

  0.0,                                 // Computed Parameter: ds_isNonZero_Y0
                                       //  Referenced by: '<S1>/ds_isNonZero'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S1>/Memory1'

  5.0,                                 // Expression: 5
                                       //  Referenced by: '<S1>/Theta_Offset [rad]'

  3.1415926535897931,                  // Expression: pi
                                       //  Referenced by: '<S1>/Constant'

  6.2831853071795862,                  // Expression: 2*pi
                                       //  Referenced by: '<S1>/Constant2'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S1>/Memory2'

  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S1>/Switch'

  0.0,                                 // Computed Parameter: V_opt_Y0
                                       //  Referenced by: '<S2>/V_opt'

  6.2831853071795862,                  // Expression: 2*pi
                                       //  Referenced by: '<S33>/Constant1'

  6.2831853071795862,                  // Expression: 2*pi
                                       //  Referenced by: '<S36>/Constant1'

  0.001,                               // Computed Parameter: DiscreteTimeIntegrator_gainval
                                       //  Referenced by: '<S35>/Discrete-Time Integrator'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S35>/Discrete-Time Integrator'

  5.0,                                 // Expression: 5
                                       //  Referenced by: '<S36>/Theta_Offset [rad]'

  6.2831853071795862,                  // Expression: 2*pi
                                       //  Referenced by: '<S36>/Constant2'

  3.1415926535897931,                  // Expression: pi
                                       //  Referenced by: '<S36>/Constant'

  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S36>/Switch'

  6.2831853071795862,                  // Expression: 2*pi
                                       //  Referenced by: '<S36>/Constant3'

  1.0E-9,                              // Expression: 1e-9
                                       //  Referenced by: '<S4>/Gain'

  0.001,                               // Computed Parameter: DiscreteTimeIntegrator_gainva_j
                                       //  Referenced by: '<S32>/Discrete-Time Integrator'

  5.0,                                 // Expression: 5
                                       //  Referenced by: '<S33>/Theta_Offset [rad]'

  6.2831853071795862,                  // Expression: 2*pi
                                       //  Referenced by: '<S33>/Constant2'

  3.1415926535897931,                  // Expression: pi
                                       //  Referenced by: '<S33>/Constant'

  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S33>/Switch'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S32>/PD_x'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S32>/PD_y'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_mc
                                       //  Referenced by: '<S12>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_a
                                       //  Referenced by: '<S13>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_av
                                       //  Referenced by: '<S14>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_jx
                                       //  Referenced by: '<S15>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_b
                                       //  Referenced by: '<S16>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_l
                                       //  Referenced by: '<S17>/Constant'


  {
    0.0                                // Data
  }                                    // Computed Parameter: Constant_Value_p
                                       //  Referenced by: '<S18>/Constant'

};

//
// File trailer for generated code.
//
// [EOF]
//
