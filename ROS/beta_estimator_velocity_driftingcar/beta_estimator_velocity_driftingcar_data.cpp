//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: beta_estimator_velocity_driftingcar_data.cpp
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
#include "beta_estimator_velocity_driftingcar.h"
#include "beta_estimator_velocity_driftingcar_private.h"

// Block parameters (auto storage)
P_beta_estimator_velocity_dri_T beta_estimator_velocity_drift_P = {
  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Theta
  },                                   // Computed Parameter: Out1_Y0
                                       //  Referenced by: '<S9>/Out1'


  {
    0.0,                               // X
    0.0,                               // Y
    0.0                                // Theta
  },                                   // Computed Parameter: Constant_Value
                                       //  Referenced by: '<S8>/Constant'

  -1.0,                                // Expression: SampleTime
                                       //  Referenced by: '<S2>/Get Parameter3'

  0.01,                                // Computed Parameter: DiscreteTimeIntegrator_gainval
                                       //  Referenced by: '<S1>/Discrete-Time Integrator'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S1>/Discrete-Time Integrator'

  628.31853071795865,                  // Expression: 2*pi*100
                                       //  Referenced by: '<S1>/Kp_x'

  628.31853071795865,                  // Expression: 2*pi*100
                                       //  Referenced by: '<S1>/Kp_y'


  {
    0.0                                // Data
  }                                    // Computed Parameter: Constant_Value_p
                                       //  Referenced by: '<S6>/Constant'

};

//
// File trailer for generated code.
//
// [EOF]
//
