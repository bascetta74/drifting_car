//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: drifting_controller_stabilization_data.cpp
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
#include "drifting_controller_stabilization.h"
#include "drifting_controller_stabilization_private.h"

// Block parameters (auto storage)
P_drifting_controller_stabili_T drifting_controller_stabiliza_P = {
  //  Variable: Den_Filter
  //  Referenced by:
  //    '<S1>/Discrete Transfer Fcn1'
  //    '<S1>/Discrete Transfer Fcn2'

  { 1.0, 0.1111111111111111 },
  0.0,                                // Variable: Fdrag
                                       //  Referenced by: '<S1>/Constant5'

  3.006,                               // Variable: Fxr_eqpoint
                                       //  Referenced by: '<S1>/Constant4'

  13.0,                                // Variable: Imax
                                       //  Referenced by: '<S1>/Gain1'

  0.2621,                              // Variable: K_Vx_Fxr
                                       //  Referenced by: '<S1>/Gain4'

  -0.6106,                             // Variable: K_Vx_delta
                                       //  Referenced by: '<S1>/Gain'

  -4.3547,                             // Variable: K_Vy_Fxr
                                       //  Referenced by: '<S1>/Gain5'

  -0.9532,                             // Variable: K_Vy_delta
                                       //  Referenced by: '<S1>/Gain2'

  0.6018,                              // Variable: K_r_Fxr
                                       //  Referenced by: '<S1>/Gain6'

  0.2143,                              // Variable: K_r_delta
                                       //  Referenced by: '<S1>/Gain3'

  0.0029382382323558795,               // Variable: Kt
                                       //  Referenced by: '<S1>/Gain1'


  //  Variable: Num_Filter
  //  Referenced by:
  //    '<S1>/Discrete Transfer Fcn1'
  //    '<S1>/Discrete Transfer Fcn2'

  { 0.55555555555555558, 0.55555555555555558 },
  0.049,                               // Variable: Rw
                                       //  Referenced by: '<S1>/Gain1'

  1.0,                                 // Variable: Vx_eqpoint
                                       //  Referenced by: '<S1>/Constant'

  -0.788,                              // Variable: Vy_eqpoint
                                       //  Referenced by: '<S1>/Constant1'

  -0.3490658503988659,                 // Variable: delta_eqpoint
                                       //  Referenced by: '<S1>/Constant3'

  0.78539816339744828,                 // Variable: delta_max
                                       //  Referenced by: '<S1>/Saturation'

  -0.78539816339744828,                // Variable: delta_min
                                       //  Referenced by: '<S1>/Saturation'

  2.31807,                             // Variable: r_eqpoint
                                       //  Referenced by: '<S1>/Constant2'

  0.09799,                             // Variable: tau
                                       //  Referenced by: '<S1>/Gain1'


  {
    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // OrientationCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // AngularVelocityCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // LinearAccelerationCovariance

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
      0.0,                             // X
      0.0,                             // Y
      0.0,                             // Z
      0.0                              // W
    },                                 // Orientation

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // AngularVelocity

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // LinearAcceleration
  },                                   // Computed Parameter: Out1_Y0
                                       //  Referenced by: '<S25>/Out1'


  {
    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // OrientationCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // AngularVelocityCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // LinearAccelerationCovariance

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
      0.0,                             // X
      0.0,                             // Y
      0.0,                             // Z
      0.0                              // W
    },                                 // Orientation

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // AngularVelocity

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // LinearAcceleration
  },                                   // Computed Parameter: Constant_Value
                                       //  Referenced by: '<S11>/Constant'


  {
    0.0,                               // SpeedRef
    0.0,                               // SteerRef
    0U                                 // State
  },                                   // Computed Parameter: Constant_Value_p
                                       //  Referenced by: '<S4>/Constant'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S1>/Discrete Transfer Fcn2'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S1>/Discrete Transfer Fcn1'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S3>/Constant'

  0.5,                                 // Expression: 0.5
                                       //  Referenced by: '<S9>/1//2'

  2.0,                                 // Expression: 2
                                       //  Referenced by: '<S18>/Gain'

  2.0,                                 // Expression: 2
                                       //  Referenced by: '<S18>/Gain1'

  0.5,                                 // Expression: 0.5
                                       //  Referenced by: '<S18>/Constant'

  2.0,                                 // Expression: 2
                                       //  Referenced by: '<S18>/Gain2'

  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S1>/Saturation1'

  -1.0,                                // Expression: -1
                                       //  Referenced by: '<S1>/Saturation1'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Out1_Y0_o
                                       //  Referenced by: '<S24>/Out1'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_f
                                       //  Referenced by: '<S10>/Constant'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Out1_Y0_f
                                       //  Referenced by: '<S26>/Out1'


  {
    0.0                                // Data
  },                                   // Computed Parameter: Constant_Value_o
                                       //  Referenced by: '<S12>/Constant'

  2U                                   // Computed Parameter: Constant2_Value
                                       //  Referenced by: '<Root>/Constant2'

};

//
// File trailer for generated code.
//
// [EOF]
//
