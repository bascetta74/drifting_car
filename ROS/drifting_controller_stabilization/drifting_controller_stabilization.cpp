//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: drifting_controller_stabilization.cpp
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
#define drifting_contro_MessageQueueLen (1)

// Block signals (auto storage)
B_drifting_controller_stabili_T drifting_controller_stabiliza_B;

// Block states (auto storage)
DW_drifting_controller_stabil_T drifting_controller_stabiliz_DW;

// Real-time model
RT_MODEL_drifting_controller__T drifting_controller_stabiliz_M_;
RT_MODEL_drifting_controller__T *const drifting_controller_stabiliz_M =
  &drifting_controller_stabiliz_M_;
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u1 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u0 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2((real_T)tmp_0, (real_T)tmp);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

// Model step function
void drifting_controller_stabilization_step(void)
{
  boolean_T varargout_1;
  real_T DiscreteTransferFcn2_tmp;
  real_T rtb_sincos_o1;
  real_T rtb_sincos_o1_idx_0;
  real_T rtb_sincos_o2_idx_0;
  real_T rtb_sincos_o2_idx_1;
  real_T rtb_sincos_o1_idx_1;

  // Outputs for Atomic SubSystem: '<S3>/Subscribe'
  // Start for MATLABSystem: '<S10>/SourceBlock' incorporates:
  //   Inport: '<S24>/In1'
  //   MATLABSystem: '<S10>/SourceBlock'

  varargout_1 = Sub_drifting_controller_stabilization_129.getLatestMessage
    (&drifting_controller_stabiliza_B.varargout_2_c);

  // Outputs for Enabled SubSystem: '<S10>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S24>/Enable'

  if (varargout_1) {
    drifting_controller_stabiliza_B.In1_g =
      drifting_controller_stabiliza_B.varargout_2_c;
  }

  // End of Start for MATLABSystem: '<S10>/SourceBlock'
  // End of Outputs for SubSystem: '<S10>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S3>/Subscribe'

  // DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn2'
  DiscreteTransferFcn2_tmp = (drifting_controller_stabiliza_B.In1_g.Data -
    drifting_controller_stabiliza_P.Den_Filter[1] *
    drifting_controller_stabiliz_DW.DiscreteTransferFcn2_states) /
    drifting_controller_stabiliza_P.Den_Filter[0];
  drifting_controller_stabiliza_B.Product3_o =
    drifting_controller_stabiliza_P.Num_Filter[0] * DiscreteTransferFcn2_tmp +
    drifting_controller_stabiliza_P.Num_Filter[1] *
    drifting_controller_stabiliz_DW.DiscreteTransferFcn2_states;

  // Outputs for Atomic SubSystem: '<S3>/Subscribe2'
  // Start for MATLABSystem: '<S12>/SourceBlock' incorporates:
  //   Inport: '<S26>/In1'
  //   MATLABSystem: '<S12>/SourceBlock'

  varargout_1 = Sub_drifting_controller_stabilization_134.getLatestMessage
    (&drifting_controller_stabiliza_B.varargout_2_c);

  // Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S26>/Enable'

  if (varargout_1) {
    drifting_controller_stabiliza_B.In1_m =
      drifting_controller_stabiliza_B.varargout_2_c;
  }

  // End of Start for MATLABSystem: '<S12>/SourceBlock'
  // End of Outputs for SubSystem: '<S12>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S3>/Subscribe2'

  // DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1'
  drifting_controller_stabiliza_B.DiscreteTransferFcn1_tmp =
    (drifting_controller_stabiliza_B.In1_m.Data -
     drifting_controller_stabiliza_P.Den_Filter[1] *
     drifting_controller_stabiliz_DW.DiscreteTransferFcn1_states) /
    drifting_controller_stabiliza_P.Den_Filter[0];
  drifting_controller_stabiliza_B.Subtract2 =
    drifting_controller_stabiliza_P.Num_Filter[0] *
    drifting_controller_stabiliza_B.DiscreteTransferFcn1_tmp +
    drifting_controller_stabiliza_P.Num_Filter[1] *
    drifting_controller_stabiliz_DW.DiscreteTransferFcn1_states;

  // Sum: '<S1>/Subtract' incorporates:
  //   Constant: '<S1>/Constant'
  //   Product: '<S1>/Product1'
  //   Trigonometry: '<S1>/Trigonometric Function1'

  drifting_controller_stabiliza_B.Subtract = cos
    (drifting_controller_stabiliza_B.Product3_o) *
    drifting_controller_stabiliza_B.Subtract2 -
    drifting_controller_stabiliza_P.Vx_eqpoint;

  // Sum: '<S1>/Subtract2' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Product: '<S1>/Product'
  //   Trigonometry: '<S1>/Trigonometric Function'

  drifting_controller_stabiliza_B.Subtract2 = sin
    (drifting_controller_stabiliza_B.Product3_o) *
    drifting_controller_stabiliza_B.Subtract2 -
    drifting_controller_stabiliza_P.Vy_eqpoint;

  // Outputs for Atomic SubSystem: '<S3>/Subscribe1'
  // Start for MATLABSystem: '<S11>/SourceBlock' incorporates:
  //   Inport: '<S25>/In1'
  //   MATLABSystem: '<S11>/SourceBlock'

  varargout_1 = Sub_drifting_controller_stabilization_164.getLatestMessage
    (&drifting_controller_stabiliza_B.varargout_2);

  // Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S25>/Enable'

  if (varargout_1) {
    drifting_controller_stabiliza_B.In1 =
      drifting_controller_stabiliza_B.varargout_2;
  }

  // End of Start for MATLABSystem: '<S11>/SourceBlock'
  // End of Outputs for SubSystem: '<S11>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S3>/Subscribe1'

  // Sqrt: '<S22>/sqrt' incorporates:
  //   Product: '<S23>/Product'
  //   Product: '<S23>/Product1'
  //   Product: '<S23>/Product2'
  //   Product: '<S23>/Product3'
  //   Sum: '<S23>/Sum'

  drifting_controller_stabiliza_B.Product3_o = sqrt
    (((drifting_controller_stabiliza_B.In1.Orientation.W *
       drifting_controller_stabiliza_B.In1.Orientation.W +
       drifting_controller_stabiliza_B.In1.Orientation.X *
       drifting_controller_stabiliza_B.In1.Orientation.X) +
      drifting_controller_stabiliza_B.In1.Orientation.Y *
      drifting_controller_stabiliza_B.In1.Orientation.Y) +
     drifting_controller_stabiliza_B.In1.Orientation.Z *
     drifting_controller_stabiliza_B.In1.Orientation.Z);

  // Product: '<S21>/Product'
  drifting_controller_stabiliza_B.Product1_p =
    drifting_controller_stabiliza_B.In1.Orientation.W /
    drifting_controller_stabiliza_B.Product3_o;

  // Product: '<S21>/Product1'
  drifting_controller_stabiliza_B.Product3_n =
    drifting_controller_stabiliza_B.In1.Orientation.X /
    drifting_controller_stabiliza_B.Product3_o;

  // Product: '<S21>/Product2'
  drifting_controller_stabiliza_B.sqrt_m =
    drifting_controller_stabiliza_B.In1.Orientation.Y /
    drifting_controller_stabiliza_B.Product3_o;

  // Product: '<S21>/Product3'
  drifting_controller_stabiliza_B.Product3_o =
    drifting_controller_stabiliza_B.In1.Orientation.Z /
    drifting_controller_stabiliza_B.Product3_o;

  // Fcn: '<S8>/fcn3'
  rtb_sincos_o1_idx_1 = (drifting_controller_stabiliza_B.Product3_n *
    drifting_controller_stabiliza_B.Product3_o -
    drifting_controller_stabiliza_B.Product1_p *
    drifting_controller_stabiliza_B.sqrt_m) * -2.0;

  // Trigonometry: '<S8>/trigFcn'
  if (rtb_sincos_o1_idx_1 > 1.0) {
    rtb_sincos_o1_idx_1 = 1.0;
  } else {
    if (rtb_sincos_o1_idx_1 < -1.0) {
      rtb_sincos_o1_idx_1 = -1.0;
    }
  }

  // Fcn: '<S8>/fcn4'
  drifting_controller_stabiliza_B.Saturation =
    (drifting_controller_stabiliza_B.sqrt_m *
     drifting_controller_stabiliza_B.Product3_o +
     drifting_controller_stabiliza_B.Product1_p *
     drifting_controller_stabiliza_B.Product3_n) * 2.0;

  // Fcn: '<S8>/fcn5'
  drifting_controller_stabiliza_B.Product1_p =
    ((drifting_controller_stabiliza_B.Product1_p *
      drifting_controller_stabiliza_B.Product1_p -
      drifting_controller_stabiliza_B.Product3_n *
      drifting_controller_stabiliza_B.Product3_n) -
     drifting_controller_stabiliza_B.sqrt_m *
     drifting_controller_stabiliza_B.sqrt_m) +
    drifting_controller_stabiliza_B.Product3_o *
    drifting_controller_stabiliza_B.Product3_o;

  // Gain: '<S9>/1//2' incorporates:
  //   Constant: '<S3>/Constant'
  //   Trigonometry: '<S8>/Trigonometric Function3'
  //   Trigonometry: '<S8>/trigFcn'

  rtb_sincos_o1_idx_0 = drifting_controller_stabiliza_P.u2_Gain *
    drifting_controller_stabiliza_P.Constant_Value_m;
  rtb_sincos_o1_idx_1 = drifting_controller_stabiliza_P.u2_Gain * asin
    (rtb_sincos_o1_idx_1);
  drifting_controller_stabiliza_B.Product3_o =
    drifting_controller_stabiliza_P.u2_Gain * rt_atan2d_snf
    (drifting_controller_stabiliza_B.Saturation,
     drifting_controller_stabiliza_B.Product1_p);

  // Trigonometry: '<S9>/sincos'
  rtb_sincos_o2_idx_0 = cos(rtb_sincos_o1_idx_0);
  rtb_sincos_o1_idx_0 = sin(rtb_sincos_o1_idx_0);
  rtb_sincos_o2_idx_1 = cos(rtb_sincos_o1_idx_1);
  rtb_sincos_o1_idx_1 = sin(rtb_sincos_o1_idx_1);
  drifting_controller_stabiliza_B.sqrt_m = cos
    (drifting_controller_stabiliza_B.Product3_o);
  rtb_sincos_o1 = sin(drifting_controller_stabiliza_B.Product3_o);

  // Fcn: '<S9>/q1' incorporates:
  //   Trigonometry: '<S9>/sincos'

  drifting_controller_stabiliza_B.Saturation = rtb_sincos_o2_idx_0 *
    rtb_sincos_o2_idx_1 * rtb_sincos_o1 - rtb_sincos_o1_idx_0 *
    rtb_sincos_o1_idx_1 * drifting_controller_stabiliza_B.sqrt_m;

  // UnaryMinus: '<S13>/Unary Minus'
  drifting_controller_stabiliza_B.Product1_p =
    -drifting_controller_stabiliza_B.Saturation;

  // Fcn: '<S9>/q0' incorporates:
  //   Trigonometry: '<S9>/sincos'

  drifting_controller_stabiliza_B.Product3_o = rtb_sincos_o2_idx_0 *
    rtb_sincos_o2_idx_1 * drifting_controller_stabiliza_B.sqrt_m +
    rtb_sincos_o1_idx_0 * rtb_sincos_o1_idx_1 * rtb_sincos_o1;

  // Product: '<S14>/Product1'
  drifting_controller_stabiliza_B.Product3_n =
    drifting_controller_stabiliza_B.Saturation *
    drifting_controller_stabiliza_B.Saturation;

  // Fcn: '<S9>/q2' incorporates:
  //   Trigonometry: '<S9>/sincos'

  drifting_controller_stabiliza_B.Saturation = rtb_sincos_o2_idx_0 *
    rtb_sincos_o1_idx_1 * drifting_controller_stabiliza_B.sqrt_m +
    rtb_sincos_o1_idx_0 * rtb_sincos_o2_idx_1 * rtb_sincos_o1;

  // Fcn: '<S9>/q3' incorporates:
  //   Trigonometry: '<S9>/sincos'

  drifting_controller_stabiliza_B.sqrt_m = rtb_sincos_o1_idx_0 *
    rtb_sincos_o2_idx_1 * drifting_controller_stabiliza_B.sqrt_m -
    rtb_sincos_o2_idx_0 * rtb_sincos_o1_idx_1 * rtb_sincos_o1;

  // Sum: '<S14>/Sum' incorporates:
  //   Product: '<S14>/Product'
  //   Product: '<S14>/Product2'
  //   Product: '<S14>/Product3'

  drifting_controller_stabiliza_B.Product3_n =
    ((drifting_controller_stabiliza_B.Product3_o *
      drifting_controller_stabiliza_B.Product3_o +
      drifting_controller_stabiliza_B.Product3_n) +
     drifting_controller_stabiliza_B.Saturation *
     drifting_controller_stabiliza_B.Saturation) +
    drifting_controller_stabiliza_B.sqrt_m *
    drifting_controller_stabiliza_B.sqrt_m;

  // Product: '<S6>/Divide1'
  drifting_controller_stabiliza_B.Product1_p /=
    drifting_controller_stabiliza_B.Product3_n;

  // Product: '<S6>/Divide'
  drifting_controller_stabiliza_B.Product3_o /=
    drifting_controller_stabiliza_B.Product3_n;

  // Product: '<S6>/Divide2' incorporates:
  //   UnaryMinus: '<S13>/Unary Minus1'

  drifting_controller_stabiliza_B.Saturation =
    -drifting_controller_stabiliza_B.Saturation /
    drifting_controller_stabiliza_B.Product3_n;

  // Product: '<S6>/Divide3' incorporates:
  //   UnaryMinus: '<S13>/Unary Minus2'

  drifting_controller_stabiliza_B.Product3_n =
    -drifting_controller_stabiliza_B.sqrt_m /
    drifting_controller_stabiliza_B.Product3_n;

  // Sqrt: '<S19>/sqrt' incorporates:
  //   Product: '<S20>/Product'
  //   Product: '<S20>/Product1'
  //   Product: '<S20>/Product2'
  //   Product: '<S20>/Product3'
  //   Sum: '<S20>/Sum'

  drifting_controller_stabiliza_B.sqrt_m = sqrt
    (((drifting_controller_stabiliza_B.Product3_o *
       drifting_controller_stabiliza_B.Product3_o +
       drifting_controller_stabiliza_B.Product1_p *
       drifting_controller_stabiliza_B.Product1_p) +
      drifting_controller_stabiliza_B.Saturation *
      drifting_controller_stabiliza_B.Saturation) +
     drifting_controller_stabiliza_B.Product3_n *
     drifting_controller_stabiliza_B.Product3_n);

  // Product: '<S15>/Product1'
  drifting_controller_stabiliza_B.Product1_p /=
    drifting_controller_stabiliza_B.sqrt_m;

  // Product: '<S15>/Product3'
  drifting_controller_stabiliza_B.Product3_n /=
    drifting_controller_stabiliza_B.sqrt_m;

  // Product: '<S15>/Product'
  drifting_controller_stabiliza_B.Product3_o /=
    drifting_controller_stabiliza_B.sqrt_m;

  // Product: '<S15>/Product2'
  drifting_controller_stabiliza_B.Saturation /=
    drifting_controller_stabiliza_B.sqrt_m;

  // Sum: '<S1>/Subtract1' incorporates:
  //   Constant: '<S18>/Constant'
  //   Constant: '<S1>/Constant2'
  //   Gain: '<S18>/Gain'
  //   Gain: '<S18>/Gain1'
  //   Gain: '<S18>/Gain2'
  //   Product: '<S18>/Product'
  //   Product: '<S18>/Product1'
  //   Product: '<S18>/Product2'
  //   Product: '<S18>/Product3'
  //   Product: '<S18>/Product4'
  //   Product: '<S18>/Product5'
  //   Product: '<S18>/Product6'
  //   Product: '<S18>/Product7'
  //   Product: '<S18>/Product8'
  //   Sum: '<S18>/Sum'
  //   Sum: '<S18>/Sum1'
  //   Sum: '<S18>/Sum2'
  //   Sum: '<S18>/Sum3'

  drifting_controller_stabiliza_B.Saturation =
    (((drifting_controller_stabiliza_B.Product1_p *
       drifting_controller_stabiliza_B.Product3_n +
       drifting_controller_stabiliza_B.Product3_o *
       drifting_controller_stabiliza_B.Saturation) *
      drifting_controller_stabiliza_P.Gain_Gain *
      drifting_controller_stabiliza_B.In1.AngularVelocity.X +
      (drifting_controller_stabiliza_B.Saturation *
       drifting_controller_stabiliza_B.Product3_n -
       drifting_controller_stabiliza_B.Product3_o *
       drifting_controller_stabiliza_B.Product1_p) *
      drifting_controller_stabiliza_P.Gain1_Gain *
      drifting_controller_stabiliza_B.In1.AngularVelocity.Y) +
     ((drifting_controller_stabiliza_P.Constant_Value_d -
       drifting_controller_stabiliza_B.Product1_p *
       drifting_controller_stabiliza_B.Product1_p) -
      drifting_controller_stabiliza_B.Saturation *
      drifting_controller_stabiliza_B.Saturation) *
     drifting_controller_stabiliza_P.Gain2_Gain *
     drifting_controller_stabiliza_B.In1.AngularVelocity.Z) -
    drifting_controller_stabiliza_P.r_eqpoint;

  // Gain: '<S1>/Gain1' incorporates:
  //   Constant: '<S1>/Constant4'
  //   Constant: '<S1>/Constant5'
  //   Gain: '<S1>/Gain4'
  //   Gain: '<S1>/Gain5'
  //   Gain: '<S1>/Gain6'
  //   Sum: '<S1>/Add1'
  //   Sum: '<S1>/Add3'
  //   Sum: '<S1>/Subtract3'

  rtb_sincos_o1_idx_1 = ((((-drifting_controller_stabiliza_P.K_Vx_Fxr *
    drifting_controller_stabiliza_B.Subtract +
    -drifting_controller_stabiliza_P.K_Vy_Fxr *
    drifting_controller_stabiliza_B.Subtract2) +
    -drifting_controller_stabiliza_P.K_r_Fxr *
    drifting_controller_stabiliza_B.Saturation) +
    drifting_controller_stabiliza_P.Fxr_eqpoint) +
    drifting_controller_stabiliza_P.Fdrag) * (drifting_controller_stabiliza_P.Rw
    * drifting_controller_stabiliza_P.tau / drifting_controller_stabiliza_P.Kt /
    drifting_controller_stabiliza_P.Imax);

  // Saturate: '<S1>/Saturation1'
  if (rtb_sincos_o1_idx_1 > drifting_controller_stabiliza_P.Saturation1_UpperSat)
  {
    rtb_sincos_o1_idx_1 = drifting_controller_stabiliza_P.Saturation1_UpperSat;
  } else {
    if (rtb_sincos_o1_idx_1 <
        drifting_controller_stabiliza_P.Saturation1_LowerSat) {
      rtb_sincos_o1_idx_1 = drifting_controller_stabiliza_P.Saturation1_LowerSat;
    }
  }

  // End of Saturate: '<S1>/Saturation1'

  // BusAssignment: '<S2>/Bus Assignment1'
  drifting_controller_stabiliza_B.BusAssignment1.SpeedRef = rtb_sincos_o1_idx_1;

  // Sum: '<S1>/Add' incorporates:
  //   Constant: '<S1>/Constant3'
  //   Gain: '<S1>/Gain'
  //   Gain: '<S1>/Gain2'
  //   Gain: '<S1>/Gain3'
  //   Sum: '<S1>/Add2'

  rtb_sincos_o1_idx_1 = ((-drifting_controller_stabiliza_P.K_Vx_delta *
    drifting_controller_stabiliza_B.Subtract +
    -drifting_controller_stabiliza_P.K_Vy_delta *
    drifting_controller_stabiliza_B.Subtract2) +
    -drifting_controller_stabiliza_P.K_r_delta *
    drifting_controller_stabiliza_B.Saturation) +
    drifting_controller_stabiliza_P.delta_eqpoint;

  // Saturate: '<S1>/Saturation'
  if (rtb_sincos_o1_idx_1 > drifting_controller_stabiliza_P.delta_max) {
    rtb_sincos_o1_idx_1 = drifting_controller_stabiliza_P.delta_max;
  } else {
    if (rtb_sincos_o1_idx_1 < drifting_controller_stabiliza_P.delta_min) {
      rtb_sincos_o1_idx_1 = drifting_controller_stabiliza_P.delta_min;
    }
  }

  // End of Saturate: '<S1>/Saturation'

  // BusAssignment: '<S2>/Bus Assignment1' incorporates:
  //   Constant: '<Root>/Constant2'

  drifting_controller_stabiliza_B.BusAssignment1.SteerRef = rtb_sincos_o1_idx_1;
  drifting_controller_stabiliza_B.BusAssignment1.State =
    drifting_controller_stabiliza_P.Constant2_Value;

  // Outputs for Atomic SubSystem: '<S2>/Publish1'
  // Start for MATLABSystem: '<S5>/SinkBlock' incorporates:
  //   MATLABSystem: '<S5>/SinkBlock'

  Pub_drifting_controller_stabilization_75.publish
    (&drifting_controller_stabiliza_B.BusAssignment1);

  // End of Outputs for SubSystem: '<S2>/Publish1'

  // Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn2'
  drifting_controller_stabiliz_DW.DiscreteTransferFcn2_states =
    DiscreteTransferFcn2_tmp;

  // Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1'
  drifting_controller_stabiliz_DW.DiscreteTransferFcn1_states =
    drifting_controller_stabiliza_B.DiscreteTransferFcn1_tmp;
}

// Model initialize function
void drifting_controller_stabilization_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // initialize error status
  rtmSetErrorStatus(drifting_controller_stabiliz_M, (NULL));

  // block I/O
  (void) memset(((void *) &drifting_controller_stabiliza_B), 0,
                sizeof(B_drifting_controller_stabili_T));

  // states (dwork)
  (void) memset((void *)&drifting_controller_stabiliz_DW, 0,
                sizeof(DW_drifting_controller_stabil_T));

  {
    static const char_T tmp[15] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l', 'l',
      'e', 'r', '_', 'c', 'm', 'd' };

    static const char_T tmp_0[9] = { '/', 'i', 'm', 'u', '/', 'd', 'a', 't', 'a'
    };

    static const char_T tmp_1[22] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'V' };

    static const char_T tmp_2[25] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'b', 'e',
      't', 'a' };

    char_T tmp_3[16];
    char_T tmp_4[10];
    char_T tmp_5[23];
    int32_T i;

    // Start for Atomic SubSystem: '<S3>/Subscribe'
    // Start for MATLABSystem: '<S10>/SourceBlock'
    drifting_controller_stabiliz_DW.obj_b.isInitialized = 0;
    drifting_controller_stabiliz_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      drifting_controller_stabiliza_B.cv0[i] = tmp_2[i];
    }

    drifting_controller_stabiliza_B.cv0[25] = '\x00';
    Sub_drifting_controller_stabilization_129.createSubscriber
      (drifting_controller_stabiliza_B.cv0, drifting_contro_MessageQueueLen);

    // End of Start for MATLABSystem: '<S10>/SourceBlock'
    // End of Start for SubSystem: '<S3>/Subscribe'

    // Start for Atomic SubSystem: '<S3>/Subscribe2'
    // Start for MATLABSystem: '<S12>/SourceBlock'
    drifting_controller_stabiliz_DW.obj_k.isInitialized = 0;
    drifting_controller_stabiliz_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_5[i] = tmp_1[i];
    }

    tmp_5[22] = '\x00';
    Sub_drifting_controller_stabilization_134.createSubscriber(tmp_5,
      drifting_contro_MessageQueueLen);

    // End of Start for MATLABSystem: '<S12>/SourceBlock'
    // End of Start for SubSystem: '<S3>/Subscribe2'

    // Start for Atomic SubSystem: '<S3>/Subscribe1'
    // Start for MATLABSystem: '<S11>/SourceBlock'
    drifting_controller_stabiliz_DW.obj_o.isInitialized = 0;
    drifting_controller_stabiliz_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_4[i] = tmp_0[i];
    }

    tmp_4[9] = '\x00';
    Sub_drifting_controller_stabilization_164.createSubscriber(tmp_4,
      drifting_contro_MessageQueueLen);

    // End of Start for MATLABSystem: '<S11>/SourceBlock'
    // End of Start for SubSystem: '<S3>/Subscribe1'

    // Start for Atomic SubSystem: '<S2>/Publish1'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    drifting_controller_stabiliz_DW.obj.isInitialized = 0;
    drifting_controller_stabiliz_DW.obj.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      tmp_3[i] = tmp[i];
    }

    tmp_3[15] = '\x00';
    Pub_drifting_controller_stabilization_75.createPublisher(tmp_3,
      drifting_contro_MessageQueueLen);

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
    // End of Start for SubSystem: '<S2>/Publish1'

    // InitializeConditions for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn2' 
    drifting_controller_stabiliz_DW.DiscreteTransferFcn2_states =
      drifting_controller_stabiliza_P.DiscreteTransferFcn2_InitialSta;

    // InitializeConditions for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn1' 
    drifting_controller_stabiliz_DW.DiscreteTransferFcn1_states =
      drifting_controller_stabiliza_P.DiscreteTransferFcn1_InitialSta;

    // SystemInitialize for Atomic SubSystem: '<S3>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S10>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S24>/Out1'
    drifting_controller_stabiliza_B.In1_g =
      drifting_controller_stabiliza_P.Out1_Y0_o;

    // End of SystemInitialize for SubSystem: '<S10>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S3>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<S3>/Subscribe2'
    // SystemInitialize for Enabled SubSystem: '<S12>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S26>/Out1'
    drifting_controller_stabiliza_B.In1_m =
      drifting_controller_stabiliza_P.Out1_Y0_f;

    // End of SystemInitialize for SubSystem: '<S12>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S3>/Subscribe2'

    // SystemInitialize for Atomic SubSystem: '<S3>/Subscribe1'
    // SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S25>/Out1'
    drifting_controller_stabiliza_B.In1 =
      drifting_controller_stabiliza_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S3>/Subscribe1'
  }
}

// Model terminate function
void drifting_controller_stabilization_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S3>/Subscribe'
  // Start for MATLABSystem: '<S10>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S10>/SourceBlock'

  if (drifting_controller_stabiliz_DW.obj_b.isInitialized == 1) {
    drifting_controller_stabiliz_DW.obj_b.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S10>/SourceBlock'
  // End of Terminate for SubSystem: '<S3>/Subscribe'

  // Terminate for Atomic SubSystem: '<S3>/Subscribe2'
  // Start for MATLABSystem: '<S12>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S12>/SourceBlock'

  if (drifting_controller_stabiliz_DW.obj_k.isInitialized == 1) {
    drifting_controller_stabiliz_DW.obj_k.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S12>/SourceBlock'
  // End of Terminate for SubSystem: '<S3>/Subscribe2'

  // Terminate for Atomic SubSystem: '<S3>/Subscribe1'
  // Start for MATLABSystem: '<S11>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S11>/SourceBlock'

  if (drifting_controller_stabiliz_DW.obj_o.isInitialized == 1) {
    drifting_controller_stabiliz_DW.obj_o.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S11>/SourceBlock'
  // End of Terminate for SubSystem: '<S3>/Subscribe1'

  // Terminate for Atomic SubSystem: '<S2>/Publish1'
  // Start for MATLABSystem: '<S5>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S5>/SinkBlock'

  if (drifting_controller_stabiliz_DW.obj.isInitialized == 1) {
    drifting_controller_stabiliz_DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S5>/SinkBlock'
  // End of Terminate for SubSystem: '<S2>/Publish1'
}

//
// File trailer for generated code.
//
// [EOF]
//
