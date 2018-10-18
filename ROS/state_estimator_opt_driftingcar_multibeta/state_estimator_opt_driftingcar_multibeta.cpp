//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: state_estimator_opt_driftingcar_multibeta.cpp
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
#define state_estimator_MessageQueueLen (1)

// Block signals (auto storage)
B_state_estimator_opt_driftin_T state_estimator_opt_driftingc_B;

// Block states (auto storage)
DW_state_estimator_opt_drifti_T state_estimator_opt_drifting_DW;

// Real-time model
RT_MODEL_state_estimator_opt__T state_estimator_opt_drifting_M_;
RT_MODEL_state_estimator_opt__T *const state_estimator_opt_drifting_M =
  &state_estimator_opt_drifting_M_;
real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2((real_T)u0_0, (real_T)u1_0);
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

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  boolean_T yEq;
  real_T q;
  y = u0;
  if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf(u1)))))
  {
    if (u1 != 0.0) {
      y = (rtNaN);
    }
  } else if (u0 == 0.0) {
    y = u1 * 0.0;
  } else {
    if (u1 != 0.0) {
      y = fmod(u0, u1);
      yEq = (y == 0.0);
      if ((!yEq) && (u1 > floor(u1))) {
        q = fabs(u0 / u1);
        yEq = (fabs(q - floor(q + 0.5)) <= DBL_EPSILON * q);
      }

      if (yEq) {
        y = u1 * 0.0;
      } else {
        if ((u0 < 0.0) != (u1 < 0.0)) {
          y += u1;
        }
      }
    }
  }

  return y;
}

// Model step function
void state_estimator_opt_driftingcar_multibeta_step(void)
{
  boolean_T varargout_1;
  real_T rtb_Product1_i;
  real_T rtb_fcn5;
  real_T rtb_betaunwrapped;
  real_T rtb_Switch2;
  real_T rtb_VectorConcatenate_idx_0;
  real_T rtb_TmpSignalConversionAtSFun_0;

  // Outputs for Atomic SubSystem: '<S4>/Subscribe'
  // Start for MATLABSystem: '<S27>/SourceBlock' incorporates:
  //   Inport: '<S31>/In1'
  //   MATLABSystem: '<S27>/SourceBlock'

  varargout_1 =
    Sub_state_estimator_opt_driftingcar_multibeta_132.getLatestMessage
    (&state_estimator_opt_driftingc_B.varargout_2);

  // Outputs for Enabled SubSystem: '<S27>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S31>/Enable'

  if (varargout_1) {
    state_estimator_opt_driftingc_B.In1 =
      state_estimator_opt_driftingc_B.varargout_2;
  }

  // End of Outputs for SubSystem: '<S27>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S4>/Subscribe'

  // Sqrt: '<S29>/sqrt' incorporates:
  //   Product: '<S30>/Product'
  //   Product: '<S30>/Product1'
  //   Product: '<S30>/Product2'
  //   Product: '<S30>/Product3'
  //   Sum: '<S30>/Sum'

  state_estimator_opt_driftingc_B.Product3_m = sqrt
    (((state_estimator_opt_driftingc_B.In1.Pose.Orientation.W *
       state_estimator_opt_driftingc_B.In1.Pose.Orientation.W +
       state_estimator_opt_driftingc_B.In1.Pose.Orientation.X *
       state_estimator_opt_driftingc_B.In1.Pose.Orientation.X) +
      state_estimator_opt_driftingc_B.In1.Pose.Orientation.Y *
      state_estimator_opt_driftingc_B.In1.Pose.Orientation.Y) +
     state_estimator_opt_driftingc_B.In1.Pose.Orientation.Z *
     state_estimator_opt_driftingc_B.In1.Pose.Orientation.Z);

  // Product: '<S28>/Product'
  rtb_fcn5 = state_estimator_opt_driftingc_B.In1.Pose.Orientation.W /
    state_estimator_opt_driftingc_B.Product3_m;

  // Product: '<S28>/Product1'
  rtb_Product1_i = state_estimator_opt_driftingc_B.In1.Pose.Orientation.X /
    state_estimator_opt_driftingc_B.Product3_m;

  // Product: '<S28>/Product2'
  state_estimator_opt_driftingc_B.Product2_o =
    state_estimator_opt_driftingc_B.In1.Pose.Orientation.Y /
    state_estimator_opt_driftingc_B.Product3_m;

  // Product: '<S28>/Product3'
  state_estimator_opt_driftingc_B.Product3_m =
    state_estimator_opt_driftingc_B.In1.Pose.Orientation.Z /
    state_estimator_opt_driftingc_B.Product3_m;

  // Trigonometry: '<S26>/Trigonometric Function1' incorporates:
  //   Fcn: '<S26>/fcn1'
  //   Fcn: '<S26>/fcn2'

  rtb_VectorConcatenate_idx_0 = rt_atan2d_snf((rtb_Product1_i *
    state_estimator_opt_driftingc_B.Product2_o + rtb_fcn5 *
    state_estimator_opt_driftingc_B.Product3_m) * 2.0, ((rtb_fcn5 * rtb_fcn5 +
    rtb_Product1_i * rtb_Product1_i) -
    state_estimator_opt_driftingc_B.Product2_o *
    state_estimator_opt_driftingc_B.Product2_o) -
    state_estimator_opt_driftingc_B.Product3_m *
    state_estimator_opt_driftingc_B.Product3_m);

  // S-Function (sdspunwrap2): '<S36>/Unwrap1'
  if (state_estimator_opt_drifting_DW.Unwrap1_FirstStep) {
    state_estimator_opt_drifting_DW.Unwrap1_Prev = rtb_VectorConcatenate_idx_0;
    state_estimator_opt_drifting_DW.Unwrap1_FirstStep = false;
  }

  state_estimator_opt_driftingc_B.Product3_m =
    state_estimator_opt_drifting_DW.Unwrap1_Cumsum;
  rtb_betaunwrapped = rtb_VectorConcatenate_idx_0 -
    state_estimator_opt_drifting_DW.Unwrap1_Prev;
  rtb_TmpSignalConversionAtSFun_0 = rtb_betaunwrapped - floor((rtb_betaunwrapped
    + 3.1415926535897931) / 6.2831853071795862) * 6.2831853071795862;
  if ((rtb_TmpSignalConversionAtSFun_0 == -3.1415926535897931) &&
      (rtb_betaunwrapped > 0.0)) {
    rtb_TmpSignalConversionAtSFun_0 = 3.1415926535897931;
  }

  rtb_betaunwrapped = rtb_TmpSignalConversionAtSFun_0 - rtb_betaunwrapped;
  if (fabs(rtb_betaunwrapped) > 3.1415926535897931) {
    state_estimator_opt_driftingc_B.Product3_m =
      state_estimator_opt_drifting_DW.Unwrap1_Cumsum + rtb_betaunwrapped;
  }

  state_estimator_opt_drifting_DW.Unwrap1_Prev = rtb_VectorConcatenate_idx_0;
  state_estimator_opt_drifting_DW.Unwrap1_Cumsum =
    state_estimator_opt_driftingc_B.Product3_m;

  // Sum: '<S36>/Add' incorporates:
  //   Constant: '<S36>/Theta_Offset [rad]'
  //   S-Function (sdspunwrap2): '<S36>/Unwrap1'

  state_estimator_opt_driftingc_B.Product3_m = (rtb_VectorConcatenate_idx_0 +
    state_estimator_opt_driftingc_B.Product3_m) +
    state_estimator_opt_driftingc_P.Theta_Offsetrad_Value_m;

  // Math: '<S36>/Math Function2' incorporates:
  //   Constant: '<S36>/Constant2'
  //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator'
  //   Sum: '<S36>/Subtract2'

  rtb_fcn5 = rt_modd_snf
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2] -
     state_estimator_opt_driftingc_B.Product3_m,
     state_estimator_opt_driftingc_P.Constant2_Value_i);

  // Switch: '<S36>/Switch' incorporates:
  //   Constant: '<S36>/Constant'
  //   Product: '<S36>/Divide'
  //   Rounding: '<S36>/Floor'

  if (floor(rtb_fcn5 / state_estimator_opt_driftingc_P.Constant_Value_js) >=
      state_estimator_opt_driftingc_P.Switch_Threshold_a) {
    // BusAssignment: '<S3>/Bus Assignment1' incorporates:
    //   Constant: '<S36>/Constant1'
    //   Sum: '<S36>/Subtract3'

    state_estimator_opt_driftingc_B.BusAssignment1.Data = rtb_fcn5 -
      state_estimator_opt_driftingc_P.Constant1_Value_k;
  } else {
    // BusAssignment: '<S3>/Bus Assignment1'
    state_estimator_opt_driftingc_B.BusAssignment1.Data = rtb_fcn5;
  }

  // End of Switch: '<S36>/Switch'

  // Outputs for Atomic SubSystem: '<S3>/Publish1'
  // Start for MATLABSystem: '<S19>/SinkBlock' incorporates:
  //   MATLABSystem: '<S19>/SinkBlock'

  Pub_state_estimator_opt_driftingcar_multibeta_170.publish
    (&state_estimator_opt_driftingc_B.BusAssignment1);

  // End of Outputs for SubSystem: '<S3>/Publish1'

  // Math: '<S36>/Math Function3' incorporates:
  //   Constant: '<S36>/Constant3'

  state_estimator_opt_driftingc_B.Product3_m = rt_modd_snf
    (state_estimator_opt_driftingc_B.Product3_m,
     state_estimator_opt_driftingc_P.Constant3_Value);

  // BusAssignment: '<S3>/Bus Assignment2'
  state_estimator_opt_driftingc_B.BusAssignment2.Data =
    state_estimator_opt_driftingc_B.Product3_m;

  // Outputs for Atomic SubSystem: '<S3>/Publish2'
  // Start for MATLABSystem: '<S20>/SinkBlock' incorporates:
  //   MATLABSystem: '<S20>/SinkBlock'

  Pub_state_estimator_opt_driftingcar_multibeta_171.publish
    (&state_estimator_opt_driftingc_B.BusAssignment2);

  // End of Outputs for SubSystem: '<S3>/Publish2'

  // Sum: '<S4>/Add' incorporates:
  //   Gain: '<S4>/Gain'

  rtb_fcn5 = state_estimator_opt_driftingc_P.Gain_Gain *
    state_estimator_opt_driftingc_B.In1.Header.Stamp.Nsec +
    state_estimator_opt_driftingc_B.In1.Header.Stamp.Sec;

  // Outputs for Enabled SubSystem: '<Root>/BetaOptEstimator1' incorporates:
  //   EnablePort: '<S1>/Enable'

  // Outputs for Atomic SubSystem: '<S4>/Subscribe'
  // Start for MATLABSystem: '<S27>/SourceBlock' incorporates:
  //   MATLABSystem: '<S27>/SourceBlock'

  if (varargout_1) {
    // S-Function (sdspunwrap2): '<S1>/Unwrap1'
    if (state_estimator_opt_drifting_DW.Unwrap1_FirstStep_m) {
      state_estimator_opt_drifting_DW.Unwrap1_Prev_j =
        rtb_VectorConcatenate_idx_0;
      state_estimator_opt_drifting_DW.Unwrap1_FirstStep_m = false;
    }

    state_estimator_opt_driftingc_B.Product3_m =
      state_estimator_opt_drifting_DW.Unwrap1_Cumsum_c;
    rtb_betaunwrapped = rtb_VectorConcatenate_idx_0 -
      state_estimator_opt_drifting_DW.Unwrap1_Prev_j;
    rtb_TmpSignalConversionAtSFun_0 = rtb_betaunwrapped - floor
      ((rtb_betaunwrapped + 3.1415926535897931) / 6.2831853071795862) *
      6.2831853071795862;
    if ((rtb_TmpSignalConversionAtSFun_0 == -3.1415926535897931) &&
        (rtb_betaunwrapped > 0.0)) {
      rtb_TmpSignalConversionAtSFun_0 = 3.1415926535897931;
    }

    rtb_betaunwrapped = rtb_TmpSignalConversionAtSFun_0 - rtb_betaunwrapped;
    if (fabs(rtb_betaunwrapped) > 3.1415926535897931) {
      state_estimator_opt_driftingc_B.Product3_m =
        state_estimator_opt_drifting_DW.Unwrap1_Cumsum_c + rtb_betaunwrapped;
    }

    rtb_Switch2 = rtb_VectorConcatenate_idx_0 +
      state_estimator_opt_driftingc_B.Product3_m;
    state_estimator_opt_drifting_DW.Unwrap1_Prev_j = rtb_VectorConcatenate_idx_0;
    state_estimator_opt_drifting_DW.Unwrap1_Cumsum_c =
      state_estimator_opt_driftingc_B.Product3_m;

    // End of S-Function (sdspunwrap2): '<S1>/Unwrap1'

    // Sum: '<S8>/Diff' incorporates:
    //   UnitDelay: '<S8>/UD'
    //
    //  Block description for '<S8>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S8>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_driftingc_B.Product2_o =
      state_estimator_opt_driftingc_B.In1.Pose.Position.X -
      state_estimator_opt_drifting_DW.UD_DSTATE_mn;

    // Sum: '<S7>/Diff' incorporates:
    //   UnitDelay: '<S7>/UD'
    //
    //  Block description for '<S7>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S7>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_driftingc_B.Product3_m =
      state_estimator_opt_driftingc_B.In1.Pose.Position.Y -
      state_estimator_opt_drifting_DW.UD_DSTATE_b;

    // Sqrt: '<S1>/Sqrt' incorporates:
    //   Math: '<S1>/Math Function'
    //   Math: '<S1>/Math Function1'
    //   Sum: '<S1>/Add1'
    //
    //  About '<S1>/Math Function':
    //   Operator: magnitude^2
    //
    //  About '<S1>/Math Function1':
    //   Operator: magnitude^2

    rtb_Product1_i = sqrt(state_estimator_opt_driftingc_B.Product2_o *
                          state_estimator_opt_driftingc_B.Product2_o +
                          state_estimator_opt_driftingc_B.Product3_m *
                          state_estimator_opt_driftingc_B.Product3_m);

    // Switch: '<S1>/Switch2' incorporates:
    //   Memory: '<S1>/Memory1'
    //   Memory: '<S1>/Memory2'
    //   Switch: '<S1>/Switch3'
    //   Trigonometry: '<S1>/Trigonometric Function1'

    if (rtb_Product1_i > state_estimator_opt_driftingc_P.ds_nonzero_threshold) {
      state_estimator_opt_driftingc_B.Product2_o = rt_atan2d_snf
        (state_estimator_opt_driftingc_B.Product3_m,
         state_estimator_opt_driftingc_B.Product2_o);
    } else {
      rtb_Switch2 = state_estimator_opt_drifting_DW.Memory1_PreviousInput;
      state_estimator_opt_driftingc_B.Product2_o =
        state_estimator_opt_drifting_DW.Memory2_PreviousInput;
    }

    // End of Switch: '<S1>/Switch2'

    // S-Function (sdspunwrap2): '<S1>/Unwrap'
    if (state_estimator_opt_drifting_DW.Unwrap_FirstStep) {
      state_estimator_opt_drifting_DW.Unwrap_Prev =
        state_estimator_opt_driftingc_B.Product2_o;
      state_estimator_opt_drifting_DW.Unwrap_FirstStep = false;
    }

    state_estimator_opt_driftingc_B.Product3_m =
      state_estimator_opt_drifting_DW.Unwrap_Cumsum;
    rtb_betaunwrapped = state_estimator_opt_driftingc_B.Product2_o -
      state_estimator_opt_drifting_DW.Unwrap_Prev;
    rtb_TmpSignalConversionAtSFun_0 = rtb_betaunwrapped - floor
      ((rtb_betaunwrapped + 3.1415926535897931) / 6.2831853071795862) *
      6.2831853071795862;
    if ((rtb_TmpSignalConversionAtSFun_0 == -3.1415926535897931) &&
        (rtb_betaunwrapped > 0.0)) {
      rtb_TmpSignalConversionAtSFun_0 = 3.1415926535897931;
    }

    rtb_betaunwrapped = rtb_TmpSignalConversionAtSFun_0 - rtb_betaunwrapped;
    if (fabs(rtb_betaunwrapped) > 3.1415926535897931) {
      state_estimator_opt_driftingc_B.Product3_m =
        state_estimator_opt_drifting_DW.Unwrap_Cumsum + rtb_betaunwrapped;
    }

    state_estimator_opt_drifting_DW.Unwrap_Prev =
      state_estimator_opt_driftingc_B.Product2_o;
    state_estimator_opt_drifting_DW.Unwrap_Cumsum =
      state_estimator_opt_driftingc_B.Product3_m;

    // Math: '<S1>/Math Function2' incorporates:
    //   Constant: '<S1>/Constant2'
    //   Constant: '<S1>/Theta_Offset [rad]'
    //   S-Function (sdspunwrap2): '<S1>/Unwrap'
    //   Sum: '<S1>/Add'
    //   Sum: '<S1>/Subtract2'

    state_estimator_opt_driftingc_B.Product3_m = rt_modd_snf
      ((state_estimator_opt_driftingc_B.Product2_o +
        state_estimator_opt_driftingc_B.Product3_m) - (rtb_Switch2 +
        state_estimator_opt_driftingc_P.Theta_Offsetrad_Value),
       state_estimator_opt_driftingc_P.Constant2_Value);

    // Switch: '<S1>/Switch' incorporates:
    //   Constant: '<S1>/Constant'
    //   Constant: '<S1>/Constant1'
    //   Product: '<S1>/Divide'
    //   Rounding: '<S1>/Floor'
    //   Sum: '<S1>/Subtract3'

    if (floor(state_estimator_opt_driftingc_B.Product3_m /
              state_estimator_opt_driftingc_P.Constant_Value_j) >=
        state_estimator_opt_driftingc_P.Switch_Threshold) {
      state_estimator_opt_driftingc_B.Betapipi =
        state_estimator_opt_driftingc_B.Product3_m -
        state_estimator_opt_driftingc_P.Constant1_Value;
    } else {
      state_estimator_opt_driftingc_B.Betapipi =
        state_estimator_opt_driftingc_B.Product3_m;
    }

    // End of Switch: '<S1>/Switch'

    // Switch: '<S1>/Switch1' incorporates:
    //   Constant: '<S1>/Constant4'
    //   Constant: '<S1>/Constant5'

    if (rtb_Product1_i > state_estimator_opt_driftingc_P.ds_nonzero_threshold) {
      state_estimator_opt_driftingc_B.Switch1 =
        state_estimator_opt_driftingc_P.Constant4_Value;
    } else {
      state_estimator_opt_driftingc_B.Switch1 =
        state_estimator_opt_driftingc_P.Constant5_Value;
    }

    // End of Switch: '<S1>/Switch1'

    // Update for UnitDelay: '<S8>/UD'
    //
    //  Block description for '<S8>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_mn =
      state_estimator_opt_driftingc_B.In1.Pose.Position.X;

    // Update for UnitDelay: '<S7>/UD'
    //
    //  Block description for '<S7>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_b =
      state_estimator_opt_driftingc_B.In1.Pose.Position.Y;

    // Update for Memory: '<S1>/Memory1'
    state_estimator_opt_drifting_DW.Memory1_PreviousInput = rtb_Switch2;

    // Update for Memory: '<S1>/Memory2'
    state_estimator_opt_drifting_DW.Memory2_PreviousInput =
      state_estimator_opt_driftingc_B.Product2_o;
  }

  // End of Outputs for SubSystem: '<Root>/BetaOptEstimator1'

  // Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S2>/Enable'

  // Product: '<Root>/Product' incorporates:
  //   MATLABSystem: '<S27>/SourceBlock'
  //   Start for MATLABSystem: '<S27>/SourceBlock'

  if (state_estimator_opt_driftingc_B.Switch1 * (real_T)varargout_1 > 0.0) {
    // Sum: '<S11>/Diff' incorporates:
    //   UnitDelay: '<S11>/UD'
    //
    //  Block description for '<S11>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S11>/UD':
    //
    //   Store in Global RAM

    rtb_Switch2 = state_estimator_opt_driftingc_B.In1.Pose.Position.X -
      state_estimator_opt_drifting_DW.UD_DSTATE;

    // Math: '<S2>/Math Function1'
    //
    //  About '<S2>/Math Function1':
    //   Operator: magnitude^2

    rtb_Product1_i = rtb_Switch2 * rtb_Switch2;

    // Sum: '<S10>/Diff' incorporates:
    //   UnitDelay: '<S10>/UD'
    //
    //  Block description for '<S10>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S10>/UD':
    //
    //   Store in Global RAM

    rtb_Switch2 = state_estimator_opt_driftingc_B.In1.Pose.Position.Y -
      state_estimator_opt_drifting_DW.UD_DSTATE_m;

    // Product: '<S2>/Divide' incorporates:
    //   Math: '<S2>/Math Function'
    //   Sqrt: '<S2>/Sqrt'
    //   Sum: '<S2>/Add1'
    //   Sum: '<S9>/Diff'
    //   UnitDelay: '<S9>/UD'
    //
    //  About '<S2>/Math Function':
    //   Operator: magnitude^2
    //
    //  Block description for '<S9>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S9>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_driftingc_B.Divide = sqrt(rtb_Switch2 * rtb_Switch2 +
      rtb_Product1_i) / (rtb_fcn5 - state_estimator_opt_drifting_DW.UD_DSTATE_a);

    // Update for UnitDelay: '<S11>/UD'
    //
    //  Block description for '<S11>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE =
      state_estimator_opt_driftingc_B.In1.Pose.Position.X;

    // Update for UnitDelay: '<S10>/UD'
    //
    //  Block description for '<S10>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_m =
      state_estimator_opt_driftingc_B.In1.Pose.Position.Y;

    // Update for UnitDelay: '<S9>/UD'
    //
    //  Block description for '<S9>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_a = rtb_fcn5;
  }

  // End of Product: '<Root>/Product'
  // End of Outputs for SubSystem: '<Root>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S4>/Subscribe'

  // BusAssignment: '<S3>/Bus Assignment3'
  state_estimator_opt_driftingc_B.BusAssignment3.Data =
    state_estimator_opt_driftingc_B.Divide;

  // Outputs for Atomic SubSystem: '<S3>/Publish3'
  // Start for MATLABSystem: '<S21>/SinkBlock' incorporates:
  //   MATLABSystem: '<S21>/SinkBlock'

  Pub_state_estimator_opt_driftingcar_multibeta_172.publish
    (&state_estimator_opt_driftingc_B.BusAssignment3);

  // End of Outputs for SubSystem: '<S3>/Publish3'

  // BusAssignment: '<S3>/Bus Assignment4'
  state_estimator_opt_driftingc_B.BusAssignment4.Data =
    state_estimator_opt_driftingc_B.Betapipi;

  // Outputs for Atomic SubSystem: '<S3>/Publish4'
  // Start for MATLABSystem: '<S22>/SinkBlock' incorporates:
  //   MATLABSystem: '<S22>/SinkBlock'

  Pub_state_estimator_opt_driftingcar_multibeta_268.publish
    (&state_estimator_opt_driftingc_B.BusAssignment4);

  // End of Outputs for SubSystem: '<S3>/Publish4'

  // S-Function (sdspunwrap2): '<S33>/Unwrap1'
  if (state_estimator_opt_drifting_DW.Unwrap1_FirstStep_f) {
    state_estimator_opt_drifting_DW.Unwrap1_Prev_o = rtb_VectorConcatenate_idx_0;
    state_estimator_opt_drifting_DW.Unwrap1_FirstStep_f = false;
  }

  state_estimator_opt_driftingc_B.Product3_m =
    state_estimator_opt_drifting_DW.Unwrap1_Cumsum_b;
  rtb_betaunwrapped = rtb_VectorConcatenate_idx_0 -
    state_estimator_opt_drifting_DW.Unwrap1_Prev_o;
  rtb_TmpSignalConversionAtSFun_0 = rtb_betaunwrapped - floor((rtb_betaunwrapped
    + 3.1415926535897931) / 6.2831853071795862) * 6.2831853071795862;
  if ((rtb_TmpSignalConversionAtSFun_0 == -3.1415926535897931) &&
      (rtb_betaunwrapped > 0.0)) {
    rtb_TmpSignalConversionAtSFun_0 = 3.1415926535897931;
  }

  rtb_betaunwrapped = rtb_TmpSignalConversionAtSFun_0 - rtb_betaunwrapped;
  if (fabs(rtb_betaunwrapped) > 3.1415926535897931) {
    state_estimator_opt_driftingc_B.Product3_m =
      state_estimator_opt_drifting_DW.Unwrap1_Cumsum_b + rtb_betaunwrapped;
  }

  state_estimator_opt_drifting_DW.Unwrap1_Prev_o = rtb_VectorConcatenate_idx_0;
  state_estimator_opt_drifting_DW.Unwrap1_Cumsum_b =
    state_estimator_opt_driftingc_B.Product3_m;

  // Math: '<S33>/Math Function2' incorporates:
  //   Constant: '<S33>/Constant2'
  //   Constant: '<S33>/Theta_Offset [rad]'
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
  //   S-Function (sdspunwrap2): '<S33>/Unwrap1'
  //   Sum: '<S33>/Add'
  //   Sum: '<S33>/Subtract2'

  state_estimator_opt_driftingc_B.Product3_m = rt_modd_snf
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2] -
     ((rtb_VectorConcatenate_idx_0 + state_estimator_opt_driftingc_B.Product3_m)
      + state_estimator_opt_driftingc_P.Theta_Offsetrad_Value_d),
     state_estimator_opt_driftingc_P.Constant2_Value_m);

  // Switch: '<S33>/Switch' incorporates:
  //   Constant: '<S33>/Constant'
  //   Product: '<S33>/Divide'
  //   Rounding: '<S33>/Floor'

  if (floor(state_estimator_opt_driftingc_B.Product3_m /
            state_estimator_opt_driftingc_P.Constant_Value_m) >=
      state_estimator_opt_driftingc_P.Switch_Threshold_j) {
    // BusAssignment: '<S3>/Bus Assignment5' incorporates:
    //   Constant: '<S33>/Constant1'
    //   Sum: '<S33>/Subtract3'

    state_estimator_opt_driftingc_B.BusAssignment5.Data =
      state_estimator_opt_driftingc_B.Product3_m -
      state_estimator_opt_driftingc_P.Constant1_Value_h;
  } else {
    // BusAssignment: '<S3>/Bus Assignment5'
    state_estimator_opt_driftingc_B.BusAssignment5.Data =
      state_estimator_opt_driftingc_B.Product3_m;
  }

  // End of Switch: '<S33>/Switch'

  // Outputs for Atomic SubSystem: '<S3>/Publish5'
  // Start for MATLABSystem: '<S23>/SinkBlock' incorporates:
  //   MATLABSystem: '<S23>/SinkBlock'

  Pub_state_estimator_opt_driftingcar_multibeta_318.publish
    (&state_estimator_opt_driftingc_B.BusAssignment5);

  // End of Outputs for SubSystem: '<S3>/Publish5'

  // BusAssignment: '<S3>/Bus Assignment6' incorporates:
  //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator'

  state_estimator_opt_driftingc_B.BusAssignment6.Data =
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2];

  // Outputs for Atomic SubSystem: '<S3>/Publish6'
  // Start for MATLABSystem: '<S24>/SinkBlock' incorporates:
  //   MATLABSystem: '<S24>/SinkBlock'

  Pub_state_estimator_opt_driftingcar_multibeta_322.publish
    (&state_estimator_opt_driftingc_B.BusAssignment6);

  // End of Outputs for SubSystem: '<S3>/Publish6'

  // BusAssignment: '<S3>/Bus Assignment7' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  state_estimator_opt_driftingc_B.BusAssignment7.Data =
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2];

  // Outputs for Atomic SubSystem: '<S3>/Publish7'
  // Start for MATLABSystem: '<S25>/SinkBlock' incorporates:
  //   MATLABSystem: '<S25>/SinkBlock'

  Pub_state_estimator_opt_driftingcar_multibeta_326.publish
    (&state_estimator_opt_driftingc_B.BusAssignment7);

  // End of Outputs for SubSystem: '<S3>/Publish7'

  // DiscreteTransferFcn: '<S32>/PD_x' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
  //   Sum: '<S32>/Sum'

  state_estimator_opt_driftingc_B.Product3_m =
    (state_estimator_opt_driftingc_B.In1.Pose.Position.X -
     state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[0]) -
    state_estimator_opt_driftingc_P.Den_PDa[1] *
    state_estimator_opt_drifting_DW.PD_x_states;
  rtb_Product1_i = state_estimator_opt_driftingc_P.Num_PDa[0] *
    state_estimator_opt_driftingc_B.Product3_m +
    state_estimator_opt_driftingc_P.Num_PDa[1] *
    state_estimator_opt_drifting_DW.PD_x_states;

  // DiscreteTransferFcn: '<S32>/PD_y' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
  //   Sum: '<S32>/Sum1'

  rtb_fcn5 = (state_estimator_opt_driftingc_B.In1.Pose.Position.Y -
              state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[1])
    - state_estimator_opt_driftingc_P.Den_PDa[1] *
    state_estimator_opt_drifting_DW.PD_y_states;
  rtb_VectorConcatenate_idx_0 = state_estimator_opt_driftingc_P.Num_PDa[0] *
    rtb_fcn5 + state_estimator_opt_driftingc_P.Num_PDa[1] *
    state_estimator_opt_drifting_DW.PD_y_states;

  // MATLAB Function: '<S32>/unicycle_feedback_acceleration' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
  //   SignalConversion: '<S34>/TmpSignal ConversionAt SFunction Inport2'

  //  State and input variables
  // MATLAB Function 'beta estimator acceleration/Beta estimator acceleration/unicycle_feedback_acceleration': '<S34>:1' 
  // '<S34>:1:4' x     = state(1);
  // [m]
  // '<S34>:1:5' y     = state(2);
  // [m]
  // '<S34>:1:6' gamma = state(3);
  //  theta+beta [rad]
  // '<S34>:1:7' v     = state(4);
  // [m/s]
  // '<S34>:1:9' ax    = input(1);
  // [m/s^2]
  // '<S34>:1:10' ay    = input(2);
  // [m/s^2]
  //  Model equations
  // '<S34>:1:13' dstate = zeros(4,1);
  // '<S34>:1:14' dstate(1) = v*cos(gamma);
  // dx/dt
  // '<S34>:1:15' dstate(2) = v*sin(gamma);
  // dy/dt
  // '<S34>:1:16' dstate(3) = (ay*cos(gamma)-ax*sin(gamma))/v;
  state_estimator_opt_driftingc_B.Product2_o = (rtb_VectorConcatenate_idx_0 *
    cos(state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2]) -
    rtb_Product1_i * sin
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2])) /
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[3];

  // d(gamma)/dt
  // '<S34>:1:17' dstate(4) = ax*cos(gamma)+ay*sin(gamma);
  rtb_VectorConcatenate_idx_0 = rtb_Product1_i * cos
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2]) +
    rtb_VectorConcatenate_idx_0 * sin
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2]);

  // SignalConversion: '<S37>/TmpSignal ConversionAt SFunction Inport2' incorporates:
  //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator'
  //   Gain: '<S35>/Gain'
  //   Gain: '<S35>/Gain1'
  //   MATLAB Function: '<S35>/unicycle_feedback_velocity'
  //   Sum: '<S35>/Sum'
  //   Sum: '<S35>/Sum1'

  // dv/dt
  rtb_betaunwrapped = (state_estimator_opt_driftingc_B.In1.Pose.Position.X -
                       state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE
                       [0]) * state_estimator_opt_driftingc_P.Kp;
  rtb_TmpSignalConversionAtSFun_0 =
    (state_estimator_opt_driftingc_B.In1.Pose.Position.Y -
     state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[1]) *
    state_estimator_opt_driftingc_P.Kp;

  // MATLAB Function: '<S35>/unicycle_feedback_velocity' incorporates:
  //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator'

  //  State and input variables
  // MATLAB Function 'beta estimator velocity/Beta estimator velocity/unicycle_feedback_velocity': '<S37>:1' 
  // '<S37>:1:4' x     = state(1);
  // '<S37>:1:5' y     = state(2);
  // '<S37>:1:6' gamma = state(3);
  //  theta+beta
  // '<S37>:1:8' vxp   = input(1);
  // '<S37>:1:9' vyp   = input(2);
  //  Parameters
  // '<S37>:1:12' p = param(1);
  //  Model equations
  // '<S37>:1:15' dstate = zeros(3,1);
  // '<S37>:1:16' dstate(1) = vxp*cos(gamma)^2+vyp*sin(gamma)*cos(gamma);
  rtb_Product1_i = cos
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2]);
  rtb_Switch2 = rtb_Product1_i * rtb_Product1_i * rtb_betaunwrapped +
    rtb_TmpSignalConversionAtSFun_0 * sin
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2]) * cos
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2]);

  // '<S37>:1:17' dstate(2) = vxp*sin(gamma)*cos(gamma)+vyp*sin(gamma)^2;
  rtb_Product1_i = sin
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2]);

  // Update for DiscreteIntegrator: '<S35>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S35>/Constant'
  //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator'
  //   MATLAB Function: '<S35>/unicycle_feedback_velocity'

  // '<S37>:1:18' dstate(3) = (vyp*cos(gamma)-vxp*sin(gamma))/p;
  state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[0] +=
    state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_gainval * rtb_Switch2;
  state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[1] +=
    (rtb_betaunwrapped * sin
     (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2]) * cos
     (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2]) +
     rtb_Product1_i * rtb_Product1_i * rtb_TmpSignalConversionAtSFun_0) *
    state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_gainval;
  state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2] +=
    (rtb_TmpSignalConversionAtSFun_0 * cos
     (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2]) -
     rtb_betaunwrapped * sin
     (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2])) /
    state_estimator_opt_driftingc_P.p *
    state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_gainval;

  // Update for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
  //   MATLAB Function: '<S32>/unicycle_feedback_acceleration'

  state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[0] +=
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[3] * cos
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2]) *
    state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_gainva_j;
  state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[1] +=
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[3] * sin
    (state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2]) *
    state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_gainva_j;
  state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2] +=
    state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_gainva_j *
    state_estimator_opt_driftingc_B.Product2_o;
  state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[3] +=
    state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_gainva_j *
    rtb_VectorConcatenate_idx_0;

  // Update for DiscreteTransferFcn: '<S32>/PD_x'
  state_estimator_opt_drifting_DW.PD_x_states =
    state_estimator_opt_driftingc_B.Product3_m;

  // Update for DiscreteTransferFcn: '<S32>/PD_y'
  state_estimator_opt_drifting_DW.PD_y_states = rtb_fcn5;
}

// Model initialize function
void state_estimator_opt_driftingcar_multibeta_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // initialize error status
  rtmSetErrorStatus(state_estimator_opt_drifting_M, (NULL));

  // block I/O
  (void) memset(((void *) &state_estimator_opt_driftingc_B), 0,
                sizeof(B_state_estimator_opt_driftin_T));

  // states (dwork)
  (void) memset((void *)&state_estimator_opt_drifting_DW, 0,
                sizeof(DW_state_estimator_opt_drifti_T));

  {
    static const char_T tmp[30] = { '/', 's', 't', 'a', 't', 'e', '_', 'e', 's',
      't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'g', 'a', 'm',
      'm', 'a', '_', 'a', 'c', 'c' };

    static const char_T tmp_0[30] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'g', 'a',
      'm', 'm', 'a', '_', 'v', 'e', 'l' };

    static const char_T tmp_1[29] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'b', 'e',
      't', 'a', '_', 'a', 'c', 'c' };

    static const char_T tmp_2[25] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'b', 'e',
      't', 'a' };

    static const char_T tmp_3[22] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'V' };

    static const char_T tmp_4[26] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 't', 'h',
      'e', 't', 'a' };

    static const char_T tmp_5[29] = { '/', 's', 't', 'a', 't', 'e', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'o', 'p', 't', '_', 'b', 'e',
      't', 'a', '_', 'v', 'e', 'l' };

    static const char_T tmp_6[9] = { '/', 'c', 'a', 'r', '/', 'p', 'o', 's', 'e'
    };

    char_T tmp_7[26];
    char_T tmp_8[23];
    char_T tmp_9[10];
    int32_T i;

    // Start for Atomic SubSystem: '<S4>/Subscribe'
    // Start for MATLABSystem: '<S27>/SourceBlock'
    state_estimator_opt_drifting_DW.obj_n.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 9; i++) {
      tmp_9[i] = tmp_6[i];
    }

    tmp_9[9] = '\x00';
    Sub_state_estimator_opt_driftingcar_multibeta_132.createSubscriber(tmp_9,
      state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S27>/SourceBlock'
    // End of Start for SubSystem: '<S4>/Subscribe'

    // Start for Atomic SubSystem: '<S3>/Publish1'
    // Start for MATLABSystem: '<S19>/SinkBlock'
    state_estimator_opt_drifting_DW.obj_dt.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj_dt.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      state_estimator_opt_driftingc_B.cv1[i] = tmp_5[i];
    }

    state_estimator_opt_driftingc_B.cv1[29] = '\x00';
    Pub_state_estimator_opt_driftingcar_multibeta_170.createPublisher
      (state_estimator_opt_driftingc_B.cv1, state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S19>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish1'

    // Start for Atomic SubSystem: '<S3>/Publish2'
    // Start for MATLABSystem: '<S20>/SinkBlock'
    state_estimator_opt_drifting_DW.obj_cl.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj_cl.isInitialized = 1;
    for (i = 0; i < 26; i++) {
      state_estimator_opt_driftingc_B.cv2[i] = tmp_4[i];
    }

    state_estimator_opt_driftingc_B.cv2[26] = '\x00';
    Pub_state_estimator_opt_driftingcar_multibeta_171.createPublisher
      (state_estimator_opt_driftingc_B.cv2, state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S20>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish2'

    // Start for Atomic SubSystem: '<S3>/Publish3'
    // Start for MATLABSystem: '<S21>/SinkBlock'
    state_estimator_opt_drifting_DW.obj_d.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 22; i++) {
      tmp_8[i] = tmp_3[i];
    }

    tmp_8[22] = '\x00';
    Pub_state_estimator_opt_driftingcar_multibeta_172.createPublisher(tmp_8,
      state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S21>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish3'

    // Start for Atomic SubSystem: '<S3>/Publish4'
    // Start for MATLABSystem: '<S22>/SinkBlock'
    state_estimator_opt_drifting_DW.obj_b.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      tmp_7[i] = tmp_2[i];
    }

    tmp_7[25] = '\x00';
    Pub_state_estimator_opt_driftingcar_multibeta_268.createPublisher(tmp_7,
      state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S22>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish4'

    // Start for Atomic SubSystem: '<S3>/Publish5'
    // Start for MATLABSystem: '<S23>/SinkBlock'
    state_estimator_opt_drifting_DW.obj_g.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      state_estimator_opt_driftingc_B.cv1[i] = tmp_1[i];
    }

    state_estimator_opt_driftingc_B.cv1[29] = '\x00';
    Pub_state_estimator_opt_driftingcar_multibeta_318.createPublisher
      (state_estimator_opt_driftingc_B.cv1, state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S23>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish5'

    // Start for Atomic SubSystem: '<S3>/Publish6'
    // Start for MATLABSystem: '<S24>/SinkBlock'
    state_estimator_opt_drifting_DW.obj_c.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 30; i++) {
      state_estimator_opt_driftingc_B.cv0[i] = tmp_0[i];
    }

    state_estimator_opt_driftingc_B.cv0[30] = '\x00';
    Pub_state_estimator_opt_driftingcar_multibeta_322.createPublisher
      (state_estimator_opt_driftingc_B.cv0, state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S24>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish6'

    // Start for Atomic SubSystem: '<S3>/Publish7'
    // Start for MATLABSystem: '<S25>/SinkBlock'
    state_estimator_opt_drifting_DW.obj.isInitialized = 0;
    state_estimator_opt_drifting_DW.obj.isInitialized = 1;
    for (i = 0; i < 30; i++) {
      state_estimator_opt_driftingc_B.cv0[i] = tmp[i];
    }

    state_estimator_opt_driftingc_B.cv0[30] = '\x00';
    Pub_state_estimator_opt_driftingcar_multibeta_326.createPublisher
      (state_estimator_opt_driftingc_B.cv0, state_estimator_MessageQueueLen);

    // End of Start for MATLABSystem: '<S25>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish7'

    // InitializeConditions for DiscreteIntegrator: '<S35>/Discrete-Time Integrator' 
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[0] =
      state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_IC;
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[1] =
      state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_IC;
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE[2] =
      state_estimator_opt_driftingc_P.DiscreteTimeIntegrator_IC;

    // InitializeConditions for S-Function (sdspunwrap2): '<S36>/Unwrap1'
    state_estimator_opt_drifting_DW.Unwrap1_FirstStep = true;
    state_estimator_opt_drifting_DW.Unwrap1_Cumsum = 0.0;

    // InitializeConditions for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' 
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[0] =
      state_estimator_opt_driftingc_P.acc_integrator_initial_condition[0];
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[1] =
      state_estimator_opt_driftingc_P.acc_integrator_initial_condition[1];
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[2] =
      state_estimator_opt_driftingc_P.acc_integrator_initial_condition[2];
    state_estimator_opt_drifting_DW.DiscreteTimeIntegrator_DSTATE_j[3] =
      state_estimator_opt_driftingc_P.acc_integrator_initial_condition[3];

    // InitializeConditions for S-Function (sdspunwrap2): '<S33>/Unwrap1'
    state_estimator_opt_drifting_DW.Unwrap1_FirstStep_f = true;
    state_estimator_opt_drifting_DW.Unwrap1_Cumsum_b = 0.0;

    // InitializeConditions for DiscreteTransferFcn: '<S32>/PD_x'
    state_estimator_opt_drifting_DW.PD_x_states =
      state_estimator_opt_driftingc_P.PD_x_InitialStates;

    // InitializeConditions for DiscreteTransferFcn: '<S32>/PD_y'
    state_estimator_opt_drifting_DW.PD_y_states =
      state_estimator_opt_driftingc_P.PD_y_InitialStates;

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S27>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S31>/Out1'
    state_estimator_opt_driftingc_B.In1 =
      state_estimator_opt_driftingc_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S27>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe'

    // SystemInitialize for Enabled SubSystem: '<Root>/BetaOptEstimator1'
    // InitializeConditions for S-Function (sdspunwrap2): '<S1>/Unwrap1'
    state_estimator_opt_drifting_DW.Unwrap1_FirstStep_m = true;
    state_estimator_opt_drifting_DW.Unwrap1_Cumsum_c = 0.0;

    // InitializeConditions for UnitDelay: '<S8>/UD'
    //
    //  Block description for '<S8>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_mn =
      state_estimator_opt_driftingc_P.Difference1_ICPrevInput;

    // InitializeConditions for UnitDelay: '<S7>/UD'
    //
    //  Block description for '<S7>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_b =
      state_estimator_opt_driftingc_P.Difference_ICPrevInput;

    // InitializeConditions for Memory: '<S1>/Memory1'
    state_estimator_opt_drifting_DW.Memory1_PreviousInput =
      state_estimator_opt_driftingc_P.Memory1_X0;

    // InitializeConditions for Memory: '<S1>/Memory2'
    state_estimator_opt_drifting_DW.Memory2_PreviousInput =
      state_estimator_opt_driftingc_P.Memory2_X0;

    // InitializeConditions for S-Function (sdspunwrap2): '<S1>/Unwrap'
    state_estimator_opt_drifting_DW.Unwrap_FirstStep = true;
    state_estimator_opt_drifting_DW.Unwrap_Cumsum = 0.0;

    // SystemInitialize for Outport: '<S1>/beta_opt'
    state_estimator_opt_driftingc_B.Betapipi =
      state_estimator_opt_driftingc_P.beta_opt_Y0;

    // SystemInitialize for Outport: '<S1>/ds_isNonZero'
    state_estimator_opt_driftingc_B.Switch1 =
      state_estimator_opt_driftingc_P.ds_isNonZero_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/BetaOptEstimator1'

    // SystemInitialize for Enabled SubSystem: '<Root>/Enabled Subsystem'
    // InitializeConditions for UnitDelay: '<S11>/UD'
    //
    //  Block description for '<S11>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE =
      state_estimator_opt_driftingc_P.Difference2_ICPrevInput;

    // InitializeConditions for UnitDelay: '<S10>/UD'
    //
    //  Block description for '<S10>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_m =
      state_estimator_opt_driftingc_P.Difference1_ICPrevInput_n;

    // InitializeConditions for UnitDelay: '<S9>/UD'
    //
    //  Block description for '<S9>/UD':
    //
    //   Store in Global RAM

    state_estimator_opt_drifting_DW.UD_DSTATE_a =
      state_estimator_opt_driftingc_P.Difference_ICPrevInput_d;

    // SystemInitialize for Outport: '<S2>/V_opt'
    state_estimator_opt_driftingc_B.Divide =
      state_estimator_opt_driftingc_P.V_opt_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/Enabled Subsystem'
  }
}

// Model terminate function
void state_estimator_opt_driftingcar_multibeta_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S4>/Subscribe'
  // Start for MATLABSystem: '<S27>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S27>/SourceBlock'

  if (state_estimator_opt_drifting_DW.obj_n.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj_n.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S27>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe'

  // Terminate for Atomic SubSystem: '<S3>/Publish1'
  // Start for MATLABSystem: '<S19>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S19>/SinkBlock'

  if (state_estimator_opt_drifting_DW.obj_dt.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj_dt.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S19>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish1'

  // Terminate for Atomic SubSystem: '<S3>/Publish2'
  // Start for MATLABSystem: '<S20>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S20>/SinkBlock'

  if (state_estimator_opt_drifting_DW.obj_cl.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj_cl.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S20>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish2'

  // Terminate for Atomic SubSystem: '<S3>/Publish3'
  // Start for MATLABSystem: '<S21>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S21>/SinkBlock'

  if (state_estimator_opt_drifting_DW.obj_d.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj_d.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S21>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish3'

  // Terminate for Atomic SubSystem: '<S3>/Publish4'
  // Start for MATLABSystem: '<S22>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S22>/SinkBlock'

  if (state_estimator_opt_drifting_DW.obj_b.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj_b.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S22>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish4'

  // Terminate for Atomic SubSystem: '<S3>/Publish5'
  // Start for MATLABSystem: '<S23>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S23>/SinkBlock'

  if (state_estimator_opt_drifting_DW.obj_g.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj_g.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S23>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish5'

  // Terminate for Atomic SubSystem: '<S3>/Publish6'
  // Start for MATLABSystem: '<S24>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S24>/SinkBlock'

  if (state_estimator_opt_drifting_DW.obj_c.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj_c.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S24>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish6'

  // Terminate for Atomic SubSystem: '<S3>/Publish7'
  // Start for MATLABSystem: '<S25>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S25>/SinkBlock'

  if (state_estimator_opt_drifting_DW.obj.isInitialized == 1) {
    state_estimator_opt_drifting_DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S25>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish7'
}

//
// File trailer for generated code.
//
// [EOF]
//
