//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: beta_estimator_velocity_driftingcar.cpp
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
#define beta_esti_ParameterInitialValue (0.0)
#define beta_estimator__MessageQueueLen (1)

// Block signals (auto storage)
B_beta_estimator_velocity_dri_T beta_estimator_velocity_drift_B;

// Block states (auto storage)
DW_beta_estimator_velocity_dr_T beta_estimator_velocity_drif_DW;

// Real-time model
RT_MODEL_beta_estimator_veloc_T beta_estimator_velocity_drif_M_;
RT_MODEL_beta_estimator_veloc_T *const beta_estimator_velocity_drif_M =
  &beta_estimator_velocity_drif_M_;

// Model step function
void beta_estimator_velocity_driftingcar_step(void)
{
  real_T x;
  boolean_T p;
  real_T value;
  boolean_T varargout_1;
  SL_Bus_beta_estimator_velocity_driftingcar_std_msgs_Float64 rtb_BusAssignment1;
  real_T rtb_TmpSignalConversionAtSFun_0;
  real_T rtb_TmpSignalConversionAtSFun_1;
  real_T rtb_dstate_idx_0;

  // BusAssignment: '<S3>/Bus Assignment1' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'

  rtb_BusAssignment1.Data =
    beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2];

  // Outputs for Atomic SubSystem: '<S3>/Publish1'
  // Start for MATLABSystem: '<S7>/SinkBlock' incorporates:
  //   MATLABSystem: '<S7>/SinkBlock'

  Pub_beta_estimator_velocity_driftingcar_75.publish(&rtb_BusAssignment1);

  // End of Outputs for SubSystem: '<S3>/Publish1'

  // Outputs for Atomic SubSystem: '<S4>/Subscribe'
  // Start for MATLABSystem: '<S8>/SourceBlock' incorporates:
  //   Inport: '<S9>/In1'
  //   MATLABSystem: '<S8>/SourceBlock'

  varargout_1 = Sub_beta_estimator_velocity_driftingcar_77.getLatestMessage
    (&beta_estimator_velocity_drift_B.varargout_2);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S9>/Enable'

  if (varargout_1) {
    beta_estimator_velocity_drift_B.In1 =
      beta_estimator_velocity_drift_B.varargout_2;
  }

  // End of Start for MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S4>/Subscribe'

  // Start for MATLABSystem: '<S2>/Get Parameter3' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter3'

  varargout_1 = false;
  p = true;
  if (!(beta_estimator_velocity_drif_DW.obj.SampleTime ==
        beta_estimator_velocity_drift_P.GetParameter3_SampleTime)) {
    p = false;
  }

  if (p) {
    varargout_1 = true;
  }

  if (!varargout_1) {
    beta_estimator_velocity_drif_DW.obj.SampleTime =
      beta_estimator_velocity_drift_P.GetParameter3_SampleTime;
  }

  ParamGet_beta_estimator_velocity_driftingcar_63.get_parameter(&value);

  // SignalConversion: '<S5>/TmpSignal ConversionAt SFunction Inport2' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   Gain: '<S1>/Kp_x'
  //   Gain: '<S1>/Kp_y'
  //   MATLAB Function: '<S1>/unicycle_feedback_velocity'
  //   Sum: '<S1>/Sum'
  //   Sum: '<S1>/Sum1'

  rtb_TmpSignalConversionAtSFun_0 = (beta_estimator_velocity_drift_B.In1.X -
    beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[0]) *
    beta_estimator_velocity_drift_P.Kp_x_Gain;
  rtb_TmpSignalConversionAtSFun_1 = (beta_estimator_velocity_drift_B.In1.Y -
    beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[1]) *
    beta_estimator_velocity_drift_P.Kp_y_Gain;

  // MATLAB Function: '<S1>/unicycle_feedback_velocity' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'

  //  State and input variables
  // MATLAB Function 'Beta estimator velocity/unicycle_feedback_velocity': '<S5>:1' 
  // '<S5>:1:4' x     = state(1);
  // '<S5>:1:5' y     = state(2);
  // '<S5>:1:6' gamma = state(3);
  //  theta+beta
  // '<S5>:1:8' vxp   = input(1);
  // '<S5>:1:9' vyp   = input(2);
  //  Parameters
  // '<S5>:1:12' p = param(1);
  //  Model equations
  // '<S5>:1:15' dstate = zeros(3,1);
  // '<S5>:1:16' dstate(1) = vxp*cos(gamma)^2+vyp*sin(gamma)*cos(gamma);
  x = cos(beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2]);
  rtb_dstate_idx_0 = x * x * rtb_TmpSignalConversionAtSFun_0 +
    rtb_TmpSignalConversionAtSFun_1 * sin
    (beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2]) * cos
    (beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2]);

  // '<S5>:1:17' dstate(2) = vxp*sin(gamma)*cos(gamma)+vyp*sin(gamma)^2;
  x = sin(beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2]);

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   MATLAB Function: '<S1>/unicycle_feedback_velocity'
  //   MATLABSystem: '<S2>/Get Parameter3'
  //   Start for MATLABSystem: '<S2>/Get Parameter3'

  // '<S5>:1:18' dstate(3) = (vyp*cos(gamma)-vxp*sin(gamma))/p;
  beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[0] +=
    beta_estimator_velocity_drift_P.DiscreteTimeIntegrator_gainval *
    rtb_dstate_idx_0;
  beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[1] +=
    (rtb_TmpSignalConversionAtSFun_0 * sin
     (beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2]) * cos
     (beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2]) + x * x *
     rtb_TmpSignalConversionAtSFun_1) *
    beta_estimator_velocity_drift_P.DiscreteTimeIntegrator_gainval;
  beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2] +=
    (rtb_TmpSignalConversionAtSFun_1 * cos
     (beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2]) -
     rtb_TmpSignalConversionAtSFun_0 * sin
     (beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2])) / value
    * beta_estimator_velocity_drift_P.DiscreteTimeIntegrator_gainval;
}

// Model initialize function
void beta_estimator_velocity_driftingcar_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(beta_estimator_velocity_drif_M, (NULL));

  // block I/O
  (void) memset(((void *) &beta_estimator_velocity_drift_B), 0,
                sizeof(B_beta_estimator_velocity_dri_T));

  // states (dwork)
  (void) memset((void *)&beta_estimator_velocity_drif_DW, 0,
                sizeof(DW_beta_estimator_velocity_dr_T));

  {
    static const char_T tmp[17] = { '/', 'b', 'e', 't', 'a', '_', 'e', 's', 't',
      'i', 'm', 'a', 't', 'o', 'r', '/', 'p' };

    static const char_T tmp_0[16] = { '/', 'c', 'a', 'r', '/', 'g', 'r', 'o',
      'u', 'n', 'd', '_', 'p', 'o', 's', 'e' };

    static const char_T tmp_1[25] = { '/', 'g', 'a', 'm', 'm', 'a', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'v', 'e', 'l', 'o', 'c', 'i',
      't', 'y' };

    char_T tmp_2[18];
    char_T tmp_3[17];
    int32_T i;

    // Start for Atomic SubSystem: '<S3>/Publish1'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    beta_estimator_velocity_drif_DW.obj_a.isInitialized = 0;
    beta_estimator_velocity_drif_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      beta_estimator_velocity_drift_B.cv0[i] = tmp_1[i];
    }

    beta_estimator_velocity_drift_B.cv0[25] = '\x00';
    Pub_beta_estimator_velocity_driftingcar_75.createPublisher
      (beta_estimator_velocity_drift_B.cv0, beta_estimator__MessageQueueLen);

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish1'

    // Start for Atomic SubSystem: '<S4>/Subscribe'
    // Start for MATLABSystem: '<S8>/SourceBlock'
    beta_estimator_velocity_drif_DW.obj_l.isInitialized = 0;
    beta_estimator_velocity_drif_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      tmp_3[i] = tmp_0[i];
    }

    tmp_3[16] = '\x00';
    Sub_beta_estimator_velocity_driftingcar_77.createSubscriber(tmp_3,
      beta_estimator__MessageQueueLen);

    // End of Start for MATLABSystem: '<S8>/SourceBlock'
    // End of Start for SubSystem: '<S4>/Subscribe'

    // Start for MATLABSystem: '<S2>/Get Parameter3'
    beta_estimator_velocity_drif_DW.obj.isInitialized = 0;
    beta_estimator_velocity_drif_DW.obj.SampleTime =
      beta_estimator_velocity_drift_P.GetParameter3_SampleTime;
    beta_estimator_velocity_drif_DW.obj.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      tmp_2[i] = tmp[i];
    }

    tmp_2[17] = '\x00';
    ParamGet_beta_estimator_velocity_driftingcar_63.initialize(tmp_2);
    ParamGet_beta_estimator_velocity_driftingcar_63.initialize_error_codes(0U,
      1U, 2U, 3U);
    ParamGet_beta_estimator_velocity_driftingcar_63.set_initial_value
      (beta_esti_ParameterInitialValue);

    // End of Start for MATLABSystem: '<S2>/Get Parameter3'

    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' 
    beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[0] =
      beta_estimator_velocity_drift_P.DiscreteTimeIntegrator_IC;
    beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[1] =
      beta_estimator_velocity_drift_P.DiscreteTimeIntegrator_IC;
    beta_estimator_velocity_drif_DW.DiscreteTimeIntegrator_DSTATE[2] =
      beta_estimator_velocity_drift_P.DiscreteTimeIntegrator_IC;

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S9>/Out1'
    beta_estimator_velocity_drift_B.In1 =
      beta_estimator_velocity_drift_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe'
  }
}

// Model terminate function
void beta_estimator_velocity_driftingcar_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S3>/Publish1'
  // Start for MATLABSystem: '<S7>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S7>/SinkBlock'

  if (beta_estimator_velocity_drif_DW.obj_a.isInitialized == 1) {
    beta_estimator_velocity_drif_DW.obj_a.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S7>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish1'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe'
  // Start for MATLABSystem: '<S8>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S8>/SourceBlock'

  if (beta_estimator_velocity_drif_DW.obj_l.isInitialized == 1) {
    beta_estimator_velocity_drif_DW.obj_l.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S8>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe'

  // Start for MATLABSystem: '<S2>/Get Parameter3' incorporates:
  //   Terminate for MATLABSystem: '<S2>/Get Parameter3'

  if (beta_estimator_velocity_drif_DW.obj.isInitialized == 1) {
    beta_estimator_velocity_drif_DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S2>/Get Parameter3'
}

//
// File trailer for generated code.
//
// [EOF]
//
