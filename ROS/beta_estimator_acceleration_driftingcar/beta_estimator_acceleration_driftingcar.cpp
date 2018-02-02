//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: beta_estimator_acceleration_driftingcar.cpp
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
#include "beta_estimator_acceleration_driftingcar.h"
#include "beta_estimator_acceleration_driftingcar_private.h"
#define beta_esti_ParameterInitialValue (0.0)
#define beta_estimator__MessageQueueLen (1)

// Block signals (auto storage)
B_beta_estimator_acceleration_T beta_estimator_acceleration_d_B;

// Block states (auto storage)
DW_beta_estimator_acceleratio_T beta_estimator_acceleration__DW;

// Real-time model
RT_MODEL_beta_estimator_accel_T beta_estimator_acceleration__M_;
RT_MODEL_beta_estimator_accel_T *const beta_estimator_acceleration__M =
  &beta_estimator_acceleration__M_;

// Model step function
void beta_estimator_acceleration_driftingcar_step(void)
{
  boolean_T p;
  SL_Bus_beta_estimator_accelerati_Pose2D_de4qul varargout_2;
  boolean_T varargout_1;
  SL_Bus_beta_estimator_accelerati_Float64_woid8t rtb_BusAssignment1;
  real_T rtb_PD_y;
  real_T rtb_PD_x;
  real_T rtb_dstate_idx_3;

  // BusAssignment: '<S3>/Bus Assignment1' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'

  rtb_BusAssignment1.Data =
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2];

  // Outputs for Atomic SubSystem: '<S3>/Publish1'
  // Start for MATLABSystem: '<S7>/SinkBlock' incorporates:
  //   MATLABSystem: '<S7>/SinkBlock'

  Pub_beta_estimator_acceleration_driftingcar_57.publish(&rtb_BusAssignment1);

  // End of Outputs for SubSystem: '<S3>/Publish1'

  // Outputs for Atomic SubSystem: '<S4>/Subscribe'
  // Start for MATLABSystem: '<S8>/SourceBlock' incorporates:
  //   Inport: '<S9>/In1'
  //   MATLABSystem: '<S8>/SourceBlock'

  varargout_1 = Sub_beta_estimator_acceleration_driftingcar_60.getLatestMessage(
    &varargout_2);

  // Outputs for Enabled SubSystem: '<S8>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S9>/Enable'

  if (varargout_1) {
    beta_estimator_acceleration_d_B.In1 = varargout_2;
  }

  // End of Start for MATLABSystem: '<S8>/SourceBlock'
  // End of Outputs for SubSystem: '<S8>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<S4>/Subscribe'

  // Start for MATLABSystem: '<S2>/Get Parameter3' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter3'

  varargout_1 = false;
  p = true;
  if (!(beta_estimator_acceleration__DW.obj.SampleTime ==
        beta_estimator_acceleration_d_P.GetParameter3_SampleTime)) {
    p = false;
  }

  if (p) {
    varargout_1 = true;
  }

  if (!varargout_1) {
    beta_estimator_acceleration__DW.obj.SampleTime =
      beta_estimator_acceleration_d_P.GetParameter3_SampleTime;
  }

  ParamGet_beta_estimator_acceleration_driftingcar_72.get_parameter(&rtb_PD_y);

  // Start for MATLABSystem: '<S2>/Get Parameter1' incorporates:
  //   MATLABSystem: '<S2>/Get Parameter1'

  varargout_1 = false;
  p = true;
  if (!(beta_estimator_acceleration__DW.obj_b.SampleTime ==
        beta_estimator_acceleration_d_P.GetParameter1_SampleTime)) {
    p = false;
  }

  if (p) {
    varargout_1 = true;
  }

  if (!varargout_1) {
    beta_estimator_acceleration__DW.obj_b.SampleTime =
      beta_estimator_acceleration_d_P.GetParameter1_SampleTime;
  }

  ParamGet_beta_estimator_acceleration_driftingcar_74.get_parameter(&rtb_PD_x);

  // End of Start for MATLABSystem: '<S2>/Get Parameter1'

  // DiscreteTransferFcn: '<S1>/PD_x' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   MATLABSystem: '<S2>/Get Parameter3'
  //   Start for MATLABSystem: '<S2>/Get Parameter3'
  //   Sum: '<S1>/Sum'

  rtb_PD_x = (beta_estimator_acceleration_d_B.In1.X -
              beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[0]) *
    rtb_PD_y;

  // DiscreteTransferFcn: '<S1>/PD_y' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   MATLABSystem: '<S2>/Get Parameter3'
  //   Start for MATLABSystem: '<S2>/Get Parameter3'
  //   Sum: '<S1>/Sum1'

  rtb_PD_y *= beta_estimator_acceleration_d_B.In1.Y -
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[1];

  // MATLAB Function: '<S1>/unicycle_feedback_acceleration' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   SignalConversion: '<S5>/TmpSignal ConversionAt SFunction Inport2'

  //  State and input variables
  // MATLAB Function 'Beta estimator acceleration/unicycle_feedback_acceleration': '<S5>:1' 
  // '<S5>:1:4' x     = state(1);
  // '<S5>:1:5' y     = state(2);
  // '<S5>:1:6' gamma = state(3);
  //  theta+beta
  // '<S5>:1:7' v     = state(4);
  // '<S5>:1:9' ax    = input(1);
  // '<S5>:1:10' ay    = input(2);
  //  Model equations
  // '<S5>:1:13' dstate = zeros(4,1);
  // '<S5>:1:14' dstate(1) = v*cos(gamma);
  // '<S5>:1:15' dstate(2) = v*sin(gamma);
  // '<S5>:1:16' dstate(3) = (ay*cos(gamma)-ax*sin(gamma))/v;
  // '<S5>:1:17' dstate(4) = ax*cos(gamma)+ay*sin(gamma);
  rtb_dstate_idx_3 = rtb_PD_x * cos
    (beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2]) +
    rtb_PD_y * sin
    (beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2]);

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  //   MATLAB Function: '<S1>/unicycle_feedback_acceleration'
  //   SignalConversion: '<S5>/TmpSignal ConversionAt SFunction Inport2'

  beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[0] +=
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[3] * cos
    (beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2]) *
    beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_gainval;
  beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[1] +=
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[3] * sin
    (beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2]) *
    beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_gainval;
  beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2] += (rtb_PD_y *
    cos(beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2]) -
    rtb_PD_x * sin
    (beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2])) /
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[3] *
    beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_gainval;
  beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[3] +=
    beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_gainval *
    rtb_dstate_idx_3;
}

// Model initialize function
void beta_estimator_acceleration_driftingcar_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(beta_estimator_acceleration__M, (NULL));

  // block I/O
  (void) memset(((void *) &beta_estimator_acceleration_d_B), 0,
                sizeof(B_beta_estimator_acceleration_T));

  // states (dwork)
  (void) memset((void *)&beta_estimator_acceleration__DW, 0,
                sizeof(DW_beta_estimator_acceleratio_T));

  {
    static const char_T tmp[47] = { '/', 'b', 'e', 't', 'a', '_', 'e', 's', 't',
      'i', 'm', 'a', 't', 'o', 'r', '_', 'a', 'c', 'c', 'e', 'l', 'e', 'r', 'a',
      't', 'i', 'o', 'n', '_', 'd', 'r', 'i', 'f', 't', 'i', 'n', 'g', 'c', 'a',
      'r', '/', 'P', 'D', '_', 'D', 'e', 'n' };

    static const char_T tmp_0[47] = { '/', 'b', 'e', 't', 'a', '_', 'e', 's',
      't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'a', 'c', 'c', 'e', 'l', 'e', 'r',
      'a', 't', 'i', 'o', 'n', '_', 'd', 'r', 'i', 'f', 't', 'i', 'n', 'g', 'c',
      'a', 'r', '/', 'P', 'D', '_', 'N', 'u', 'm' };

    static const char_T tmp_1[16] = { '/', 'c', 'a', 'r', '/', 'g', 'r', 'o',
      'u', 'n', 'd', '_', 'p', 'o', 's', 'e' };

    static const char_T tmp_2[29] = { '/', 'g', 'a', 'm', 'm', 'a', '_', 'e',
      's', 't', 'i', 'm', 'a', 't', 'o', 'r', '_', 'a', 'c', 'c', 'e', 'l', 'e',
      'r', 'a', 't', 'i', 'o', 'n' };

    char_T tmp_3[17];
    char_T tmp_4[30];
    int32_T i;

    // Start for Atomic SubSystem: '<S3>/Publish1'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    beta_estimator_acceleration__DW.obj_bg.isInitialized = 0;
    beta_estimator_acceleration__DW.obj_bg.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      tmp_4[i] = tmp_2[i];
    }

    tmp_4[29] = '\x00';
    Pub_beta_estimator_acceleration_driftingcar_57.createPublisher(tmp_4,
      beta_estimator__MessageQueueLen);

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of Start for SubSystem: '<S3>/Publish1'

    // Start for Atomic SubSystem: '<S4>/Subscribe'
    // Start for MATLABSystem: '<S8>/SourceBlock'
    beta_estimator_acceleration__DW.obj_c.isInitialized = 0;
    beta_estimator_acceleration__DW.obj_c.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      tmp_3[i] = tmp_1[i];
    }

    tmp_3[16] = '\x00';
    Sub_beta_estimator_acceleration_driftingcar_60.createSubscriber(tmp_3,
      beta_estimator__MessageQueueLen);

    // End of Start for MATLABSystem: '<S8>/SourceBlock'
    // End of Start for SubSystem: '<S4>/Subscribe'

    // Start for MATLABSystem: '<S2>/Get Parameter3'
    beta_estimator_acceleration__DW.obj.isInitialized = 0;
    beta_estimator_acceleration__DW.obj.SampleTime =
      beta_estimator_acceleration_d_P.GetParameter3_SampleTime;
    beta_estimator_acceleration__DW.obj.isInitialized = 1;
    for (i = 0; i < 47; i++) {
      beta_estimator_acceleration_d_B.cv0[i] = tmp_0[i];
    }

    beta_estimator_acceleration_d_B.cv0[47] = '\x00';
    ParamGet_beta_estimator_acceleration_driftingcar_72.initialize
      (beta_estimator_acceleration_d_B.cv0);
    ParamGet_beta_estimator_acceleration_driftingcar_72.initialize_error_codes
      (0U, 1U, 2U, 3U);
    ParamGet_beta_estimator_acceleration_driftingcar_72.set_initial_value
      (beta_esti_ParameterInitialValue);

    // End of Start for MATLABSystem: '<S2>/Get Parameter3'

    // Start for MATLABSystem: '<S2>/Get Parameter1'
    beta_estimator_acceleration__DW.obj_b.isInitialized = 0;
    beta_estimator_acceleration__DW.obj_b.SampleTime =
      beta_estimator_acceleration_d_P.GetParameter1_SampleTime;
    beta_estimator_acceleration__DW.obj_b.isInitialized = 1;
    for (i = 0; i < 47; i++) {
      beta_estimator_acceleration_d_B.cv0[i] = tmp[i];
    }

    beta_estimator_acceleration_d_B.cv0[47] = '\x00';
    ParamGet_beta_estimator_acceleration_driftingcar_74.initialize
      (beta_estimator_acceleration_d_B.cv0);
    ParamGet_beta_estimator_acceleration_driftingcar_74.initialize_error_codes
      (0U, 1U, 2U, 3U);
    ParamGet_beta_estimator_acceleration_driftingcar_74.set_initial_value
      (beta_esti_ParameterInitialValue);

    // End of Start for MATLABSystem: '<S2>/Get Parameter1'

    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' 
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[0] =
      beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_IC;
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[1] =
      beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_IC;
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[2] =
      beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_IC;
    beta_estimator_acceleration__DW.DiscreteTimeIntegrator_DSTATE[3] =
      beta_estimator_acceleration_d_P.DiscreteTimeIntegrator_IC;

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S8>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S9>/Out1'
    beta_estimator_acceleration_d_B.In1 =
      beta_estimator_acceleration_d_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S8>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe'
  }
}

// Model terminate function
void beta_estimator_acceleration_driftingcar_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S3>/Publish1'
  // Start for MATLABSystem: '<S7>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S7>/SinkBlock'

  if (beta_estimator_acceleration__DW.obj_bg.isInitialized == 1) {
    beta_estimator_acceleration__DW.obj_bg.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S7>/SinkBlock'
  // End of Terminate for SubSystem: '<S3>/Publish1'

  // Terminate for Atomic SubSystem: '<S4>/Subscribe'
  // Start for MATLABSystem: '<S8>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S8>/SourceBlock'

  if (beta_estimator_acceleration__DW.obj_c.isInitialized == 1) {
    beta_estimator_acceleration__DW.obj_c.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S8>/SourceBlock'
  // End of Terminate for SubSystem: '<S4>/Subscribe'

  // Start for MATLABSystem: '<S2>/Get Parameter3' incorporates:
  //   Terminate for MATLABSystem: '<S2>/Get Parameter3'

  if (beta_estimator_acceleration__DW.obj.isInitialized == 1) {
    beta_estimator_acceleration__DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S2>/Get Parameter3'

  // Start for MATLABSystem: '<S2>/Get Parameter1' incorporates:
  //   Terminate for MATLABSystem: '<S2>/Get Parameter1'

  if (beta_estimator_acceleration__DW.obj_b.isInitialized == 1) {
    beta_estimator_acceleration__DW.obj_b.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S2>/Get Parameter1'
}

//
// File trailer for generated code.
//
// [EOF]
//
