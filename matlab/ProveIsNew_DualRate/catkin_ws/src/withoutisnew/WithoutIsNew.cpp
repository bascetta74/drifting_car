//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: WithoutIsNew.cpp
//
// Code generated for Simulink model 'WithoutIsNew'.
//
// Model version                  : 1.29
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Fri Jan 26 16:53:38 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "WithoutIsNew.h"
#include "WithoutIsNew_private.h"
#define WithoutIsNew_MessageQueueLen   (1)

// Block signals (auto storage)
B_WithoutIsNew_T WithoutIsNew_B;

// Block states (auto storage)
DW_WithoutIsNew_T WithoutIsNew_DW;

// Real-time model
RT_MODEL_WithoutIsNew_T WithoutIsNew_M_;
RT_MODEL_WithoutIsNew_T *const WithoutIsNew_M = &WithoutIsNew_M_;

// Model step function
void WithoutIsNew_step(void)
{
  boolean_T varargout_1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S2>/SourceBlock' incorporates:
  //   Inport: '<S5>/In1'
  //   MATLABSystem: '<S2>/SourceBlock'

  varargout_1 = Sub_WithoutIsNew_40.getLatestMessage(&WithoutIsNew_B.varargout_2);

  // Outputs for Enabled SubSystem: '<S2>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S5>/Enable'

  if (varargout_1) {
    WithoutIsNew_B.In1 = WithoutIsNew_B.varargout_2;
  }

  // End of Start for MATLABSystem: '<S2>/SourceBlock'
  // End of Outputs for SubSystem: '<S2>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // BusAssignment: '<S1>/Bus Assignment' incorporates:
  //   Constant: '<S3>/Constant'
  //   DataTypeConversion: '<S1>/Data Type Conversion'

  WithoutIsNew_B.BusAssignment = WithoutIsNew_P.Constant_Value_c;
  WithoutIsNew_B.BusAssignment.Point.X = WithoutIsNew_B.In1.Header.Seq;
  WithoutIsNew_B.BusAssignment.Point.Y = WithoutIsNew_B.In1.Header.Stamp.Sec;
  WithoutIsNew_B.BusAssignment.Point.Z = WithoutIsNew_B.In1.Header.Stamp.Nsec;

  // Outputs for Atomic SubSystem: '<S1>/Publish2'
  // Start for MATLABSystem: '<S4>/SinkBlock' incorporates:
  //   MATLABSystem: '<S4>/SinkBlock'

  Pub_WithoutIsNew_66.publish(&WithoutIsNew_B.BusAssignment);

  // End of Outputs for SubSystem: '<S1>/Publish2'
}

// Model initialize function
void WithoutIsNew_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(WithoutIsNew_M, (NULL));

  // block I/O
  (void) memset(((void *) &WithoutIsNew_B), 0,
                sizeof(B_WithoutIsNew_T));

  // states (dwork)
  (void) memset((void *)&WithoutIsNew_DW, 0,
                sizeof(DW_WithoutIsNew_T));

  {
    static const char_T tmp[21] = { '/', 'C', 'h', 'a', 't', 't', 'e', 'r', '_',
      'W', 'i', 't', 'h', 'o', 'u', 't', 'I', 's', 'N', 'e', 'w' };

    static const char_T tmp_0[8] = { '/', 'c', 'h', 'a', 't', 't', 'e', 'r' };

    char_T tmp_1[22];
    char_T tmp_2[9];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S2>/SourceBlock'
    WithoutIsNew_DW.obj_f.isInitialized = 0;
    WithoutIsNew_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_2[i] = tmp_0[i];
    }

    tmp_2[8] = '\x00';
    Sub_WithoutIsNew_40.createSubscriber(tmp_2, WithoutIsNew_MessageQueueLen);

    // End of Start for MATLABSystem: '<S2>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    WithoutIsNew_DW.obj.isInitialized = 0;
    WithoutIsNew_DW.obj.isInitialized = 1;
    for (i = 0; i < 21; i++) {
      tmp_1[i] = tmp[i];
    }

    tmp_1[21] = '\x00';
    Pub_WithoutIsNew_66.createPublisher(tmp_1, WithoutIsNew_MessageQueueLen);

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of Start for SubSystem: '<S1>/Publish2'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S2>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S5>/Out1'
    WithoutIsNew_B.In1 = WithoutIsNew_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S2>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void WithoutIsNew_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S2>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S2>/SourceBlock'

  if (WithoutIsNew_DW.obj_f.isInitialized == 1) {
    WithoutIsNew_DW.obj_f.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S2>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<S1>/Publish2'
  // Start for MATLABSystem: '<S4>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S4>/SinkBlock'

  if (WithoutIsNew_DW.obj.isInitialized == 1) {
    WithoutIsNew_DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S4>/SinkBlock'
  // End of Terminate for SubSystem: '<S1>/Publish2'
}

//
// File trailer for generated code.
//
// [EOF]
//
