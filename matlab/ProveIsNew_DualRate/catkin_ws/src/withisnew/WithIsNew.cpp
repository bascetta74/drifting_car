//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: WithIsNew.cpp
//
// Code generated for Simulink model 'WithIsNew'.
//
// Model version                  : 1.28
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Fri Jan 26 16:55:18 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "WithIsNew.h"
#include "WithIsNew_private.h"
#define WithIsNew_MessageQueueLen      (1)

// Block signals (auto storage)
B_WithIsNew_T WithIsNew_B;

// Block states (auto storage)
DW_WithIsNew_T WithIsNew_DW;

// Real-time model
RT_MODEL_WithIsNew_T WithIsNew_M_;
RT_MODEL_WithIsNew_T *const WithIsNew_M = &WithIsNew_M_;

// Model step function
void WithIsNew_step(void)
{
  boolean_T varargout_1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S2>/SourceBlock' incorporates:
  //   MATLABSystem: '<S2>/SourceBlock'

  varargout_1 = Sub_WithIsNew_40.getLatestMessage(&WithIsNew_B.varargout_2);

  // Outputs for Enabled SubSystem: '<Root>/Signals Publisher' incorporates:
  //   EnablePort: '<S1>/Enable'

  // Outputs for Enabled SubSystem: '<S2>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S5>/Enable'

  if (varargout_1) {
    // BusAssignment: '<S1>/Bus Assignment' incorporates:
    //   Constant: '<S3>/Constant'
    //   DataTypeConversion: '<S1>/Data Type Conversion'

    WithIsNew_B.BusAssignment = WithIsNew_P.Constant_Value_c;
    WithIsNew_B.BusAssignment.Point.X = WithIsNew_B.varargout_2.Header.Seq;
    WithIsNew_B.BusAssignment.Point.Y = WithIsNew_B.varargout_2.Header.Stamp.Sec;
    WithIsNew_B.BusAssignment.Point.Z =
      WithIsNew_B.varargout_2.Header.Stamp.Nsec;

    // Outputs for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S4>/SinkBlock' incorporates:
    //   MATLABSystem: '<S4>/SinkBlock'

    Pub_WithIsNew_8.publish(&WithIsNew_B.BusAssignment);

    // End of Outputs for SubSystem: '<S1>/Publish2'
  }

  // End of Start for MATLABSystem: '<S2>/SourceBlock'
  // End of Outputs for SubSystem: '<S2>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Signals Publisher'
  // End of Outputs for SubSystem: '<Root>/Subscribe'
}

// Model initialize function
void WithIsNew_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(WithIsNew_M, (NULL));

  // states (dwork)
  (void) memset((void *)&WithIsNew_DW, 0,
                sizeof(DW_WithIsNew_T));

  {
    static const char_T tmp[18] = { '/', 'C', 'h', 'a', 't', 't', 'e', 'r', '_',
      'W', 'i', 't', 'h', 'I', 's', 'N', 'e', 'w' };

    static const char_T tmp_0[8] = { '/', 'c', 'h', 'a', 't', 't', 'e', 'r' };

    char_T tmp_1[19];
    char_T tmp_2[9];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S2>/SourceBlock'
    WithIsNew_DW.obj_f.isInitialized = 0;
    WithIsNew_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_2[i] = tmp_0[i];
    }

    tmp_2[8] = '\x00';
    Sub_WithIsNew_40.createSubscriber(tmp_2, WithIsNew_MessageQueueLen);

    // End of Start for MATLABSystem: '<S2>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Enabled SubSystem: '<Root>/Signals Publisher'
    // Start for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    WithIsNew_DW.obj.isInitialized = 0;
    WithIsNew_DW.obj.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      tmp_1[i] = tmp[i];
    }

    tmp_1[18] = '\x00';
    Pub_WithIsNew_8.createPublisher(tmp_1, WithIsNew_MessageQueueLen);

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of Start for SubSystem: '<S1>/Publish2'
    // End of Start for SubSystem: '<Root>/Signals Publisher'
  }
}

// Model terminate function
void WithIsNew_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S2>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S2>/SourceBlock'

  if (WithIsNew_DW.obj_f.isInitialized == 1) {
    WithIsNew_DW.obj_f.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S2>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Enabled SubSystem: '<Root>/Signals Publisher'
  // Terminate for Atomic SubSystem: '<S1>/Publish2'
  // Start for MATLABSystem: '<S4>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S4>/SinkBlock'

  if (WithIsNew_DW.obj.isInitialized == 1) {
    WithIsNew_DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S4>/SinkBlock'
  // End of Terminate for SubSystem: '<S1>/Publish2'
  // End of Terminate for SubSystem: '<Root>/Signals Publisher'
}

//
// File trailer for generated code.
//
// [EOF]
//
