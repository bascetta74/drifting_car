//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: AtomicDualRate.cpp
//
// Code generated for Simulink model 'AtomicDualRate'.
//
// Model version                  : 1.31
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Tue Jan 30 10:13:45 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "AtomicDualRate.h"
#include "AtomicDualRate_private.h"
#define AtomicDualRate_MessageQueueLen (1)

// Block signals (auto storage)
B_AtomicDualRate_T AtomicDualRate_B;

// Block states (auto storage)
DW_AtomicDualRate_T AtomicDualRate_DW;

// Real-time model
RT_MODEL_AtomicDualRate_T AtomicDualRate_M_;
RT_MODEL_AtomicDualRate_T *const AtomicDualRate_M = &AtomicDualRate_M_;
static void rate_scheduler(void);

//
//   This function updates active task flag for each subrate.
// The function is called at model base rate, hence the
// generated code self-manages all its subrates.
//
static void rate_scheduler(void)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (AtomicDualRate_M->Timing.TaskCounters.TID[1])++;
  if ((AtomicDualRate_M->Timing.TaskCounters.TID[1]) > 4) {// Sample time: [0.05s, 0.0s] 
    AtomicDualRate_M->Timing.TaskCounters.TID[1] = 0;
  }
}

// Model step function
void AtomicDualRate_step(void)
{
  boolean_T varargout_1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S3>/SourceBlock' incorporates:
  //   Inport: '<S8>/In1'
  //   MATLABSystem: '<S3>/SourceBlock'

  varargout_1 = Sub_AtomicDualRate_40.getLatestMessage
    (&AtomicDualRate_B.varargout_2);

  // Outputs for Enabled SubSystem: '<S3>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S8>/Enable'

  if (varargout_1) {
    AtomicDualRate_B.In1 = AtomicDualRate_B.varargout_2;
  }

  // End of Start for MATLABSystem: '<S3>/SourceBlock'
  // End of Outputs for SubSystem: '<S3>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Outputs for Atomic SubSystem: '<Root>/Publisher_Fast'
  // BusAssignment: '<S1>/Bus Assignment' incorporates:
  //   Constant: '<S4>/Constant'
  //   DataTypeConversion: '<S1>/Data Type Conversion'

  AtomicDualRate_B.BusAssignment_l = AtomicDualRate_P.Constant_Value_c;
  AtomicDualRate_B.BusAssignment_l.Point.X = AtomicDualRate_B.In1.Header.Seq;
  AtomicDualRate_B.BusAssignment_l.Point.Y =
    AtomicDualRate_B.In1.Header.Stamp.Sec;
  AtomicDualRate_B.BusAssignment_l.Point.Z =
    AtomicDualRate_B.In1.Header.Stamp.Nsec;

  // Outputs for Atomic SubSystem: '<S1>/Publish2'
  // Start for MATLABSystem: '<S5>/SinkBlock' incorporates:
  //   MATLABSystem: '<S5>/SinkBlock'

  Pub_AtomicDualRate_66.publish(&AtomicDualRate_B.BusAssignment_l);

  // End of Outputs for SubSystem: '<S1>/Publish2'
  // End of Outputs for SubSystem: '<Root>/Publisher_Fast'

  // RateTransition: '<Root>/Rate Transition'
  if (AtomicDualRate_M->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Atomic SubSystem: '<Root>/Publisher_Slow'
    // BusAssignment: '<S2>/Bus Assignment' incorporates:
    //   Constant: '<S6>/Constant'
    //   DataTypeConversion: '<S2>/Data Type Conversion'

    AtomicDualRate_B.BusAssignment_l = AtomicDualRate_P.Constant_Value_a;
    AtomicDualRate_B.BusAssignment_l.Point.X = AtomicDualRate_B.In1.Header.Seq;
    AtomicDualRate_B.BusAssignment_l.Point.Y =
      AtomicDualRate_B.In1.Header.Stamp.Sec;
    AtomicDualRate_B.BusAssignment_l.Point.Z =
      AtomicDualRate_B.In1.Header.Stamp.Nsec;

    // Outputs for Atomic SubSystem: '<S2>/Publish2'
    // Start for MATLABSystem: '<S7>/SinkBlock' incorporates:
    //   MATLABSystem: '<S7>/SinkBlock'

    Pub_AtomicDualRate_90.publish(&AtomicDualRate_B.BusAssignment_l);

    // End of Outputs for SubSystem: '<S2>/Publish2'
    // End of Outputs for SubSystem: '<Root>/Publisher_Slow'
  }

  // End of RateTransition: '<Root>/Rate Transition'
  rate_scheduler();
}

// Model initialize function
void AtomicDualRate_initialize(void)
{
  // Registration code

  // initialize real-time model
  (void) memset((void *)AtomicDualRate_M, 0,
                sizeof(RT_MODEL_AtomicDualRate_T));

  // block I/O
  (void) memset(((void *) &AtomicDualRate_B), 0,
                sizeof(B_AtomicDualRate_T));

  // states (dwork)
  (void) memset((void *)&AtomicDualRate_DW, 0,
                sizeof(DW_AtomicDualRate_T));

  {
    static const char_T tmp[13] = { '/', 'C', 'h', 'a', 't', 't', 'e', 'r', '_',
      'S', 'l', 'o', 'w' };

    static const char_T tmp_0[13] = { '/', 'C', 'h', 'a', 't', 't', 'e', 'r',
      '_', 'F', 'a', 's', 't' };

    static const char_T tmp_1[8] = { '/', 'c', 'h', 'a', 't', 't', 'e', 'r' };

    char_T tmp_2[14];
    char_T tmp_3[9];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S3>/SourceBlock'
    AtomicDualRate_DW.obj_f.isInitialized = 0;
    AtomicDualRate_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_3[i] = tmp_1[i];
    }

    tmp_3[8] = '\x00';
    Sub_AtomicDualRate_40.createSubscriber(tmp_3, AtomicDualRate_MessageQueueLen);

    // End of Start for MATLABSystem: '<S3>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Atomic SubSystem: '<Root>/Publisher_Fast'
    // Start for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    AtomicDualRate_DW.obj_m.isInitialized = 0;
    AtomicDualRate_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_2[i] = tmp_0[i];
    }

    tmp_2[13] = '\x00';
    Pub_AtomicDualRate_66.createPublisher(tmp_2, AtomicDualRate_MessageQueueLen);

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
    // End of Start for SubSystem: '<S1>/Publish2'
    // End of Start for SubSystem: '<Root>/Publisher_Fast'

    // Start for Atomic SubSystem: '<Root>/Publisher_Slow'
    // Start for Atomic SubSystem: '<S2>/Publish2'
    // Start for MATLABSystem: '<S7>/SinkBlock'
    AtomicDualRate_DW.obj.isInitialized = 0;
    AtomicDualRate_DW.obj.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_2[i] = tmp[i];
    }

    tmp_2[13] = '\x00';
    Pub_AtomicDualRate_90.createPublisher(tmp_2, AtomicDualRate_MessageQueueLen);

    // End of Start for MATLABSystem: '<S7>/SinkBlock'
    // End of Start for SubSystem: '<S2>/Publish2'
    // End of Start for SubSystem: '<Root>/Publisher_Slow'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S3>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S8>/Out1'
    AtomicDualRate_B.In1 = AtomicDualRate_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S3>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
  }
}

// Model terminate function
void AtomicDualRate_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S3>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S3>/SourceBlock'

  if (AtomicDualRate_DW.obj_f.isInitialized == 1) {
    AtomicDualRate_DW.obj_f.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S3>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publisher_Fast'
  // Terminate for Atomic SubSystem: '<S1>/Publish2'
  // Start for MATLABSystem: '<S5>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S5>/SinkBlock'

  if (AtomicDualRate_DW.obj_m.isInitialized == 1) {
    AtomicDualRate_DW.obj_m.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S5>/SinkBlock'
  // End of Terminate for SubSystem: '<S1>/Publish2'
  // End of Terminate for SubSystem: '<Root>/Publisher_Fast'

  // Terminate for Atomic SubSystem: '<Root>/Publisher_Slow'
  // Terminate for Atomic SubSystem: '<S2>/Publish2'
  // Start for MATLABSystem: '<S7>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S7>/SinkBlock'

  if (AtomicDualRate_DW.obj.isInitialized == 1) {
    AtomicDualRate_DW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S7>/SinkBlock'
  // End of Terminate for SubSystem: '<S2>/Publish2'
  // End of Terminate for SubSystem: '<Root>/Publisher_Slow'
}

//
// File trailer for generated code.
//
// [EOF]
//
