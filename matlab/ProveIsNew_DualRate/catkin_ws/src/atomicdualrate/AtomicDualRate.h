//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: AtomicDualRate.h
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
#ifndef RTW_HEADER_AtomicDualRate_h_
#define RTW_HEADER_AtomicDualRate_h_
#include <string.h>
#include <stddef.h>
#ifndef AtomicDualRate_COMMON_INCLUDES_
# define AtomicDualRate_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 // AtomicDualRate_COMMON_INCLUDES_

#include "AtomicDualRate_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (auto storage)
typedef struct {
  SL_Bus_AtomicDualRate_geometry_msgs_PoseArray In1;// '<S8>/In1'
  SL_Bus_AtomicDualRate_geometry_msgs_PoseArray varargout_2;
  SL_Bus_AtomicDualRate_geometry_msgs_PointStamped BusAssignment_l;// '<S1>/Bus Assignment' 
} B_AtomicDualRate_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  void *SourceBlock_PWORK;             // '<S3>/SourceBlock'
  void *SinkBlock_PWORK;               // '<S7>/SinkBlock'
  void *SinkBlock_PWORK_j;             // '<S5>/SinkBlock'
  robotics_slros_internal_block_T obj; // '<S7>/SinkBlock'
  robotics_slros_internal_block_T obj_m;// '<S5>/SinkBlock'
  robotics_slros_internal_blo_p_T obj_f;// '<S3>/SourceBlock'
} DW_AtomicDualRate_T;

// Parameters (auto storage)
struct P_AtomicDualRate_T_ {
  SL_Bus_AtomicDualRate_geometry_msgs_PoseArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                        //  Referenced by: '<S8>/Out1'

  SL_Bus_AtomicDualRate_geometry_msgs_PoseArray Constant_Value;// Computed Parameter: Constant_Value
                                                               //  Referenced by: '<S3>/Constant'

  SL_Bus_AtomicDualRate_geometry_msgs_PointStamped Constant_Value_c;// Computed Parameter: Constant_Value_c
                                                                    //  Referenced by: '<S4>/Constant'

  SL_Bus_AtomicDualRate_geometry_msgs_PointStamped Constant_Value_a;// Computed Parameter: Constant_Value_a
                                                                    //  Referenced by: '<S6>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_AtomicDualRate_T {
  const char_T *errorStatus;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_AtomicDualRate_T AtomicDualRate_P;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B_AtomicDualRate_T AtomicDualRate_B;

// Block states (auto storage)
extern DW_AtomicDualRate_T AtomicDualRate_DW;

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void AtomicDualRate_initialize(void);
  extern void AtomicDualRate_step(void);
  extern void AtomicDualRate_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_AtomicDualRate_T *const AtomicDualRate_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S2>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S2>/Data Type Conversion2' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'AtomicDualRate'
//  '<S1>'   : 'AtomicDualRate/Publisher_Fast'
//  '<S2>'   : 'AtomicDualRate/Publisher_Slow'
//  '<S3>'   : 'AtomicDualRate/Subscribe'
//  '<S4>'   : 'AtomicDualRate/Publisher_Fast/Blank Message'
//  '<S5>'   : 'AtomicDualRate/Publisher_Fast/Publish2'
//  '<S6>'   : 'AtomicDualRate/Publisher_Slow/Blank Message'
//  '<S7>'   : 'AtomicDualRate/Publisher_Slow/Publish2'
//  '<S8>'   : 'AtomicDualRate/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_AtomicDualRate_h_

//
// File trailer for generated code.
//
// [EOF]
//
