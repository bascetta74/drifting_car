//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: WithIsNew.h
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
#ifndef RTW_HEADER_WithIsNew_h_
#define RTW_HEADER_WithIsNew_h_
#include <stddef.h>
#include <string.h>
#ifndef WithIsNew_COMMON_INCLUDES_
# define WithIsNew_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 // WithIsNew_COMMON_INCLUDES_

#include "WithIsNew_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (auto storage)
typedef struct {
  SL_Bus_WithIsNew_geometry_msgs_PoseArray varargout_2;
  SL_Bus_WithIsNew_geometry_msgs_PointStamped BusAssignment;// '<S1>/Bus Assignment' 
} B_WithIsNew_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  void *SourceBlock_PWORK;             // '<S2>/SourceBlock'
  void *SinkBlock_PWORK;               // '<S4>/SinkBlock'
  robotics_slros_internal_block_T obj; // '<S4>/SinkBlock'
  robotics_slros_internal_blo_e_T obj_f;// '<S2>/SourceBlock'
} DW_WithIsNew_T;

// Parameters (auto storage)
struct P_WithIsNew_T_ {
  SL_Bus_WithIsNew_geometry_msgs_PoseArray Out1_Y0;// Computed Parameter: Out1_Y0
                                                   //  Referenced by: '<S5>/Out1'

  SL_Bus_WithIsNew_geometry_msgs_PoseArray Constant_Value;// Computed Parameter: Constant_Value
                                                          //  Referenced by: '<S2>/Constant'

  SL_Bus_WithIsNew_geometry_msgs_PointStamped Constant_Value_c;// Computed Parameter: Constant_Value_c
                                                               //  Referenced by: '<S3>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_WithIsNew_T {
  const char_T *errorStatus;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_WithIsNew_T WithIsNew_P;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B_WithIsNew_T WithIsNew_B;

// Block states (auto storage)
extern DW_WithIsNew_T WithIsNew_DW;

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
  extern void WithIsNew_initialize(void);
  extern void WithIsNew_step(void);
  extern void WithIsNew_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_WithIsNew_T *const WithIsNew_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion


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
//  '<Root>' : 'WithIsNew'
//  '<S1>'   : 'WithIsNew/Signals Publisher'
//  '<S2>'   : 'WithIsNew/Subscribe'
//  '<S3>'   : 'WithIsNew/Signals Publisher/Blank Message'
//  '<S4>'   : 'WithIsNew/Signals Publisher/Publish2'
//  '<S5>'   : 'WithIsNew/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_WithIsNew_h_

//
// File trailer for generated code.
//
// [EOF]
//
