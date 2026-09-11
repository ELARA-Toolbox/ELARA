//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeStaticResiduum_data.cpp
//
// Code generation for function 'computeStaticResiduum_data'
//

// Include files
#include "computeStaticResiduum_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

const volatile char_T *emlrtBreakCheckR2012bFlagVar{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131675U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "computeStaticResiduum",                              // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

omp_lock_t emlrtLockGlobal;

omp_nest_lock_t computeStaticResiduum_nestLockGlobal;

// End of code generation (computeStaticResiduum_data.cpp)
