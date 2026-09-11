//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// getSimResFromStateTrajectory_initialize.cpp
//
// Code generation for function 'getSimResFromStateTrajectory_initialize'
//

// Include files
#include "getSimResFromStateTrajectory_initialize.h"
#include "_coder_getSimResFromStateTrajectory_mex.h"
#include "getSimResFromStateTrajectory_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void getSimResFromStateTrajectory_once();

// Function Definitions
static void getSimResFromStateTrajectory_once()
{
  mex_InitInfAndNan();
}

void getSimResFromStateTrajectory_initialize()
{
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar =
      emlrtGetBreakCheckFlagAddressR2022b(emlrtRootTLSGlobal);
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    getSimResFromStateTrajectory_once();
  }
}

// End of code generation (getSimResFromStateTrajectory_initialize.cpp)
