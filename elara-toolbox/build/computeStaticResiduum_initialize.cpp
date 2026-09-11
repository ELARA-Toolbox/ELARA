//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeStaticResiduum_initialize.cpp
//
// Code generation for function 'computeStaticResiduum_initialize'
//

// Include files
#include "computeStaticResiduum_initialize.h"
#include "_coder_computeStaticResiduum_mex.h"
#include "computeStaticResiduum_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void computeStaticResiduum_once();

// Function Definitions
static void computeStaticResiduum_once()
{
  mex_InitInfAndNan();
}

void computeStaticResiduum_initialize()
{
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar =
      emlrtGetBreakCheckFlagAddressR2022b(emlrtRootTLSGlobal);
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    computeStaticResiduum_once();
  }
}

// End of code generation (computeStaticResiduum_initialize.cpp)
