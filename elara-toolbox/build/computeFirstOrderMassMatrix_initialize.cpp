//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFirstOrderMassMatrix_initialize.cpp
//
// Code generation for function 'computeFirstOrderMassMatrix_initialize'
//

// Include files
#include "computeFirstOrderMassMatrix_initialize.h"
#include "_coder_computeFirstOrderMassMatrix_mex.h"
#include "computeFirstOrderMassMatrix_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void computeFirstOrderMassMatrix_once();

// Function Definitions
static void computeFirstOrderMassMatrix_once()
{
  mex_InitInfAndNan();
}

void computeFirstOrderMassMatrix_initialize()
{
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar =
      emlrtGetBreakCheckFlagAddressR2022b(emlrtRootTLSGlobal);
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    computeFirstOrderMassMatrix_once();
  }
}

// End of code generation (computeFirstOrderMassMatrix_initialize.cpp)
