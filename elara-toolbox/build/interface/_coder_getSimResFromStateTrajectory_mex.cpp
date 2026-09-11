//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_getSimResFromStateTrajectory_mex.cpp
//
// Code generation for function '_coder_getSimResFromStateTrajectory_mex'
//

// Include files
#include "_coder_getSimResFromStateTrajectory_mex.h"
#include "_coder_getSimResFromStateTrajectory_api.h"
#include "getSimResFromStateTrajectory_data.h"
#include "getSimResFromStateTrajectory_initialize.h"
#include "getSimResFromStateTrajectory_terminate.h"
#include "rt_nonfinite.h"
#include <stdexcept>

void emlrtExceptionBridge();
void emlrtExceptionBridge()
{
  throw std::runtime_error("");
}
// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&getSimResFromStateTrajectory_atexit);
  getSimResFromStateTrajectory_initialize();
  try {
    unsafe_getSimResFromStateTrajectory_mexFunction(nlhs, plhs, nrhs, prhs);
    getSimResFromStateTrajectory_terminate();
  } catch (...) {
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           (void *)&emlrtExceptionBridge, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_getSimResFromStateTrajectory_mexFunction(int32_T nlhs,
                                                     mxArray *plhs[1],
                                                     int32_T nrhs,
                                                     const mxArray *prhs[4])
{
  const mxArray *outputs;
  // Check for proper number of arguments.
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 4, 4, 28, "getSimResFromStateTrajectory");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 28,
                        "getSimResFromStateTrajectory");
  }
  // Call the function.
  c_getSimResFromStateTrajectory_(prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

// End of code generation (_coder_getSimResFromStateTrajectory_mex.cpp)
