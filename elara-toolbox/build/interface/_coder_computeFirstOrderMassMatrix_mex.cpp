//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_computeFirstOrderMassMatrix_mex.cpp
//
// Code generation for function '_coder_computeFirstOrderMassMatrix_mex'
//

// Include files
#include "_coder_computeFirstOrderMassMatrix_mex.h"
#include "_coder_computeFirstOrderMassMatrix_api.h"
#include "computeFirstOrderMassMatrix_data.h"
#include "computeFirstOrderMassMatrix_initialize.h"
#include "computeFirstOrderMassMatrix_terminate.h"
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
  mexAtExit(&computeFirstOrderMassMatrix_atexit);
  computeFirstOrderMassMatrix_initialize();
  try {
    unsafe_computeFirstOrderMassMatrix_mexFunction(nlhs, plhs, nrhs, prhs);
    computeFirstOrderMassMatrix_terminate();
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

void unsafe_computeFirstOrderMassMatrix_mexFunction(int32_T nlhs,
                                                    mxArray *plhs[1],
                                                    int32_T nrhs,
                                                    const mxArray *prhs[3])
{
  const mxArray *outputs;
  // Check for proper number of arguments.
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 3, 4, 27, "computeFirstOrderMassMatrix");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 27,
                        "computeFirstOrderMassMatrix");
  }
  // Call the function.
  computeFirstOrderMassMatrix_api(prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

// End of code generation (_coder_computeFirstOrderMassMatrix_mex.cpp)
