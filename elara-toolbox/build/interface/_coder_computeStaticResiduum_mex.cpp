//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_computeStaticResiduum_mex.cpp
//
// Code generation for function '_coder_computeStaticResiduum_mex'
//

// Include files
#include "_coder_computeStaticResiduum_mex.h"
#include "_coder_computeStaticResiduum_api.h"
#include "computeStaticResiduum.h"
#include "computeStaticResiduum_data.h"
#include "computeStaticResiduum_initialize.h"
#include "computeStaticResiduum_terminate.h"
#include "rt_nonfinite.h"
#include "omp.h"
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
  mexAtExit(&computeStaticResiduum_atexit);
  emlrtLoadLibrary("C:\\ProgramData\\MATLAB\\SupportPackages\\R2025b\\3P."
                   "instrset\\mingw_w64.instrset\\bin\\libgomp-1.dll");
  // Initialize the memory manager.
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&computeStaticResiduum_nestLockGlobal);
  computeStaticResiduum_initialize();
  try {
    unsafe_computeStaticResiduum_mexFunction(nlhs, plhs, nrhs, prhs);
    computeStaticResiduum_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&computeStaticResiduum_nestLockGlobal);
  } catch (...) {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(&computeStaticResiduum_nestLockGlobal);
    emlrtReportParallelRunTimeError(emlrtRootTLSGlobal);
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(),
                           (void *)&emlrtExceptionBridge, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_computeStaticResiduum_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                              int32_T nrhs,
                                              const mxArray *prhs[4])
{
  const mxArray *outputs;
  // Check for proper number of arguments.
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 4, 4, 21, "computeStaticResiduum");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 21,
                        "computeStaticResiduum");
  }
  // Call the function.
  computeStaticResiduum_api(prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

// End of code generation (_coder_computeStaticResiduum_mex.cpp)
