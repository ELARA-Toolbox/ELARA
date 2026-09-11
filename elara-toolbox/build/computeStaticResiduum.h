//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeStaticResiduum.h
//
// Code generation for function 'computeStaticResiduum'
//

#pragma once

// Include files
#include "SimulationParameters.h"
#include "SystemNum.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Function Declarations
void computeStaticResiduum(const elara::SystemNum *MBSys,
                           const elara::SimulationParameters *simPars,
                           const coder::array<real_T, 1U> &q,
                           const coder::array<real_T, 1U> &u,
                           coder::array<real_T, 1U> &res);

emlrtCTX emlrtGetRootTLSGlobal();

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

// End of code generation (computeStaticResiduum.h)
