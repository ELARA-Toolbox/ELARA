//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// SimulationParameters.h
//
// Code generation for function 'SimulationParameters'
//

#pragma once

// Include files
#include "ExternalWrench.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Type Definitions
namespace elara {
class SimulationParameters {
public:
  coder::array<real_T, 1U> q0;
  coder::array<real_T, 1U> qDot0;
  real_T tEnd;
  real_T g;
  coder::array<real_T, 1U> uConst;
  coder::array<real_T, 2U> uSampleValues;
  coder::array<real_T, 1U> uSampleTimes;
  ExternalWrench externalWrench_b;
  ExternalWrench externalWrench_s;
};

} // namespace elara

// End of code generation (SimulationParameters.h)
