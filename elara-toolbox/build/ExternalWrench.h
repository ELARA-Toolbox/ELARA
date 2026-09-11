//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ExternalWrench.h
//
// Code generation for function 'ExternalWrench'
//

#pragma once

// Include files
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
class ExternalWrench {
public:
  coder::array<real_T, 1U> startTime;
  coder::array<real_T, 1U> endTime;
  coder::array<real_T, 1U> interpolationType;
  coder::array<real_T, 3U> maximumWrench;
};

} // namespace elara

// End of code generation (ExternalWrench.h)
