//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// SimulationResults.h
//
// Code generation for function 'SimulationResults'
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
class SimulationResults {
public:
  coder::array<real_T, 3U> eta;
  coder::array<real_T, 4U> g;
  coder::array<real_T, 2U> q;
  coder::array<real_T, 2U> q_dot;
  coder::array<real_T, 1U> tout;
};

} // namespace elara

// End of code generation (SimulationResults.h)
