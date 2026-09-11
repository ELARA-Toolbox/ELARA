//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// getSimResFromStateTrajectory.h
//
// Code generation for function 'getSimResFromStateTrajectory'
//

#pragma once

// Include files
#include "SimulationResults.h"
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
void getSimResFromStateTrajectory(const elara::SystemNum *MBSys,
                                  const coder::array<real_T, 1U> &tout,
                                  const coder::array<real_T, 2U> &q,
                                  const coder::array<real_T, 2U> &q_dot,
                                  elara::SimulationResults *simRes);

// End of code generation (getSimResFromStateTrajectory.h)
