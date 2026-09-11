//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// SystemNum.h
//
// Code generation for function 'SystemNum'
//

#pragma once

// Include files
#include "FramePropertiesNum.h"
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
class SystemNum {
public:
  boolean_T isCantilever;
  real_T g0[16];
  coder::array<real_T, 2U> LinkAdjacencyMatrix;
  coder::array<real_T, 2U> FrameAdjacencyMatrix;
  real_T nLinks;
  real_T nJoints;
  real_T nFrames;
  real_T nDoF;
  real_T nInputs;
  coder::array<uint16_T, 2U> linkFrameIndices;
  uint16_T indexTCPFrame;
  real_T g_B_TCP[16];
  FramePropertiesNum frames;
  coder::array<real_T, 1U> cSys;
  coder::array<real_T, 1U> dSys;
  coder::array<real_T, 1U> qRef;
};

} // namespace elara

// End of code generation (SystemNum.h)
