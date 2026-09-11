//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// FramePropertiesNum.h
//
// Code generation for function 'FramePropertiesNum'
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
class FramePropertiesNum {
public:
  coder::array<uint8_T, 1U> nDof;
  coder::array<uint8_T, 1U> jointType;
  coder::array<uint16_T, 2U> qIndices;
  coder::array<uint16_T, 2U> uIndices;
  coder::array<uint16_T, 2U> linkIndex;
  coder::array<uint16_T, 2U> parent;
  coder::array<uint16_T, 2U> ancestors;
  coder::array<real_T, 2U> X;
  coder::array<real_T, 3U> g_ref;
  coder::array<real_T, 3U> BaPadded;
  coder::array<real_T, 2U> xiC;
  coder::array<real_T, 1U> l;
  coder::array<real_T, 3U> MGen;
  coder::array<real_T, 1U> m;
  coder::array<real_T, 3U> g_a;
  coder::array<real_T, 2U> x_a;
  coder::array<real_T, 2U> m_a;
  coder::array<real_T, 5U> g_cm;
};

} // namespace elara

// End of code generation (FramePropertiesNum.h)
