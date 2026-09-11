//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeStaticResiduum.cpp
//
// Code generation for function 'computeStaticResiduum'
//

// Include files
#include "computeStaticResiduum.h"
#include "FramePropertiesNum.h"
#include "SimulationParameters.h"
#include "SystemNum.h"
#include "computeStaticResiduum_data.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "coder_array.h"
#include "mwmathutil.h"
#include "omp.h"
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <emmintrin.h>

// Variable Definitions
static emlrtMCInfo emlrtMCI{
    17,                      // lineNo
    5,                       // colNo
    "computeStaticResiduum", // fName
    "C:\\Users\\Pfeiffer\\Desktop\\elara\\ELARA\\elara-toolbox\\elara\\system-"
    "equations\\computeStaticResiduum.m" // pName
};

static emlrtMCInfo b_emlrtMCI{
    19,                      // lineNo
    5,                       // colNo
    "computeStaticResiduum", // fName
    "C:\\Users\\Pfeiffer\\Desktop\\elara\\ELARA\\elara-toolbox\\elara\\system-"
    "equations\\computeStaticResiduum.m" // pName
};

static emlrtMCInfo c_emlrtMCI{
    27,      // lineNo
    5,       // colNo
    "error", // fName
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\lang\\error.m" // pName
};

// Function Declarations
static void b_error(const mxArray *m, emlrtMCInfo &location);

static void binary_expand_op(coder::array<real_T, 2U> &in1,
                             const elara::SystemNum *in2);

static void binary_expand_op_1(emlrtCTX aTLS, coder::array<real_T, 1U> &in1,
                               const elara::SystemNum *in2,
                               const coder::array<real_T, 1U> &in3);

static void plus(emlrtCTX aTLS, coder::array<real_T, 1U> &in1,
                 const coder::array<real_T, 1U> &in2);

// Function Definitions
static void b_error(const mxArray *m, emlrtMCInfo &location)
{
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, nullptr, 1, &m, "error", true,
                        &location);
}

static void binary_expand_op(coder::array<real_T, 2U> &in1,
                             const elara::SystemNum *in2)
{
  coder::array<real_T, 2U> r;
  int32_T aux_1_1;
  int32_T loop_ub;
  int32_T stride_1_1;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  if (in1.size(1) == 1) {
    loop_ub = static_cast<int32_T>(in2->nFrames);
  } else {
    loop_ub = in1.size(1);
  }
  r.set_size(6, loop_ub);
  stride_1_1 = (in1.size(1) != 1);
  aux_1_1 = 0;
  for (int32_T i{0}; i < loop_ub; i++) {
    for (int32_T i1{0}; i1 < 6; i1++) {
      r[i1 + 6 * i] = in1[i1 + 6 * aux_1_1];
    }
    aux_1_1 += stride_1_1;
  }
  in1.set_size(6, loop_ub);
  for (int32_T i{0}; i < loop_ub; i++) {
    for (int32_T i1{0}; i1 < 6; i1++) {
      stride_1_1 = i1 + 6 * i;
      in1[stride_1_1] = r[stride_1_1];
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

static void binary_expand_op_1(emlrtCTX aTLS, coder::array<real_T, 1U> &in1,
                               const elara::SystemNum *in2,
                               const coder::array<real_T, 1U> &in3)
{
  coder::array<real_T, 1U> b_in2;
  int32_T binary_expand_op_1_numThreads;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  int32_T stride_2_0;
  int32_T stride_3_0;
  emlrtHeapReferenceStackEnterFcnR2012b(aTLS);
  if (in1.size(0) == 1) {
    if (in2->qRef.size(0) == 1) {
      stride_0_0 = in3.size(0);
    } else {
      stride_0_0 = in2->qRef.size(0);
    }
    if (stride_0_0 == 1) {
      loop_ub = in2->cSys.size(0);
    } else if (in2->qRef.size(0) == 1) {
      loop_ub = in3.size(0);
    } else {
      loop_ub = in2->qRef.size(0);
    }
  } else {
    loop_ub = in1.size(0);
  }
  b_in2.set_size(loop_ub);
  stride_0_0 = (in2->cSys.size(0) != 1);
  stride_1_0 = (in3.size(0) != 1);
  stride_2_0 = (in2->qRef.size(0) != 1);
  stride_3_0 = (in1.size(0) != 1);
  if (static_cast<int32_T>(loop_ub < 1600)) {
    for (int32_T i{0}; i < loop_ub; i++) {
      b_in2[i] = in2->cSys[i * stride_0_0] *
                     (in3[i * stride_1_0] - in2->qRef[i * stride_2_0]) -
                 in1[i * stride_3_0];
    }
  } else {
    emlrtEnterParallelRegion(aTLS, static_cast<boolean_T>(omp_in_parallel()));
    binary_expand_op_1_numThreads =
        emlrtAllocRegionTLSs(aTLS, static_cast<boolean_T>(omp_in_parallel()),
                             omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(binary_expand_op_1_numThreads)

    for (int32_T i = 0; i < loop_ub; i++) {
      b_in2[i] = in2->cSys[i * stride_0_0] *
                     (in3[i * stride_1_0] - in2->qRef[i * stride_2_0]) -
                 in1[i * stride_3_0];
    }
    emlrtExitParallelRegion(aTLS, static_cast<boolean_T>(omp_in_parallel()));
  }
  in1.set_size(loop_ub);
  for (int32_T i1{0}; i1 < loop_ub; i1++) {
    in1[i1] = b_in2[i1];
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(aTLS);
}

static void plus(emlrtCTX aTLS, coder::array<real_T, 1U> &in1,
                 const coder::array<real_T, 1U> &in2)
{
  coder::array<real_T, 1U> b_in1;
  int32_T loop_ub;
  int32_T plus_numThreads;
  int32_T stride_0_0;
  int32_T stride_1_0;
  emlrtHeapReferenceStackEnterFcnR2012b(aTLS);
  if (in2.size(0) == 1) {
    loop_ub = in1.size(0);
  } else {
    loop_ub = in2.size(0);
  }
  b_in1.set_size(loop_ub);
  stride_0_0 = (in1.size(0) != 1);
  stride_1_0 = (in2.size(0) != 1);
  if (static_cast<int32_T>(loop_ub < 1600)) {
    for (int32_T i{0}; i < loop_ub; i++) {
      b_in1[i] = in1[i * stride_0_0] + in2[i * stride_1_0];
    }
  } else {
    emlrtEnterParallelRegion(aTLS, static_cast<boolean_T>(omp_in_parallel()));
    plus_numThreads =
        emlrtAllocRegionTLSs(aTLS, static_cast<boolean_T>(omp_in_parallel()),
                             omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel for num_threads(plus_numThreads)

    for (int32_T i = 0; i < loop_ub; i++) {
      b_in1[i] = in1[i * stride_0_0] + in2[i * stride_1_0];
    }
    emlrtExitParallelRegion(aTLS, static_cast<boolean_T>(omp_in_parallel()));
  }
  in1.set_size(loop_ub);
  for (int32_T i1{0}; i1 < loop_ub; i1++) {
    in1[i1] = b_in1[i1];
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(aTLS);
}

void computeStaticResiduum(const elara::SystemNum *MBSys,
                           const elara::SimulationParameters *simPars,
                           const coder::array<real_T, 1U> &q,
                           const coder::array<real_T, 1U> &u,
                           coder::array<real_T, 1U> &res)
{
  static const int32_T iv[2]{1, 55};
  static const int32_T iv1[2]{1, 38};
  static const int32_T iv2[2]{1, 29};
  static const char_T b_u[55]{
      'V', 'e', 'c', 't', 'o', 'r', ' ', 'o', 'f', ' ', 'g', 'e', 'n', 'e',
      'r', 'a', 'l', 'i', 'z', 'e', 'd', ' ', 'c', 'o', 'o', 'r', 'd', 'i',
      'n', 'a', 't', 'e', 's', ' ', 'h', 'a', 's', ' ', 'w', 'r', 'o', 'n',
      'g', ' ', 'd', 'i', 'm', 'e', 'n', 's', 'i', 'o', 'n', 's', '.'};
  static const char_T c_u[38]{'V', 'e', 'c', 't', 'o', 'r', ' ', 'o', 'f', ' ',
                              'i', 'n', 'p', 'u', 't', 's', ' ', 'h', 'a', 's',
                              ' ', 'w', 'r', 'o', 'n', 'g', ' ', 'd', 'i', 'm',
                              'e', 'n', 's', 'i', 'o', 'n', 's', '.'};
  static const char_T d_u[29]{'I', 'n', 'v', 'a', 'l', 'i', 'd', ' ', 'j', 'o',
                              'i', 'n', 't', ' ', 't', 'y', 'p', 'e', ' ', 's',
                              'p', 'e', 'c', 'i', 'f', 'i', 'e', 'd', '.'};
  static const int8_T iv3[4]{0, 0, 0, 1};
  static const int8_T d_b[3]{0, 0, 1};
  __m128d r;
  __m128d r1;
  __m128d r2;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  coder::array<real_T, 3U> J;
  coder::array<real_T, 3U> g;
  coder::array<real_T, 3U> g_rel;
  coder::array<real_T, 2U> B;
  coder::array<real_T, 2U> c_b;
  coder::array<real_T, 2U> f_frame_b;
  coder::array<real_T, 2U> r3;
  coder::array<real_T, 2U> r5;
  coder::array<real_T, 2U> r9;
  coder::array<real_T, 1U> g_y;
  coder::array<real_T, 1U> qi;
  coder::array<uint16_T, 2U> qIndices;
  coder::array<uint16_T, 2U> uIndices;
  coder::array<uint16_T, 1U> r4;
  coder::array<uint16_T, 1U> r6;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *m;
  const mxArray *m1;
  const mxArray *m2;
  const mxArray *propValues;
  const mxArray *y;
  real_T BaMat_data[36];
  real_T h_y[36];
  real_T tmp_data[36];
  real_T A[16];
  real_T R[16];
  real_T R_tmp[9];
  real_T b_R[9];
  real_T b_R_tmp[9];
  real_T omh[9];
  real_T e_b[6];
  real_T xi_c[6];
  real_T b_MBSys[3];
  real_T c_R_tmp[3];
  real_T alpha1;
  real_T beta1;
  real_T t10;
  real_T t12;
  real_T t13;
  real_T t14;
  real_T t15;
  real_T t16;
  real_T t18;
  real_T t19;
  real_T t2;
  real_T t3;
  real_T t4;
  real_T t5;
  real_T t6;
  real_T t7;
  real_T t8;
  int32_T b_b;
  int32_T b_loop_ub;
  int32_T b_n;
  int32_T c_loop_ub;
  int32_T i;
  int32_T i2{0};
  int32_T jj;
  int32_T jp1j;
  int32_T loop_ub;
  int32_T mmj;
  int32_T n;
  uint16_T a;
  uint16_T b;
  const char_T *b_propClasses{"coder.internal.string"};
  const char_T *b_propNames{"Value"};
  const char_T *c_propClasses{"coder.internal.string"};
  const char_T *c_propNames{"Value"};
  const char_T *propClasses{"coder.internal.string"};
  const char_T *propNames{"Value"};
  char_T TRANSA1;
  char_T TRANSB1;
  int8_T i3;
  boolean_T tf;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  //     %% Compute Residuum of Static Equilbrium Equation
  //  Multibody system
  //  Vector of generalized coordinates (1,nDoF)
  //  Vector of inputs (1,nInputs)
  //  Check argument sizes
  if (!(q.size(0) == MBSys->nDoF)) {
    y = nullptr;
    m = nullptr;
    emlrtAssign(&y, emlrtCreateClassInstance2022a(emlrtRootTLSGlobal,
                                                  "coder.internal.string"));
    m = nullptr;
    c_y = nullptr;
    propValues = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 55, propValues, &b_u[0]);
    emlrtAssign(&c_y, propValues);
    emlrtAssign(&m, c_y);
    propValues = m;
    emlrtSetAllProperties(emlrtRootTLSGlobal, &y, 0, 1,
                          (const char_T **)&propNames,
                          (const char_T **)&propClasses, &propValues);
    emlrtAssign(&y, emlrtConvertInstanceToRedirectSource(
                        emlrtRootTLSGlobal, y, 0, "coder.internal.string"));
    b_error(y, emlrtMCI);
  }
  if (!(u.size(0) == MBSys->nInputs)) {
    b_y = nullptr;
    m1 = nullptr;
    emlrtAssign(&b_y, emlrtCreateClassInstance2022a(emlrtRootTLSGlobal,
                                                    "coder.internal.string"));
    m1 = nullptr;
    d_y = nullptr;
    propValues = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 38, propValues, &c_u[0]);
    emlrtAssign(&d_y, propValues);
    emlrtAssign(&m1, d_y);
    propValues = m1;
    emlrtSetAllProperties(emlrtRootTLSGlobal, &b_y, 0, 1,
                          (const char_T **)&b_propNames,
                          (const char_T **)&b_propClasses, &propValues);
    emlrtAssign(&b_y, emlrtConvertInstanceToRedirectSource(
                          emlrtRootTLSGlobal, b_y, 0, "coder.internal.string"));
    b_error(b_y, b_emlrtMCI);
  }
  //  Forward Kinematics and Jacobians
  //             %% Compute Kinematics for Full Multibody System
  //  i.e., the configuration of all body frames (= CoM frames / node
  //  frames)
  //  System coordinates (nDoF, 1)
  //  Absolute configurations of all body frames
  //  Relative configurations between body frames (joint
  //  transformations)
  //  Compute relative joint transformations
  //             %% Compute the relative transformations of all joints (rigid
  //             and flexible)
  //  for given relative coordinates
  //  System coordinates  (nDoF, 1)
  //  SE3 Matrices with relative configurations between body frames
  i = static_cast<int32_T>(MBSys->nFrames);
  g_rel.set_size(4, 4, static_cast<int32_T>(MBSys->nFrames));
  jj = static_cast<int32_T>(MBSys->nFrames) << 4;
  if (jj - 1 >= 0) {
    std::memset(&g_rel[0], 0, static_cast<uint32_T>(jj) * sizeof(real_T));
  }
  for (int32_T iFrm{0}; iFrm < i; iFrm++) {
    a = MBSys->frames.qIndices[2 * iFrm];
    b = MBSys->frames.qIndices[2 * iFrm + 1];
    if (a > b) {
      jp1j = 0;
      n = 0;
    } else {
      jp1j = a - 1;
      n = b;
    }
    mmj = n - jp1j;
    qi.set_size(mmj);
    for (int32_T b_i{0}; b_i < mmj; b_i++) {
      qi[b_i] = q[jp1j + b_i];
    }
    switch (MBSys->frames.jointType[iFrm]) {
    case 1U: {
      // %% Screw joint
      //     %% Exponential map for SE(3) with constant screw axis
      //  Implements the Exponential map for SE(3): exp : se(3) -> SE(3)
      //  with a constant screw axis (twist) X and magnetude ("angle") theta.
      //
      //  We use the standard formulas for the SE3 exponential mapping, e.g.,
      //  from [MLS94].
      //  Maximilian Herrmann
      //  Chair of Automatic Control
      //  TUM School of Engineering and Design
      //  Technical University of Munich
      //  Screw axis / twist
      //  Magnetude / angle
      if (mmj != 0) {
        t8 = q[jp1j];
      }
      //  SKEWSO3 Hat map for so(3) / 3 dimensions R3 -> so(3), i.e. 3x3 matrix
      //  Implementation by:
      //  Maximilian Herrmann
      //  Chair of Automatic Control
      //  TUM School of Engineering and Design
      //  Technical University of Munich
      omh[0] = 0.0;
      mmj = 6 * iFrm + 2;
      alpha1 = MBSys->frames.X[mmj];
      omh[3] = -alpha1;
      b_b = 6 * iFrm + 1;
      t15 = MBSys->frames.X[b_b];
      omh[6] = t15;
      omh[1] = alpha1;
      omh[4] = 0.0;
      alpha1 = MBSys->frames.X[6 * iFrm];
      omh[7] = -alpha1;
      omh[2] = -t15;
      omh[5] = alpha1;
      omh[8] = 0.0;
      //  Formula (2.14), p. 28 in [MLS94]
      std::memset(&R_tmp[0], 0, 9U * sizeof(real_T));
      beta1 = muDoubleScalarSin(t8);
      t14 = 1.0 - muDoubleScalarCos(t8);
      std::memset(&b_R[0], 0, 9U * sizeof(real_T));
      for (int32_T b_i{0}; b_i < 3; b_i++) {
        R_tmp[b_i + 3 * b_i] = 1.0;
        t19 = omh[3 * b_i];
        r = _mm_loadu_pd(&omh[0]);
        r1 = _mm_loadu_pd(&b_R[3 * b_i]);
        _mm_storeu_pd(&b_R[3 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(t19))));
        jp1j = 3 * b_i + 2;
        b_R[jp1j] += -t15 * t19;
        t19 = omh[3 * b_i + 1];
        r = _mm_loadu_pd(&omh[3]);
        r1 = _mm_loadu_pd(&b_R[3 * b_i]);
        _mm_storeu_pd(&b_R[3 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(t19))));
        b_R[jp1j] += alpha1 * t19;
        t19 = omh[jp1j];
        r = _mm_loadu_pd(&omh[6]);
        r1 = _mm_loadu_pd(&b_R[3 * b_i]);
        _mm_storeu_pd(&b_R[3 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(t19))));
        b_R[jp1j] += 0.0 * t19;
      }
      __m128d r7;
      __m128d r8;
      //  Formula (2.36), p. 42 in [MLS94]
      r = _mm_loadu_pd(&omh[0]);
      r2 = _mm_loadu_pd(&R_tmp[0]);
      r1 = _mm_loadu_pd(&b_R[0]);
      r7 = _mm_set1_pd(beta1);
      r8 = _mm_set1_pd(t14);
      r = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r, r7)), _mm_mul_pd(r1, r8));
      _mm_storeu_pd(&b_R[0], r);
      _mm_storeu_pd(&R_tmp[0], _mm_sub_pd(r2, r));
      r = _mm_loadu_pd(&omh[2]);
      r2 = _mm_loadu_pd(&R_tmp[2]);
      r1 = _mm_loadu_pd(&b_R[2]);
      r = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r, r7)), _mm_mul_pd(r1, r8));
      _mm_storeu_pd(&b_R[2], r);
      _mm_storeu_pd(&R_tmp[2], _mm_sub_pd(r2, r));
      r = _mm_loadu_pd(&omh[4]);
      r2 = _mm_loadu_pd(&R_tmp[4]);
      r1 = _mm_loadu_pd(&b_R[4]);
      r = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r, r7)), _mm_mul_pd(r1, r8));
      _mm_storeu_pd(&b_R[4], r);
      _mm_storeu_pd(&R_tmp[4], _mm_sub_pd(r2, r));
      r = _mm_loadu_pd(&omh[6]);
      r2 = _mm_loadu_pd(&R_tmp[6]);
      r1 = _mm_loadu_pd(&b_R[6]);
      r = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r, r7)), _mm_mul_pd(r1, r8));
      _mm_storeu_pd(&b_R[6], r);
      _mm_storeu_pd(&R_tmp[6], _mm_sub_pd(r2, r));
      alpha1 = (R_tmp[8] + 0.0 * beta1) + b_R[8] * t14;
      b_R[8] = alpha1;
      R_tmp[8] -= alpha1;
      std::memset(&b_R_tmp[0], 0, 9U * sizeof(real_T));
      std::memset(&b_MBSys[0], 0, 3U * sizeof(real_T));
      alpha1 = b_MBSys[0];
      beta1 = b_MBSys[1];
      t19 = b_MBSys[2];
      for (int32_T b_i{0}; b_i < 3; b_i++) {
        n = b_i + 6 * iFrm;
        t14 = MBSys->frames.X[n + 3];
        t15 = omh[3 * b_i];
        r = _mm_loadu_pd(&R_tmp[0]);
        r1 = _mm_loadu_pd(&b_R_tmp[3 * b_i]);
        _mm_storeu_pd(&b_R_tmp[3 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(t15))));
        b_n = 3 * b_i + 2;
        b_R_tmp[b_n] += R_tmp[2] * t15;
        t15 = MBSys->frames.X[6 * iFrm] * MBSys->frames.X[n];
        omh[3 * b_i] = t15;
        alpha1 += t15 * t14;
        jp1j = 3 * b_i + 1;
        t15 = omh[jp1j];
        r = _mm_loadu_pd(&R_tmp[3]);
        r1 = _mm_loadu_pd(&b_R_tmp[3 * b_i]);
        _mm_storeu_pd(&b_R_tmp[3 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(t15))));
        b_R_tmp[b_n] += R_tmp[5] * t15;
        t15 = MBSys->frames.X[b_b] * MBSys->frames.X[n];
        omh[jp1j] = t15;
        beta1 += t15 * t14;
        t15 = omh[b_n];
        r = _mm_loadu_pd(&R_tmp[6]);
        r1 = _mm_loadu_pd(&b_R_tmp[3 * b_i]);
        _mm_storeu_pd(&b_R_tmp[3 * b_i],
                      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(t15))));
        b_R_tmp[b_n] += R_tmp[8] * t15;
        t15 = MBSys->frames.X[mmj] * MBSys->frames.X[n];
        omh[b_n] = t15;
        t19 += t15 * t14;
      }
      b_MBSys[2] = t19;
      b_MBSys[1] = beta1;
      b_MBSys[0] = alpha1;
      for (int32_T b_i{0}; b_i < 3; b_i++) {
        n = b_i << 2;
        R[n] = b_R[3 * b_i];
        R[n + 1] = b_R[3 * b_i + 1];
        R[n + 2] = b_R[3 * b_i + 2];
        R[b_i + 12] = ((b_R_tmp[b_i] * MBSys->frames.X[6 * iFrm + 3] +
                        b_R_tmp[b_i + 3] * MBSys->frames.X[6 * iFrm + 4]) +
                       b_R_tmp[b_i + 6] * MBSys->frames.X[6 * iFrm + 5]) +
                      b_MBSys[b_i] * t8;
      }
      for (int32_T b_i{0}; b_i < 4; b_i++) {
        n = b_i << 2;
        R[n + 3] = iv3[b_i];
        b_n = 4 * b_i + 16 * iFrm;
        g_rel[b_n] = 0.0;
        g_rel[b_n + 1] = 0.0;
        g_rel[b_n + 2] = 0.0;
        g_rel[b_n + 3] = 0.0;
        for (int32_T j{0}; j < 4; j++) {
          r = _mm_loadu_pd(&g_rel[b_n]);
          r1 = _mm_set1_pd(R[j + n]);
          jp1j = 4 * j + 16 * iFrm;
          _mm_storeu_pd(
              &g_rel[b_n],
              _mm_add_pd(
                  r, _mm_mul_pd(_mm_loadu_pd(&MBSys->frames.g_ref[jp1j]), r1)));
          r = _mm_loadu_pd(&g_rel[b_n + 2]);
          _mm_storeu_pd(
              &g_rel[b_n + 2],
              _mm_add_pd(
                  r, _mm_mul_pd(_mm_loadu_pd(&MBSys->frames.g_ref[jp1j + 2]),
                                r1)));
        }
      }
    } break;
    case 2U:
      // %% Flexible joint
      //             %% Get frame's Ba matrix
      //  Helper function to get the Ba matrix with correct dimensions
      //  from the stored array padded with zeros
      //  index of the frame to get
      n = MBSys->frames.nDof[iFrm];
      for (int32_T b_i{0}; b_i < n; b_i++) {
        for (int32_T j{0}; j < 6; j++) {
          b_n = j + 6 * b_i;
          BaMat_data[b_n] = MBSys->frames.BaPadded[b_n + 36 * iFrm];
        }
      }
      if ((MBSys->frames.nDof[iFrm] == 0) || (mmj == 0)) {
        for (int32_T b_i{0}; b_i < 6; b_i++) {
          xi_c[b_i] = 0.0;
        }
      } else {
        TRANSB1 = 'N';
        TRANSA1 = 'N';
        alpha1 = 1.0;
        beta1 = 0.0;
        m_t = (ptrdiff_t)6;
        n_t = (ptrdiff_t)1;
        k_t = (ptrdiff_t) static_cast<int32_T>(MBSys->frames.nDof[iFrm]);
        lda_t = (ptrdiff_t)6;
        ldb_t = (ptrdiff_t)mmj;
        ldc_t = (ptrdiff_t)6;
        dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &BaMat_data[0],
              &lda_t, &qi[0], &ldb_t, &beta1, &xi_c[0], &ldc_t);
      }
      r = _mm_loadu_pd(&xi_c[0]);
      r1 = _mm_set1_pd(MBSys->frames.l[iFrm]);
      _mm_storeu_pd(
          &xi_c[0],
          _mm_mul_pd(_mm_add_pd(r, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm])),
                     r1));
      r = _mm_loadu_pd(&xi_c[2]);
      _mm_storeu_pd(
          &xi_c[2],
          _mm_mul_pd(
              _mm_add_pd(r, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 2])),
              r1));
      r = _mm_loadu_pd(&xi_c[4]);
      _mm_storeu_pd(
          &xi_c[4],
          _mm_mul_pd(
              _mm_add_pd(r, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 4])),
              r1));
      //     %% Cayley map for SE(3)
      //  Implements the Cayley map for SE(3): cay : se(3) -> SE(3)
      //  Source: [Dem+14, p.10], eq. 19
      //  Follows convention for se3 elements in vector form: [omega; v]
      //  Input xi: se(3) element in *vector* form
      //  Output g: Corresponding element of SE3 in matrix form
      //  Maximilian Herrmann
      //  Chair of Automatic Control
      //  TUM School of Engineering and Design
      //  Technical University of Munich
      //     %% Original implementation using "analytic" functions
      //     %{
      //     om = xi(1:3);
      //     v  = xi(4:6);
      //     omH = skew(om);
      //
      //     R = caySO3( om );
      //     x = ( 4 / (4 + om.'*om) ) * ( eye(3) + 1/2 * omH + 1/4 * (om*om.')
      //     ) * v; g = SE3Matrix(R, x);
      //     %}
      //     %% More efficient code generated from MATLAB
      //  (using the above initial implementation)
      //  Code used:
      //  xi = sym('xi', [6,1]);
      //  C = caySE3( xi );
      //  matlabFunction(C, "Vars", {xi}, "File","tempFun", "Optimize",true);
      t2 = xi_c[0] * xi_c[0];
      t3 = xi_c[1] * xi_c[1];
      t4 = xi_c[2] * xi_c[2];
      t5 = xi_c[0] / 2.0;
      t6 = xi_c[1] / 2.0;
      t7 = xi_c[2] / 2.0;
      alpha1 = xi_c[0] * xi_c[1];
      t10 = alpha1 / 4.0;
      beta1 = xi_c[0] * xi_c[2];
      t12 = beta1 / 4.0;
      t19 = xi_c[1] * xi_c[2];
      t13 = t19 / 4.0;
      t14 = t2 / 2.0;
      t15 = t3 / 2.0;
      t16 = t4 / 2.0;
      t18 = 1.0 / (((t2 + t3) + t4) + 4.0);
      g_rel[16 * iFrm] = t18 * (t15 + t16) * -4.0 + 1.0;
      g_rel[16 * iFrm + 1] = t18 * (t5 * xi_c[1] + xi_c[2]) * 4.0;
      g_rel[16 * iFrm + 2] = t18 * (xi_c[1] - beta1 / 2.0) * -4.0;
      g_rel[16 * iFrm + 3] = 0.0;
      g_rel[16 * iFrm + 4] = t18 * (xi_c[2] - alpha1 / 2.0) * -4.0;
      g_rel[16 * iFrm + 5] = t18 * (t14 + t16) * -4.0 + 1.0;
      g_rel[16 * iFrm + 6] = t18 * (t6 * xi_c[2] + xi_c[0]) * 4.0;
      g_rel[16 * iFrm + 7] = 0.0;
      g_rel[16 * iFrm + 8] = t18 * (t5 * xi_c[2] + xi_c[1]) * 4.0;
      g_rel[16 * iFrm + 9] = t18 * (xi_c[0] - t19 / 2.0) * -4.0;
      g_rel[16 * iFrm + 10] = t18 * (t14 + t15) * -4.0 + 1.0;
      g_rel[16 * iFrm + 11] = 0.0;
      beta1 = t18 * xi_c[4];
      t19 = t18 * xi_c[3];
      alpha1 = t18 * xi_c[5];
      g_rel[16 * iFrm + 12] =
          (t19 * (t2 / 4.0 + 1.0) * 4.0 + alpha1 * (t6 + t12) * 4.0) -
          beta1 * (t7 - t10) * 4.0;
      g_rel[16 * iFrm + 13] =
          (beta1 * (t3 / 4.0 + 1.0) * 4.0 + t19 * (t7 + t10) * 4.0) -
          alpha1 * (t5 - t13) * 4.0;
      g_rel[16 * iFrm + 14] =
          (alpha1 * (t4 / 4.0 + 1.0) * 4.0 + beta1 * (t5 + t13) * 4.0) -
          t19 * (t6 - t12) * 4.0;
      g_rel[16 * iFrm + 15] = 1.0;
      break;
    default:
      e_y = nullptr;
      m2 = nullptr;
      emlrtAssign(&e_y, emlrtCreateClassInstance2022a(emlrtRootTLSGlobal,
                                                      "coder.internal.string"));
      m2 = nullptr;
      f_y = nullptr;
      propValues = emlrtCreateCharArray(2, &iv2[0]);
      emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 29, propValues, &d_u[0]);
      emlrtAssign(&f_y, propValues);
      emlrtAssign(&m2, f_y);
      propValues = m2;
      emlrtSetAllProperties(emlrtRootTLSGlobal, &e_y, 0, 1,
                            (const char_T **)&c_propNames,
                            (const char_T **)&c_propClasses, &propValues);
      emlrtAssign(
          &e_y, emlrtConvertInstanceToRedirectSource(emlrtRootTLSGlobal, e_y, 0,
                                                     "coder.internal.string"));
      b_error(e_y, c_emlrtMCI);
      break;
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  //  Compute kinematics
  //             %% Compute Kinematics for Full Multibody System
  //  i.e., the configuration of all body frames (= CoM frames / node
  //  frames)
  //  with *given* relative joint transformations g_ij
  //  Array of relative configurations between body frames
  //  dimensions (4,4,nFrames)
  //  Absolute configurations of all body frames
  //             %% Compute kinematics
  //  Kinematics without Joint Frames, Section 2.3 (CoM Frames = Body Frames
  //  only)
  g.set_size(4, 4, static_cast<int32_T>(MBSys->nFrames));
  if (jj - 1 >= 0) {
    std::memset(&g[0], 0, static_cast<uint32_T>(jj) * sizeof(real_T));
  }
  //  First frame
  for (int32_T b_i{0}; b_i < 4; b_i++) {
    g[4 * b_i] = 0.0;
    g[4 * b_i + 1] = 0.0;
    jp1j = 4 * b_i + 2;
    g[jp1j] = 0.0;
    g[4 * b_i + 3] = 0.0;
    for (int32_T j{0}; j < 4; j++) {
      r = _mm_loadu_pd(&g[4 * b_i]);
      b_n = j << 2;
      r1 = _mm_set1_pd(g_rel[j + 4 * b_i]);
      _mm_storeu_pd(
          &g[4 * b_i],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&MBSys->g0[b_n]), r1)));
      r = _mm_loadu_pd(&g[jp1j]);
      _mm_storeu_pd(
          &g[jp1j],
          _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&MBSys->g0[b_n + 2]), r1)));
    }
  }
  //  Other frames
  mmj = static_cast<int32_T>(MBSys->nFrames - 1.0);
  for (int32_T iFrm{0}; iFrm < mmj; iFrm++) {
    n = MBSys->frames.parent[iFrm + 1];
    std::memset(&R[0], 0, sizeof(real_T) << 4);
    for (int32_T b_i{0}; b_i < 4; b_i++) {
      b_n = b_i << 2;
      for (int32_T j{0}; j < 4; j++) {
        jp1j = 4 * j + 16 * (n - 1);
        r = _mm_loadu_pd(&g[jp1j]);
        r1 = _mm_loadu_pd(&R[b_n]);
        r2 = _mm_set1_pd(g_rel[(j + 4 * b_i) + 16 * (iFrm + 1)]);
        _mm_storeu_pd(&R[b_n], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&g[jp1j + 2]);
        r1 = _mm_loadu_pd(&R[b_n + 2]);
        _mm_storeu_pd(&R[b_n + 2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      }
    }
    for (int32_T b_i{0}; b_i < 4; b_i++) {
      n = b_i << 2;
      jp1j = 4 * b_i + 16 * (iFrm + 1);
      g[jp1j] = R[n];
      g[jp1j + 1] = R[n + 1];
      g[jp1j + 2] = R[n + 2];
      g[jp1j + 3] = R[n + 3];
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  //             %% Compute Geometric Jacobian Matrix for Full Multibody System
  //  with *given* relative joint transformations g_ij
  //  System coordinates (nDoF, 1)
  //  Array of relative configurations between body frames
  //  dimensions (4,4,nFrames)
  //  Jacobian matrices with dimensions
  //  6 x (nAllwd_1*nSegments1 + ... + nAllwdB*nSegmentsB + nLinks) x nFrames
  //  where B is the nr. of flexible beams in the system
  //  Array holding all Jacobians
  loop_ub = static_cast<int32_T>(MBSys->nDoF);
  J.set_size(6, static_cast<int32_T>(MBSys->nDoF),
             static_cast<int32_T>(MBSys->nFrames));
  n = 6 * static_cast<int32_T>(MBSys->nDoF) *
      static_cast<int32_T>(MBSys->nFrames);
  if (n - 1 >= 0) {
    std::memset(&J[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
  }
  for (int32_T iFrm{0}; iFrm < i; iFrm++) {
    for (int32_T ii{0}; ii <= iFrm; ii++) {
      //  Column indices of the current block
      //             %% Get all indices in the configuration vector for the
      //             current frame
      //  Helper function to get all indices from the stored first and
      //  last index
      a = MBSys->frames.qIndices[2 * ii];
      b = MBSys->frames.qIndices[2 * ii + 1];
      if (b < a) {
        n = -1;
      } else {
        n = static_cast<uint16_T>(b - a);
      }
      qIndices.set_size(1, n + 1);
      for (int32_T b_i{0}; b_i <= n; b_i++) {
        qIndices[b_i] = static_cast<uint16_T>(a + static_cast<uint32_T>(b_i));
      }
      //  Compute block columns for current frame
      if (ii == iFrm) {
        switch (MBSys->frames.jointType[iFrm]) {
        case 1U:
          n = qIndices.size(1);
          r4.set_size(qIndices.size(1));
          for (int32_T b_i{0}; b_i < n; b_i++) {
            r4[b_i] = static_cast<uint16_T>(qIndices[b_i] - 1);
          }
          for (int32_T b_i{0}; b_i < 6; b_i++) {
            xi_c[b_i] = MBSys->frames.X[b_i + 6 * iFrm];
          }
          for (int32_T b_i{0}; b_i < n; b_i++) {
            for (int32_T j{0}; j < 6; j++) {
              J[(j + 6 * r4[b_i]) + 6 * J.size(1) * iFrm] = xi_c[j + 6 * b_i];
            }
          }
          break;
        case 2U: {
          mmj = qIndices.size(1);
          r3.set_size(1, qIndices.size(1));
          for (int32_T b_i{0}; b_i < mmj; b_i++) {
            r3[b_i] = q[qIndices[b_i] - 1];
          }
          //             %% Get frame's Ba matrix
          //  Helper function to get the Ba matrix with correct dimensions
          //  from the stored array padded with zeros
          //  index of the frame to get
          b_n = MBSys->frames.nDof[iFrm];
          for (int32_T b_i{0}; b_i < b_n; b_i++) {
            for (int32_T j{0}; j < 6; j++) {
              n = j + 6 * b_i;
              BaMat_data[n] = MBSys->frames.BaPadded[n + 36 * iFrm];
            }
          }
          if ((MBSys->frames.nDof[iFrm] == 0) || (qIndices.size(1) == 0)) {
            for (int32_T b_i{0}; b_i < 6; b_i++) {
              xi_c[b_i] = 0.0;
            }
          } else {
            TRANSB1 = 'T';
            TRANSA1 = 'N';
            alpha1 = 1.0;
            beta1 = 0.0;
            m_t = (ptrdiff_t)6;
            n_t = (ptrdiff_t)1;
            k_t = (ptrdiff_t) static_cast<int32_T>(MBSys->frames.nDof[iFrm]);
            lda_t = (ptrdiff_t)6;
            ldb_t = (ptrdiff_t)1;
            ldc_t = (ptrdiff_t)6;
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &BaMat_data[0],
                  &lda_t, &r3[0], &ldb_t, &beta1, &xi_c[0], &ldc_t);
          }
          r4.set_size(qIndices.size(1));
          for (int32_T b_i{0}; b_i < mmj; b_i++) {
            r4[b_i] = static_cast<uint16_T>(qIndices[b_i] - 1);
          }
          real_T b_t10;
          real_T b_t12;
          real_T t12_tmp;
          real_T t9;
          r = _mm_loadu_pd(&xi_c[0]);
          r1 = _mm_set1_pd(-1.0);
          r2 = _mm_set1_pd(MBSys->frames.l[iFrm]);
          _mm_storeu_pd(
              &xi_c[0],
              _mm_mul_pd(
                  _mm_mul_pd(
                      _mm_add_pd(r, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm])),
                      r1),
                  r2));
          r = _mm_loadu_pd(&xi_c[2]);
          _mm_storeu_pd(
              &xi_c[2],
              _mm_mul_pd(
                  _mm_mul_pd(
                      _mm_add_pd(
                          r, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 2])),
                      r1),
                  r2));
          r = _mm_loadu_pd(&xi_c[4]);
          _mm_storeu_pd(
              &xi_c[4],
              _mm_mul_pd(
                  _mm_mul_pd(
                      _mm_add_pd(
                          r, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 4])),
                      r1),
                  r2));
          //     %% Right-Trivialized Derivative of the Cayley map for SE(3)
          //  Implements the right-trivialized derivative for the Cayley map
          //  (also referred to as the right-trivialized tangent map) for SE(3):
          //  dcay : se(3) -> se(3) as the corresponding 6x6 linear matrix
          //  operator Source: [Kob14], eq. 17; cf. [Dem+14, p.11];
          //  cf. [KM11] and other sources for background information
          //  Follows convention for se3 elements in vector form: [omega; v]
          //  Input xi: se(3) element in *vector* form
          //  Output T: 6x6 matrix
          //  Maximilian Herrmann
          //  Chair of Automatic Control
          //  TUM School of Engineering and Design
          //  Technical University of Munich
          //     %% Original implementation using "analytic" functions
          //     %{
          //     om = xi(1:3);
          //     v  = xi(4:end);
          //     omH = skew(om);
          //     vH  = skew(v);
          //
          //     T = zeros(6, 6, class(xi));
          //     T(1:3, 1:3) = 2 / (4 + om.'*om) * ( 2*eye(3) + omH );
          //     T(1:3, 4:6) = zeros(3);
          //     T(4:6, 1:3) = 1 / (4 + om.'*om) * vH * ( 2*eye(3) + omH );
          //     T(4:6, 4:6) = eye(3) + ( 1 / (4 + om.'*om) * ( 2*omH + omH^2 )
          //     );
          //     %}
          //     %% More efficient code generated from MATLAB
          //  (using the above initial implementation)
          //  Code used:
          //  xi = sym('xi', [6,1]);
          //  T = cayRTDSE3( xi );
          //  matlabFunction(T, "Vars", {xi}, "File","tempFun",
          //  "Optimize",true);
          t5 = xi_c[0] * xi_c[1];
          t6 = xi_c[0] * xi_c[2];
          t7 = xi_c[1] * xi_c[2];
          t10 = xi_c[0] * 2.0;
          t12 = xi_c[1] * 2.0;
          t13 = xi_c[2] * 2.0;
          t8 = xi_c[0] * xi_c[0];
          t9 = xi_c[1] * xi_c[1];
          b_t10 = xi_c[2] * xi_c[2];
          t12_tmp = t8 + t9;
          b_t12 = 1.0 / ((t12_tmp + b_t10) + 4.0);
          alpha1 = b_t12 * xi_c[3] * 2.0;
          beta1 = b_t12 * xi_c[4] * 2.0;
          t19 = b_t12 * xi_c[5] * 2.0;
          t14 = b_t12 * xi_c[0];
          t15 = t14 * xi_c[3];
          t16 = b_t12 * xi_c[1];
          t18 = t16 * xi_c[4];
          t2 = b_t12 * xi_c[2];
          t3 = t2 * xi_c[5];
          t4 = MBSys->frames.l[iFrm] * (b_t12 * 4.0);
          h_y[0] = t4;
          h_y[1] = MBSys->frames.l[iFrm] * (t13 * b_t12);
          h_y[2] = MBSys->frames.l[iFrm] * (t16 * -2.0);
          h_y[3] = MBSys->frames.l[iFrm] * (-t18 - t3);
          h_y[4] = MBSys->frames.l[iFrm] * (t19 + t16 * xi_c[3]);
          h_y[5] = MBSys->frames.l[iFrm] * (-beta1 + t2 * xi_c[3]);
          h_y[6] = MBSys->frames.l[iFrm] * (t2 * -2.0);
          h_y[7] = t4;
          h_y[8] = MBSys->frames.l[iFrm] * (t10 * b_t12);
          h_y[9] = MBSys->frames.l[iFrm] * (-t19 + t14 * xi_c[4]);
          h_y[10] = MBSys->frames.l[iFrm] * (-t15 - t3);
          h_y[11] = MBSys->frames.l[iFrm] * (alpha1 + t2 * xi_c[4]);
          h_y[12] = MBSys->frames.l[iFrm] * (t12 * b_t12);
          h_y[13] = MBSys->frames.l[iFrm] * (t14 * -2.0);
          h_y[14] = t4;
          h_y[15] = MBSys->frames.l[iFrm] * (beta1 + t14 * xi_c[5]);
          h_y[16] = MBSys->frames.l[iFrm] * (-alpha1 + t16 * xi_c[5]);
          h_y[17] = MBSys->frames.l[iFrm] * (-t15 - t18);
          alpha1 = MBSys->frames.l[iFrm] * 0.0;
          h_y[18] = alpha1;
          h_y[19] = alpha1;
          h_y[20] = alpha1;
          h_y[21] = MBSys->frames.l[iFrm] * (-b_t12 * (t9 + b_t10) + 1.0);
          h_y[22] = MBSys->frames.l[iFrm] * (b_t12 * (t5 + t13));
          h_y[23] = MBSys->frames.l[iFrm] * (b_t12 * (t6 - t12));
          h_y[24] = alpha1;
          h_y[25] = alpha1;
          h_y[26] = alpha1;
          h_y[27] = MBSys->frames.l[iFrm] * (b_t12 * (t5 - t13));
          h_y[28] = MBSys->frames.l[iFrm] * (-b_t12 * (t8 + b_t10) + 1.0);
          h_y[29] = MBSys->frames.l[iFrm] * (b_t12 * (t7 + t10));
          h_y[30] = alpha1;
          h_y[31] = alpha1;
          h_y[32] = alpha1;
          h_y[33] = MBSys->frames.l[iFrm] * (b_t12 * (t6 + t12));
          h_y[34] = MBSys->frames.l[iFrm] * (b_t12 * (t7 - t10));
          h_y[35] = MBSys->frames.l[iFrm] * (-b_t12 * t12_tmp + 1.0);
          //             %% Get frame's Ba matrix
          //  Helper function to get the Ba matrix with correct dimensions
          //  from the stored array padded with zeros
          //  index of the frame to get
          for (int32_T b_i{0}; b_i < b_n; b_i++) {
            for (int32_T j{0}; j < 6; j++) {
              n = j + 6 * b_i;
              BaMat_data[n] = MBSys->frames.BaPadded[n + 36 * iFrm];
            }
          }
          if (MBSys->frames.nDof[iFrm] != 0) {
            TRANSB1 = 'N';
            TRANSA1 = 'N';
            alpha1 = 1.0;
            beta1 = 0.0;
            m_t = (ptrdiff_t)6;
            n_t = (ptrdiff_t) static_cast<int32_T>(MBSys->frames.nDof[iFrm]);
            k_t = (ptrdiff_t)6;
            lda_t = (ptrdiff_t)6;
            ldb_t = (ptrdiff_t)6;
            ldc_t = (ptrdiff_t)6;
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &h_y[0],
                  &lda_t, &BaMat_data[0], &ldb_t, &beta1, &tmp_data[0], &ldc_t);
          }
          for (int32_T b_i{0}; b_i < mmj; b_i++) {
            for (int32_T j{0}; j < 6; j++) {
              J[(j + 6 * r4[b_i]) + 6 * J.size(1) * iFrm] =
                  tmp_data[j + 6 * b_i];
            }
          }
        } break;
        default:
          //  error
          break;
        }
      } else if (ii < iFrm) {
        boolean_T exitg1;
        tf = false;
        jp1j = 0;
        exitg1 = false;
        while ((!exitg1) && (jp1j <= MBSys->frames.ancestors.size(0) - 1)) {
          if (static_cast<uint32_T>(ii) + 1U ==
              MBSys->frames
                  .ancestors[jp1j + MBSys->frames.ancestors.size(0) * iFrm]) {
            tf = true;
            exitg1 = true;
          } else {
            jp1j++;
          }
        }
        if (tf) {
          b_n = qIndices.size(1);
          r4.set_size(qIndices.size(1));
          //  *Inverse* large Ad representation (6x6 matrix) of an element of
          //  SE3 Follows convention for se3 elements in vector form: [omega; v]
          //  Input g: SE3 element in (4x4) matrix representation
          //  Maximilian Herrmann
          //  Chair of Automatic Control
          //  TUM School of Engineering and Design
          //  Technical University of Munich
          //     %% Original implementation using "analytic" functions
          //     %{
          //     R = g(1:3, 1:3);
          //     p = g(1:3, 4);
          //     % Inverse Ad representation, taken from [MLS94, p. 56]
          //     AdInv = [
          //         R.',          zeros(3);
          //         -R.'*skew(p), R.';
          //         ];
          //     %}
          //     %% More efficient code generated from MATLAB
          //  (using the above initial implementation)
          //  Code used:
          //  g = sym('g', [4,4]);
          //  AdInv = lAdSE3Inv( g );
          //  matlabFunction(AdInv, "Vars", {g}, "File","tempFun",
          //  "Optimize",true);
          alpha1 = g_rel[16 * iFrm];
          h_y[0] = alpha1;
          beta1 = g_rel[16 * iFrm + 4];
          h_y[1] = beta1;
          t19 = g_rel[16 * iFrm + 8];
          h_y[2] = t19;
          t14 = g_rel[16 * iFrm + 1];
          t15 = g_rel[16 * iFrm + 14];
          t16 = g_rel[16 * iFrm + 13];
          t6 = g_rel[16 * iFrm + 2];
          h_y[3] = -t14 * t15 + t16 * t6;
          t18 = g_rel[16 * iFrm + 5];
          t2 = g_rel[16 * iFrm + 6];
          h_y[4] = -t18 * t15 + t16 * t2;
          t3 = g_rel[16 * iFrm + 9];
          t4 = g_rel[16 * iFrm + 10];
          h_y[5] = -t3 * t15 + t16 * t4;
          h_y[6] = t14;
          h_y[7] = t18;
          h_y[8] = t3;
          t5 = g_rel[16 * iFrm + 12];
          h_y[9] = alpha1 * t15 - t5 * t6;
          h_y[10] = beta1 * t15 - t5 * t2;
          h_y[11] = t19 * t15 - t5 * t4;
          h_y[12] = t6;
          h_y[13] = t2;
          h_y[14] = t4;
          h_y[15] = -alpha1 * t16 + t5 * t14;
          h_y[16] = -beta1 * t16 + t5 * t18;
          h_y[17] = -t19 * t16 + t5 * t3;
          h_y[18] = 0.0;
          h_y[19] = 0.0;
          h_y[20] = 0.0;
          h_y[21] = alpha1;
          h_y[22] = beta1;
          h_y[23] = t19;
          h_y[24] = 0.0;
          h_y[25] = 0.0;
          h_y[26] = 0.0;
          h_y[27] = t14;
          h_y[28] = t18;
          h_y[29] = t3;
          h_y[30] = 0.0;
          h_y[31] = 0.0;
          h_y[32] = 0.0;
          h_y[33] = t6;
          h_y[34] = t2;
          h_y[35] = t4;
          n = MBSys->frames.parent[iFrm];
          c_b.set_size(6, qIndices.size(1));
          for (int32_T b_i{0}; b_i < b_n; b_i++) {
            r4[b_i] = static_cast<uint16_T>(qIndices[b_i] - 1);
            for (int32_T j{0}; j < 6; j++) {
              c_b[j + 6 * b_i] =
                  J[(j + 6 * (qIndices[b_i] - 1)) + 6 * J.size(1) * (n - 1)];
            }
          }
          if (qIndices.size(1) == 0) {
            r5.set_size(6, 0);
          } else {
            TRANSB1 = 'N';
            TRANSA1 = 'N';
            alpha1 = 1.0;
            beta1 = 0.0;
            m_t = (ptrdiff_t)6;
            n_t = (ptrdiff_t)qIndices.size(1);
            k_t = (ptrdiff_t)6;
            lda_t = (ptrdiff_t)6;
            ldb_t = (ptrdiff_t)6;
            ldc_t = (ptrdiff_t)6;
            r5.set_size(6, qIndices.size(1));
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &h_y[0],
                  &lda_t, &c_b[0], &ldb_t, &beta1, &r5[0], &ldc_t);
          }
          for (int32_T b_i{0}; b_i < b_n; b_i++) {
            for (int32_T j{0}; j < 6; j++) {
              J[(j + 6 * r4[b_i]) + 6 * J.size(1) * iFrm] = r5[j + 6 * b_i];
            }
          }
        }
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  //  Generalized forces (stress and inputs)
  //             %% Compute the system input matrix
  //  "Fast" function -- with given relative deformations
  //  Array of relative configurations between body frames
  //  dimensions (4,4,nFrames)
  B.set_size(static_cast<int32_T>(MBSys->nDoF),
             static_cast<int32_T>(MBSys->nInputs));
  jp1j =
      static_cast<int32_T>(MBSys->nDoF) * static_cast<int32_T>(MBSys->nInputs);
  if (jp1j - 1 >= 0) {
    std::memset(&B[0], 0, static_cast<uint32_T>(jp1j) * sizeof(real_T));
  }
  for (int32_T b_iFrm{0}; b_iFrm < i; b_iFrm++) {
    a = MBSys->frames.uIndices[2 * b_iFrm];
    if (a != 0) {
      //             %% Get all indices in the input vector for the current
      //             frame
      //  Helper function to get all indices from the stored first and
      //  last index
      jp1j = 2 * b_iFrm + 1;
      b = MBSys->frames.uIndices[jp1j];
      if (b < a) {
        b_n = -1;
      } else {
        b_n = static_cast<uint16_T>(b - a);
      }
      uIndices.set_size(1, b_n + 1);
      for (int32_T b_i{0}; b_i <= b_n; b_i++) {
        uIndices[b_i] = static_cast<uint16_T>(a + static_cast<uint32_T>(b_i));
      }
      //             %% Get all indices in the configuration vector for the
      //             current frame
      //  Helper function to get all indices from the stored first and
      //  last index
      a = MBSys->frames.qIndices[2 * b_iFrm];
      b = MBSys->frames.qIndices[jp1j];
      if (b < a) {
        b_n = -1;
      } else {
        b_n = static_cast<uint16_T>(b - a);
      }
      qIndices.set_size(1, b_n + 1);
      for (int32_T b_i{0}; b_i <= b_n; b_i++) {
        qIndices[b_i] = static_cast<uint16_T>(a + static_cast<uint32_T>(b_i));
      }
      switch (MBSys->frames.jointType[b_iFrm]) {
      case 1U:
        //  Rigid joint (scalar input)
        n = qIndices.size(1);
        r4.set_size(qIndices.size(1));
        if (n - 1 >= 0) {
          std::copy(&qIndices[0], &qIndices[n], &r4[0]);
        }
        b_n = uIndices.size(1);
        r6.set_size(uIndices.size(1));
        if (b_n - 1 >= 0) {
          std::copy(&uIndices[0], &uIndices[b_n], &r6[0]);
        }
        for (int32_T b_i{0}; b_i < b_n; b_i++) {
          for (int32_T j{0}; j < n; j++) {
            B[(r4[j] + B.size(0) * (r6[b_i] - 1)) - 1] = 1.0;
          }
        }
        break;
      case 2U: {
        int32_T i1;
        //  Flexible joint (multiple cable inputs)
        t6 = MBSys->frames.l[b_iFrm];
        i1 = uIndices.size(1);
        if (uIndices.size(1) - 1 >= 0) {
          i2 = MBSys->frames.nDof[b_iFrm];
          b_loop_ub = i2;
          c_loop_ub = qIndices.size(1);
        }
        if (i1 - 1 >= 0) {
          for (int32_T b_i{0}; b_i < b_loop_ub; b_i++) {
            for (int32_T j{0}; j < 6; j++) {
              n = j + 6 * b_i;
              tmp_data[n] = MBSys->frames.BaPadded[n + 36 * b_iFrm];
            }
          }
        }
        for (int32_T ii{0}; ii < i1; ii++) {
          int8_T ipiv[4];
          //  Cable configurations at adjacent nodes
          //  Discrete deformation gradient cable routing
          //  Tangent vector is in elements 4:6
          for (int32_T b_i{0}; b_i < 4; b_i++) {
            n = b_i << 2;
            b_n = 4 * b_i + 16 * b_iFrm;
            R[n] = g_rel[b_n];
            jp1j = 4 * b_i + 32 * b_iFrm;
            A[n] =
                MBSys->frames.g_cm[jp1j + 32 * MBSys->frames.g_cm.size(3) * ii];
            R[n + 1] = g_rel[b_n + 1];
            A[n + 1] =
                MBSys->frames
                    .g_cm[(jp1j + 32 * MBSys->frames.g_cm.size(3) * ii) + 1];
            R[n + 2] = g_rel[b_n + 2];
            A[n + 2] =
                MBSys->frames
                    .g_cm[(jp1j + 32 * MBSys->frames.g_cm.size(3) * ii) + 2];
            R[n + 3] = g_rel[b_n + 3];
            A[n + 3] =
                MBSys->frames
                    .g_cm[(jp1j + 32 * MBSys->frames.g_cm.size(3) * ii) + 3];
            ipiv[b_i] = static_cast<int8_T>(b_i + 1);
          }
          for (int32_T j{0}; j < 3; j++) {
            mmj = 2 - j;
            b_b = j * 5;
            jj = j * 5;
            jp1j = b_b + 2;
            n = 5 - j;
            b_n = 0;
            alpha1 = muDoubleScalarAbs(A[jj]);
            for (int32_T b_i{2}; b_i < n; b_i++) {
              beta1 = muDoubleScalarAbs(A[(b_b + b_i) - 1]);
              if (beta1 > alpha1) {
                b_n = b_i - 1;
                alpha1 = beta1;
              }
            }
            if (A[jj + b_n] != 0.0) {
              if (b_n != 0) {
                n = j + b_n;
                ipiv[j] = static_cast<int8_T>(n + 1);
                alpha1 = A[j];
                A[j] = A[n];
                A[n] = alpha1;
                alpha1 = A[j + 4];
                A[j + 4] = A[n + 4];
                A[n + 4] = alpha1;
                alpha1 = A[j + 8];
                A[j + 8] = A[n + 8];
                A[n + 8] = alpha1;
                alpha1 = A[j + 12];
                A[j + 12] = A[n + 12];
                A[n + 12] = alpha1;
              }
              n = (jj - j) + 4;
              for (int32_T b_i{jp1j}; b_i <= n; b_i++) {
                A[b_i - 1] /= A[jj];
              }
            }
            n = jj;
            for (int32_T b_i{0}; b_i <= mmj; b_i++) {
              alpha1 = A[(b_b + (b_i << 2)) + 4];
              if (alpha1 != 0.0) {
                b_n = n + 6;
                jp1j = (n - j) + 8;
                for (int32_T iFrm{b_n}; iFrm <= jp1j; iFrm++) {
                  A[iFrm - 1] += A[((jj + iFrm) - n) - 5] * -alpha1;
                }
              }
              n += 4;
            }
            i3 = ipiv[j];
            if (i3 != j + 1) {
              alpha1 = R[j];
              R[j] = R[i3 - 1];
              R[i3 - 1] = alpha1;
              alpha1 = R[j + 4];
              R[j + 4] = R[i3 + 3];
              R[i3 + 3] = alpha1;
              alpha1 = R[j + 8];
              R[j + 8] = R[i3 + 7];
              R[i3 + 7] = alpha1;
              alpha1 = R[j + 12];
              R[j + 12] = R[i3 + 11];
              R[i3 + 11] = alpha1;
            }
          }
          for (int32_T b_i{0}; b_i < 4; b_i++) {
            n = b_i << 2;
            for (int32_T iFrm{0}; iFrm < 4; iFrm++) {
              b_n = iFrm << 2;
              jp1j = iFrm + n;
              if (R[jp1j] != 0.0) {
                mmj = iFrm + 2;
                for (int32_T j{mmj}; j < 5; j++) {
                  b_b = (j + n) - 1;
                  R[b_b] -= R[jp1j] * A[(j + b_n) - 1];
                }
              }
            }
          }
          for (int32_T b_i{0}; b_i < 4; b_i++) {
            n = b_i << 2;
            for (int32_T j{3}; j >= 0; j--) {
              b_n = j << 2;
              jp1j = j + n;
              alpha1 = R[jp1j];
              if (alpha1 != 0.0) {
                R[jp1j] = alpha1 / A[j + b_n];
                for (int32_T iFrm{0}; iFrm < j; iFrm++) {
                  mmj = iFrm + n;
                  R[mmj] -= R[jp1j] * A[iFrm + b_n];
                }
              }
            }
          }
          std::memset(&A[0], 0, sizeof(real_T) << 4);
          for (int32_T b_i{0}; b_i < 4; b_i++) {
            n = b_i << 2;
            for (int32_T j{0}; j < 4; j++) {
              b_n = j << 2;
              r = _mm_loadu_pd(&R[b_n]);
              r1 = _mm_loadu_pd(&A[n]);
              r2 = _mm_set1_pd(
                  MBSys->frames.g_cm[(((j + 4 * b_i) + 32 * b_iFrm) +
                                      32 * MBSys->frames.g_cm.size(3) * ii) +
                                     16]);
              _mm_storeu_pd(&A[n], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
              r = _mm_loadu_pd(&R[b_n + 2]);
              r1 = _mm_loadu_pd(&A[n + 2]);
              _mm_storeu_pd(&A[n + 2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
            }
          }
          //     %% Inverse of the Cayley map for SE(3)
          //  Implements the inverse Cayley map for SE(3): cay : SE(3) -> se(3)
          //  Source: [Dem+14, p.10]
          //  Follows convention for se3 elements in vector form: [omega; v]
          //  Input g:   Element of SE3 in matrix form
          //  Output xi: Corresponding se(3) element in *vector* form
          //  Maximilian Herrmann
          //  Chair of Automatic Control
          //  TUM School of Engineering and Design
          //  Technical University of Munich
          //     %% Inverse of the Cayley map for SO(3)
          //  Implements the inverse Cayley map for SO(3): cayInv : SO3(3) ->
          //  so(3) Source: [Dem+14, p.9], eq. 15 Input Lambda: Rotation matrix
          //  Output omega: Corresponding so(3) element in *vector* form
          //  Maximilian Herrmann
          //  Chair of Automatic Control
          //  TUM School of Engineering and Design
          //  Technical University of Munich
          alpha1 = 2.0 / (((A[0] + A[5]) + A[10]) + 1.0);
          for (int32_T b_i{0}; b_i < 3; b_i++) {
            n = b_i << 2;
            omh[3 * b_i] = alpha1 * (A[n] - A[b_i]);
            omh[3 * b_i + 1] = alpha1 * (A[n + 1] - A[b_i + 4]);
            omh[3 * b_i + 2] = alpha1 * (A[n + 2] - A[b_i + 8]);
          }
          //  SKEWINV Inverse hat map for 2, 3 and 6 dimensions:
          //  so(2) -> R1
          //  so(3) -> R3
          //  se(3) -> R6
          //  Important:
          //  For se3, the convention for the vector in R6 is
          //     x = [ omega; v ],
          //  where omega is the angular and v the translationalcomponent.
          //  Implementation by:
          //  Maximilian Herrmann
          //  Chair of Automatic Control
          //  TUM School of Engineering and Design
          //  Technical University of Munich
          //  so(3) hat map
          //  e.g. [Lee08, p. 28]
          //  Inverse hat map for so(3)
          std::memset(&R_tmp[0], 0, 9U * sizeof(real_T));
          R_tmp[0] = 1.0;
          R_tmp[4] = 1.0;
          R_tmp[8] = 1.0;
          r = _mm_loadu_pd(&A[0]);
          r1 = _mm_loadu_pd(&R_tmp[0]);
          _mm_storeu_pd(&R_tmp[0], _mm_add_pd(r, r1));
          R_tmp[2] += A[2];
          r = _mm_loadu_pd(&A[4]);
          r1 = _mm_loadu_pd(&R_tmp[3]);
          _mm_storeu_pd(&R_tmp[3], _mm_add_pd(r, r1));
          R_tmp[5] += A[6];
          r = _mm_loadu_pd(&A[8]);
          r1 = _mm_loadu_pd(&R_tmp[6]);
          _mm_storeu_pd(&R_tmp[6], _mm_add_pd(r, r1));
          R_tmp[8] += A[10];
          n = 0;
          b_n = 1;
          jp1j = 2;
          alpha1 = muDoubleScalarAbs(R_tmp[0]);
          beta1 = muDoubleScalarAbs(R_tmp[1]);
          if (beta1 > alpha1) {
            alpha1 = beta1;
            n = 1;
            b_n = 0;
          }
          if (muDoubleScalarAbs(R_tmp[2]) > alpha1) {
            n = 2;
            b_n = 1;
            jp1j = 0;
          }
          R_tmp[b_n] /= R_tmp[n];
          R_tmp[jp1j] /= R_tmp[n];
          R_tmp[b_n + 3] -= R_tmp[b_n] * R_tmp[n + 3];
          R_tmp[jp1j + 3] -= R_tmp[jp1j] * R_tmp[n + 3];
          R_tmp[b_n + 6] -= R_tmp[b_n] * R_tmp[n + 6];
          R_tmp[jp1j + 6] -= R_tmp[jp1j] * R_tmp[n + 6];
          if (muDoubleScalarAbs(R_tmp[jp1j + 3]) >
              muDoubleScalarAbs(R_tmp[b_n + 3])) {
            mmj = b_n;
            b_n = jp1j;
            jp1j = mmj;
          }
          R_tmp[jp1j + 3] /= R_tmp[b_n + 3];
          R_tmp[jp1j + 6] -= R_tmp[jp1j + 3] * R_tmp[b_n + 6];
          alpha1 = A[n + 12];
          c_R_tmp[1] = A[b_n + 12] - alpha1 * R_tmp[b_n];
          c_R_tmp[2] = (A[jp1j + 12] - alpha1 * R_tmp[jp1j]) -
                       c_R_tmp[1] * R_tmp[jp1j + 3];
          c_R_tmp[2] /= R_tmp[jp1j + 6];
          c_R_tmp[0] = alpha1 - c_R_tmp[2] * R_tmp[n + 6];
          c_R_tmp[1] -= c_R_tmp[2] * R_tmp[b_n + 6];
          c_R_tmp[1] /= R_tmp[b_n + 3];
          c_R_tmp[0] -= c_R_tmp[1] * R_tmp[n + 3];
          c_R_tmp[0] /= R_tmp[n];
          xi_c[0] = omh[5] / t6;
          xi_c[1] = omh[6] / t6;
          xi_c[2] = omh[1] / t6;
          r = _mm_loadu_pd(&c_R_tmp[0]);
          _mm_storeu_pd(&xi_c[3], _mm_div_pd(_mm_mul_pd(_mm_set1_pd(2.0), r),
                                             _mm_set1_pd(t6)));
          xi_c[5] = 2.0 * c_R_tmp[2] / t6;
          //  Compute matrix entry
          //             %% Get frame's Ba matrix
          //  Helper function to get the Ba matrix with correct dimensions
          //  from the stored array padded with zeros
          //  index of the frame to get
          _mm_storeu_pd(
              &c_R_tmp[0],
              _mm_add_pd(_mm_loadu_pd(
                             &MBSys->frames
                                  .g_cm[(32 * b_iFrm +
                                         32 * MBSys->frames.g_cm.size(3) * ii) +
                                        12]),
                         _mm_loadu_pd(
                             &MBSys->frames
                                  .g_cm[(32 * b_iFrm +
                                         32 * MBSys->frames.g_cm.size(3) * ii) +
                                        28])));
          c_R_tmp[2] =
              MBSys->frames
                  .g_cm[(32 * b_iFrm + 32 * MBSys->frames.g_cm.size(3) * ii) +
                        14] +
              MBSys->frames
                  .g_cm[(32 * b_iFrm + 32 * MBSys->frames.g_cm.size(3) * ii) +
                        30];
          //  SKEW Hat map for 2, 3 and 6 dimensions:
          //  R1 -> so(2), i.e. 2x2 matrix
          //  R3 -> so(3), i.e. 3x3 matrix
          //  R6 -> se(3), i.e. 4x4 matrix
          //  Important:
          //  For se3, the convention for the vector in R6 is
          //     x = [ omega; v ],
          //  where omega is the angular and v the translationalcomponent.
          //  Implementation by:
          //  Maximilian Herrmann
          //  Chair of Automatic Control
          //  TUM School of Engineering and Design
          //  Technical University of Munich
          //  so(3) hat map
          //  e.g. [Lee08, p. 28]
          //  SKEWSO3 Hat map for so(3) / 3 dimensions R3 -> so(3), i.e. 3x3
          //  matrix Implementation by:
          //  Maximilian Herrmann
          //  Chair of Automatic Control
          //  TUM School of Engineering and Design
          //  Technical University of Munich
          omh[0] = 0.0;
          omh[3] = 0.5 * -c_R_tmp[2];
          omh[6] = 0.5 * c_R_tmp[1];
          omh[1] = 0.5 * c_R_tmp[2];
          omh[4] = 0.0;
          omh[7] = 0.5 * -c_R_tmp[0];
          omh[2] = 0.5 * -c_R_tmp[1];
          omh[5] = 0.5 * c_R_tmp[0];
          omh[8] = 0.0;
          std::memset(&c_R_tmp[0], 0, 3U * sizeof(real_T));
          r = _mm_loadu_pd(&omh[0]);
          r1 = _mm_loadu_pd(&c_R_tmp[0]);
          _mm_storeu_pd(&c_R_tmp[0],
                        _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(xi_c[3]))));
          c_R_tmp[2] += omh[2] * xi_c[3];
          r = _mm_loadu_pd(&omh[3]);
          r1 = _mm_loadu_pd(&c_R_tmp[0]);
          _mm_storeu_pd(&c_R_tmp[0],
                        _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(xi_c[4]))));
          c_R_tmp[2] += xi_c[4] * omh[5];
          r = _mm_loadu_pd(&omh[6]);
          r1 = _mm_loadu_pd(&c_R_tmp[0]);
          _mm_storeu_pd(&c_R_tmp[0],
                        _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(xi_c[5]))));
          c_R_tmp[2] += 0.0 * xi_c[5];
          e_b[0] = c_R_tmp[0];
          e_b[3] = xi_c[3];
          e_b[1] = c_R_tmp[1];
          e_b[4] = xi_c[4];
          e_b[2] = c_R_tmp[2];
          e_b[5] = xi_c[5];
          if (i2 == 0) {
            g_y.set_size(0);
          } else {
            TRANSB1 = 'N';
            TRANSA1 = 'T';
            alpha1 = 1.0;
            beta1 = 0.0;
            m_t = (ptrdiff_t)i2;
            n_t = (ptrdiff_t)1;
            k_t = (ptrdiff_t)6;
            lda_t = (ptrdiff_t)6;
            ldb_t = (ptrdiff_t)6;
            ldc_t = (ptrdiff_t)i2;
            g_y.set_size(i2);
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &tmp_data[0],
                  &lda_t, &e_b[0], &ldb_t, &beta1, &g_y[0], &ldc_t);
          }
          b_n = qIndices.size(1);
          r4.set_size(qIndices.size(1));
          for (int32_T b_i{0}; b_i < c_loop_ub; b_i++) {
            r4[b_i] = static_cast<uint16_T>(qIndices[b_i] - 1);
          }
          alpha1 = 3.3121686421112381E-170;
          beta1 = muDoubleScalarAbs(xi_c[3]);
          if (beta1 > 3.3121686421112381E-170) {
            t14 = 1.0;
            alpha1 = beta1;
          } else {
            t19 = beta1 / 3.3121686421112381E-170;
            t14 = t19 * t19;
          }
          beta1 = muDoubleScalarAbs(xi_c[4]);
          if (beta1 > alpha1) {
            t19 = alpha1 / beta1;
            t14 = t14 * t19 * t19 + 1.0;
            alpha1 = beta1;
          } else {
            t19 = beta1 / alpha1;
            t14 += t19 * t19;
          }
          beta1 = muDoubleScalarAbs(xi_c[5]);
          if (beta1 > alpha1) {
            t19 = alpha1 / beta1;
            t14 = t14 * t19 * t19 + 1.0;
            alpha1 = beta1;
          } else {
            t19 = beta1 / alpha1;
            t14 += t19 * t19;
          }
          t14 = alpha1 * muDoubleScalarSqrt(t14);
          tf = muDoubleScalarIsNaN(t14);
          if (tf) {
            n = 0;
            int32_T exitg2;
            do {
              exitg2 = 0;
              if (n < 3) {
                if (muDoubleScalarIsNaN(xi_c[n + 3])) {
                  exitg2 = 1;
                } else {
                  n++;
                }
              } else {
                t14 = rtInf;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
          alpha1 = -t6 / t14;
          n = uIndices[ii];
          for (int32_T b_i{0}; b_i < b_n; b_i++) {
            B[r4[b_i] + B.size(0) * (n - 1)] = alpha1 * g_y[b_i];
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
          }
        }
      } break;
      default:
        //  error
        break;
      }
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  if ((B.size(0) == 0) || (B.size(1) == 0) || (u.size(0) == 0)) {
    res.set_size(static_cast<int32_T>(MBSys->nDoF));
    for (int32_T b_i{0}; b_i < loop_ub; b_i++) {
      res[b_i] = 0.0;
    }
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)B.size(0);
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)B.size(1);
    lda_t = (ptrdiff_t)B.size(0);
    ldb_t = (ptrdiff_t)u.size(0);
    ldc_t = (ptrdiff_t)B.size(0);
    res.set_size(static_cast<int32_T>(MBSys->nDoF));
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &B[0], &lda_t,
          (real_T *)&u[0], &ldb_t, &beta1, &res[0], &ldc_t);
  }
  if (q.size(0) == 1) {
    n = MBSys->qRef.size(0);
  } else {
    n = q.size(0);
  }
  if (MBSys->cSys.size(0) == 1) {
    jp1j = n;
  } else {
    jp1j = MBSys->cSys.size(0);
  }
  if ((q.size(0) == MBSys->qRef.size(0)) && (MBSys->cSys.size(0) == n) &&
      (jp1j == res.size(0))) {
    n = MBSys->cSys.size(0);
    res.set_size(MBSys->cSys.size(0));
    jp1j = (MBSys->cSys.size(0) / 2) << 1;
    b_n = jp1j - 2;
    for (int32_T b_i{0}; b_i <= b_n; b_i += 2) {
      r = _mm_loadu_pd(&res[b_i]);
      _mm_storeu_pd(
          &res[b_i],
          _mm_sub_pd(_mm_mul_pd(_mm_loadu_pd(&MBSys->cSys[b_i]),
                                _mm_sub_pd(_mm_loadu_pd(&q[b_i]),
                                           _mm_loadu_pd(&MBSys->qRef[b_i]))),
                     r));
    }
    for (int32_T b_i{jp1j}; b_i < n; b_i++) {
      res[b_i] = MBSys->cSys[b_i] * (q[b_i] - MBSys->qRef[b_i]) - res[b_i];
    }
  } else {
    binary_expand_op_1(emlrtRootTLSGlobal, res, MBSys, q);
  }
  //  Placeholder values for external forces
  //  Get gravity and external spatial forces transformed to the body-fixed
  //  frames
  //     %% Compute the body-fixed forces for all frames: Gravity and Ext.
  //     Forces
  f_frame_b.set_size(6, static_cast<int32_T>(MBSys->nFrames));
  n = 6 * static_cast<int32_T>(MBSys->nFrames);
  if (n - 1 >= 0) {
    std::memset(&f_frame_b[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
  }
  for (int32_T j{0}; j < i; j++) {
    std::memset(&c_R_tmp[0], 0, 3U * sizeof(real_T));
    t15 = MBSys->frames.m_a[j];
    std::memset(&b_MBSys[0], 0, 3U * sizeof(real_T));
    t16 = c_R_tmp[0];
    t6 = c_R_tmp[1];
    t18 = c_R_tmp[2];
    alpha1 = b_MBSys[0];
    beta1 = b_MBSys[1];
    t19 = b_MBSys[2];
    for (int32_T b_i{0}; b_i < 3; b_i++) {
      i3 = d_b[b_i];
      n = b_i + 16 * j;
      t14 = g[n];
      omh[3 * b_i] = t14;
      t16 += t14 * static_cast<real_T>(i3);
      alpha1 += MBSys->frames.m[j] * t14 * static_cast<real_T>(i3);
      t14 = g[n + 4];
      omh[3 * b_i + 1] = t14;
      t6 += t14 * static_cast<real_T>(i3);
      beta1 += MBSys->frames.m[j] * t14 * static_cast<real_T>(i3);
      t14 = g[n + 8];
      omh[3 * b_i + 2] = t14;
      t18 += t14 * static_cast<real_T>(i3);
      t19 += MBSys->frames.m[j] * t14 * static_cast<real_T>(i3);
    }
    b_MBSys[2] = t19;
    b_MBSys[1] = beta1;
    b_MBSys[0] = alpha1;
    alpha1 = MBSys->frames.x_a[3 * j + 1];
    beta1 = MBSys->frames.x_a[3 * j + 2];
    xi_c[0] = simPars->g * (t15 * (alpha1 * t18 - t6 * beta1));
    t19 = MBSys->frames.x_a[3 * j];
    xi_c[1] = simPars->g * (t15 * (t16 * beta1 - t19 * t18));
    xi_c[2] = simPars->g * (t15 * (t19 * t6 - t16 * alpha1));
    r = _mm_loadu_pd(&b_MBSys[0]);
    _mm_storeu_pd(&xi_c[3], _mm_mul_pd(_mm_set1_pd(simPars->g), r));
    r = _mm_set1_pd(-0.0);
    _mm_storeu_pd(&c_R_tmp[0], r);
    _mm_storeu_pd(&b_MBSys[0], r);
    xi_c[5] = simPars->g * b_MBSys[2];
    alpha1 = c_R_tmp[0];
    beta1 = c_R_tmp[1];
    t19 = b_MBSys[0];
    t14 = b_MBSys[1];
    for (int32_T b_i{0}; b_i < 3; b_i++) {
      t15 = omh[b_i];
      t16 = omh[b_i + 3];
      t6 = omh[b_i + 6] * -0.0;
      n = b_i + 6 * j;
      f_frame_b[n] = ((t15 * alpha1 + t16 * beta1) + t6) + xi_c[b_i];
      f_frame_b[n + 3] = ((t15 * t19 + t16 * t14) + t6) + xi_c[b_i + 3];
    }
    //  % External spatial forces
    // % Gravity
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  if (static_cast<int32_T>(MBSys->nFrames) == f_frame_b.size(1)) {
    f_frame_b.set_size(6, static_cast<int32_T>(MBSys->nFrames));
  } else {
    binary_expand_op(f_frame_b, MBSys);
  }
  //  Compute sum of generalized forces
  for (int32_T iFrm{0}; iFrm < i; iFrm++) {
    r9.set_size(6, loop_ub);
    for (int32_T b_i{0}; b_i < loop_ub; b_i++) {
      for (int32_T j{0}; j < 6; j++) {
        n = j + 6 * b_i;
        r9[n] = J[n + 6 * J.size(1) * iFrm];
      }
    }
    if (J.size(1) == 0) {
      g_y.set_size(0);
    } else {
      TRANSB1 = 'N';
      TRANSA1 = 'T';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)J.size(1);
      n_t = (ptrdiff_t)1;
      k_t = (ptrdiff_t)6;
      lda_t = (ptrdiff_t)6;
      ldb_t = (ptrdiff_t)6;
      ldc_t = (ptrdiff_t)J.size(1);
      g_y.set_size(loop_ub);
      dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &r9[0], &lda_t,
            &f_frame_b[6 * iFrm], &ldb_t, &beta1, &g_y[0], &ldc_t);
    }
    n = res.size(0);
    if (res.size(0) == g_y.size(0)) {
      jp1j = (res.size(0) / 2) << 1;
      b_n = jp1j - 2;
      for (int32_T b_i{0}; b_i <= b_n; b_i += 2) {
        r = _mm_loadu_pd(&res[b_i]);
        r1 = _mm_loadu_pd(&g_y[b_i]);
        _mm_storeu_pd(&res[b_i], _mm_add_pd(r, r1));
      }
      for (int32_T b_i{jp1j}; b_i < n; b_i++) {
        res[b_i] = res[b_i] + g_y[b_i];
      }
    } else {
      plus(emlrtRootTLSGlobal, res, g_y);
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

emlrtCTX emlrtGetRootTLSGlobal()
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

// End of code generation (computeStaticResiduum.cpp)
