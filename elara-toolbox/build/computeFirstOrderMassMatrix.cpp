//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFirstOrderMassMatrix.cpp
//
// Code generation for function 'computeFirstOrderMassMatrix'
//

// Include files
#include "computeFirstOrderMassMatrix.h"
#include "FramePropertiesNum.h"
#include "SystemNum.h"
#include "computeFirstOrderMassMatrix_data.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "coder_array.h"
#include "mwmathutil.h"
#include <cstddef>
#include <cstring>
#include <emmintrin.h>

// Variable Definitions
static emlrtMCInfo emlrtMCI{
    27,      // lineNo
    5,       // colNo
    "error", // fName
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\lang\\error.m" // pName
};

// Function Declarations
static void b_error(const mxArray *m, emlrtMCInfo &location);

// Function Definitions
static void b_error(const mxArray *m, emlrtMCInfo &location)
{
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, nullptr, 1, &m, "error", true,
                        &location);
}

void computeFirstOrderMassMatrix(real_T, const coder::array<real_T, 1U> &x,
                                 const elara::SystemNum *MBSys,
                                 coder::array<real_T, 2U> &M_fo)
{
  static const int32_T iv[2]{1, 29};
  static const char_T u[29]{'I', 'n', 'v', 'a', 'l', 'i', 'd', ' ', 'j', 'o',
                            'i', 'n', 't', ' ', 't', 'y', 'p', 'e', ' ', 's',
                            'p', 'e', 'c', 'i', 'f', 'i', 'e', 'd', '.'};
  static const int8_T iv1[4]{0, 0, 0, 1};
  __m128d r3;
  __m128d r4;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  coder::array<real_T, 3U> J;
  coder::array<real_T, 3U> g_rel;
  coder::array<real_T, 2U> M;
  coder::array<real_T, 2U> b_b;
  coder::array<real_T, 2U> c_b;
  coder::array<real_T, 2U> d_y;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r2;
  coder::array<real_T, 2U> r5;
  coder::array<real_T, 1U> qi;
  coder::array<uint16_T, 2U> qIndices;
  coder::array<uint16_T, 1U> r1;
  coder::array<int8_T, 2U> b_I;
  const mxArray *b_m;
  const mxArray *b_y;
  const mxArray *propValues;
  const mxArray *y;
  real_T BaMat_data[36];
  real_T e_y[36];
  real_T tmp_data[36];
  real_T R[9];
  real_T b_R_tmp[9];
  real_T c_R_tmp[9];
  real_T omh[9];
  real_T c_y[6];
  real_T b_MBSys[3];
  real_T alpha1;
  real_T b_t;
  real_T beta1;
  real_T t10;
  real_T t12;
  real_T t13;
  real_T t13_tmp;
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
  real_T t9;
  int32_T BaMat_data_tmp;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T i2;
  int32_T loop_ub;
  int32_T m;
  int32_T n;
  uint16_T a;
  uint16_T b;
  const char_T *propClasses{"coder.internal.string"};
  const char_T *propNames{"Value"};
  char_T TRANSA1;
  char_T TRANSB1;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  //     %% Compute the overall mass matrix for a multibody system in
  //     first-order form
  //  I.e., the mass matrix diag(I, M) with dimension 2*nDof x 2*nDof
  //  Integration time (from ode solver)
  //  Not needed for the function, but "~" is not allowed for codegen
  //  State vector [q; q_dot] (2*nDof,1)
  if (MBSys->nDoF < 0.0) {
    b_t = 0.0;
  } else {
    b_t = MBSys->nDoF;
  }
  m = static_cast<int32_T>(b_t);
  b_I.set_size(static_cast<int32_T>(b_t), static_cast<int32_T>(b_t));
  n = static_cast<int32_T>(b_t) * static_cast<int32_T>(b_t);
  if (n - 1 >= 0) {
    std::memset(&b_I[0], 0, static_cast<uint32_T>(n) * sizeof(int8_T));
  }
  if (static_cast<int32_T>(b_t) > 0) {
    for (int32_T i{0}; i < m; i++) {
      b_I[i + b_I.size(0) * i] = 1;
    }
  }
  //             %% Compute the system mass matrix
  //  System coordinates (nDoF, 1)
  //  System mass matrix
  //  Array of frame Jacobian matrices
  //  Get Jacobians
  //             %% Compute Geometric Jacobian Matrix for Full Multibody System
  //  System coordinates (nDoF, 1)
  //  Jacobian matrices with dimensions
  //  6 x (nAllwd_1*nSegments1 + ... + nAllwdB*nSegmentsB + nLinks) x nFrames
  //  where B is the nr. of flexible beams in the system
  //  Relative configurations between body frames
  //  Compute relative joint transformations
  //             %% Compute the relative transformations of all joints (rigid
  //             and flexible)
  //  for given relative coordinates
  //  System coordinates  (nDoF, 1)
  //  SE3 Matrices with relative configurations between body frames
  b_i = static_cast<int32_T>(MBSys->nFrames);
  g_rel.set_size(4, 4, static_cast<int32_T>(MBSys->nFrames));
  n = static_cast<int32_T>(MBSys->nFrames) << 4;
  if (n - 1 >= 0) {
    std::memset(&g_rel[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
  }
  for (int32_T iFrm{0}; iFrm < b_i; iFrm++) {
    a = MBSys->frames.qIndices[2 * iFrm];
    b = MBSys->frames.qIndices[2 * iFrm + 1];
    if (a > b) {
      BaMat_data_tmp = 0;
      n = 0;
    } else {
      BaMat_data_tmp = a - 1;
      n = b;
    }
    loop_ub = n - BaMat_data_tmp;
    qi.set_size(loop_ub);
    for (int32_T i{0}; i < loop_ub; i++) {
      qi[i] = x[BaMat_data_tmp + i];
    }
    switch (MBSys->frames.jointType[iFrm]) {
    case 1U: {
      real_T b_R[16];
      int8_T R_tmp[9];
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
      if (loop_ub != 0) {
        t9 = x[BaMat_data_tmp];
      }
      //  SKEWSO3 Hat map for so(3) / 3 dimensions R3 -> so(3), i.e. 3x3 matrix
      //  Implementation by:
      //  Maximilian Herrmann
      //  Chair of Automatic Control
      //  TUM School of Engineering and Design
      //  Technical University of Munich
      omh[0] = 0.0;
      i2 = 6 * iFrm + 2;
      alpha1 = MBSys->frames.X[i2];
      omh[3] = -alpha1;
      b_loop_ub = 6 * iFrm + 1;
      t19 = MBSys->frames.X[b_loop_ub];
      omh[6] = t19;
      omh[1] = alpha1;
      omh[4] = 0.0;
      alpha1 = MBSys->frames.X[6 * iFrm];
      omh[7] = -alpha1;
      omh[2] = -t19;
      omh[5] = alpha1;
      omh[8] = 0.0;
      //  Formula (2.14), p. 28 in [MLS94]
      for (int32_T i{0}; i < 9; i++) {
        R_tmp[i] = 0;
      }
      t13_tmp = muDoubleScalarSin(t9);
      t14 = 1.0 - muDoubleScalarCos(t9);
      std::memset(&R[0], 0, 9U * sizeof(real_T));
      for (int32_T i{0}; i < 3; i++) {
        R_tmp[i + 3 * i] = 1;
        beta1 = omh[3 * i];
        r3 = _mm_loadu_pd(&omh[0]);
        r4 = _mm_loadu_pd(&R[3 * i]);
        _mm_storeu_pd(&R[3 * i],
                      _mm_add_pd(r4, _mm_mul_pd(r3, _mm_set1_pd(beta1))));
        n = 3 * i + 2;
        R[n] += -t19 * beta1;
        beta1 = omh[3 * i + 1];
        r3 = _mm_loadu_pd(&omh[3]);
        r4 = _mm_loadu_pd(&R[3 * i]);
        _mm_storeu_pd(&R[3 * i],
                      _mm_add_pd(r4, _mm_mul_pd(r3, _mm_set1_pd(beta1))));
        R[n] += alpha1 * beta1;
        beta1 = omh[n];
        r3 = _mm_loadu_pd(&omh[6]);
        r4 = _mm_loadu_pd(&R[3 * i]);
        _mm_storeu_pd(&R[3 * i],
                      _mm_add_pd(r4, _mm_mul_pd(r3, _mm_set1_pd(beta1))));
        R[n] += 0.0 * beta1;
      }
      //  Formula (2.36), p. 42 in [MLS94]
      for (int32_T i{0}; i < 9; i++) {
        n = R_tmp[i];
        alpha1 = (static_cast<real_T>(n) + omh[i] * t13_tmp) + R[i] * t14;
        R[i] = alpha1;
        b_R_tmp[i] = static_cast<real_T>(n) - alpha1;
      }
      std::memset(&c_R_tmp[0], 0, 9U * sizeof(real_T));
      std::memset(&b_MBSys[0], 0, 3U * sizeof(real_T));
      alpha1 = b_MBSys[0];
      beta1 = b_MBSys[1];
      t19 = b_MBSys[2];
      for (int32_T i{0}; i < 3; i++) {
        n = i + 6 * iFrm;
        t13_tmp = MBSys->frames.X[n + 3];
        t14 = omh[3 * i];
        r3 = _mm_loadu_pd(&b_R_tmp[0]);
        r4 = _mm_loadu_pd(&c_R_tmp[3 * i]);
        _mm_storeu_pd(&c_R_tmp[3 * i],
                      _mm_add_pd(r4, _mm_mul_pd(r3, _mm_set1_pd(t14))));
        BaMat_data_tmp = 3 * i + 2;
        c_R_tmp[BaMat_data_tmp] += b_R_tmp[2] * t14;
        t14 = MBSys->frames.X[6 * iFrm] * MBSys->frames.X[n];
        omh[3 * i] = t14;
        alpha1 += t14 * t13_tmp;
        loop_ub = 3 * i + 1;
        t14 = omh[loop_ub];
        r3 = _mm_loadu_pd(&b_R_tmp[3]);
        r4 = _mm_loadu_pd(&c_R_tmp[3 * i]);
        _mm_storeu_pd(&c_R_tmp[3 * i],
                      _mm_add_pd(r4, _mm_mul_pd(r3, _mm_set1_pd(t14))));
        c_R_tmp[BaMat_data_tmp] += b_R_tmp[5] * t14;
        t14 = MBSys->frames.X[b_loop_ub] * MBSys->frames.X[n];
        omh[loop_ub] = t14;
        beta1 += t14 * t13_tmp;
        t14 = omh[BaMat_data_tmp];
        r3 = _mm_loadu_pd(&b_R_tmp[6]);
        r4 = _mm_loadu_pd(&c_R_tmp[3 * i]);
        _mm_storeu_pd(&c_R_tmp[3 * i],
                      _mm_add_pd(r4, _mm_mul_pd(r3, _mm_set1_pd(t14))));
        c_R_tmp[BaMat_data_tmp] += b_R_tmp[8] * t14;
        t14 = MBSys->frames.X[i2] * MBSys->frames.X[n];
        omh[BaMat_data_tmp] = t14;
        t19 += t14 * t13_tmp;
      }
      b_MBSys[2] = t19;
      b_MBSys[1] = beta1;
      b_MBSys[0] = alpha1;
      for (int32_T i{0}; i < 3; i++) {
        n = i << 2;
        b_R[n] = R[3 * i];
        b_R[n + 1] = R[3 * i + 1];
        b_R[n + 2] = R[3 * i + 2];
        b_R[i + 12] = ((c_R_tmp[i] * MBSys->frames.X[6 * iFrm + 3] +
                        c_R_tmp[i + 3] * MBSys->frames.X[6 * iFrm + 4]) +
                       c_R_tmp[i + 6] * MBSys->frames.X[6 * iFrm + 5]) +
                      b_MBSys[i] * t9;
      }
      for (int32_T i{0}; i < 4; i++) {
        n = i << 2;
        b_R[n + 3] = iv1[i];
        BaMat_data_tmp = 4 * i + 16 * iFrm;
        g_rel[BaMat_data_tmp] = 0.0;
        g_rel[BaMat_data_tmp + 1] = 0.0;
        g_rel[BaMat_data_tmp + 2] = 0.0;
        g_rel[BaMat_data_tmp + 3] = 0.0;
        for (int32_T i1{0}; i1 < 4; i1++) {
          r3 = _mm_loadu_pd(&g_rel[BaMat_data_tmp]);
          r4 = _mm_set1_pd(b_R[i1 + n]);
          loop_ub = 4 * i1 + 16 * iFrm;
          _mm_storeu_pd(
              &g_rel[BaMat_data_tmp],
              _mm_add_pd(
                  r3,
                  _mm_mul_pd(_mm_loadu_pd(&MBSys->frames.g_ref[loop_ub]), r4)));
          r3 = _mm_loadu_pd(&g_rel[BaMat_data_tmp + 2]);
          _mm_storeu_pd(
              &g_rel[BaMat_data_tmp + 2],
              _mm_add_pd(r3, _mm_mul_pd(_mm_loadu_pd(
                                            &MBSys->frames.g_ref[loop_ub + 2]),
                                        r4)));
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
      for (int32_T i{0}; i < n; i++) {
        for (int32_T i1{0}; i1 < 6; i1++) {
          BaMat_data_tmp = i1 + 6 * i;
          BaMat_data[BaMat_data_tmp] =
              MBSys->frames.BaPadded[BaMat_data_tmp + 36 * iFrm];
        }
      }
      if ((MBSys->frames.nDof[iFrm] == 0) || (loop_ub == 0)) {
        for (int32_T i{0}; i < 6; i++) {
          c_y[i] = 0.0;
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
        ldb_t = (ptrdiff_t)loop_ub;
        ldc_t = (ptrdiff_t)6;
        dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &BaMat_data[0],
              &lda_t, &qi[0], &ldb_t, &beta1, &c_y[0], &ldc_t);
      }
      r3 = _mm_loadu_pd(&c_y[0]);
      r4 = _mm_set1_pd(MBSys->frames.l[iFrm]);
      _mm_storeu_pd(
          &c_y[0],
          _mm_mul_pd(_mm_add_pd(r3, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm])),
                     r4));
      r3 = _mm_loadu_pd(&c_y[2]);
      _mm_storeu_pd(
          &c_y[2],
          _mm_mul_pd(
              _mm_add_pd(r3, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 2])),
              r4));
      r3 = _mm_loadu_pd(&c_y[4]);
      _mm_storeu_pd(
          &c_y[4],
          _mm_mul_pd(
              _mm_add_pd(r3, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 4])),
              r4));
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
      t2 = c_y[0] * c_y[0];
      t3 = c_y[1] * c_y[1];
      t4 = c_y[2] * c_y[2];
      t5 = c_y[0] / 2.0;
      t6 = c_y[1] / 2.0;
      t7 = c_y[2] / 2.0;
      alpha1 = c_y[0] * c_y[1];
      t10 = alpha1 / 4.0;
      beta1 = c_y[0] * c_y[2];
      t12 = beta1 / 4.0;
      t13_tmp = c_y[1] * c_y[2];
      t13 = t13_tmp / 4.0;
      t14 = t2 / 2.0;
      t15 = t3 / 2.0;
      t16 = t4 / 2.0;
      t18 = 1.0 / (((t2 + t3) + t4) + 4.0);
      g_rel[16 * iFrm] = t18 * (t15 + t16) * -4.0 + 1.0;
      g_rel[16 * iFrm + 1] = t18 * (t5 * c_y[1] + c_y[2]) * 4.0;
      g_rel[16 * iFrm + 2] = t18 * (c_y[1] - beta1 / 2.0) * -4.0;
      g_rel[16 * iFrm + 3] = 0.0;
      g_rel[16 * iFrm + 4] = t18 * (c_y[2] - alpha1 / 2.0) * -4.0;
      g_rel[16 * iFrm + 5] = t18 * (t14 + t16) * -4.0 + 1.0;
      g_rel[16 * iFrm + 6] = t18 * (t6 * c_y[2] + c_y[0]) * 4.0;
      g_rel[16 * iFrm + 7] = 0.0;
      g_rel[16 * iFrm + 8] = t18 * (t5 * c_y[2] + c_y[1]) * 4.0;
      g_rel[16 * iFrm + 9] = t18 * (c_y[0] - t13_tmp / 2.0) * -4.0;
      g_rel[16 * iFrm + 10] = t18 * (t14 + t15) * -4.0 + 1.0;
      g_rel[16 * iFrm + 11] = 0.0;
      beta1 = t18 * c_y[4];
      t19 = t18 * c_y[3];
      alpha1 = t18 * c_y[5];
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
      y = nullptr;
      b_m = nullptr;
      emlrtAssign(&y, emlrtCreateClassInstance2022a(emlrtRootTLSGlobal,
                                                    "coder.internal.string"));
      b_m = nullptr;
      b_y = nullptr;
      propValues = emlrtCreateCharArray(2, &iv[0]);
      emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 29, propValues, &u[0]);
      emlrtAssign(&b_y, propValues);
      emlrtAssign(&b_m, b_y);
      propValues = b_m;
      emlrtSetAllProperties(emlrtRootTLSGlobal, &y, 0, 1,
                            (const char_T **)&propNames,
                            (const char_T **)&propClasses, &propValues);
      emlrtAssign(&y, emlrtConvertInstanceToRedirectSource(
                          emlrtRootTLSGlobal, y, 0, "coder.internal.string"));
      b_error(y, emlrtMCI);
      break;
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  //  Compute Jacobians
  //             %% Compute Geometric Jacobian Matrix for Full Multibody System
  //  with *given* relative joint transformations g_ij
  //  System coordinates (nDoF, 1)
  //  Array of relative configurations between body frames
  //  dimensions (4,4,nFrames)
  //  Jacobian matrices with dimensions
  //  6 x (nAllwd_1*nSegments1 + ... + nAllwdB*nSegmentsB + nLinks) x nFrames
  //  where B is the nr. of flexible beams in the system
  //  Array holding all Jacobians
  b_loop_ub = static_cast<int32_T>(MBSys->nDoF);
  J.set_size(6, static_cast<int32_T>(MBSys->nDoF),
             static_cast<int32_T>(MBSys->nFrames));
  n = 6 * static_cast<int32_T>(MBSys->nDoF) *
      static_cast<int32_T>(MBSys->nFrames);
  if (n - 1 >= 0) {
    std::memset(&J[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
  }
  for (int32_T iFrm{0}; iFrm < b_i; iFrm++) {
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
      for (int32_T i{0}; i <= n; i++) {
        qIndices[i] = static_cast<uint16_T>(a + static_cast<uint32_T>(i));
      }
      //  Compute block columns for current frame
      if (ii == iFrm) {
        switch (MBSys->frames.jointType[iFrm]) {
        case 1U:
          n = qIndices.size(1);
          r1.set_size(qIndices.size(1));
          for (int32_T i{0}; i < n; i++) {
            r1[i] = static_cast<uint16_T>(qIndices[i] - 1);
          }
          for (int32_T i{0}; i < 6; i++) {
            c_y[i] = MBSys->frames.X[i + 6 * iFrm];
          }
          for (int32_T i{0}; i < n; i++) {
            for (int32_T i1{0}; i1 < 6; i1++) {
              J[(i1 + 6 * r1[i]) + 6 * J.size(1) * iFrm] = c_y[i1 + 6 * i];
            }
          }
          break;
        case 2U: {
          loop_ub = qIndices.size(1);
          r.set_size(1, qIndices.size(1));
          for (int32_T i{0}; i < loop_ub; i++) {
            r[i] = x[qIndices[i] - 1];
          }
          //             %% Get frame's Ba matrix
          //  Helper function to get the Ba matrix with correct dimensions
          //  from the stored array padded with zeros
          //  index of the frame to get
          BaMat_data_tmp = MBSys->frames.nDof[iFrm];
          for (int32_T i{0}; i < BaMat_data_tmp; i++) {
            for (int32_T i1{0}; i1 < 6; i1++) {
              n = i1 + 6 * i;
              BaMat_data[n] = MBSys->frames.BaPadded[n + 36 * iFrm];
            }
          }
          if ((MBSys->frames.nDof[iFrm] == 0) || (qIndices.size(1) == 0)) {
            for (int32_T i{0}; i < 6; i++) {
              c_y[i] = 0.0;
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
                  &lda_t, &r[0], &ldb_t, &beta1, &c_y[0], &ldc_t);
          }
          r1.set_size(qIndices.size(1));
          for (int32_T i{0}; i < loop_ub; i++) {
            r1[i] = static_cast<uint16_T>(qIndices[i] - 1);
          }
          __m128d r6;
          real_T b_t10;
          real_T b_t12;
          real_T t12_tmp;
          r3 = _mm_loadu_pd(&c_y[0]);
          r4 = _mm_set1_pd(-1.0);
          r6 = _mm_set1_pd(MBSys->frames.l[iFrm]);
          _mm_storeu_pd(
              &c_y[0],
              _mm_mul_pd(
                  _mm_mul_pd(_mm_add_pd(r3, _mm_loadu_pd(
                                                &MBSys->frames.xiC[6 * iFrm])),
                             r4),
                  r6));
          r3 = _mm_loadu_pd(&c_y[2]);
          _mm_storeu_pd(
              &c_y[2],
              _mm_mul_pd(
                  _mm_mul_pd(
                      _mm_add_pd(
                          r3, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 2])),
                      r4),
                  r6));
          r3 = _mm_loadu_pd(&c_y[4]);
          _mm_storeu_pd(
              &c_y[4],
              _mm_mul_pd(
                  _mm_mul_pd(
                      _mm_add_pd(
                          r3, _mm_loadu_pd(&MBSys->frames.xiC[6 * iFrm + 4])),
                      r4),
                  r6));
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
          t4 = c_y[0] * c_y[1];
          t5 = c_y[0] * c_y[2];
          t6 = c_y[1] * c_y[2];
          t7 = c_y[0] * 2.0;
          t10 = c_y[1] * 2.0;
          t12 = c_y[2] * 2.0;
          t13 = c_y[0] * c_y[0];
          t9 = c_y[1] * c_y[1];
          b_t10 = c_y[2] * c_y[2];
          t12_tmp = t13 + t9;
          b_t12 = 1.0 / ((t12_tmp + b_t10) + 4.0);
          alpha1 = b_t12 * c_y[3] * 2.0;
          beta1 = b_t12 * c_y[4] * 2.0;
          t19 = b_t12 * c_y[5] * 2.0;
          t13_tmp = b_t12 * c_y[0];
          t14 = t13_tmp * c_y[3];
          t15 = b_t12 * c_y[1];
          t16 = t15 * c_y[4];
          t18 = b_t12 * c_y[2];
          t2 = t18 * c_y[5];
          t3 = MBSys->frames.l[iFrm] * (b_t12 * 4.0);
          e_y[0] = t3;
          e_y[1] = MBSys->frames.l[iFrm] * (t12 * b_t12);
          e_y[2] = MBSys->frames.l[iFrm] * (t15 * -2.0);
          e_y[3] = MBSys->frames.l[iFrm] * (-t16 - t2);
          e_y[4] = MBSys->frames.l[iFrm] * (t19 + t15 * c_y[3]);
          e_y[5] = MBSys->frames.l[iFrm] * (-beta1 + t18 * c_y[3]);
          e_y[6] = MBSys->frames.l[iFrm] * (t18 * -2.0);
          e_y[7] = t3;
          e_y[8] = MBSys->frames.l[iFrm] * (t7 * b_t12);
          e_y[9] = MBSys->frames.l[iFrm] * (-t19 + t13_tmp * c_y[4]);
          e_y[10] = MBSys->frames.l[iFrm] * (-t14 - t2);
          e_y[11] = MBSys->frames.l[iFrm] * (alpha1 + t18 * c_y[4]);
          e_y[12] = MBSys->frames.l[iFrm] * (t10 * b_t12);
          e_y[13] = MBSys->frames.l[iFrm] * (t13_tmp * -2.0);
          e_y[14] = t3;
          e_y[15] = MBSys->frames.l[iFrm] * (beta1 + t13_tmp * c_y[5]);
          e_y[16] = MBSys->frames.l[iFrm] * (-alpha1 + t15 * c_y[5]);
          e_y[17] = MBSys->frames.l[iFrm] * (-t14 - t16);
          alpha1 = MBSys->frames.l[iFrm] * 0.0;
          e_y[18] = alpha1;
          e_y[19] = alpha1;
          e_y[20] = alpha1;
          e_y[21] = MBSys->frames.l[iFrm] * (-b_t12 * (t9 + b_t10) + 1.0);
          e_y[22] = MBSys->frames.l[iFrm] * (b_t12 * (t4 + t12));
          e_y[23] = MBSys->frames.l[iFrm] * (b_t12 * (t5 - t10));
          e_y[24] = alpha1;
          e_y[25] = alpha1;
          e_y[26] = alpha1;
          e_y[27] = MBSys->frames.l[iFrm] * (b_t12 * (t4 - t12));
          e_y[28] = MBSys->frames.l[iFrm] * (-b_t12 * (t13 + b_t10) + 1.0);
          e_y[29] = MBSys->frames.l[iFrm] * (b_t12 * (t6 + t7));
          e_y[30] = alpha1;
          e_y[31] = alpha1;
          e_y[32] = alpha1;
          e_y[33] = MBSys->frames.l[iFrm] * (b_t12 * (t5 + t10));
          e_y[34] = MBSys->frames.l[iFrm] * (b_t12 * (t6 - t7));
          e_y[35] = MBSys->frames.l[iFrm] * (-b_t12 * t12_tmp + 1.0);
          //             %% Get frame's Ba matrix
          //  Helper function to get the Ba matrix with correct dimensions
          //  from the stored array padded with zeros
          //  index of the frame to get
          for (int32_T i{0}; i < BaMat_data_tmp; i++) {
            for (int32_T i1{0}; i1 < 6; i1++) {
              n = i1 + 6 * i;
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
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &e_y[0],
                  &lda_t, &BaMat_data[0], &ldb_t, &beta1, &tmp_data[0], &ldc_t);
          }
          for (int32_T i{0}; i < loop_ub; i++) {
            for (int32_T i1{0}; i1 < 6; i1++) {
              J[(i1 + 6 * r1[i]) + 6 * J.size(1) * iFrm] = tmp_data[i1 + 6 * i];
            }
          }
        } break;
        default:
          //  error
          break;
        }
      } else if (ii < iFrm) {
        boolean_T exitg1;
        boolean_T tf;
        tf = false;
        n = 0;
        exitg1 = false;
        while ((!exitg1) && (n <= MBSys->frames.ancestors.size(0) - 1)) {
          if (static_cast<uint32_T>(ii) + 1U ==
              MBSys->frames
                  .ancestors[n + MBSys->frames.ancestors.size(0) * iFrm]) {
            tf = true;
            exitg1 = true;
          } else {
            n++;
          }
        }
        if (tf) {
          BaMat_data_tmp = qIndices.size(1);
          r1.set_size(qIndices.size(1));
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
          e_y[0] = alpha1;
          beta1 = g_rel[16 * iFrm + 4];
          e_y[1] = beta1;
          t19 = g_rel[16 * iFrm + 8];
          e_y[2] = t19;
          t13_tmp = g_rel[16 * iFrm + 1];
          t14 = g_rel[16 * iFrm + 14];
          t15 = g_rel[16 * iFrm + 13];
          t16 = g_rel[16 * iFrm + 2];
          e_y[3] = -t13_tmp * t14 + t15 * t16;
          t18 = g_rel[16 * iFrm + 5];
          t2 = g_rel[16 * iFrm + 6];
          e_y[4] = -t18 * t14 + t15 * t2;
          t3 = g_rel[16 * iFrm + 9];
          t4 = g_rel[16 * iFrm + 10];
          e_y[5] = -t3 * t14 + t15 * t4;
          e_y[6] = t13_tmp;
          e_y[7] = t18;
          e_y[8] = t3;
          t5 = g_rel[16 * iFrm + 12];
          e_y[9] = alpha1 * t14 - t5 * t16;
          e_y[10] = beta1 * t14 - t5 * t2;
          e_y[11] = t19 * t14 - t5 * t4;
          e_y[12] = t16;
          e_y[13] = t2;
          e_y[14] = t4;
          e_y[15] = -alpha1 * t15 + t5 * t13_tmp;
          e_y[16] = -beta1 * t15 + t5 * t18;
          e_y[17] = -t19 * t15 + t5 * t3;
          e_y[18] = 0.0;
          e_y[19] = 0.0;
          e_y[20] = 0.0;
          e_y[21] = alpha1;
          e_y[22] = beta1;
          e_y[23] = t19;
          e_y[24] = 0.0;
          e_y[25] = 0.0;
          e_y[26] = 0.0;
          e_y[27] = t13_tmp;
          e_y[28] = t18;
          e_y[29] = t3;
          e_y[30] = 0.0;
          e_y[31] = 0.0;
          e_y[32] = 0.0;
          e_y[33] = t16;
          e_y[34] = t2;
          e_y[35] = t4;
          n = MBSys->frames.parent[iFrm];
          b_b.set_size(6, qIndices.size(1));
          for (int32_T i{0}; i < BaMat_data_tmp; i++) {
            r1[i] = static_cast<uint16_T>(qIndices[i] - 1);
            for (int32_T i1{0}; i1 < 6; i1++) {
              b_b[i1 + 6 * i] =
                  J[(i1 + 6 * (qIndices[i] - 1)) + 6 * J.size(1) * (n - 1)];
            }
          }
          if (qIndices.size(1) == 0) {
            r2.set_size(6, 0);
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
            r2.set_size(6, qIndices.size(1));
            dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &e_y[0],
                  &lda_t, &b_b[0], &ldb_t, &beta1, &r2[0], &ldc_t);
          }
          for (int32_T i{0}; i < BaMat_data_tmp; i++) {
            for (int32_T i1{0}; i1 < 6; i1++) {
              J[(i1 + 6 * r1[i]) + 6 * J.size(1) * iFrm] = r2[i1 + 6 * i];
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
  //  Compute mass matrix
  //             %% Compute the system mass matrix
  //  Array of geometric Jacobians
  //  System mass matrix
  M.set_size(static_cast<int32_T>(MBSys->nDoF),
             static_cast<int32_T>(MBSys->nDoF));
  n = static_cast<int32_T>(MBSys->nDoF) * static_cast<int32_T>(MBSys->nDoF);
  if (n - 1 >= 0) {
    std::memset(&M[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
  }
  for (int32_T iFrm{0}; iFrm < b_i; iFrm++) {
    c_b.set_size(6, b_loop_ub);
    for (int32_T i{0}; i < b_loop_ub; i++) {
      for (int32_T i1{0}; i1 < 6; i1++) {
        BaMat_data_tmp = i1 + 6 * i;
        c_b[BaMat_data_tmp] = J[BaMat_data_tmp + 6 * J.size(1) * iFrm];
      }
    }
    if (J.size(1) == 0) {
      d_y.set_size(0, 6);
    } else {
      TRANSB1 = 'N';
      TRANSA1 = 'T';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)J.size(1);
      n_t = (ptrdiff_t)6;
      k_t = (ptrdiff_t)6;
      lda_t = (ptrdiff_t)6;
      ldb_t = (ptrdiff_t)6;
      ldc_t = (ptrdiff_t)J.size(1);
      d_y.set_size(b_loop_ub, 6);
      dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &c_b[0], &lda_t,
            (real_T *)&MBSys->frames.MGen[36 * iFrm], &ldb_t, &beta1, &d_y[0],
            &ldc_t);
    }
    c_b.set_size(6, b_loop_ub);
    for (int32_T i{0}; i < b_loop_ub; i++) {
      for (int32_T i1{0}; i1 < 6; i1++) {
        BaMat_data_tmp = i1 + 6 * i;
        c_b[BaMat_data_tmp] = J[BaMat_data_tmp + 6 * J.size(1) * iFrm];
      }
    }
    if ((d_y.size(0) == 0) || (J.size(1) == 0)) {
      r5.set_size(d_y.size(0), b_loop_ub);
      n = d_y.size(0) * J.size(1);
      if (n - 1 >= 0) {
        std::memset(&r5[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
      }
    } else {
      TRANSB1 = 'N';
      TRANSA1 = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)d_y.size(0);
      n_t = (ptrdiff_t)J.size(1);
      k_t = (ptrdiff_t)6;
      lda_t = (ptrdiff_t)d_y.size(0);
      ldb_t = (ptrdiff_t)6;
      ldc_t = (ptrdiff_t)d_y.size(0);
      r5.set_size(d_y.size(0), b_loop_ub);
      dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &d_y[0], &lda_t,
            &c_b[0], &ldb_t, &beta1, &r5[0], &ldc_t);
    }
    if ((M.size(0) == r5.size(0)) && (M.size(1) == r5.size(1))) {
      n = M.size(0) * M.size(1);
      BaMat_data_tmp = (n / 2) << 1;
      loop_ub = BaMat_data_tmp - 2;
      for (int32_T i{0}; i <= loop_ub; i += 2) {
        r3 = _mm_loadu_pd(&M[i]);
        r4 = _mm_loadu_pd(&r5[i]);
        _mm_storeu_pd(&M[i], _mm_add_pd(r3, r4));
      }
      for (int32_T i{BaMat_data_tmp}; i < n; i++) {
        M[i] = M[i] + r5[i];
      }
    } else {
      plus(M, r5);
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  BaMat_data_tmp = b_I.size(0) + M.size(0);
  b_loop_ub = b_I.size(1) + M.size(1);
  M_fo.set_size(BaMat_data_tmp, b_loop_ub);
  n = BaMat_data_tmp * b_loop_ub;
  for (int32_T i{0}; i < n; i++) {
    M_fo[i] = 0.0;
  }
  if ((b_I.size(0) > 0) && (b_I.size(1) > 0)) {
    for (int32_T i{0}; i < m; i++) {
      for (int32_T i1{0}; i1 < m; i1++) {
        M_fo[i1 + M_fo.size(0) * i] = b_I[i1 + b_I.size(0) * i];
      }
    }
  }
  if ((M.size(0) > 0) && (M.size(1) > 0)) {
    if (b_I.size(0) + 1 > BaMat_data_tmp) {
      loop_ub = 0;
      BaMat_data_tmp = 0;
    } else {
      loop_ub = static_cast<int32_T>(b_t);
    }
    if (b_I.size(1) + 1 > b_loop_ub) {
      i2 = 0;
      b_loop_ub = 0;
    } else {
      i2 = static_cast<int32_T>(b_t);
    }
    BaMat_data_tmp -= loop_ub;
    n = b_loop_ub - i2;
    for (int32_T i{0}; i < n; i++) {
      for (int32_T i1{0}; i1 < BaMat_data_tmp; i1++) {
        M_fo[(loop_ub + i1) + M_fo.size(0) * (i2 + i)] =
            M[i1 + BaMat_data_tmp * i];
      }
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

// End of code generation (computeFirstOrderMassMatrix.cpp)
