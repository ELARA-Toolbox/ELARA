//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_computeFirstOrderMassMatrix_api.cpp
//
// Code generation for function '_coder_computeFirstOrderMassMatrix_api'
//

// Include files
#include "_coder_computeFirstOrderMassMatrix_api.h"
#include "FramePropertiesNum.h"
#include "SystemNum.h"
#include "computeFirstOrderMassMatrix.h"
#include "computeFirstOrderMassMatrix_data.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Declarations
static void b_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[16]);

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<uint16_T, 2U> &y);

static boolean_T b_emlrt_marshallIn(const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void b_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint8_T, 1U> &ret);

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 2U> &y);

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 3U> &y);

static void b_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 5U> &ret);

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 1U> &y);

static void c_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 1U> &ret);

static uint16_T c_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static void c_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<uint16_T, 2U> &y);

static void c_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 3U> &ret);

static void c_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 2U> &y);

static void d_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint16_T, 2U> &ret);

static real_T d_emlrt_marshallIn(const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void d_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 3U> &ret);

static void d_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 2U> &y);

static void d_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 1U> &ret);

static boolean_T e_emlrt_marshallIn(const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static void e_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret);

static void e_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint16_T, 2U> &ret);

static void emlrt_marshallIn(const mxArray *b_nullptr, const char_T *identifier,
                             coder::array<real_T, 1U> &y);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 1U> &y);

static void emlrt_marshallIn(const mxArray *b_nullptr, const char_T *identifier,
                             elara::SystemNum &y);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             elara::SystemNum &y);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 2U> &y);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 3U> &y);

static real_T emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[16]);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             elara::FramePropertiesNum &y);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 5U> &y);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<uint8_T, 1U> &y);

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<uint16_T, 2U> &y);

static real_T emlrt_marshallIn(const mxArray *b_nullptr,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(coder::array<real_T, 2U> &u);

static void f_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint16_T, 2U> &ret);

static uint16_T f_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

static void f_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret);

static void g_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret);

static void h_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret);

// Function Definitions
static void b_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[16])
{
  static const int32_T dims[2]{4, 4};
  real_T(*r)[16];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[16])emlrtMxGetData(src);
  for (int32_T i{0}; i < 16; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static boolean_T b_emlrt_marshallIn(const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = e_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void b_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint8_T, 1U> &ret)
{
  static const int32_T dims{-1};
  int32_T i;
  boolean_T b{true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "uint8", false, 1U,
                            (const void *)&dims, &b, &i);
  ret.set_size(i);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 1, false);
  emlrtDestroyArray(&src);
}

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<uint16_T, 2U> &y)
{
  e_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 2U> &y)
{
  f_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 3U> &y)
{
  d_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void b_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 5U> &ret)
{
  static const int32_T dims[5]{4, 4, 2, -1, -1};
  int32_T iv[5];
  boolean_T bv[5]{false, false, false, true, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 5U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1], iv[2], iv[3], iv[4]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

static void b_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 1U> &y)
{
  d_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void c_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 1U> &ret)
{
  static const int32_T dims{-1};
  int32_T i;
  boolean_T b{true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret.prealloc(i);
  ret.set_size(i);
  ret.set(static_cast<real_T *>(emlrtMxGetData(src)), ret.size(0));
  emlrtDestroyArray(&src);
}

static uint16_T c_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint16_T y;
  y = f_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void c_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<uint16_T, 2U> &y)
{
  f_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void c_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 3U> &ret)
{
  static const int32_T dims[3]{4, 4, -1};
  int32_T iv[3];
  boolean_T bv[3]{false, false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 3U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1], iv[2]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

static void c_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 2U> &y)
{
  g_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void d_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 1U> &ret)
{
  static const int32_T dims{-1};
  int32_T i;
  boolean_T b{true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret.set_size(i);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

static void d_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint16_T, 2U> &ret)
{
  static const int32_T dims[2]{2, -1};
  int32_T iv[2];
  boolean_T bv[2]{false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "uint16", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 2, false);
  emlrtDestroyArray(&src);
}

static real_T d_emlrt_marshallIn(const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

static void d_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 3U> &ret)
{
  static const int32_T dims[3]{6, 6, -1};
  int32_T iv[3];
  boolean_T bv[3]{false, false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 3U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1], iv[2]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

static void d_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               coder::array<real_T, 2U> &y)
{
  h_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret)
{
  static const int32_T dims[2]{-1, -1};
  int32_T iv[2];
  boolean_T bv[2]{true, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

static boolean_T e_emlrt_marshallIn(const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  boolean_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void e_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint16_T, 2U> &ret)
{
  static const int32_T dims[2]{1, -1};
  int32_T iv[2];
  boolean_T bv[2]{false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "uint16", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 2, false);
  emlrtDestroyArray(&src);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 1U> &y)
{
  c_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const mxArray *b_nullptr, const char_T *identifier,
                             elara::SystemNum &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             elara::SystemNum &y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[16];
  const char_T *propClasses[16]{
      "elara.internal.System", "elara.internal.System", "elara.internal.System",
      "elara.internal.System", "elara.internal.System", "elara.internal.System",
      "elara.internal.System", "elara.internal.System", "elara.internal.System",
      "elara.internal.System", "elara.internal.System", "elara.internal.System",
      "elara.SystemNum",       "elara.SystemNum",       "elara.SystemNum",
      "elara.SystemNum"};
  const char_T *propNames[16]{"isCantilever",
                              "g0",
                              "LinkAdjacencyMatrix",
                              "FrameAdjacencyMatrix",
                              "nLinks",
                              "nJoints",
                              "nFrames",
                              "nDoF",
                              "nInputs",
                              "linkFrameIndices",
                              "indexTCPFrame",
                              "g_B_TCP",
                              "frames",
                              "cSys",
                              "dSys",
                              "qRef"};
  for (int32_T i{0}; i < 16; i++) {
    propValues[i] = nullptr;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a(emlrtRootTLSGlobal, parentId, u, "elara.SystemNum");
  emlrtGetAllProperties(emlrtRootTLSGlobal, u, 0, 16,
                        (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "isCantilever";
  y.isCantilever = b_emlrt_marshallIn(emlrtAlias(propValues[0]), &thisId);
  thisId.fIdentifier = "g0";
  emlrt_marshallIn(emlrtAlias(propValues[1]), &thisId, y.g0);
  thisId.fIdentifier = "LinkAdjacencyMatrix";
  emlrt_marshallIn(emlrtAlias(propValues[2]), &thisId, y.LinkAdjacencyMatrix);
  thisId.fIdentifier = "FrameAdjacencyMatrix";
  emlrt_marshallIn(emlrtAlias(propValues[3]), &thisId, y.FrameAdjacencyMatrix);
  thisId.fIdentifier = "nLinks";
  y.nLinks = emlrt_marshallIn(emlrtAlias(propValues[4]), &thisId);
  thisId.fIdentifier = "nJoints";
  y.nJoints = emlrt_marshallIn(emlrtAlias(propValues[5]), &thisId);
  thisId.fIdentifier = "nFrames";
  y.nFrames = emlrt_marshallIn(emlrtAlias(propValues[6]), &thisId);
  thisId.fIdentifier = "nDoF";
  y.nDoF = emlrt_marshallIn(emlrtAlias(propValues[7]), &thisId);
  thisId.fIdentifier = "nInputs";
  y.nInputs = emlrt_marshallIn(emlrtAlias(propValues[8]), &thisId);
  thisId.fIdentifier = "linkFrameIndices";
  emlrt_marshallIn(emlrtAlias(propValues[9]), &thisId, y.linkFrameIndices);
  thisId.fIdentifier = "indexTCPFrame";
  y.indexTCPFrame = c_emlrt_marshallIn(emlrtAlias(propValues[10]), &thisId);
  thisId.fIdentifier = "g_B_TCP";
  emlrt_marshallIn(emlrtAlias(propValues[11]), &thisId, y.g_B_TCP);
  thisId.fIdentifier = "frames";
  emlrt_marshallIn(emlrtAlias(propValues[12]), &thisId, y.frames);
  thisId.fIdentifier = "cSys";
  b_emlrt_marshallIn(emlrtAlias(propValues[13]), &thisId, y.cSys);
  thisId.fIdentifier = "dSys";
  b_emlrt_marshallIn(emlrtAlias(propValues[14]), &thisId, y.dSys);
  thisId.fIdentifier = "qRef";
  b_emlrt_marshallIn(emlrtAlias(propValues[15]), &thisId, y.qRef);
  emlrtDestroyArrays(16, &propValues[0]);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 2U> &y)
{
  e_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = d_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 3U> &y)
{
  c_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[16])
{
  b_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<uint16_T, 2U> &y)
{
  d_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<uint8_T, 1U> &y)
{
  b_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             coder::array<real_T, 5U> &y)
{
  b_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T emlrt_marshallIn(const mxArray *b_nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static void emlrt_marshallIn(const mxArray *b_nullptr, const char_T *identifier,
                             coder::array<real_T, 1U> &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

static void emlrt_marshallIn(const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             elara::FramePropertiesNum &y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[18];
  const char_T *propClasses[18]{
      "elara.internal.FrameProperties", "elara.internal.FrameProperties",
      "elara.internal.FrameProperties", "elara.internal.FrameProperties",
      "elara.internal.FrameProperties", "elara.internal.FrameProperties",
      "elara.internal.FrameProperties", "elara.internal.FrameProperties",
      "elara.internal.FrameProperties", "elara.internal.FrameProperties",
      "elara.internal.FrameProperties", "elara.internal.FrameProperties",
      "elara.FramePropertiesNum",       "elara.FramePropertiesNum",
      "elara.FramePropertiesNum",       "elara.FramePropertiesNum",
      "elara.FramePropertiesNum",       "elara.FramePropertiesNum"};
  const char_T *propNames[18]{
      "nDof",      "jointType", "qIndices", "uIndices", "linkIndex", "parent",
      "ancestors", "X",         "g_ref",    "BaPadded", "xiC",       "l",
      "MGen",      "m",         "g_a",      "x_a",      "m_a",       "g_cm"};
  for (int32_T i{0}; i < 18; i++) {
    propValues[i] = nullptr;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a(emlrtRootTLSGlobal, parentId, u,
                           "elara.FramePropertiesNum");
  emlrtGetAllProperties(emlrtRootTLSGlobal, u, 0, 18,
                        (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "nDof";
  emlrt_marshallIn(emlrtAlias(propValues[0]), &thisId, y.nDof);
  thisId.fIdentifier = "jointType";
  emlrt_marshallIn(emlrtAlias(propValues[1]), &thisId, y.jointType);
  thisId.fIdentifier = "qIndices";
  emlrt_marshallIn(emlrtAlias(propValues[2]), &thisId, y.qIndices);
  thisId.fIdentifier = "uIndices";
  emlrt_marshallIn(emlrtAlias(propValues[3]), &thisId, y.uIndices);
  thisId.fIdentifier = "linkIndex";
  b_emlrt_marshallIn(emlrtAlias(propValues[4]), &thisId, y.linkIndex);
  thisId.fIdentifier = "parent";
  b_emlrt_marshallIn(emlrtAlias(propValues[5]), &thisId, y.parent);
  thisId.fIdentifier = "ancestors";
  c_emlrt_marshallIn(emlrtAlias(propValues[6]), &thisId, y.ancestors);
  thisId.fIdentifier = "X";
  b_emlrt_marshallIn(emlrtAlias(propValues[7]), &thisId, y.X);
  thisId.fIdentifier = "g_ref";
  emlrt_marshallIn(emlrtAlias(propValues[8]), &thisId, y.g_ref);
  thisId.fIdentifier = "BaPadded";
  b_emlrt_marshallIn(emlrtAlias(propValues[9]), &thisId, y.BaPadded);
  thisId.fIdentifier = "xiC";
  b_emlrt_marshallIn(emlrtAlias(propValues[10]), &thisId, y.xiC);
  thisId.fIdentifier = "l";
  b_emlrt_marshallIn(emlrtAlias(propValues[11]), &thisId, y.l);
  thisId.fIdentifier = "MGen";
  b_emlrt_marshallIn(emlrtAlias(propValues[12]), &thisId, y.MGen);
  thisId.fIdentifier = "m";
  b_emlrt_marshallIn(emlrtAlias(propValues[13]), &thisId, y.m);
  thisId.fIdentifier = "g_a";
  emlrt_marshallIn(emlrtAlias(propValues[14]), &thisId, y.g_a);
  thisId.fIdentifier = "x_a";
  c_emlrt_marshallIn(emlrtAlias(propValues[15]), &thisId, y.x_a);
  thisId.fIdentifier = "m_a";
  d_emlrt_marshallIn(emlrtAlias(propValues[16]), &thisId, y.m_a);
  thisId.fIdentifier = "g_cm";
  emlrt_marshallIn(emlrtAlias(propValues[17]), &thisId, y.g_cm);
  emlrtDestroyArrays(18, &propValues[0]);
  emlrtDestroyArray(&u);
}

static const mxArray *emlrt_marshallOut(coder::array<real_T, 2U> &u)
{
  static const int32_T iv[2]{0, 0};
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, u.size(), 2);
  u.set_owner(false);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<uint16_T, 2U> &ret)
{
  static const int32_T dims[2]{-1, -1};
  int32_T iv[2];
  boolean_T bv[2]{true, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "uint16", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 2, false);
  emlrtDestroyArray(&src);
}

static uint16_T f_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  uint16_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "uint16", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<uint16_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

static void f_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret)
{
  static const int32_T dims[2]{6, -1};
  int32_T iv[2];
  boolean_T bv[2]{false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

static void g_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret)
{
  static const int32_T dims[2]{3, -1};
  int32_T iv[2];
  boolean_T bv[2]{false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

static void h_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               coder::array<real_T, 2U> &ret)
{
  static const int32_T dims[2]{1, -1};
  int32_T iv[2];
  boolean_T bv[2]{false, true};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret.set_size(iv[0], iv[1]);
  emlrtImportArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 8, false);
  emlrtDestroyArray(&src);
}

void computeFirstOrderMassMatrix_api(const mxArray *const prhs[3],
                                     const mxArray **plhs)
{
  elara::SystemNum MBSys;
  coder::array<real_T, 2U> M_fo;
  coder::array<real_T, 1U> x;
  const mxArray *prhs_copy_idx_1;
  real_T t;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  prhs_copy_idx_1 = emlrtProtectR2012b(prhs[1], 1, false, -1);
  // Marshall function inputs
  t = emlrt_marshallIn(emlrtAliasP(prhs[0]), "t");
  x.set_owner(false);
  emlrt_marshallIn(emlrtAlias(prhs_copy_idx_1), "x", x);
  emlrt_marshallIn(emlrtAliasP(prhs[2]), "MBSys", MBSys);
  // Invoke the target function
  computeFirstOrderMassMatrix(t, x, &MBSys, M_fo);
  // Marshall function outputs
  M_fo.set_owner(false);
  *plhs = emlrt_marshallOut(M_fo);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

// End of code generation (_coder_computeFirstOrderMassMatrix_api.cpp)
