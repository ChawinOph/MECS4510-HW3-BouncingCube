/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_crossover2pt_gene_api.c
 *
 * Code generation for function '_coder_crossover2pt_gene_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "crossover2pt_gene.h"
#include "_coder_crossover2pt_gene_api.h"
#include "crossover2pt_gene_data.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[1500];
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[1500];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *parents,
  const char_T *identifier))[1500];
static const mxArray *emlrt_marshallOut(const real_T u[1500]);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[1500]
{
  real_T (*y)[1500];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[1500]
{
  real_T (*ret)[1500];
  static const int32_T dims[2] = { 15, 100 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[1500])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *parents,
  const char_T *identifier))[1500]
{
  real_T (*y)[1500];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(parents), &thisId);
  emlrtDestroyArray(&parents);
  return y;
}
  static const mxArray *emlrt_marshallOut(const real_T u[1500])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv4[2] = { 0, 0 };

  static const int32_T iv5[2] = { 15, 100 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv4, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, iv5, 2);
  emlrtAssign(&y, m0);
  return y;
}

void crossover2pt_gene_api(const mxArray * const prhs[1], int32_T nlhs, const
  mxArray *plhs[1])
{
  real_T (*children)[1500];
  real_T (*parents)[1500];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  (void)nlhs;
  st.tls = emlrtRootTLSGlobal;
  children = (real_T (*)[1500])mxMalloc(sizeof(real_T [1500]));

  /* Marshall function inputs */
  parents = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "parents");

  /* Invoke the target function */
  crossover2pt_gene(&st, *parents, *children);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*children);
}

/* End of code generation (_coder_crossover2pt_gene_api.c) */
