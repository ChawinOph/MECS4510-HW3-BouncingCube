/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * crossover2pt_gene_terminate.c
 *
 * Code generation for function 'crossover2pt_gene_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "crossover2pt_gene.h"
#include "crossover2pt_gene_terminate.h"
#include "_coder_crossover2pt_gene_mex.h"
#include "crossover2pt_gene_data.h"

/* Function Definitions */
void crossover2pt_gene_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void crossover2pt_gene_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (crossover2pt_gene_terminate.c) */
