/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_crossover2pt_gene_mex.c
 *
 * Code generation for function '_coder_crossover2pt_gene_mex'
 *
 */

/* Include files */
#include "crossover2pt_gene.h"
#include "_coder_crossover2pt_gene_mex.h"
#include "crossover2pt_gene_terminate.h"
#include "_coder_crossover2pt_gene_api.h"
#include "crossover2pt_gene_initialize.h"
#include "crossover2pt_gene_data.h"

/* Function Declarations */
static void crossover2pt_gene_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[1]);

/* Function Definitions */
static void crossover2pt_gene_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[1])
{
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        17, "crossover2pt_gene");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 17,
                        "crossover2pt_gene");
  }

  /* Call the function. */
  crossover2pt_gene_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  crossover2pt_gene_terminate();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(crossover2pt_gene_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  crossover2pt_gene_initialize();

  /* Dispatch the entry-point. */
  crossover2pt_gene_mexFunction(nlhs, plhs, nrhs, prhs);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_crossover2pt_gene_mex.c) */
