/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sort1.c
 *
 * Code generation for function 'sort1'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "crossover2pt_gene.h"
#include "sort1.h"

/* Function Definitions */
void sort(real_T x[200])
{
  int32_T j;
  int32_T k;
  real_T vwork[2];
  real_T tmp;
  for (j = 0; j < 100; j++) {
    for (k = 0; k < 2; k++) {
      vwork[k] = x[j + k * 100];
    }

    if ((vwork[0] <= vwork[1]) || muDoubleScalarIsNaN(vwork[1])) {
    } else {
      tmp = vwork[0];
      vwork[0] = vwork[1];
      vwork[1] = tmp;
    }

    for (k = 0; k < 2; k++) {
      x[j + k * 100] = vwork[k];
    }
  }
}

/* End of code generation (sort1.c) */
