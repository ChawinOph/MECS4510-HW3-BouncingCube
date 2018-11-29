/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * crossover2pt_gene.h
 *
 * Code generation for function 'crossover2pt_gene'
 *
 */

#ifndef CROSSOVER2PT_GENE_H
#define CROSSOVER2PT_GENE_H

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "crossover2pt_gene_types.h"

/* Function Declarations */
extern void crossover2pt_gene(const emlrtStack *sp, const real_T parents[1500],
  real_T children[1500]);

#endif

/* End of code generation (crossover2pt_gene.h) */
