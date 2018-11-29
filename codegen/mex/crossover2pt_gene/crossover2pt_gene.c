/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * crossover2pt_gene.c
 *
 * Code generation for function 'crossover2pt_gene'
 *
 */

/* Include files */
#include <string.h>
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "crossover2pt_gene.h"
#include "rand.h"
#include "sort1.h"
#include "crossover2pt_gene_data.h"

/* Variable Definitions */
static emlrtDCInfo emlrtDCI = { 13,    /* lineNo */
  21,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo emlrtBCI = { 1,     /* iFirst */
  15,                                  /* iLast */
  13,                                  /* lineNo */
  21,                                  /* colNo */
  "parA",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = { 13,  /* lineNo */
  37,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = { 1,   /* iFirst */
  15,                                  /* iLast */
  13,                                  /* lineNo */
  37,                                  /* colNo */
  "parB",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = { 13,  /* lineNo */
  49,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { 1,   /* iFirst */
  15,                                  /* iLast */
  13,                                  /* lineNo */
  49,                                  /* colNo */
  "parB",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = { 13,  /* lineNo */
  65,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { 1,   /* iFirst */
  15,                                  /* iLast */
  13,                                  /* lineNo */
  65,                                  /* colNo */
  "parA",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo e_emlrtDCI = { 14,  /* lineNo */
  21,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { 1,   /* iFirst */
  15,                                  /* iLast */
  14,                                  /* lineNo */
  21,                                  /* colNo */
  "parB",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = { 14,  /* lineNo */
  37,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { 1,   /* iFirst */
  15,                                  /* iLast */
  14,                                  /* lineNo */
  37,                                  /* colNo */
  "parA",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = { 14,  /* lineNo */
  49,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = { 1,   /* iFirst */
  15,                                  /* iLast */
  14,                                  /* lineNo */
  49,                                  /* colNo */
  "parA",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo h_emlrtDCI = { 14,  /* lineNo */
  65,                                  /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { 1,   /* iFirst */
  15,                                  /* iLast */
  14,                                  /* lineNo */
  65,                                  /* colNo */
  "parB",                              /* aName */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  20,                                  /* lineNo */
  9,                                   /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m"/* pName */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  21,                                  /* lineNo */
  9,                                   /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m"/* pName */
};

static emlrtECInfo c_emlrtECI = { -1,  /* nDims */
  25,                                  /* lineNo */
  9,                                   /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m"/* pName */
};

static emlrtECInfo d_emlrtECI = { -1,  /* nDims */
  26,                                  /* lineNo */
  9,                                   /* colNo */
  "crossover2pt_gene",                 /* fName */
  "C:\\Users\\Roar\\MECS4510-HW3-BouncingCube\\crossover2pt_gene.m"/* pName */
};

/* Function Definitions */
void crossover2pt_gene(const emlrtStack *sp, const real_T parents[1500], real_T
  children[1500])
{
  boolean_T b0;
  boolean_T b1;
  boolean_T b2;
  boolean_T b3;
  real_T r[100];
  int32_T k;
  real_T b_r[100];
  real_T cut[200];
  int32_T i;
  int32_T b_i;
  int32_T loop_ub;
  real_T d0;
  int32_T i0;
  int32_T i1;
  int32_T i2;
  int32_T geneC_size[1];
  int32_T i3;
  int32_T b_loop_ub;
  real_T geneC_data[45];
  int32_T geneD_size[1];
  real_T geneD_data[45];
  int32_T iv0[1];
  int32_T iv1[1];
  int32_T iv2[1];
  int32_T iv3[1];
  b0 = false;
  b1 = false;
  b2 = false;
  b3 = false;

  /*  #codegen */
  /*  This crossover function takes and returns genes, not bots */
  memset(&children[0], 0, 1500U * sizeof(real_T));
  b_rand(r);
  for (k = 0; k < 100; k++) {
    r[k] = 1.0 + muDoubleScalarFloor(r[k] * 15.0);
  }

  b_rand(b_r);
  for (k = 0; k < 100; k++) {
    b_r[k] = 1.0 + muDoubleScalarFloor(b_r[k] * 15.0);
  }

  for (k = 0; k < 100; k++) {
    cut[k] = r[k];
    cut[100 + k] = b_r[k];
  }

  sort(cut);

  /*  children = starfish_robot.empty(0,length(parents)); */
  i = 0;
  while (i < 50) {
    b_i = i << 1;

    /*      parA = parents(i).gene; */
    /*      parB = parents(i+1).gene; */
    if (1.0 > cut[b_i]) {
      loop_ub = 0;
    } else {
      if (cut[b_i] != (int32_T)muDoubleScalarFloor(cut[b_i])) {
        emlrtIntegerCheckR2012b(cut[b_i], &emlrtDCI, sp);
      }

      loop_ub = (int32_T)cut[b_i];
      if (!((loop_ub >= 1) && (loop_ub <= 15))) {
        emlrtDynamicBoundsCheckR2012b(loop_ub, 1, 15, &emlrtBCI, sp);
      }
    }

    if (cut[b_i] + 1.0 > cut[100 + b_i]) {
      k = 0;
      i0 = 0;
    } else {
      d0 = cut[b_i] + 1.0;
      if (d0 != (int32_T)muDoubleScalarFloor(d0)) {
        emlrtIntegerCheckR2012b(d0, &b_emlrtDCI, sp);
      }

      k = (int32_T)d0;
      if (!((k >= 1) && (k <= 15))) {
        emlrtDynamicBoundsCheckR2012b(k, 1, 15, &b_emlrtBCI, sp);
      }

      k--;
      d0 = cut[100 + b_i];
      if (d0 != (int32_T)muDoubleScalarFloor(d0)) {
        emlrtIntegerCheckR2012b(d0, &c_emlrtDCI, sp);
      }

      i0 = (int32_T)d0;
      if (!((i0 >= 1) && (i0 <= 15))) {
        emlrtDynamicBoundsCheckR2012b(i0, 1, 15, &c_emlrtBCI, sp);
      }
    }

    if (cut[100 + b_i] + 1.0 > 15.0) {
      i1 = 1;
      i2 = 1;
    } else {
      d0 = cut[100 + b_i] + 1.0;
      if (d0 != (int32_T)muDoubleScalarFloor(d0)) {
        emlrtIntegerCheckR2012b(d0, &d_emlrtDCI, sp);
      }

      i1 = (int32_T)d0;
      if (!((i1 >= 1) && (i1 <= 15))) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, 15, &d_emlrtBCI, sp);
      }

      i2 = 16;
    }

    geneC_size[0] = (((loop_ub + i0) - k) + i2) - i1;
    for (i3 = 0; i3 < loop_ub; i3++) {
      geneC_data[i3] = parents[i3 + 15 * b_i];
    }

    b_loop_ub = i0 - k;
    for (i3 = 0; i3 < b_loop_ub; i3++) {
      geneC_data[i3 + loop_ub] = parents[(k + i3) + 15 * (b_i + 1)];
    }

    b_loop_ub = i2 - i1;
    for (i2 = 0; i2 < b_loop_ub; i2++) {
      geneC_data[((i2 + loop_ub) + i0) - k] = parents[((i1 + i2) + 15 * b_i) - 1];
    }

    if (1.0 > cut[b_i]) {
      loop_ub = 0;
    } else {
      if (cut[b_i] != (int32_T)muDoubleScalarFloor(cut[b_i])) {
        emlrtIntegerCheckR2012b(cut[b_i], &e_emlrtDCI, sp);
      }

      loop_ub = (int32_T)cut[b_i];
      if (!((loop_ub >= 1) && (loop_ub <= 15))) {
        emlrtDynamicBoundsCheckR2012b(loop_ub, 1, 15, &e_emlrtBCI, sp);
      }
    }

    if (cut[b_i] + 1.0 > cut[100 + b_i]) {
      k = 0;
      i0 = 0;
    } else {
      d0 = cut[b_i] + 1.0;
      if (d0 != (int32_T)muDoubleScalarFloor(d0)) {
        emlrtIntegerCheckR2012b(d0, &f_emlrtDCI, sp);
      }

      k = (int32_T)d0;
      if (!((k >= 1) && (k <= 15))) {
        emlrtDynamicBoundsCheckR2012b(k, 1, 15, &f_emlrtBCI, sp);
      }

      k--;
      d0 = cut[100 + b_i];
      if (d0 != (int32_T)muDoubleScalarFloor(d0)) {
        emlrtIntegerCheckR2012b(d0, &g_emlrtDCI, sp);
      }

      i0 = (int32_T)d0;
      if (!((i0 >= 1) && (i0 <= 15))) {
        emlrtDynamicBoundsCheckR2012b(i0, 1, 15, &g_emlrtBCI, sp);
      }
    }

    if (cut[100 + b_i] + 1.0 > 15.0) {
      i1 = 1;
      i2 = 1;
    } else {
      d0 = cut[100 + b_i] + 1.0;
      if (d0 != (int32_T)muDoubleScalarFloor(d0)) {
        emlrtIntegerCheckR2012b(d0, &h_emlrtDCI, sp);
      }

      i1 = (int32_T)d0;
      if (!((i1 >= 1) && (i1 <= 15))) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, 15, &h_emlrtBCI, sp);
      }

      i2 = 16;
    }

    geneD_size[0] = (((loop_ub + i0) - k) + i2) - i1;
    for (i3 = 0; i3 < loop_ub; i3++) {
      geneD_data[i3] = parents[i3 + 15 * (b_i + 1)];
    }

    b_loop_ub = i0 - k;
    for (i3 = 0; i3 < b_loop_ub; i3++) {
      geneD_data[i3 + loop_ub] = parents[(k + i3) + 15 * b_i];
    }

    b_loop_ub = i2 - i1;
    for (i2 = 0; i2 < b_loop_ub; i2++) {
      geneD_data[((i2 + loop_ub) + i0) - k] = parents[((i1 + i2) + 15 * (b_i + 1))
        - 1];
    }

    /* Child is placed in same index as more similar parent */
    if (cut[100 + b_i] - cut[b_i] < 8.0) {
      /*          children(i) = starfish_robot(geneC); */
      /*          children(i+1) = starfish_robot(geneD); */
      if (!b0) {
        iv1[0] = 15;
        b0 = true;
      }

      emlrtSubAssignSizeCheckR2012b(&iv1[0], 1, &geneC_size[0], 1, &emlrtECI, sp);
      memcpy(&children[b_i * 15], &geneC_data[0], 15U * sizeof(real_T));
      if (!b1) {
        iv3[0] = 15;
        b1 = true;
      }

      emlrtSubAssignSizeCheckR2012b(&iv3[0], 1, &geneD_size[0], 1, &b_emlrtECI,
        sp);
      memcpy(&children[b_i * 15 + 15], &geneD_data[0], 15U * sizeof(real_T));
    } else {
      /*          children(i) = starfish_robot(geneD); */
      /*          children(i+1) = starfish_robot(geneC); */
      if (!b2) {
        iv0[0] = 15;
        b2 = true;
      }

      emlrtSubAssignSizeCheckR2012b(&iv0[0], 1, &geneD_size[0], 1, &c_emlrtECI,
        sp);
      memcpy(&children[b_i * 15], &geneD_data[0], 15U * sizeof(real_T));
      if (!b3) {
        iv2[0] = 15;
        b3 = true;
      }

      emlrtSubAssignSizeCheckR2012b(&iv2[0], 1, &geneC_size[0], 1, &d_emlrtECI,
        sp);
      memcpy(&children[b_i * 15 + 15], &geneC_data[0], 15U * sizeof(real_T));
    }

    i++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }
}

/* End of code generation (crossover2pt_gene.c) */
