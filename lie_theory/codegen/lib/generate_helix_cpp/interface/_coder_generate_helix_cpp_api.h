//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_generate_helix_cpp_api.h
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 16-Feb-2023 15:04:13
//

#ifndef _CODER_GENERATE_HELIX_CPP_API_H
#define _CODER_GENERATE_HELIX_CPP_API_H

// Include Files
#include "coder_array_mex.h"
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void generate_helix_cpp(real_T a, real_T b, real_T dt, real_T b_time,
                        coder::array<real_T, 3U> *poses);

void generate_helix_cpp_api(const mxArray *const prhs[4], const mxArray **plhs);

void generate_helix_cpp_atexit();

void generate_helix_cpp_initialize();

void generate_helix_cpp_terminate();

void generate_helix_cpp_xil_shutdown();

void generate_helix_cpp_xil_terminate();

#endif
//
// File trailer for _coder_generate_helix_cpp_api.h
//
// [EOF]
//
