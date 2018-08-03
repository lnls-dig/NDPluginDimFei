
#pragma once

#define lapack_complex_float std::complex<float>
#define lapack_complex_double std::complex<double>

#include "lapacke.h"


int solve_lin_sys(int n, float* A, float* b, float* x);
