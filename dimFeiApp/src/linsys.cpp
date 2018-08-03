#include "stdafx.h"
#include "linsys.h"

int solve_lin_sys(int n, float* A, float* b, float* x)
{
    const lapack_int num = n;
    const lapack_int nrhs = 1;
    const lapack_int lda = num;
    const lapack_int ldb = nrhs;

    int i;
    lapack_int info;
    lapack_int* ipiv = new lapack_int[num];

    float* a_lapacke = new float[num*lda];
    float* b_lapacke = new float[num*nrhs];

    for(i=0; i<num*lda; ++i)
        a_lapacke[i] = A[i];
    for(i=0; i<num*nrhs; ++i)
        b_lapacke[i] = b[i];

    info = LAPACKE_sgesv(LAPACK_ROW_MAJOR, n, nrhs, a_lapacke, lda, ipiv, b_lapacke, ldb);

    for(i=0; i<num*nrhs; ++i)
        x[i] = b_lapacke[i];

    delete[] ipiv;
    delete[] a_lapacke;
    delete[] b_lapacke;

    return info;
};
