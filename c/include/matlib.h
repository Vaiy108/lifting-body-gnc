/* matlib.h -- fixed-size dense matrix operations, no dynamic allocation.
 *
 * All matrices are caller-allocated double[MATLIB_MAXN][MATLIB_MAXN]
 * (or smaller, sub-addressed); dimensions are passed explicitly.
 * Sized for the 15-state ESKF (N_ERR=15) with headroom.
 */
#ifndef MATLIB_H
#define MATLIB_H

#ifdef __cplusplus
extern "C" {
#endif

#define MATLIB_MAXN 16

void mat_zero(double *M, int rows, int cols, int stride);
void mat_eye(double *M, int n, int stride);
void mat_mult(const double *A, int ar, int ac, int astride,
             const double *B, int br, int bc, int bstride,
             double *C, int cstride);
void mat_transpose(const double *A, int ar, int ac, int astride,
                   double *At, int atstride);
void mat_add(const double *A, const double *B, double *C, int r, int c,
            int astride, int bstride, int cstride);
void mat_sub(const double *A, const double *B, double *C, int r, int c,
            int astride, int bstride, int cstride);
void mat_scale(const double *A, double s, double *C, int r, int c,
              int astride, int cstride);

/* Solve A x = b for square A (n x n), via Gauss-Jordan with partial
 * pivoting. A is overwritten (not preserved). Returns 0 on success,
 * -1 if singular (pivot below threshold). No allocation. */
int mat_solve(double A[MATLIB_MAXN][MATLIB_MAXN], int n,
             double b[MATLIB_MAXN][MATLIB_MAXN], int bcols);

#ifdef __cplusplus
}
#endif

#endif /* MATLIB_H */
