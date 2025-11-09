#include "mat.h"


void mat_zero_volatile(volatile Mat *M, int r, int c) {
    M->filas = r; M->columnas = c;
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            M->d[i][j] = 0.0;
}


// ===== Implementaciones =====
void mat_zero(Mat *M, int r, int c) {
    M->filas = r; M->columnas = c;
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            M->d[i][j] = 0.0;
}

void mat_copy(Mat *dst, const Mat *src) {
    dst->filas = src->filas; dst->columnas = src->columnas;
    for (int i = 0; i < src->filas; i++)
        for (int j = 0; j < src->columnas; j++)
            dst->d[i][j] = src->d[i][j];
}


void mat_copy_volatile(Mat *dst, volatile Mat *src) {
    dst->filas = src->filas; dst->columnas = src->columnas;
    for (int i = 0; i < src->filas; i++)
        for (int j = 0; j < src->columnas; j++)
            dst->d[i][j] = src->d[i][j];
}

void mat_add(const Mat *A, const Mat *B, Mat *C) {
    C->filas = A->filas; C->columnas = A->columnas;
    for (int i = 0; i < A->filas; i++)
        for (int j = 0; j < A->columnas; j++)
            C->d[i][j] = A->d[i][j] + B->d[i][j];
}

void mat_sut(const Mat *A, const Mat *B, Mat *C) {
    C->filas = A->filas; C->columnas = A->columnas;
    for (int i = 0; i < A->filas; i++)
        for (int j = 0; j < A->columnas; j++)
            C->d[i][j] = A->d[i][j] - B->d[i][j];
}

void mat_mul(const Mat *A, const Mat *B, Mat *C) {
    C->filas = A->filas; C->columnas = B->columnas;
    for (int i = 0; i < A->filas; i++) {
        for (int j = 0; j < B->columnas; j++) {
            double acc = 0.0;
            for (int k = 0; k < A->columnas; k++)
                acc += A->d[i][k] * B->d[k][j];
            C->d[i][j] = acc;
        }
    }
}

void ss_step(const Mat *A, const Mat *B, const Mat *C, const Mat *D,
             const Mat *xk, const Mat *uk, Mat *xk1, Mat *yk)
{
    Mat Ax, Bu, Cx, Du;
    mat_mul(A, xk, &Ax);
    mat_mul(B, uk, &Bu);
    mat_add(&Ax, &Bu, xk1);

    mat_mul(C, xk, &Cx);
    mat_mul(D, uk, &Du);
    mat_add(&Cx, &Du, yk);
}
