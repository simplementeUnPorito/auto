#ifndef MAT_H
#define MAT_H

#define MAX 10

// ===== Estructura de Matriz =====
typedef struct {
    int filas;
    int columnas;
    double d[MAX][MAX];
} Mat;

// ===== Prototipos =====
void mat_zero_volatile(volatile Mat *M, int r, int c);
void mat_zero(Mat *M, int r, int c);
void mat_copy(Mat *dst, const Mat *src);
void mat_copy_volatile(Mat *dst, volatile Mat *src);
void mat_add(const Mat *A, const Mat *B, Mat *C);
void mat_sut(const Mat *A, const Mat *B, Mat *C);
void mat_mul(const Mat *A, const Mat *B, Mat *C);
void ss_step(const Mat *A, const Mat *B, const Mat *C, const Mat *D,
             const Mat *xk, const Mat *uk, Mat *xk1, Mat *yk);

#endif // MAT_H
