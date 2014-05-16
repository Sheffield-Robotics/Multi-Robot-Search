/*
** matrix.h
**
** matrix functions
**
** (c) Steffen Gutmann, 1995-98
**
** Creation date: 28.09.95
** Last modified: 22.10.98
*/

#ifndef MATRIX_H
#define MATRIX_H

//#include "lms200/src/scanner/misc.h"
#include <stdio.h>
#include <stdlib.h>


#ifdef	__cplusplus
extern "C" {
#endif

/* error codes */
enum
{
    MATERR_MEM = 1,
    MATERR_NOMATRIX,
    MATERR_NOSQUAREMATRIX,
    MATERR_SIZE,
    MATERR_SINGULAR
};

struct Vector2
{
    double d[2];
};

struct Vector3
{
    double d[3];
};

struct Matrix2
{
    double d[2][2];
};

struct Matrix3
{
    double d[3][3];
};

typedef double elem_t;

/* generic matrix and vector */
struct Matrix
{
    int n, m;           /* rows and cols */
    elem_t **d;    	/* pointer to rows (which itself points to elements) */
};

extern struct Vector3 Vector3Zero;
extern struct Matrix3 Matrix3Zero, Matrix3One;

void Vector2Set(struct Vector2 *v, double a, double b);
/*
** Sets vector v to a, b.
*/

void Vector2Add(struct Vector2 *a, struct Vector2 *b, struct Vector2 *c);
/*
** Adds a and b and stores the result in c, a and b are not altered.
** You may pass the same pointer more than once, e.g.
** Vector2Add(a, a, a) is allowed.
*/

void Vector2MultSkalar(struct Vector2 *v, double a);
/*
** multiplies a with each vector element.
*/

double Vector2MultVector(struct Vector2 *v1, struct Vector2 *v2);
/*
** Returns transpose(v1) * v2
*/

void Vector3Set(struct Vector3 *v, double a, double b, double c);
/*
** Sets vector v to a, b, c.
*/

void Vector3Add(struct Vector3 *a, struct Vector3 *b, struct Vector3 *c);
/*
** Adds a and b and stores the result in c, a and b are not altered.
** You may pass the same pointer more than once, e.g.
** Vector3Add(a, a, a) is allowed.
*/

void Vector3MultSkalar(struct Vector3 *v, double a);
/*
** multiplies a with each vector element.
*/

double Vector3MultVector(struct Vector3 *v1, struct Vector3 *v2);
/*
** Returns transpose(v1) * v2
*/

void Matrix2Set(struct Matrix2 *m,
        double a, double b, 
	double c, double d);
/*
** Fills in matrix structure.
*/

void Matrix2Add(struct Matrix2 *A, struct Matrix2 *B, struct Matrix2 *C);
/*
** Adds A and B, result is stored in C. A and B are not altered.
** You may pass the same pointer more than once, e.g.
** Matrix2Add(A, A, A) is allowed.
*/

int Matrix2Inverse(struct Matrix2 *m);
/*
** Inverts 2x2 Matrix m. Returns FALSE if matrix is singular.
*/

void Matrix2Transpose(struct Matrix2 *m);
/*
** Transposes 2x2 Matrix m.
*/

void Matrix2MultVector(struct Matrix2 *m, struct Vector2 *x, struct Vector2 *result);
/*
** multiplies matrix with vector x. Result is stored in result,
** m and x are not altered.
*/

void Matrix2Mult(struct Matrix2 *A, struct Matrix2 *B, struct Matrix2 *C);
/*
** Multiplies A * B, result is stored in C. A and B are not altered.
*/

void GetEllipseParams(struct Matrix2 *m, double *a, double *b, double *alpha);
/*
** calculates main axis a and b and angle to x-axis.
*/

void Matrix2DrawEllipse(long cx, long cy, long a, long b, double alpha,
    void (*drawLine)(long x1, long y1, long x2, long y2));
/*
** Draws an ellipse at (cx, cy) with axis a and b and angle to x-axis
** alpha by approximating it with line segments and calling the
** supplied drawLine function.
*/

void Matrix2Print(struct Matrix2 *m);
/*
** Prints matrix to stdout.
*/

void Matrix3Set(struct Matrix3 *m,
        double a, double b, double c,
        double d, double e, double f,
        double g, double h, double i);
/*
** Fills in matrix structure.
*/

void Matrix3Magic(struct Matrix3 *m);
/*
** clears angular covariance fields.
*/

double Matrix3Determinante(struct Matrix3 *m);
/*
** Returns determinante of matrix m.
*/

double Matrix3Trace(struct Matrix3 *m);
/*
** Returns the trace of matrix m.
*/

int Matrix3Inverse(struct Matrix3 *m);
/*
** Inverts 3x3 Matrix m. Returns FALSE if matrix is singular.
*/

void Matrix3Transpose(struct Matrix3 *m);
/*
** Transposes 3x3 Matrix m.
*/

void Matrix3Add(struct Matrix3 *A, struct Matrix3 *B, struct Matrix3 *C);
/*
** Adds A and B, result is stored in C. A and B are not altered.
** You may pass the same pointer more than once, e.g.
** Matrix3Add(A, A, A) is allowed.
*/

void Matrix3MultSkalar(struct Matrix3 *m, double a);
/*
** multiplies a with each matrix element.
*/

void Matrix3MultVector(struct Matrix3 *m, struct Vector3 *x, struct Vector3 *result);
/*
** multiplies matrix with vector x. Result is stored in result,
** m and x are not altered.
*/

void Matrix3Mult(struct Matrix3 *A, struct Matrix3 *B, struct Matrix3 *C);
/*
** Multiplies A * B, result is stored in C. A and B are not altered.
*/

void Matrix3Print(struct Matrix3 *m);
/*
** Prints matrix to stdout.
*/

/* Generic matrix functions */

struct Matrix *MatrixAlloc(int n, int m);
/*
** Allocates matrix structure and data space.
** Initial element values are undefined.
*/

void MatrixFree(struct Matrix *m);
/* Frees matrix structure and data space. */

void MatrixZero(struct Matrix *m);
/* Sets all elements to 0.0. */

int MatrixDeterminante(struct Matrix *m, elem_t *det);
/* Returns determinante of matrix m. */

int MatrixInverse(struct Matrix *m, struct Matrix *res);
/*
** Calculates inverse of matrix m.
** Result is stored in matrix res (allocated by you!).
** Warning: m will be modified!
*/

int MatrixMult(struct Matrix *A, struct Matrix *B, struct Matrix *C);
/*
** Calculates A * B (matrix multiplication and stores the result in C.
** C must be allocated by you!
*/

void MatrixPrint(struct Matrix *m, FILE *file);
/* Prints matrix to file */

char *MatError(int errnum);
/* returns error string to given error number */

#ifdef	__cplusplus
}
#endif
#endif
