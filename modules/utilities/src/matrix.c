/*
** matrix.c
**
** matrix functions
**
** (c) Steffen Gutmann, 1995-98
**
** Creation date: 28.09.95
** Last modified: 27.03.98
*/

//#include "laser/src/scanner/misc.h"
#include "utilities/math/matrix.h"
#include <math.h>

#ifndef NEAR_ZERO
#define NEAR_ZERO(x) (ABS(x) < 1.0e-12)
#endif

#ifndef ABS
#define ABS(x) (((x) >= 0)? (x) : -(x))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b))? (a) : (b))
#endif

#define LARGE_VAL 1e20

#define GETDATA2(m) \
        a = m->d[0][0]; \
        b = m->d[0][1]; \
        c = m->d[1][0]; \
        d = m->d[1][1];

#define GETDATA3(m) \
        a = m->d[0][0]; \
        b = m->d[0][1]; \
        c = m->d[0][2]; \
        d = m->d[1][0]; \
        e = m->d[1][1]; \
        f = m->d[1][2]; \
        g = m->d[2][0]; \
        h = m->d[2][1]; \
        i = m->d[2][2];

struct Vector3 Vector3Zero =
{
    { 0.0, 0.0, 0.0, },
};

struct Matrix3 Matrix3Zero =
{
    { 
	{0.0, 0.0, 0.0,},
    	{0.0, 0.0, 0.0,},
    	{0.0, 0.0, 0.0,},
    }
};

struct Matrix3 Matrix3One =
{
    {
	{1.0, 0.0, 0.0,},
	{0.0, 1.0, 0.0,},
	{0.0, 0.0, 1.0,},
    }
};

void Vector2Set(struct Vector2 *v, double a, double b)
{
    if(v != NULL)
    {
	v->d[0] = a;
	v->d[1] = b;
    }
}

void Vector2Add(struct Vector2 *a, struct Vector2 *b, struct Vector2 *c)
{
    int i;

    if(a == NULL || b == NULL || c == NULL)
        return;

    for(i=0; i<2; i++)
        c->d[i] = a->d[i] + b->d[i];
}

void Vector3Set(struct Vector3 *v, double a, double b, double c)
{
    if(v != NULL)
    {
	v->d[0] = a;
	v->d[1] = b;
	v->d[2] = c;
    }
}

void Vector2MultSkalar(struct Vector2 *v, double a)
{
    int i;

    if(v != NULL)
	for(i=0; i < 2; i++)
	    v->d[i] *= a;
}

double Vector2MultVector(struct Vector2 *v1, struct Vector2 *v2)
{
    double res = 0.0;
    int i;

    if(v1 && v2)
        for(i=0; i < 2; i++)
            res += v1->d[i] * v2->d[i];

    return(res);
}


void Vector3Add(struct Vector3 *a, struct Vector3 *b, struct Vector3 *c)
{
    int i;

    if(a == NULL || b == NULL || c == NULL)
        return;

    for(i=0; i<3; i++)
        c->d[i] = a->d[i] + b->d[i];
}

void Vector3MultSkalar(struct Vector3 *v, double a)
{
    int i;

    if(v != NULL)
	for(i=0; i < 3; i++)
	    v->d[i] *= a;
}
    

double Vector3MultVector(struct Vector3 *v1, struct Vector3 *v2)
{
    double res = 0.0;
    int i;

    if(v1 && v2)
        for(i=0; i < 3; i++)
            res += v1->d[i] * v2->d[i];

    return(res);
}

void Matrix2Set(struct Matrix2 *m,
        double a, double b, double c, double d)
{
    if(m == NULL)
        return;

    m->d[0][0] = a;
    m->d[0][1] = b;
    m->d[1][0] = c;
    m->d[1][1] = d;
}

int Matrix2Inverse(struct Matrix2 *m)
{
    double a, b, c, d;
    double det;

    if(m == NULL)
        return(0);

    GETDATA2(m);                /* loads a - d */

    det = a*d - b*c;
    if(NEAR_ZERO(det))
    {
	m->d[0][0] = m->d[1][1] = LARGE_VAL;
	m->d[1][0] = m->d[0][1] = 0.0;
        return(0);
    }
	
    m->d[0][0] = d / det;
    m->d[0][1] = -b / det;
    m->d[1][0] = -c / det;
    m->d[1][1] = a / det;

    return(1);
}

void Matrix2Transpose(struct Matrix2 *m)
{
    double a, b, c, d;

    if(m == NULL)
        return;

    GETDATA2(m);                /* loads a - d */

    m->d[0][1] = c;
    m->d[1][0] = b;
}

void Matrix2Add(struct Matrix2 *A, struct Matrix2 *B, struct Matrix2 *C)
{
    int i, j;

    if(A == NULL || B == NULL || C == NULL)
        return;

    for(i=0; i<2; i++)
        for(j=0; j<2; j++)
            C->d[i][j] = A->d[i][j] + B->d[i][j];
}

void Matrix2MultVector(struct Matrix2 *m, struct Vector2 *x, struct Vector2 *result)
{
    int i, j;

    if(m == NULL || x == NULL || result == NULL)
        return;

    for(i=0; i<2; i++)
    {
        result->d[i] = 0.0;

        for(j=0; j<2; j++)
            result->d[i] += m->d[i][j] * x->d[j];
    }
}

void Matrix2Mult(struct Matrix2 *A, struct Matrix2 *B, struct Matrix2 *C)
{
    int i, j, k;
    double val;

    if(A == NULL || B == NULL || C == NULL)
        return;

    for(i=0; i < 2; i++)
        for(j=0; j < 2; j++)
        {
            val = 0.0;
            for(k=0; k < 2; k++)
                val += A->d[i][k] * B->d[k][j];
            C->d[i][j] = val;
        }
}

void GetEllipseParams(struct Matrix2 *m, double *pa, double *pb, double *palpha)
{
    double a, b, c, d;
    double h, lambda1, lambda2;
    double alpha;

    GETDATA2(m);                        /* load a - d */

    /* get eigen-values */
    h = sqrt((a - d)*(a - d)/4.0 + b*b);
    lambda1 = (a + d)/2.0 - h;
    lambda2 = (a + d)/2.0 + h;

#if 0
    if(lambda1 < 0.0)
        fprintf(stderr, "GetEllipseParams: lambda1 < 0!\n");
    if(lambda2 < 0.0)
        fprintf(stderr, "GetEllipseParams: lambda2 < 0!\n");
#endif

    alpha = atan2(lambda1 - a, b);

    if(pa && lambda1 != 0.0)
        *pa = 1 / sqrt(lambda1);
    if(pb && lambda2 != 0.0)
        *pb = 1 / sqrt(lambda2);
    if(palpha)
        *palpha = alpha;
}

void Matrix2DrawEllipse(long cx, long cy, long a, long b, double alpha,
    void (*drawLine)(long x1, long y1, long x2, long y2))
{
    double sina, cosa;
    long p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y;
    long qx, qy;
    double x, y;
    double step;
    int help;

    if(drawLine == NULL)
	return;

    if(b > a)
    {
	help = a;
	a = b;
	b = help;
	alpha += M_PI/2;
    }

    sina = sin(alpha);
    cosa = cos(alpha);

    if(a == 0)
    {
	/* ellipse is only a point */

	drawLine(cx, cy, cx, cy);
	return;
    }

    p1x = p2x = (long) (cx - a * cosa);
    p1y = p2y = (long) (cy - a * sina);
    p3x = p4x = (long) (cx + a * cosa);
    p3y = p4y = (long) (cy + a * sina);

    step = a / 50.0;
    if(step < 1.0)
	step = 1.0;

    for(x = -(double)a; ; x = MIN(x + step, 0.0))
    {
	y = sqrt(b*b * (1.0 - (x/a)*(x/a)));

	qx = (long) (cx + x*cosa - y*sina);
	qy = (long) (cy + x*sina + y*cosa);
	drawLine(p1x, p1y, qx, qy);
	p1x = qx;
	p1y = qy;

	qx = (long) (cx + x*cosa + y*sina);
	qy = (long) (cy + x*sina - y*cosa);
	drawLine(p2x, p2y, qx, qy);
	p2x = qx;
	p2y = qy;

	qx = (long) (cx - x*cosa - y*sina);
	qy = (long) (cy - x*sina + y*cosa);
	drawLine(p3x, p3y, qx, qy);
	p3x = qx;
	p3y = qy;

	qx = (long) (cx - x*cosa + y*sina);
	qy = (long) (cy - x*sina - y*cosa);
	drawLine(p4x, p4y, qx, qy);
	p4x = qx;
	p4y = qy;

	if(x >= 0.0)
	    break;
    }
}

void Matrix2Print(struct Matrix2 *m)
{
    int i, j;

    if(m != NULL)
    {
        for(i=0; i < 2; i++)
        {
            for(j=0; j < 2; j++)
                printf("%15g\t", m->d[i][j]);
            printf("\n");
        }
        printf("\n");
    }
}

void Matrix3Set(struct Matrix3 *m,
        double a, double b, double c,
        double d, double e, double f,
        double g, double h, double i)
{
    if(m == NULL)
        return;

    m->d[0][0] = a;
    m->d[0][1] = b;
    m->d[0][2] = c;
    m->d[1][0] = d;
    m->d[1][1] = e;
    m->d[1][2] = f;
    m->d[2][0] = g;
    m->d[2][1] = h;
    m->d[2][2] = i;
}

void Matrix3Magic(struct Matrix3 *m)
{
    if(m == NULL)
	return;
  
#if 0
    m->d[0][2] = 0.0;
    m->d[1][2] = 0.0;
    m->d[2][0] = 0.0;
    m->d[2][1] = 0.0;
#endif
#if 0
    m->d[0][1] = 0.0;
    m->d[1][0] = 0.0;
#endif
}

double Matrix3Determinante(struct Matrix3 *m)
{
    double a, b, c, d, e, f, g, h, i;
    double det;

    if(m == NULL)
        return(0.0);

    GETDATA3(m);                /* loads a - i */

    /*  
    ** we use the bottom line since for position covarianve
    ** matries there are often extreme numbers here.
    */
    det = g*(b*f - c*e) - h*(a*f - c*d) + i*(a*e - b*d);
    return(det);
}

double Matrix3Trace(struct Matrix3 *m)
{
    double a, b, c, d, e, f, g, h, i;
    double trace;

    if(m == NULL)
	return(0.0);

    GETDATA3(m);                /* loads a - i */

    trace = a * e * i;
    return(trace);
}

int Matrix3Inverse(struct Matrix3 *m)
{
    double a, b, c, d, e, f, g, h, i;
    double det = Matrix3Determinante(m);

    if(m == NULL)
        return(0);

    GETDATA3(m);                /* loads a - i */

    if(NEAR_ZERO(det))
    {
	/* printf("Matrix3Inverse: det = %g\n", (double)det); */
	Matrix3Set(m, 
	    1.0 / a, 0.0, 0.0,
	    0.0, 1.0 / e, 0.0,
	    0.0, 0.0, 1.0 / i);
	return(0);
    }
    else if(isnan(det))
    {
	printf("Matrix3Inverse: det = %g\n", (double)det);
	*m = Matrix3Zero;
	return(0);
    }

    m->d[0][0] = +(e*i - f*h) / det;
    m->d[0][1] = -(b*i - c*h) / det;
    m->d[0][2] = +(b*f - c*e) / det;
    m->d[1][0] = -(d*i - f*g) / det;
    m->d[1][1] = +(a*i - c*g) / det;
    m->d[1][2] = -(a*f - c*d) / det;
    m->d[2][0] = +(d*h - e*g) / det;
    m->d[2][1] = -(a*h - b*g) / det;
    m->d[2][2] = +(a*e - b*d) / det;

    return(1);
}

void Matrix3Transpose(struct Matrix3 *m)
{
    double a, b, c, d, e, f, g, h, i;

    if(m == NULL)
        return;

    GETDATA3(m);                /* loads a - i */

    m->d[0][1] = d;
    m->d[0][2] = g;
    m->d[1][0] = b;
    m->d[1][2] = h;
    m->d[2][0] = c;
    m->d[2][1] = f;
}

void Matrix3Add(struct Matrix3 *A, struct Matrix3 *B, struct Matrix3 *C)
{
    int i, j;

    if(A == NULL || B == NULL || C == NULL)
        return;

    for(i=0; i<3; i++)
        for(j=0; j<3; j++)
            C->d[i][j] = A->d[i][j] + B->d[i][j];
}

void Matrix3MultSkalar(struct Matrix3 *m, double a)
{
    if(m && a != 1.0)
    {
        int i, j;

        for(i=0; i<3; i++)
            for(j=0; j<3; j++)
                m->d[i][j] *= a;
    }
}

void Matrix3MultVector(struct Matrix3 *m, struct Vector3 *x, struct Vector3 *result)
{
    int i, j;

    if(m == NULL || x == NULL || result == NULL)
        return;

    for(i=0; i<3; i++)
    {
        result->d[i] = 0.0;

        for(j=0; j<3; j++)
            result->d[i] += m->d[i][j] * x->d[j];
    }
}

/*
** multiplies matrix with vector x. Result is stored in result,
** m and x are not altered.
*/

void Matrix3Mult(struct Matrix3 *A, struct Matrix3 *B, struct Matrix3 *C)
{
    int i, j, k;
    double val;

    if(A == NULL || B == NULL || C == NULL)
        return;

    for(i=0; i < 3; i++)
        for(j=0; j < 3; j++)
        {
            val = 0.0;
            for(k=0; k < 3; k++)
                val += A->d[i][k] * B->d[k][j];
            C->d[i][j] = val;
        }
}

void Matrix3Print(struct Matrix3 *m)
{
    int i, j;

    if(m != NULL)
    {
        for(i=0; i < 3; i++)
        {
            for(j=0; j < 3; j++)
                printf("%15g\t", m->d[i][j]);
            printf("\n");
        }
        printf("\n");
    }
}

/*
** Generic matrix functions
*/

void MatrixFree(struct Matrix *matrix)
{
    if(matrix != NULL)
    {
        int i;

        if(matrix->d != NULL)
        {
            for(i=0; i < matrix->n; i++)
                if(matrix->d[i] != NULL)
                    free(matrix->d[i]);
            free(matrix->d);
        }
        free(matrix);
    }
}

struct Matrix *MatrixAlloc(int n, int m)
{
    struct Matrix *matrix = NULL;
    int i;

    matrix = malloc(sizeof(struct Matrix));
    if(matrix == NULL)
        return(NULL);

    matrix->n = n;
    matrix->m = m;
    matrix->d = malloc(n * sizeof(elem_t *));
    if(matrix->d == NULL)
    {
        MatrixFree(matrix);
        return(NULL);
    }
    for(i=0; i<n; i++)
        matrix->d[i] = NULL;

    for(i=0; i<n; i++)
    {
        matrix->d[i] = malloc(m * sizeof(elem_t));
        if(matrix->d[i] == NULL)
        {
            MatrixFree(matrix);
            return(NULL);
        }
    }
    return(matrix);
}

void MatrixZero(struct Matrix *matrix)
{
    if(matrix != NULL)
    {
        int i, j;

        for(i=0; i < matrix->n; i++)
            for(j=0; j < matrix->m; j++)
                matrix->d[i][j] = 0.0;
    }
}

void MatrixOne(struct Matrix *matrix)
{
    if(matrix != NULL)
    {
        int i, n = MIN(matrix->n, matrix->m);

        MatrixZero(matrix);
        for(i=0; i < n; i++)
            matrix->d[i][i] = 1.0;
    }
}

void MatrixAdd(struct Matrix *A, struct Matrix *B, struct Matrix *C)
{
    int i, j, n, m;

    if(A == NULL || B == NULL || C == NULL)
        return;
    if(A->n != B->n || A->n != C->n)
	return;
    if(A->m != B->m || A->n != C->m)
	return;

    n = A->n;
    m = A->m;

    for(i=0; i<n; i++)
        for(j=0; j<m; j++)
            C->d[i][j] = A->d[i][j] + B->d[i][j];
}

static int finditem(int item, int *list, int size)
{
    int i;

    for(i=0; i < size; i++, list++)
        if(*list == item)
            return(1);
    return(0);
}

static elem_t MatrixSubDeterminante(struct Matrix *m, int n,
        int *deli, int *delj)
/*
** Calculates determinante of a sub matrix of m which has the size size n x n.
** deli specifies the rows to ignore, delj the cols to ignore.
** (There are exactly (m->n - n) rows and cols to ignore.
**
** Reference: U. Stammbach: Lineare Algebra, p. 136
*/
{
    int i, j, deln = m->n - n;
    elem_t sum = 0.0, sign = 1.0;

    if(n < 1)                   /* end of recursion */
        return(1.0);

    /*
    ** Recursive part.
    ** First thing is to determine a free row for the
    ** determinante evaluation.
    */
    for(i=0; i < m->n; i++)
        if(!finditem(i, deli, deln))
            break;

    /*
    ** Now loop through the row.
    */
    for(j=0; j < m->n; j++)
    {
        if(!finditem(j, delj, deln))
        {
            deli[deln] = i;
            delj[deln] = j;
            sum += sign * m->d[i][j] * MatrixSubDeterminante(m, n-1, deli, delj);
            sign = -sign;
        }
    }

    return(sum);
}

int MatrixDeterminante(struct Matrix *m, elem_t *det)
{
    int *deli = NULL, *delj = NULL;
    int n, err = 0;
    elem_t rc;

    if(m == NULL)
        return(MATERR_NOMATRIX);
    if(m->n != m->m)
        return(MATERR_NOSQUAREMATRIX);

    n = m->n;
    deli = malloc(n * sizeof(int));
    delj = malloc(n * sizeof(int));

    if(deli != NULL && delj != NULL)
    {
        rc = MatrixSubDeterminante(m, n, deli, delj);
        if(det != NULL)
            *det = rc;
    }
    else
        err = MATERR_MEM;

    if(delj != NULL)
        free(delj);
    if(deli != NULL)
        free(deli);

    return(err);
}

#define LUDECOMPOSE

#ifdef LUDECOMPOSE
/* 
** Algorithm from Recipes in C.
** Modified for our purpose (index from 0...(n-1), types)
*/

static int ludcmp(elem_t **a, int n, int *indx, elem_t *d)
{
    int i, imax, j, k; 
    elem_t big, dum, sum, temp;
    elem_t *vv;

    /* vv stores the implicit scaling of each row. */
    vv = malloc(n * sizeof(elem_t));	
    if(vv == NULL)
	return(MATERR_MEM);
   
    *d = 1.0;			/* no row interchanges yet. */

    /* Loop over rows to get the implicit scaling information. */
    for(i=0; i < n; i++)
    {
	big = 0.0;
	for(j=0; j < n; j++)
	    if((temp = fabs(a[i][j])) > big)
		big = temp;
	if(big == 0.0)
	{
	    free(vv);
	    return(MATERR_SINGULAR);
	}
	vv[i] = 1.0 / big;		/* save the scaling. */
    }

    for(j=0; j < n; j++)		/* loop over columns */
    {
	for(i=0; i < j; i++)		
	{
	    sum = a[i][j];
	    for(k=0; k < i; k++)
		sum -= a[i][k] * a[k][j];
	    a[i][j] = sum;
	}
	big = 0.0;			/* search for largest pivot element */
	imax = j;
	for(i=j; i < n; i++)
	{
	    sum = a[i][j];
	    for(k=0; k < j; k++)
		sum -= a[i][k] * a[k][j];
	    a[i][j] = sum;
	    if((dum = vv[i] * fabs(sum)) >= big)
	    {
		/* figure of merit for pivot is better than the best so far */
		big = dum;
		imax = i;
	    }
	}
	if(j != imax)		/* do we need to interchange rows? */
	{
	    for(k=0; k < n; k++)
	    {
		dum = a[imax][k];
		a[imax][k] = a[j][k];
		a[j][k] = dum;
	    }
	    *d = -(*d);		/* change the parity of d. */
	    vv[imax] = vv[j];	/* also interchange the scale factor. */
	}
	indx[j] = imax;
	if(a[j][j] == 0.0)
 	{
	    free(vv);
	    return(MATERR_SINGULAR);
	}

	if(j != n-1)
	{
	    dum = 1.0 / a[j][j];
	    for(i=j+1; i < n; i++)
		a[i][j] *= dum;
	}
    }
    free(vv);
    return(0);
}

static void lubksb(elem_t **a, int n, int *indx, elem_t *b)
{
    int i, ii=-1, ip, j;
    elem_t sum;

    for(i=0; i < n; i++)
    {
	ip = indx[i];
	sum = b[ip];
	b[ip] = b[i];
	if(ii >= 0)
	    for(j=ii; j <= i-1; j++)
		sum -= a[i][j] * b[j];
	else if(sum)
	    ii = i;
	b[i] = sum;
    }
    for(i=n-1; i>=0; i--)
    {
	sum = b[i];
	for(j=i+1; j < n; j++)
	    sum -= a[i][j] * b[j];
	b[i] = sum / a[i][i];
    }
}

int MatrixInverse(struct Matrix *m, struct Matrix *res)
{
    int n, rc = 0;
    struct Matrix *copy = NULL;
    elem_t d, *col = NULL;
    int i, j, *indx = NULL;

    if(m == NULL || res == NULL)
        return(MATERR_NOMATRIX);
    if(m->n != m->m || res->n != res->m)
        return(MATERR_NOSQUAREMATRIX);
    if(m->n != res->n)
        return(MATERR_SIZE);

    n = m->n;

    copy = MatrixAlloc(n, n);
    col = malloc(n * sizeof(elem_t));
    indx = malloc(n * sizeof(int));
    if(copy == NULL || indx == NULL)
	rc = MATERR_MEM;

    if(rc == 0)
    {
	MatrixZero(copy);
	MatrixAdd(copy, m, copy);

	rc = ludcmp(copy->d, n, indx, &d);
    }

    if(rc == 0)
    {
	for(j=0; j < n; j++)
	{
	    for(i=0; i < n; i++)
		col[i] = 0.0;
	    col[j] = 1.0;
	    lubksb(copy->d, n, indx, col);
	    for(i=0; i < n; i++)
		res->d[i][j] = col[i];
	}
    }

    if(indx != NULL)
	free(indx);
    if(col != NULL)
	free(col);
    MatrixFree(copy);
    
    return(rc);
}

#elif (defined GAUSS_ELIMINATION_WITH_SIMPLE_PIVOTING)
int MatrixInverse(struct Matrix *m, struct Matrix *res)
{
    int n, i, j, k, pivot;
    elem_t *help, fac, pivot_val;

    if(m == NULL || res == NULL)
        return(MATERR_NOMATRIX);
    if(m->n != m->m || res->n != res->m)
        return(MATERR_NOSQUAREMATRIX);
    if(m->n != res->n)
        return(MATERR_SIZE);

    n = m->n;
    MatrixOne(res);

    /*                  /1 * *\
    ** bring matrix on  |0 1 *| form
    **                  \0 0 1/
    */
    for(j=0; j < n; j++)                /* loop through cols */
    {
        /* find row with maximum value in col j (pivot) */
        pivot_val = 0.0;
        for(i=j; i < n; i++)
            if(fabs(m->d[i][j]) > pivot_val)
            {
                pivot_val = fabs(m->d[i][j]);
                pivot = i;
            }
        if(NEAR_ZERO(pivot_val))
            return(MATERR_SINGULAR);    /* can't calculate inverse matrix */
        if(pivot != j)
        {
            /* exchange rows */
            help = m->d[pivot];
            m->d[pivot] = m->d[j];
            m->d[j] = help;
            help = res->d[pivot];
            res->d[pivot] = res->d[j];
            res->d[j] = help;
        }

        /* Divide row by pivot element */
        fac = 1.0 / m->d[j][j];
        for(k=0; k < n; k++)
        {
            m->d[j][k] *= fac;
            res->d[j][k] *= fac;
        }

        /*
        ** no we can zero the j-th element of all rows below row j
        ** by basic row operations.
        */
        for(i=j+1; i < n; i++)
        {
            if(m->d[i][j] == 0.0)
                continue;               /* already zero */
            fac = m->d[i][j];
            for(k=0; k < n; k++)
            {
                m->d[i][k] -= m->d[j][k] * fac;
                res->d[i][k] -= res->d[j][k] * fac;
            }
        }
    }

    /*                  /1 0 0\
    ** bring matrix on  |0 1 0| form
    **                  \0 0 1/
    */
    for(j=n-1; j >= 0; j--)                /* loop through cols */
    {
        /* check col j */
        if(NEAR_ZERO(m->d[j][j]))
            return(MATERR_SINGULAR);    /* can't calculate inverse matrix */

        /*
        ** no we can zero the j-th element of all rows above row j
        ** by basic row operations.
        */
        for(i=j-1; i >= 0; i--)
        {
            if(m->d[i][j] == 0.0)
                continue;               /* already zero */
            fac = m->d[i][j];
            for(k=0; k < n; k++)
            {
                m->d[i][k] -= m->d[j][k] * fac;
                res->d[i][k] -= res->d[j][k] * fac;
            }
        }
    }

    return(0);
}
#else
/*
** Thanks to Ralf Corsepius for the following piece of code:
*/
int MatrixInverse(struct Matrix *m, struct Matrix *res)
{
  /*
   * This routine calculates a dim x dim inverse matrix.  It uses Gaussian
   * elimination on the system [matrix][inverse] = [identity].  The values
   * of the matrix then become the coefficients of the system of equations
   * that evolves from this equation.  The system is then solved for the
   * values of [inverse].  The algorithm solves for each column of [inverse]
   * in turn.  Partial pivoting is also done to reduce the numerical error
   * involved in the computations.  If singularity is detected, the routine
   * ends and returns an error.
   *
   * (See Numerical Analysis, L.W. Johnson and R.D.Riess, 1982).
   */

    int ipivot = 0, h, i, j, k;
    struct Matrix *aug;
    elem_t pivot, q;
    int dim;
    
    if(m == NULL || res == NULL)
        return(MATERR_NOMATRIX);
    if(m->n != m->m || res->n != res->m)
        return(MATERR_NOSQUAREMATRIX);
    if(m->n != res->n)
        return(MATERR_SIZE);

    dim = m->n;

    aug = MatrixAlloc(dim, dim+1);
    if(aug == NULL)
	return(MATERR_MEM);
      
    for (h = 0; h < dim; h++)   /* solve column by column */
    {
        /*
	 * Set up the augmented matrix for [matrix][inverse] = [identity]
	 */

    	for (i = 0; i < dim; i++)
    	{
	    memcpy(aug->d[i], m->d[i], dim * sizeof(elem_t));
      	    aug->d[i][dim] = (h == i) ? 1.0 : 0.0;
        }

        /*
	 * Search for the largest entry in column i, rows i through dim-1.
         * ipivot is the row index of the largest entry.
	 */

    	for (i = 0; i < dim-1; i++)
    	{
      	    pivot = 0.0;

      	    for (j = i; j < dim; j++)
      	    {
        	elem_t temp ;
        
        	temp = ABS (aug->d[j][i]);
        	if (pivot < temp)
        	{
          	    pivot = temp;
          	    ipivot = j;
        	}
      	    }
      	    if (NEAR_ZERO (pivot))     		/* singularity check */
      	    {
        	MatrixFree(aug);
        	return(MATERR_SINGULAR);
	    }

            /* interchange rows i and ipivot */

            if (ipivot != i)
      	    {
		elem_t *temp ;
        
		temp = aug->d[i];
		aug->d[i] = aug->d[ipivot];
		aug->d[ipivot] = temp;
	    }

	    /* put augmented matrix in echelon form */

	    for (k = i + 1; k < dim; k++)
	    {
		q = -aug->d[k][i] / aug->d[i][i];
		aug->d[k][i] = 0.0;
		for (j = i + 1; j < dim+1; j++)
		{
		    aug->d[k][j] = q * aug->d[i][j] + aug->d[k][j];
		}
	    }
	}

	if (NEAR_ZERO (aug->d[dim-1][dim-1]))   /* singularity check */
	{
	    MatrixFree(aug);
	    return(MATERR_SINGULAR);
    	}

        /* backsolve to obtain values for inverse matrix */

    	res->d[dim-1][h] = aug->d[dim-1][dim] / aug->d[dim-1][dim-1];

    	for (k = 1; k < dim; k++)
    	{
      	    q = 0.0;
      	    for (j = 1; j <= k; j++)
      	    {
        	q += aug->d[dim - 1 - k][dim - j] * res->d[dim - j][h];
      	    }
      	    res->d[dim-1 - k][h] = 
		(aug->d[dim - 1 - k][dim] - q) / 
		aug->d[dim - 1 - k][dim - 1 - k];
    	}
    }

    MatrixFree(aug);
    return (0);
}
#endif

int MatrixMult(struct Matrix *A, struct Matrix *B, struct Matrix *C)
{
    int i, j, k;
    elem_t val;

    if(A == NULL || B == NULL || C == NULL)
        return(MATERR_NOMATRIX);
    if(A->m != B->n || A->n != C->n || B->m != C->m)
        return(MATERR_SIZE);

    for(i=0; i < C->n; i++)
        for(j=0; j < C->m; j++)
        {
            val = 0.0;
            for(k=0; k < A->m; k++)
                val += A->d[i][k] * B->d[k][j];
            C->d[i][j] = val;
        }
    return(0);
}

void MatrixPrint(struct Matrix *m, FILE *file)
{
    int i, j;

    if(m != NULL && file != NULL)
    {
        for(i=0; i < m->n; i++)
        {
            for(j=0; j < m->m; j++)
                fprintf(file, "%15g\t", (double)m->d[i][j]);
            fprintf(file, "\n");
        }
    }
}

static char *err_strings[] =
{
    "No error!",
    "Out of memory!",
    "No matrix!",
    "No square matrix!",
    "Wrong size!",
    "Matrix singular!"
};

char *MatError(int errnum)
{

    return(err_strings[errnum]);
}


