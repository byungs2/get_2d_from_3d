#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct _Mat {
  int row;
  int column;
  float **mat_array;
} Mat;

void mat_mul (Mat *A, Mat *B, Mat *C);

void mat_init (Mat *a, float *mat_array);

Mat* mat_new (int row, int column);

void mat_devide(Mat *A, float devider);
#endif
