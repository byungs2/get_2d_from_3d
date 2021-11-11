#include "matrix.h"

void mat_mul (Mat *a, Mat *b, Mat *c) {
  float temp_arr[a->row][b->column];
  if(a->column != b->row) {
    printf("Wrong shape of matrix\n");
  } else {
    float result = 0;
    for(int i = 0; i < c->row; i++) {
      for(int j = 0; j < c->column; j++) {
        for(int k = 0; k < a->column; k++) {
          result = result + a->mat_array[i][k] * b->mat_array[k][j];
        }
        temp_arr[i][j] = result;
        result = 0;
      }
    }
  }
  for(int i = 0; i < c->row; i++) {
    for(int j = 0; j < c->column; j++) {
      c->mat_array[i][j] = temp_arr[i][j];
    }
  }
}

void mat_init (Mat *a, float *mat_array) {
  for(int i = 0; i < a->row; i++) {
    for(int j = 0; j < a->column; j++) {
      a->mat_array[i][j] = *(mat_array + i * a->column + j);
    }
  }
}

Mat* mat_new (int row, int column) {
  Mat *a = (Mat *) malloc(sizeof(Mat));
  a->row = row;
  a->column = column;
  a->mat_array = (float **) malloc(sizeof(float *) * row);
  memset(a->mat_array, 0.0, sizeof(float *) * row);
  for(int i = 0; i < row; i++) {
    a->mat_array[i] = (float *) malloc(sizeof(float) * column);
    memset(a->mat_array[i], 0.0, sizeof(float) * column);
  }
  return a;
}

void mat_devide(Mat *A, float devider) {
  for(int i = 0; i < A->row; i++) {
    for(int j = 0; j < A->column; j++) {
      A->mat_array[i][j] /= devider;
    }
  }
}
