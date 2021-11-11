#include "matrix.h"

int main(int argc, char *argv[]) {
  float intrinsic[3][3] = {{613.168701, 0, 639.633728},{0, 612.880493, 364.357513},{0, 0, 1}};
  Mat *intrinsic_mat;
  float point_arr[4][1];
  Mat *point_mat;
  Mat *image_mat;

  float rotate_arr[4][4] = {{0.999945, 0.010509, 0.000076, 0},{-0.010469,  0.995458, 0.094629, 0},{0.000918, -0.094624, 0.995513, 0}, {0, 0, 0, 1}};
  Mat *rotate_mat;
  float translation_arr[4][4] = {{1.0,0.0,0.0,-32.102242},{0.0, 1.0, 0.0, -2.036716},{0.0, 0.0, 1.0, 4.053820},{0.0, 0.0, 0.0, 1}};
  Mat *tr_mat;
  float projection_arr[3][4] = {{1, 0, 0, 0},{0, 1, 0, 0}, {0, 0, 1, 0}};
  Mat *pr_mat;

  float rt_arr[4][4] = {{0.999945, 0.010509, 0.000076, -32.102242},{-0.010469,  0.995458, 0.094629, -2.036716},{0.000918, -0.094624, 0.995513, 4.053820}, {0, 0, 0, 1}};
  Mat *rt_mat;

  intrinsic_mat = mat_new(3, 3);
  mat_init(intrinsic_mat, (float *)intrinsic);
  point_mat = mat_new(4, 1);
  image_mat = mat_new(3, 1);
  rotate_mat = mat_new(4, 4);
  mat_init(rotate_mat, (float *)rotate_arr);
  tr_mat = mat_new(4, 4);
  mat_init(tr_mat, (float *)translation_arr);
  pr_mat = mat_new(3, 4);
  mat_init(pr_mat, (float *)projection_arr);
  rt_mat = mat_new(4, 4);
  mat_init(rt_mat, (float *)rt_arr);

  for(int i = 0; i < rotate_mat->row; i++) {
    printf("\n");
    for(int j = 0; j < rotate_mat->column; j++) {
      printf(" %f ", rotate_mat->mat_array[i][j]);
    }
  }

  printf("\n");

  for(int i = 0; i < tr_mat->row; i++) {
    printf("\n");
    for(int j = 0; j < tr_mat->column; j++) {
      printf(" %f ", tr_mat->mat_array[i][j]);
    }
  }

  printf("\n");

  mat_mul(rotate_mat, tr_mat, tr_mat);
  for(int i = 0; i < tr_mat->row; i++) {
    printf("\n");
    for(int j = 0; j < tr_mat->column; j++) {
      printf(" %f ", tr_mat->mat_array[i][j]);
    }
  }
  printf("\n");

  /*
  mat_mul(pr_mat, tr_mat, pr_mat);
  for(int i = 0; i < pr_mat->row; i++) {
    printf("\n");
    for(int j = 0; j < pr_mat->column; j++) {
      printf(" %f ", pr_mat->mat_array[i][j]);
    }
  }
  */

  return 0;
}
