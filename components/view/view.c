#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "view.h"

point_t point_new(float x, float y){
  return (point_t){
    .x = x,
    .y = y,
    .w = 1
  };
};

affine_t* affine_new(float rx, float sx, float tx,
                    float sy, float ry, float ty) {
  affine_t *aff = malloc(sizeof(affine_t));
  if(aff == NULL) return NULL;
  float arr[] = {rx, sx, tx, sy, ry, ty, 0, 0, 1};
  memcpy(aff->inner, arr, sizeof(arr));
  return aff;
}

void affine_free(affine_t *A) {
  free(A);
}

void affine_multiply(affine_t *A, affine_t *B) {
  float res[9];
  float *a = A->inner;
  float *b = B->inner;
  res[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
  res[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
  res[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

  res[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
  res[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
  res[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

  res[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
  res[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
  res[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];

  memcpy(A->inner, res, sizeof(res));
}

affine_t* affine_identity() {
  return affine_new(1,0,0,0,1,0);
}

void affine_translate(affine_t *A, float x, float y) {
  affine_multiply(A, &(affine_t){
      .inner = {
        1, 0, x,
        0, 1, y,
        0, 0, 1
      }
    });
}

point_t affine_project(point_t p, affine_t *A) {
   float *a = A->inner;
   point_t ret =  {
     .x = a[0] * p.x + a[1] * p.y + a[2],
     .y = a[3] * p.x + a[4] * p.y + a[5],
     .w = a[5] * p.x + a[7] * p.y + a[8]
   };
   ret.x /= ret.w;
   ret.y /= ret.w;
   ret.w /= ret.w;
   return ret;
}

void affine_rotate(affine_t *A, float degrees) {
  affine_multiply(A, &(affine_t){
      .inner = {
        cos(degrees), -sin(degrees), 0,
        sin(degrees), cos(degrees), 0,
        0, 0, 1
      }
    });
}

void affine_scale(affine_t *A, float x, float y) {
  affine_multiply(A, &(affine_t){.inner = {
        x, 0, 0,
        0, y, 0,
        0, 0, 1
      }
    });
}
