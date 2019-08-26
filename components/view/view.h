#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#ifndef VIEW_H
#define VIEW_H
typedef struct {
  float x;
  float y;
  float w;
} point_t;

typedef struct {
  point_t p0;
  point_t p1;
} box_t;

typedef struct {
  float inner[9];
} affine_t;

typedef struct {
  box_t box;
  bool dirty;
  affine_t affine;
} view_t;
#endif
