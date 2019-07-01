#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#ifndef VIEW_H
#define VIEW_H
typedef struct {
  int16_t x;
  int16_t y;
  int16_t x1;
  int16_t x2;
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
