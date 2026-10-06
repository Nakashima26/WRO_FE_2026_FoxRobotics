#include "fox_native.h"

/* Mismo orden de operaciones que camera.py (_floor_ref) en float32, sin FMA. */
FOX_API void fox_floor_ids(const float *ff, const float *rr, const uint8_t *finite,
                           size_t n, float ox, float oy, float sh, float ch,
                           float outer, float inner, const uint8_t *palette,
                           uint8_t *img) {
  for (size_t i = 0; i < n; ++i) {
    uint8_t id = 0;
    if (finite[i]) {
      float a = ff[i] * sh;
      float b = rr[i] * ch;
      float gx = (ox + a) + b;
      float c = ff[i] * ch;
      float d = rr[i] * sh;
      float gy = (oy + c) - d;
      float ax = gx < 0.0f ? -gx : gx;
      float ay = gy < 0.0f ? -gy : gy;
      if (ax < inner && ay < inner) id = 2;
      else if (ax <= outer && ay <= outer) id = 1;
    }
    const uint8_t *col = palette + 3 * id;
    img[3 * i] = col[0];
    img[3 * i + 1] = col[1];
    img[3 * i + 2] = col[2];
  }
}
