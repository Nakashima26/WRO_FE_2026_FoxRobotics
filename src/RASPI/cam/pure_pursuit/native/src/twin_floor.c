#include <stdint.h>
#include <math.h>
#include "fox_native.h"

/* Piso de twin/camera.render_camera: por píxel que ve el piso, gira y traslada
 * (adelante, right) y marca 1 tapete / 2 isla. Mismas operaciones float32 y en
 * el mismo orden que el numpy (-ffp-contract=off, sin FMA): ids idénticos.
 * out debe venir en cero; solo se escriben las celdas idx[i]. */
FOX_API void fox_twin_floor(int64_t n, const int64_t *idx, const float *ff, const float *rr,
    float sh, float ch, float ox, float oy, float outer, float inner, uint8_t *out)
{
    for (int64_t i = 0; i < n; i++) {
        float gx = ff[i] * sh;
        gx = gx + ox;
        gx = gx + rr[i] * ch;
        float gy = ff[i] * ch;
        gy = gy + oy;
        gy = gy - rr[i] * sh;
        gx = fabsf(gx);
        gy = fabsf(gy);
        if (gx != gx || gy != gy)  /* np.maximum propaga NaN: ninguna comparación pasa */
            continue;
        float m = gx > gy ? gx : gy;
        out[idx[i]] = (uint8_t)((m <= outer) + (m < inner));
    }
}
