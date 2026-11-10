#include <stdint.h>
#include "fox_native.h"

/* LUT intercalada [valor][H,S,V]; máscaras planas [color][píxel].
 * Conserva los bits de cada rango y la unión por color de vision.hsv_masks. */
FOX_API void fox_hsv_masks(int64_t n, int32_t colors, const uint8_t *hsv,
    const uint8_t *lut, const uint8_t *bits, uint8_t *out)
{
    /* Dos colores es el caso de carrera. Sin bucle interior ni productos
     * por color, y sin reservar imágenes de tres canales intermedias. */
    if (colors == 2) {
        const uint8_t b0 = bits[0], b1 = bits[1];
        for (int64_t i = 0; i < n; ++i) {
            uint8_t code = lut[3 * hsv[3 * i]]
                & lut[3 * hsv[3 * i + 1] + 1]
                & lut[3 * hsv[3 * i + 2] + 2];
            out[i] = (code & b0) ? 255 : 0;
            out[n + i] = (code & b1) ? 255 : 0;
        }
        return;
    }
    for (int64_t i = 0; i < n; ++i) {
        uint8_t code = lut[3 * hsv[3 * i]]
            & lut[3 * hsv[3 * i + 1] + 1]
            & lut[3 * hsv[3 * i + 2] + 2];
        for (int32_t c = 0; c < colors; ++c)
            out[(int64_t)c * n + i] = (code & bits[c]) ? 255 : 0;
    }
}
