#include <stdint.h>
#include "fox_native.h"

/* DP min-plus de ring_route.plan(): mismas operaciones y mismo orden que el
 * bucle numpy (-ffp-contract=off impide FMA), así el resultado es idéntico.
 * V[a,b] = costo con d_{t-1}=a, d_t=b; back[t+1,b,c] = a que gana para (b,c). */
FOX_API void fox_route_dp(int32_t n, int32_t m, int32_t N,
    const double *G, const double *DS, const uint8_t *MASK,
    const double *pen, const double *bonus, const uint8_t *bad, const uint8_t *allowed,
    double *Vc, double *Vtmp, int16_t *back)
{
    const double INF = 1e18;
    const int64_t mm = (int64_t)m * m;
    const int64_t m3 = mm * m;
    for (int32_t t = 1; t <= N - 2; t++) {
        int32_t k1 = t % n, k2 = (t + 1) % n;
        const double *Gk = G + k1 * m3;
        const uint8_t *Mk = MASK + k1 * m3;
        const double *DSk = DS + k1 * mm;
        const double *penk = pen + k2 * mm;
        const double *bonk = bonus + k2 * mm;
        const uint8_t *badk = bad + k2 * mm;
        const uint8_t *alk = allowed + k2 * m;
        int16_t *bk = back + (t + 1) * mm;
        for (int32_t b = 0; b < m; b++) {
            for (int32_t c = 0; c < m; c++) {
                int64_t bc = (int64_t)b * m + c;
                int blocked = badk[bc] || !alk[c];
                double ds = DSk[bc], pc = penk[bc], bo = bonk[bc];
                double best = 0.0;
                int32_t arg = 0;
                for (int32_t a = 0; a < m; a++) {
                    int64_t idx = (int64_t)a * mm + bc;
                    double s = (Mk[idx] || blocked) ? INF : ds * (Gk[idx] + pc) + bo;
                    double tot = Vc[a * m + b] + s;
                    if (a == 0 || tot < best) {   /* primer mínimo, como np.argmin */
                        best = tot;
                        arg = a;
                    }
                }
                bk[bc] = (int16_t)arg;
                Vtmp[bc] = best < INF ? best : INF;
            }
        }
        for (int64_t i = 0; i < mm; i++) Vc[i] = Vtmp[i];
    }
}
