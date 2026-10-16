#include <assert.h>
#include <stdint.h>
#include <math.h>
#ifdef _OPENMP
#include <omp.h>
#endif
#include "fox_native.h"

/* DP min-plus de ring_route.plan(): mismas operaciones y mismo orden que el
 * bucle numpy (-ffp-contract=off impide FMA), así el resultado es idéntico.
 * V[a,b] = costo con d_{t-1}=a, d_t=b; back[t+1,b,c] = a que gana para (b,c).
 * Con OpenMP se reparte el bucle de b: cada (b,c) escribe solo sus celdas. */
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
#ifdef _OPENMP
#pragma omp parallel for
#endif
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

/* Igual que fox_route_dp, pero solo recorre a en [ALO,AHI] (MASK ya prohíbe lo
 * demás). Fuera de rango tot = Vc+INF >= INF; si el rango no da costo < INF se
 * repite el bucle completo. Bit a bit igual mientras Vc >= 0 (s >= 0). */
FOX_API void fox_route_dp_sparse(int32_t n, int32_t m, int32_t N,
    const double *G, const double *DS, const uint8_t *MASK,
    const double *pen, const double *bonus, const uint8_t *bad, const uint8_t *allowed,
    const int16_t *ALO, const int16_t *AHI,
    double *Vc, double *Vtmp, int16_t *back)
{
    const double INF = 1e18;
    const int64_t mm = (int64_t)m * m;
    const int64_t m3 = mm * m;
    assert(m <= 256);
    for (int32_t t = 1; t <= N - 2; t++) {
        int32_t k1 = t % n, k2 = (t + 1) % n;
        const double *Gk = G + k1 * m3;
        const uint8_t *Mk = MASK + k1 * m3;
        const double *DSk = DS + k1 * mm;
        const double *penk = pen + k2 * mm;
        const double *bonk = bonus + k2 * mm;
        const uint8_t *badk = bad + k2 * mm;
        const uint8_t *alk = allowed + k2 * m;
        const int16_t *alok = ALO + k1 * mm;
        const int16_t *ahik = AHI + k1 * mm;
        int16_t *bk = back + (t + 1) * mm;
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int32_t b = 0; b < m; b++) {
            /* si todo a está prohibido, tot = Vc+INF: primer mínimo (como np.argmin) */
            int32_t ia = 0;
            double ibv = Vc[b] + INF;
            for (int32_t a = 1; a < m; a++) {
                double tot = Vc[a * m + b] + INF;
                if (tot < ibv) { ibv = tot; ia = a; }
            }
            for (int32_t c = 0; c < m; c++) {
                int64_t bc = (int64_t)b * m + c;
                int blocked = badk[bc] || !alk[c];
                int32_t lo = alok[bc], hi = ahik[bc];
                if (blocked || lo > hi) {
                    bk[bc] = (int16_t)ia;
                    Vtmp[bc] = ibv < INF ? ibv : INF;
                    continue;
                }
                double ds = DSk[bc], pc = penk[bc], bo = bonk[bc];
                double best = 0.0;
                int32_t arg = lo;
                for (int32_t a = lo; a <= hi; a++) {
                    int64_t idx = (int64_t)a * mm + bc;
                    double s = Mk[idx] ? INF : ds * (Gk[idx] + pc) + bo;
                    double tot = Vc[a * m + b] + s;
                    if (a == lo || tot < best) {
                        best = tot;
                        arg = a;
                    }
                }
                if (!(best < INF)) {   /* sin costo válido en el rango: bucle completo */
                    best = 0.0;
                    arg = 0;
                    for (int32_t a = 0; a < m; a++) {
                        int64_t idx = (int64_t)a * mm + bc;
                        double s = Mk[idx] ? INF : ds * (Gk[idx] + pc) + bo;
                        double tot = Vc[a * m + b] + s;
                        if (a == 0 || tot < best) {
                            best = tot;
                            arg = a;
                        }
                    }
                }
                bk[bc] = (int16_t)arg;
                Vtmp[bc] = best < INF ? best : INF;
            }
        }
        for (int64_t i = 0; i < mm; i++) Vc[i] = Vtmp[i];
    }
}

/* Geometría de ring_route._dp_geometry, mismo orden de operaciones que numpy.
 * Por k1: G[k1,a,b,c] = wl + (w_κ·κ)·κ, DS[k1,b,c] = |bc|, MASK[k1,a,b,c] =
 * |κ| > kmax o !side_ok[k1,b,c]. P es (n,m,2); side_ok es (n,m,m) [k1][b][c]. */
FOX_API void fox_route_geom(int32_t n, int32_t m, const double *P, const uint8_t *side_ok,
    int32_t time_cost, double w_len, double w_kappa, double a_lat, double v_min, double v_max, double kmax,
    double *G, double *DS, uint8_t *MASK)
{
    const int64_t mm = (int64_t)m * m;
    const int64_t m3 = mm * m;
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int32_t k1 = 0; k1 < n; k1++) {
        int32_t k0 = (k1 - 1 + n) % n, k2 = (k1 + 1) % n;
        const double *PA = P + (int64_t)k0 * m * 2;
        const double *PB = P + (int64_t)k1 * m * 2;
        const double *PC = P + (int64_t)k2 * m * 2;
        double *Gk = G + k1 * m3;
        double *DSk = DS + k1 * mm;
        uint8_t *Mk = MASK + k1 * m3;
        const uint8_t *sok = side_ok + k1 * mm;
        for (int32_t a = 0; a < m; a++) {
            for (int32_t b = 0; b < m; b++) {
                for (int32_t c = 0; c < m; c++) {
                    double ab0 = PB[2 * b] - PA[2 * a], ab1 = PB[2 * b + 1] - PA[2 * a + 1];
                    double bc0 = PC[2 * c] - PB[2 * b], bc1 = PC[2 * c + 1] - PB[2 * b + 1];
                    double ac0 = PC[2 * c] - PA[2 * a], ac1 = PC[2 * c + 1] - PA[2 * a + 1];
                    double cross = ab0 * bc1 - ab1 * bc0;
                    double lab = hypot(ab0, ab1);
                    double lbc = hypot(bc0, bc1);
                    double lac = hypot(ac0, ac1);
                    double den = (lab * lbc) * lac;
                    den = den > 1e-9 ? den : 1e-9;
                    double kap = (2.0 * cross) / den;
                    double wl = w_len;
                    if (time_cost) {
                        /* v(κ) = clip(sqrt(a_lat/|κ|), v_min, v_max); en recta el factor es 1 */
                        double ak = fabs(kap);
                        ak = ak > 1e-9 ? ak : 1e-9;
                        double v = sqrt(a_lat / ak);
                        v = v < v_min ? v_min : v;
                        v = v > v_max ? v_max : v;
                        wl = w_len * (v_max / v);
                    }
                    int64_t idx = (int64_t)a * mm + (int64_t)b * m + c;
                    Gk[idx] = wl + (w_kappa * kap) * kap;
                    Mk[idx] = (fabs(kap) > kmax) || !sok[(int64_t)b * m + c] ? 1 : 0;
                    if (a == 0) DSk[(int64_t)b * m + c] = lbc;
                }
            }
        }
    }
}
