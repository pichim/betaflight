#include "chirp.h"
#include "maths.h"

#include <math.h>

void chirpInitExp(chirp_t *chirp, float f0, float f1, uint32_t N, float Ts) 
{
    chirp->f0 = f0;
    chirp->f1 = f1;                  // not used
    chirp->t1 = (float)(N - 1) * Ts; // not used
    chirp->Ts = Ts;
    chirp->alpha = 0.0f;
    chirpResetCount(chirp);
    chirp->N = N;
    chirp->beta = pow_approx(chirp->f1 / chirp->f0, 1.0f / chirp->t1);
    chirp->k0 = 2.0f * M_PI / log_approx(chirp->beta);
    chirp->k1 = chirp->k0 * chirp->f0;
    chirpResetSignals(chirp);
}

void chirpInitAlpha(chirp_t *chirp, float f0, float f1, uint32_t N, float Ts, float alpha) 
{
    chirp->f0 = f0;
    chirp->f1 = f1;                  // not used
    chirp->t1 = (float)(N - 1) * Ts; // not used
    chirp->Ts = Ts;
    chirp->alpha = alpha;
    chirpResetCount(chirp);
    chirp->N = N;
    chirp->beta = (chirp->f1 - chirp->f0) / pow_approx(chirp->t1, chirp->alpha);
    chirp->k0 = 2.0f * M_PI * chirp->f0;
    chirp->k1 = 2.0f * M_PI * chirp->beta / (chirp->alpha + 1);
    chirpResetSignals(chirp);
}

void chirpResetCount(chirp_t *chirp)
{
    chirp->count = 0;
}

void chirpResetSignals(chirp_t *chirp)
{
    chirp->fchirp = 0.0f;
    chirp->sinarg = 0.0f;
    chirp->exc = 0.0f;
}

bool chirpUpdate(chirp_t *chirp)
{
    if (chirp->count == chirp->N) {
        chirpResetSignals(chirp);
        return false;
    } else {
        if (chirp->alpha > 0.0f) {
            chirp->fchirp = chirp->f0 + chirp->beta * pow_approx((float)(chirp->count) * chirp->Ts, chirp->alpha);
            chirp->sinarg = chirp->k0*((float)(chirp->count) * chirp->Ts) + chirp->k1 * pow_approx((float)(chirp->count) * chirp->Ts, chirp->alpha + 1.0f);
        } else {
            chirp->fchirp = chirp->f0 * pow_approx(chirp->beta, (float)(chirp->count) * chirp->Ts);
            chirp->sinarg = chirp->k0 * chirp->fchirp - chirp->k1;
        }
        // wrap sinarg to 0...2*pi
        chirp->sinarg = fmod(chirp->sinarg, 2.0f * M_PI);
        // we do a cosine so that the angle will oscilate around 0 (integral of gyro)
        chirp->exc = cos_approx(chirp->sinarg);
        // frequencies below 1 Hz will lead to the same angle magnitude as at 1 Hz
        if (chirp->fchirp < 1.0f) {
            chirp->exc = chirp->fchirp * chirp->exc;
        }
        chirp->count++;
        return true;
    }
}