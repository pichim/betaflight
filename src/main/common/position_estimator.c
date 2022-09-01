#include "position_estimator.h"

#include <math.h>
#include "maths.h"

void positionEstimatorUpdateGain(positionEstimator_t *positionEstimator, float f_cut, float f_a, float dT)
{
    // define location of discrete poles
    const float k  = dT / (1 / (2 * M_PIf * f_cut) + dT);
    const float w1 = k - 1;
    const float w2 = w1;
    const float w3 = w1;

    const float wa = 2 * M_PIf * f_a;
    const float a33 = 1 / (dT*wa + 1);

    // calculate observer gain
    const float c0 = w1 * w2 * w3;
    const float c1 = 1 - (w1 * w2 + w1 * w3 + w2 * w3);
    const float c2 = dT * a33;
    const float k1 = c0 / a33 + 1;
    const float k2 = (c1 - k1 + (2 - k1) * a33) / c2;
    const float k3 = (c1 - k1 - ((w1 + w2 + w3) + a33) * a33) / (c2*c2);

    const float a12 = dT * (1 - k1);
    const float a22 = (1 - dT * k2);
    const float a32 = -dT * k3;

    positionEstimator->dT = dT;
    positionEstimator->a12 = a12;
    positionEstimator->a22 = a22;
    positionEstimator->a32 = a32;
    positionEstimator->a33 = a33;
    positionEstimator->k1 = k1;
    positionEstimator->k2 = k2;
    positionEstimator->k3 = k3;

}

void positionEstimatorInit(positionEstimator_t *positionEstimator, float position, float velocity, float accBias, uint8_t positionDiscreteDelay)
{   
    // initial states
    positionEstimator->position = position;
    positionEstimator->velocity = velocity;
    positionEstimator->accBias = accBias;

    // initial delayed position
    positionEstimator->positionDiscreteDelay = positionDiscreteDelay;
    for (uint8_t i = 0; i < positionEstimator->positionDiscreteDelay + 1; i++) {
        positionEstimator->positionPast[i] = position;
    }
    positionEstimator->positionPastIndex = 0;

}

void positionEstimatorApply(positionEstimator_t *positionEstimator, float acc, float position)
{
    // update delayed position estimation
    positionEstimator->positionPast[positionEstimator->positionPastIndex] = positionEstimator->position;
    positionEstimator->positionPastIndex++;
    if (positionEstimator->positionPastIndex > positionEstimator->positionDiscreteDelay) {
        positionEstimator->positionPastIndex = 0;
    }

    // it is assumed that acc is already without bias, rotated w.r.t. the earth frame and without gravity
    const float u1 = acc;
    const float u2 = position - positionEstimator->positionPast[positionEstimator->positionPastIndex];

    // create copies
    const float position_k = positionEstimator->position;
    const float velocity_k = positionEstimator->velocity;
    const float accBias_k = positionEstimator->accBias;

    // update filter step
    // G = [1, a12,   0, Ts*a12, k1
    //      0, a22,   0, Ts*a22, k2
    //      0, a32, a33, Ts*a32, k3]
    // y = G * [x; u1; u2]
    positionEstimator->position = position_k + positionEstimator->a12 * velocity_k                                      + positionEstimator->dT * positionEstimator->a12 * u1 + positionEstimator->k1 * u2;
    positionEstimator->velocity =              positionEstimator->a22 * velocity_k                                      + positionEstimator->dT * positionEstimator->a22 * u1 + positionEstimator->k2 * u2;
    positionEstimator->accBias =               positionEstimator->a32 * velocity_k + positionEstimator->a33 * accBias_k + positionEstimator->dT * positionEstimator->a32 * u1 + positionEstimator->k3 * u2;

}