#include "position_estimator.h"

#include <math.h>
#include "maths.h"

void positionEstimatorUpdateGain(positionEstimator_t *positionEstimator, float f_cut, float f_a, float dT)
{
    // define location of discrete poles
    const float k = dT / (1.0f / (2.0f*M_PIf*f_cut) + dT);
    const float w1 = k - 1.0f;
    const float w2 = w1;
    const float w3 = w1;

    const float wa = 2.0f * M_PIf * f_a;
    const float a33 = 1.0f / (dT*wa + 1.0f);
    positionEstimator->a33 = a33;

    // time discrete estimator / observer gain
    const float c1 = 1.0f - (w1*w2 + w1*w3 + w2*w3);
    const float k1 = (w1 * w2 * w3) / a33 + 1.0f;
    const float c2 = (c1 - k1) / (dT*a33);   
    const float k2 = c2 + (2.0f - k1) / dT;
    const float k3 = (c2 / a33 - ((w1 + w2 + w3) / a33 + 1.0f) / dT) / dT;
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

void positionEstimatorApply(positionEstimator_t *positionEstimator, float acc, float position, float dT)
{
    // update delayed position estimation
    positionEstimator->positionPast[positionEstimator->positionPastIndex] = positionEstimator->position + dT * (positionEstimator->velocity + dT * acc);
    positionEstimator->positionPastIndex++;
    if (positionEstimator->positionPastIndex > positionEstimator->positionDiscreteDelay) {
        positionEstimator->positionPastIndex = 0;
    }

    // predict position, velocity and accBias
    positionEstimator->position += dT * (positionEstimator->velocity + dT * acc);
    positionEstimator->velocity += dT * acc;
    positionEstimator->accBias = positionEstimator->a33 * positionEstimator->accBias;

    // correction based on delayed position estimate
    const float estimationError = position - positionEstimator->positionPast[positionEstimator->positionPastIndex];
    positionEstimator->position += positionEstimator->k1 * estimationError;
    positionEstimator->velocity += positionEstimator->k2 * estimationError;
    positionEstimator->accBias += positionEstimator->k3 * estimationError;
}