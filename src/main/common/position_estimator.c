#include "position_estimator.h"

#include <math.h>
#include "maths.h"

void positionEstimatorUpdateGain(positionEstimator_t *positionEstimator, float f_cut, float f_a)
{
    positionEstimatorUpdateGainComplexPoles(positionEstimator, f_cut, f_cut, 0.5f, f_a);
}

void positionEstimatorUpdateGainComplexPoles(positionEstimator_t *positionEstimator, float filterFreq1, float filterFreq2, float Q2, float f_a)
{
    const float wa = 2.0f * M_PIf * f_a;
    positionEstimator->wa = wa;

    // time continous estimator / observer gain
    const float w1 = 2.0f * M_PIf * filterFreq1;
    const float w2 = 2.0f * M_PIf * filterFreq2;
    positionEstimator->k1 = w1 + w2 / Q2 - wa;
    positionEstimator->k2 = w2 * (w2 + w1 / Q2) - positionEstimator->k1 * wa;
    positionEstimator->k3 = positionEstimator->k2 * wa - w1 * w2 * w2;
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
    // predict position, velocity and accBias
    positionEstimator->position += dT * (positionEstimator->velocity + 0.5f * acc * dT);
    positionEstimator->velocity += dT * acc;
    positionEstimator->accBias += dT * -positionEstimator->wa * positionEstimator->accBias;

    // update delayed position estimation
    positionEstimator->positionPast[positionEstimator->positionPastIndex] = positionEstimator->position;
    positionEstimator->positionPastIndex++;
    if (positionEstimator->positionPastIndex > positionEstimator->positionDiscreteDelay) {
        positionEstimator->positionPastIndex = 0;
    }

    // correction based on delayed position estimate
    const float estimationError = dT * (position - positionEstimator->positionPast[positionEstimator->positionPastIndex]);
    positionEstimator->position += positionEstimator->k1 * estimationError;
    positionEstimator->velocity += positionEstimator->k2 * estimationError;
    positionEstimator->accBias += positionEstimator->k3 * estimationError;
}