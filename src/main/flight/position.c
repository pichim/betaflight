/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "common/position_estimator.h"

#include "drivers/time.h"

static int32_t estimatedAltitudeCm = 0;                // in cm
#ifdef USE_BARO
    static pt2Filter_t baroDerivativeLpf;
    static float baroDerivativeLpfCutoffHz = 0.1f;
    static positionEstimator_t positionEstimatorZ = {.position = 0.0f, .velocity = 0.0f}; // cm, cm/s
#endif

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 2);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altSource = DEFAULT,
    .altNumSatsGpsUse = POSITION_DEFAULT_ALT_NUM_SATS_GPS_USE,
    .altNumSatsBaroFallback = POSITION_DEFAULT_ALT_NUM_SATS_BARO_FALLBACK,
);

#ifdef USE_VARIO
static int16_t estimatedVario = 0;                   // in cm/s

int16_t calculateEstimatedVario(float baroAltVelocity)
{
    baroAltVelocity = applyDeadband(baroAltVelocity, 10.0f); // cm/s, so ignore climb rates less than 0.1 m/s
    return constrain(lrintf(baroAltVelocity), -1500, 1500);
}
#endif

#if defined(USE_BARO) || defined(USE_GPS)
static bool altitudeOffsetSetBaro = false;
static bool altitudeOffsetSetGPS = false;

void calculateEstimatedAltitude()
{
    static timeUs_t previousTimeUs = 0;
    static float baroAltOffset = 0;
    static float gpsAltOffset = 0;

    float baroAlt = 0;
    float accZ = 0; // just for logging, offline data analysis can also be done with the unrotated value
    float baroAltVelocity = 0;
    float gpsAlt = 0;
    uint8_t gpsNumSat = 0;

#if defined(USE_GPS) && defined(USE_VARIO)
    float gpsVertSpeed = 0;
#endif
    float gpsTrust = 0.3; //conservative default
    bool haveBaroAlt = false;
    bool haveGpsAlt = false;
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {

        // since blackbox files showed updaterates between 117 - 125 Hz instead of 120 Hz I adjust the filters and the estimator based on the actual time
        const timeUs_t currentTimeUs = micros();
        const timeDelta_t deltaTimeUs = cmpTimeUs(currentTimeUs, previousTimeUs);
        previousTimeUs = currentTimeUs;
        float dT = deltaTimeUs * 1e-6f;

        // make sure dT is not zero for the first run, this is necessary since we divide by dT for the derivative filter
        static bool isFirstUpdate = true;
        if (isFirstUpdate) {
            dT = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);
            isFirstUpdate = false;
        }

        // run upsampling filter
        baroAlt = baroUpsampleAltitude(dT); // cm

        // initialise baro derivative filter
        static bool isBaroDerivativeFilterInitialised = false;
        if (!isBaroDerivativeFilterInitialised) {
            pt2FilterInit(&baroDerivativeLpf, pt2FilterGain(baroDerivativeLpfCutoffHz, dT));
            isBaroDerivativeFilterInitialised = true;
        }

        // run baro derivative filter
        static float lastBaroAlt = 0;
        pt2FilterUpdateCutoff(&baroDerivativeLpf, pt2FilterGain(baroDerivativeLpfCutoffHz, dT));
        baroAltVelocity = pt2FilterApply(&baroDerivativeLpf, (baroAlt - lastBaroAlt) / dT); // cm/s
        lastBaroAlt = baroAlt;

        if (baroIsCalibrated()) {
            haveBaroAlt = true;
        }

        if(haveBaroAlt) {
            // initialise position estimators
            static bool arePositionEstimatorsInitialised = false;
            if (!arePositionEstimatorsInitialised) {
                // positionDiscreteDelay: discrete time shift of position estimator / observer, ToDo: tune this parameter
                //                        can be used to compensate for delay that is applied due to prefiltering of baro,
                //                        position.c should be running at a constant update rate if this feature wants to be used.
                //                        this should be tuned based on the phase plot of the prefiltering (bf and sensor internal),
                //                        just approximalte the phase loss with a pure time delay, e.g. for a pt2 with cutoff at 2 Hz this
                //                        is approximately 12 samples delay. but: this is somewhat like applying a bit of derivative to the
                //                        position estimate, so it will lead to overshoot and could make the filter unstable if unreasonably high.
                const uint8_t positionDiscreteDelay = 12;
                // best guess of actual states, it is assumed that the craft is on the ground during this process, this could also be done after the baro calibration
                // is finished and baro has its first valid value when rearming
                const float accBiasZ = imuRotationmatrixTransformAccBodyToEarthZ() - 981.0f;
                positionEstimatorInit(&positionEstimatorZ, baroAlt, 0.0f, accBiasZ, positionDiscreteDelay);
                // f_cut: cutoff frequency, this will be a value between 0.10 - 0.30 Hz (or something), ToDo: tune this parameter, make this dependent on setpoint
                //        this should be adjusted based on the setpoint dynamics: no    setpoint change -> f_cut small , e.g. 0.10 Hz
                //                                                                large setpoint change -> f_cut bigger, e.g. 0.30 Hz, quadratic or exp could be used
                //                                     position estimator is not running in closed loop -> f_cut big   , e.g. 1.00 Hz
                //        transient slow switch-on behaviour can be accelerated with temprarily high values, e.g. f_cut = 1.00 Hz
                //        this parameter also decides on how much noise of the position and acc is within the estimate.
                // f_a:   eigendynamics of bias, zero corresponds to a pure integrator, for x-y position estimates this will not be zero
                const float f_cut = 0.1f;
                const float f_a = 0.0f;
                positionEstimatorUpdateGain(&positionEstimatorZ, f_cut, f_a);
                arePositionEstimatorsInitialised = true;
            }

            // run position estimators
            accZ = imuRotationmatrixTransformAccBodyToEarthZ();
            const float accBiasFreeZ = imuRotationmatrixTransformAccBodyToEarthAndRemoveBiasZ(positionEstimatorZ.accBias) - 981.0f; // cm/s^2
            //const float accBiasFreeZ = accZ - positionEstimatorZ.accBias - 981.0f; // here we substract the bias from accZ w.r.t. the earth frame
            positionEstimatorApply(&positionEstimatorZ, accBiasFreeZ, baroAlt, dT);
        }
        
    }
#endif

#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        gpsAlt = gpsSol.llh.altCm;
        gpsNumSat = gpsSol.numSat;
#ifdef USE_VARIO
        gpsVertSpeed = GPS_verticalSpeedInCmS;
#endif
        haveGpsAlt = true;

        if (gpsSol.hdop != 0) {
            gpsTrust = 100.0 / gpsSol.hdop;
        }
        // always use at least 10% of other sources besides gps if available
        gpsTrust = MIN(gpsTrust, 0.9f);
    }
#endif

    if (ARMING_FLAG(ARMED) && !altitudeOffsetSetBaro) {
        baroAltOffset = baroAlt;
        altitudeOffsetSetBaro = true;
    } else if (!ARMING_FLAG(ARMED) && altitudeOffsetSetBaro) {
        altitudeOffsetSetBaro = false;
    }

    baroAlt -= baroAltOffset;

    int goodGpsSats = 0;
    int badGpsSats = -1;

    if (haveBaroAlt) {
        goodGpsSats = positionConfig()->altNumSatsGpsUse;
        badGpsSats = positionConfig()->altNumSatsBaroFallback;
    }

    if (ARMING_FLAG(ARMED)) {
        if (!altitudeOffsetSetGPS && gpsNumSat >= goodGpsSats) {
            gpsAltOffset = gpsAlt - baroAlt;
            altitudeOffsetSetGPS = true;
        } else if (gpsNumSat <= badGpsSats) {
            altitudeOffsetSetGPS = false;
        }
    } else if (!ARMING_FLAG(ARMED) && altitudeOffsetSetGPS) {
        altitudeOffsetSetGPS = false;
    }

    gpsAlt -= gpsAltOffset;

    if (!altitudeOffsetSetGPS) {
        haveGpsAlt = false;
        gpsTrust = 0.0f;
    }

    if (haveGpsAlt && haveBaroAlt && positionConfig()->altSource == DEFAULT) {
        if (ARMING_FLAG(ARMED)) {
            estimatedAltitudeCm = gpsAlt * gpsTrust + baroAlt * (1 - gpsTrust);
        } else {
            estimatedAltitudeCm = gpsAlt; //absolute altitude is shown before arming, ignore baro
        }
#ifdef USE_VARIO
        // baro is a better source for vario, so ignore gpsVertSpeed
        estimatedVario = calculateEstimatedVario(baroAltVelocity);
#endif
    } else if (haveGpsAlt && (positionConfig()->altSource == GPS_ONLY || positionConfig()->altSource == DEFAULT )) {
        estimatedAltitudeCm = gpsAlt;
#if defined(USE_VARIO) && defined(USE_GPS)
        estimatedVario = gpsVertSpeed;
#endif
    } else if (haveBaroAlt && (positionConfig()->altSource == BARO_ONLY || positionConfig()->altSource == DEFAULT)) {
        estimatedAltitudeCm = baroAlt;
#ifdef USE_VARIO
        estimatedVario = calculateEstimatedVario(baroAltVelocity); // cm/s
#endif
    }

    DEBUG_SET(DEBUG_ALTITUDE, 0, (int32_t)(100 * gpsTrust));
    DEBUG_SET(DEBUG_ALTITUDE, 1, baroAlt);
    DEBUG_SET(DEBUG_ALTITUDE, 2, gpsAlt);
#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif

    DEBUG_SET(DEBUG_POSITION_ESTIMATOR_Z, 0, accZ);
    DEBUG_SET(DEBUG_POSITION_ESTIMATOR_Z, 1, baroAlt);
    DEBUG_SET(DEBUG_POSITION_ESTIMATOR_Z, 2, positionEstimatorZ.position - baroAltOffset); // remove arming offset
    DEBUG_SET(DEBUG_POSITION_ESTIMATOR_Z, 3, positionEstimatorZ.velocity);

    DEBUG_SET(DEBUG_COMPARE_ALTITUDE_FILTERS, 0, baroAlt);
    DEBUG_SET(DEBUG_COMPARE_ALTITUDE_FILTERS, 1, baroAltVelocity);
    DEBUG_SET(DEBUG_COMPARE_ALTITUDE_FILTERS, 2, positionEstimatorZ.position - baroAltOffset); // remove arming offset
    DEBUG_SET(DEBUG_COMPARE_ALTITUDE_FILTERS, 3, positionEstimatorZ.velocity);

}

bool isAltitudeOffset(void)
{
    return altitudeOffsetSetBaro || altitudeOffsetSetGPS;
}
#endif

int32_t getEstimatedAltitudeCm(void)
{
    return estimatedAltitudeCm;
}

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
