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

#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "pg/pg.h"

// Exported symbols
extern bool canUseGPSHeading;

typedef struct {
    float w,x,y,z;
} quaternion;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        // rMat = Rz(-yaw) * Ry(pitch) * Rx (roll)
        // usual rpy euler angels use the convention Rz(yaw) * Ry(pitch) * Rx (roll) with the z axis pointing up (building a right hand system)
        int16_t roll;  /** roll lies in (-180, 180) deg
                        *  rotating it 360 deg around x looks like this: 360      /   roll
                        *                                                        /     ^
                        *                                                       /      |
                        *                                                  0   /        --> actual rotation */

        int16_t pitch; /** pitch lies in (-90, 90) deg
                        *  rotating it 360 deg around y looks like this:              pitch
                        *                                                90    /\      ^     - this signal is dangerous for control
                        *                                               -90      \/    |     - rpy euler angels have a singularity at -90 deg and 90 deg
                        *                                                               --> actual rotation */

        int16_t yaw;   /** yaw lies in (0, 360) deg
                        *  rotating it 360 deg around z looks like this: 3600      /   yaw
                        *                                                         /     ^
                        *                                                        /      |
                        *                                                   0   /        --> actual rotation */
    } values;
} attitudeEulerAngles_t;
#define EULER_INITIALIZE  { { 0, 0, 0 } }

extern attitudeEulerAngles_t attitude;
extern float rMat[3][3];                    // DCM either transforms a vector from body to earth frame, e.g. Ev        = rMat * Bv  ( <->  Bv        = rMat^T * Ev )
                                            //         or rotates    a vector in the earth frame,       e.g. EvRotated = rMat * Ev  ( <->  BvRotated = rMat^T * Bv )
                                            // DCM  contains in the coloums [EeBx  , EeBy  , EeBz  ]  ( basis of the body  frame w.r.t. the earth frame )
                                            //      and      in the rows    [BeEx^T; BeEy^T, BeEz^T]  ( basis of the earth frame w.r.t. the body  frame )
                                            //  - without a mag the mahony filter yaw estimate has drift
                                            //  - the third row of the DCM (BeEz^T) is invariant to yaw, this is why the mahony filter can estimate roll and pitch
                                            //    by only using an acc and a gyro
                                            //  - the static angle offset of roll and pitch which may bee seen after rearming is acc x and y temerature drift. 
                                            //    without an additional inertial sensor like a gnss unit it is not possible to estimate this drift
typedef struct imuConfig_s {
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t imu_process_denom;
} imuConfig_t;

PG_DECLARE(imuConfig_t, imuConfig);

typedef struct imuRuntimeConfig_s {
    float dcm_ki;
    float dcm_kp;
} imuRuntimeConfig_t;

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value);

float getCosTiltAngle(void);
void getQuaternion(quaternion * q);
void imuUpdateAttitude(timeUs_t currentTimeUs);

void imuInit(void);

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw);  // in deg
void imuSetAttitudeQuat(float w, float x, float y, float z);
#if defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt);
#endif
#endif

bool imuQuaternionHeadfreeOffsetSet(void);
void imuQuaternionHeadfreeTransformVectorEarthToBody(t_fp_vector_def * v);
bool shouldInitializeGPSHeading(void);
bool isUpright(void);
void imuGetBasisVectorEzBody(float * v);
