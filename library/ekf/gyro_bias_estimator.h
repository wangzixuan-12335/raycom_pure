#ifndef __GYRO_BIAS_ESTIMATOR_H__
#define __GYRO_BIAS_ESTIMATOR_H__

#include "low_pass_filter.h"
#include "vector3d.h"
#include "matrix3x3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration gyro bias estimator constants */
/* gyro bias estimator data input lowpass filter cut-off frequency */
#define ACCEL_LOWPASS_FREQ  1.0f
#define GYRO_LOWPASS_FREQ   10.0f
/* gyro bias estimator gyro-bias output lowpass filter cut-off frequency */
#define GYRO_BIAS_LOWPASS_FREQ      0.2f
/* Static detection thresholds: estimator consider the state to be stable when adjacent data changes is less than the threshold, otherwise, it is regarded as in motion */
#define ACCEL_DIFF_STATIC_THRESHOLD 0.35f
#define GYRO_DIFF_STATIC_THRESHOLD  0.004f
/* Gyro magnitude threshold for bias update */
#define GYRO_FOR_BIAS_THRESHOLD     0.023f
/* the minimum number of samples required to be accumulated befor starting to updated the gyro bias */
#define NUM_GYRO_BIAS_SAMPLES_THRESHOLD 500
/* the sample length of enhance smooth for bias during cold start phase */
#define NUM_GYRO_BIAS_SAMPLES_INITIAL_SMOOTHING 1000
/* the number of consecutive static frames required to consider the sensor as static */
#define IS_STATIC_NUM_FRAMES_THRESHOLD 500

typedef struct {
    int minStaticFrames;
    int consecutiveIsStatic;
} IsStaticCounter;


void IsStaticCounter_Init(IsStaticCounter *counter, int minStaticFrames);
void IsStaticCounter_AppendFrame(IsStaticCounter *counter, uint8_t isStatic);
uint8_t IsStaticCounter_IsRecentlyStatic(const IsStaticCounter *counter);

typedef struct {
    LowPassFilter accelLowPass;
    LowPassFilter gyroLowPass;
    LowPassFilter gyroBiasLowPass;

    Vector3d smoothedGyroDiff;
    Vector3d smoothedAccelDiff;

    IsStaticCounter isAccelStatic;
    IsStaticCounter isGyroStatic;
} GyroscopeBiasEstimator;

extern GyroscopeBiasEstimator gyro_bias_estimator;

void GyroscopeBiasEstimator_Init(GyroscopeBiasEstimator *est);
void GyroscopeBiasEstimator_Reset(GyroscopeBiasEstimator *est);
void GyroscopeBiasEstimator_ProcessGyroscope(GyroscopeBiasEstimator *est, const Vector3d *gyro, int64_t timestampNs);
void GyroscopeBiasEstimator_ProcessAccelerometer(GyroscopeBiasEstimator *est, const Vector3d *accel, int64_t timestampNs);
void GyroscopeBiasEstimator_GetGyroBias(GyroscopeBiasEstimator *est, Vector3d *result);

#ifdef __cplusplus
}
#endif

#endif
