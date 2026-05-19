#include "gyro_bias_estimator.h"

#define FMAX(a, b) (( (a) > (b) ) ? (a) : (b))
#define FMIN(a, b) (((a) < (b)) ? (a) : (b))

GyroscopeBiasEstimator gyro_bias_estimator;

/**
 * *@brief initialize the IsStaticCounter structure.
 *  @param counter Pointer to IsStaticCounter instance.
 *  @param minStaticFrames Minimum number of consecutive static frames to consider as static.
 *  @return void
 */
void IsStaticCounter_Init(IsStaticCounter *counter, int minStaticFrames)
{
    counter->minStaticFrames = minStaticFrames;
    counter->consecutiveIsStatic = 0;
}

/**
 * *@brief Append a new frame's static status to the IsStaticCounter.
 * @param counter Pointer to IsStaticCounter instance.
 * @param isStatic 1 if the current frame is static, 0 otherwise.
 * @return void
 */
void IsStaticCounter_AppendFrame(IsStaticCounter *counter, uint8_t isStatic)
{
    if (!isStatic)
        counter->consecutiveIsStatic = 0;
    else
        counter->consecutiveIsStatic++;
}

/**
 * *@brief Check if the system has been recently static.
 * @param counter Pointer to IsStaticCounter instance.
 * @return 1 if recently static, 0 otherwise.
 */
uint8_t IsStaticCounter_IsRecentlyStatic(const IsStaticCounter *counter)
{
    return counter->consecutiveIsStatic >= counter->minStaticFrames;
}

/**
 * *@brief Update gyroscope bias estimate based on current gyro reading.
 * @param est Pointer to GyroscopeBiasEstimator instance.
 * @param gyro Pointer to current gyroscope reading.
 * @param timestampNs Current timestamp in nanoseconds.
 * @return void
 */
static void GyroscopeBiasEstimator_UpdateBias(GyroscopeBiasEstimator *est, const Vector3d *gyro, int64_t timestampNs)
{
    float gyro_len = Vector3d_length(gyro);
    if (gyro_len < GYRO_FOR_BIAS_THRESHOLD)
    {
        float updateWeight = FMAX(0.0f, 1.0f - gyro_len / GYRO_FOR_BIAS_THRESHOLD);
        updateWeight *= updateWeight;
        Vector3d filteredGyro = est->gyroLowPass.filteredData;
        lowPassFilter_addWeightedSample(&est->gyroBiasLowPass, &filteredGyro, timestampNs, updateWeight);
    }
}

/**
 * *@brief Reset the GyroscopeBiasEstimator to its initial state.
 * @param est Pointer to GyroscopeBiasEstimator instance.
 * @return void
 */
void GyroscopeBiasEstimator_Reset(GyroscopeBiasEstimator *est)
{
    Vector3d_setZero(&est->smoothedGyroDiff);
    Vector3d_setZero(&est->smoothedAccelDiff);
    lowPassFilter_init(&est->accelLowPass, ACCEL_LOWPASS_FREQ);
    lowPassFilter_init(&est->gyroLowPass, GYRO_LOWPASS_FREQ);
    lowPassFilter_init(&est->gyroBiasLowPass, GYRO_BIAS_LOWPASS_FREQ);
    IsStaticCounter_Init(&est->isAccelStatic, IS_STATIC_NUM_FRAMES_THRESHOLD);
    IsStaticCounter_Init(&est->isGyroStatic, IS_STATIC_NUM_FRAMES_THRESHOLD);
}

/**
 * *@brief Initialize the GyroscopeBiasEstimator structure.
 * @param est Pointer to GyroscopeBiasEstimator instance.
 * @return void
 */
void GyroscopeBiasEstimator_Init(GyroscopeBiasEstimator *est)
{
    lowPassFilter_init(&est->accelLowPass, ACCEL_LOWPASS_FREQ);
    lowPassFilter_init(&est->gyroLowPass, GYRO_LOWPASS_FREQ);
    lowPassFilter_init(&est->gyroBiasLowPass, GYRO_BIAS_LOWPASS_FREQ);
    IsStaticCounter_Init(&est->isAccelStatic, IS_STATIC_NUM_FRAMES_THRESHOLD);
    IsStaticCounter_Init(&est->isGyroStatic, IS_STATIC_NUM_FRAMES_THRESHOLD);
    GyroscopeBiasEstimator_Reset(est);
}

/**
 * *@brief Process a new gyroscope reading for bias estimation.
 * @param est Pointer to GyroscopeBiasEstimator instance.
 * @param gyro Pointer to current gyroscope reading.
 * @param timestampNs Current timestamp in nanoseconds.
 * @return void
 */
void GyroscopeBiasEstimator_ProcessGyroscope(GyroscopeBiasEstimator *est, const Vector3d *gyro, int64_t timestampNs)
{
    lowPassFilter_addSample(&est->gyroLowPass, gyro, timestampNs);
    Vector3d_sub(gyro, &est->gyroLowPass.filteredData, &est->smoothedGyroDiff);
    uint8_t gyroStatic = Vector3d_length(&est->smoothedGyroDiff) < GYRO_DIFF_STATIC_THRESHOLD ? 1 : 0;
    IsStaticCounter_AppendFrame(&est->isGyroStatic, gyroStatic);

    if (IsStaticCounter_IsRecentlyStatic(&est->isGyroStatic) && IsStaticCounter_IsRecentlyStatic(&est->isAccelStatic))
        GyroscopeBiasEstimator_UpdateBias(est, gyro, timestampNs);
}

/**
 * *@brief Process a new accelerometer reading for bias estimation.
 * @param est Pointer to GyroscopeBiasEstimator instance.
 * @param accel Pointer to current accelerometer reading.
 * @param timestampNs Current timestamp in nanoseconds.
 * @return void
 */
void GyroscopeBiasEstimator_ProcessAccelerometer(GyroscopeBiasEstimator *est, const Vector3d *accel, int64_t timestampNs)
{
    lowPassFilter_addSample(&est->accelLowPass, accel, timestampNs);
    Vector3d_sub(accel, &est->accelLowPass.filteredData, &est->smoothedAccelDiff);
    uint8_t accelStatic = Vector3d_length(&est->smoothedAccelDiff) < ACCEL_DIFF_STATIC_THRESHOLD ? 1 : 0;
    IsStaticCounter_AppendFrame(&est->isAccelStatic, accelStatic);
}

/**
 * *@brief Retrieve the current gyroscope bias estimate.
 * @param est Pointer to GyroscopeBiasEstimator instance.
 * @param result Pointer to Vector3d to store the bias estimate.
 * @return void
 */
void GyroscopeBiasEstimator_GetGyroBias(GyroscopeBiasEstimator *est, Vector3d *result)
{
    if (est->gyroBiasLowPass.numSamples < NUM_GYRO_BIAS_SAMPLES_THRESHOLD)
    {
        Vector3d_setZero(result);
        return;
    }
    *result = est->gyroBiasLowPass.filteredData;
    float rampUpRatio = FMIN(1.0,(est->gyroBiasLowPass.numSamples - NUM_GYRO_BIAS_SAMPLES_THRESHOLD) / NUM_GYRO_BIAS_SAMPLES_INITIAL_SMOOTHING);
    Vector3d_scale(result, rampUpRatio);
}
