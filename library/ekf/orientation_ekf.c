/**
 * @file orientation_ekf.c
 * @brief Orientation Extended Kalman Filter (EKF) implementation.
 * @author ChenChen
 * @date 2025-11-10
 * @version V1.0
 *
 * This file contains the implementation of an Extended Kalman Filter (EKF)
 * for estimating orientation using gyroscope and accelerometer data.
 */

#include "orientation_ekf.h"
#include "stdio.h"

OrientationEKF ekf;

// #define MP_INIT_VALUE 0.55f
// #define MQ_INIT_VALUE 0.1f
// #define MR_INIT_VALUE 0.3f
// #define MRACCEL_INIT_VALUE 0.03f
#define MP_INIT_VALUE 1.f
#define MQ_INIT_VALUE 0.1f
#define MR_INIT_VALUE 1.f
#define MRACCEL_INIT_VALUE 0.75f

/**
 * @brief Initialize the Orientation EKF structure.
 * 
 * This function sets up the initial state of the Extended Kalman Filter (EKF),
 * including covariance matrices, state vectors, and rotation matrices.
 * It also initializes the reference gravity and magnetic north vectors.
 *
 * @param ekf Pointer to an OrientationEKF instance.
 */
void OrientationEKF_Init(OrientationEKF *ekf) {
    Matrix3x3d_setIdentity(&ekf->so3SensorFromWorld);
    Matrix3x3d_setIdentity(&ekf->so3LastMotion);
    
    Matrix3x3d_setSameDiagonal(&ekf->mP, MP_INIT_VALUE * MP_INIT_VALUE);       // Initial process covariance
    Matrix3x3d_setSameDiagonal(&ekf->mQ, MQ_INIT_VALUE * MQ_INIT_VALUE);         // Process noise covariance
    Matrix3x3d_setSameDiagonal(&ekf->mR, MR_INIT_VALUE * MR_INIT_VALUE);       // Measurement noise covariance
    Matrix3x3d_setSameDiagonal(&ekf->mRaccel, MRACCEL_INIT_VALUE * MRACCEL_INIT_VALUE);  // Accelerometer noise covariance

    Matrix3x3d_zero(&ekf->mS);
    Matrix3x3d_zero(&ekf->mH);
    Matrix3x3d_zero(&ekf->mK);
    Vector3d_setZero(&ekf->mNu);
    Vector3d_setZero(&ekf->mz);
    Vector3d_setZero(&ekf->mh);
    Vector3d_setZero(&ekf->mu);
    Vector3d_setZero(&ekf->mx);
    Vector3d_setZero(&ekf->lastGyro);

    // Reference gravity (Z-axis down)
    Vector3d_set(&ekf->down, 0.0f, 0.0f, 9.81f);
    // Reference north (Y-axis)
    Vector3d_set(&ekf->north, 0.0f, 1.0f, 0.0f);

    ekf->alignedToGravity = 0;
    ekf->alignedToNorth = 0;
    ekf->previousAccelNorm = 0.0f;
    ekf->movingAverageAccelNormChange = 0.0f;
    ekf->timestepFilterInit = 0;
    ekf->numGyroTimestepSamples = 0;
    ekf->gyroFilterValid = 1;
}

/**
 * @brief Update accelerometer noise covariance matrix dynamically.
 *
 * Adjusts the covariance of accelerometer measurements based on the
 * change rate of the accelerometer magnitude. Larger variations in
 * acceleration imply higher measurement uncertainty.
 *
 * @param currentAccelNorm Current magnitude of the acceleration vector.
 */

void updateAccelCovariance(float currentAccelNorm)
{
    float currentAccelNormChange = fabsf(currentAccelNorm - ekf.previousAccelNorm);
    ekf.previousAccelNorm = currentAccelNorm;

    const float kSmoothingFactor = 0.5f;
    ekf.movingAverageAccelNormChange =
        kSmoothingFactor * currentAccelNormChange + (1 - kSmoothingFactor) * ekf.movingAverageAccelNormChange;

    const float kMaxAccelNormChange = 0.15f;
    float normChangeRatio = ekf.movingAverageAccelNormChange / kMaxAccelNormChange;
    normChangeRatio = normChangeRatio > 1.0f ? 1.0f : (normChangeRatio < 0.0f ? 0.0f : normChangeRatio);

    float accelNoiseSigma = MIN_ACCEL_NOISE_SIGMA +
                            normChangeRatio * (MAX_ACCEL_NOISE_SIGMA - MIN_ACCEL_NOISE_SIGMA);

    Matrix3x3d_setSameDiagonal(&ekf.mRaccel, accelNoiseSigma * accelNoiseSigma);
}

/**
 * @brief Apply a low-pass filter to the gyroscope timestep.
 *
 * Used to smooth the measured time intervals between gyro samples,
 * improving the stability of the integration step.
 *
 * @param timeStep Measured time difference between consecutive gyro samples (in seconds).
 */
void filterGyroTimestep(float timeStep) {
    const float kFilterCoeff = 0.95f;
    const float kMinSamples = 10.0f;

    if (!ekf.timestepFilterInit) {
        ekf.filteredGyroTimestep = timeStep;
        ekf.numGyroTimestepSamples = 1;
        ekf.timestepFilterInit = 1;
    } else {
        ekf.filteredGyroTimestep =
            kFilterCoeff * ekf.filteredGyroTimestep + (1.0f - kFilterCoeff) * timeStep;

        if (++ekf.numGyroTimestepSamples > kMinSamples) {
            ekf.gyroFilterValid = 1;
        }
    }
}

/**
 * @brief Update covariance matrix after applying gyroscope motion model.
 */
void updateCovariancesAfterMotion() 
{
    Matrix3x3d_transposeTo(&ekf.so3LastMotion, &ekf.updateCovariancesAfterMotionTempM1);
    Matrix3x3d_mult(&ekf.mP, &ekf.updateCovariancesAfterMotionTempM1, &ekf.updateCovariancesAfterMotionTempM2);
    Matrix3x3d_mult(&ekf.so3LastMotion, &ekf.updateCovariancesAfterMotionTempM2, &ekf.mP);
    Matrix3x3d_setIdentity(&ekf.so3LastMotion);
}

/**
 * @brief Compute the accelerometer observation function for Jacobian calculation.
 *
 * This function computes the predicted accelerometer measurement based on
 * the current estimated rotation, used to numerically approximate the
 * measurement Jacobian matrix.
 *
 * @param so3SensorFromWorldPred Predicted rotation matrix from world to sensor frame.
 * @param result Output vector representing the predicted accelerometer observation.
 */
void accObservationFunctionForNumericalJacobian(const Matrix3x3d *so3SensorFromWorldPred, Vector3d *result) {
    Matrix3x3d_multVec(so3SensorFromWorldPred, &ekf.down, &ekf.mh);
    So3Helper_sO3FromTwoVec(&ekf.mh, &ekf.mz, &ekf.accObservationFunctionForNumericalJacobianTempM);
    So3Helper_muFromSO3(&ekf.accObservationFunctionForNumericalJacobianTempM, result);
}

/**
 * @brief Compute the magnetometer observation function for Jacobian calculation.
 *
 * This function computes the predicted magnetometer measurement based on
 * the current estimated rotation, used to numerically approximate the
 * measurement Jacobian matrix.
 *
 * @param so3SensorFromWorldPred Predicted rotation matrix from world to sensor frame.
 * @param result Output vector representing the predicted magnetometer observation.
 */
void magObservationFunctionForNumericalJacobian(const Matrix3x3d *so3SensorFromWorldPred, Vector3d *result) {
    Matrix3x3d_multVec(so3SensorFromWorldPred, &ekf.north, &ekf.mh);
    So3Helper_sO3FromTwoVec(&ekf.mh, &ekf.mz, &ekf.magObservationFunctionForNumericalJacobianTempM);
    So3Helper_muFromSO3(&ekf.magObservationFunctionForNumericalJacobianTempM, result);
}

/**
 * @brief EKF prediction step using gyroscope data.
 *
 * Integrates the gyroscope angular velocity to predict the next rotation state.
 *
 * @param ekf Pointer to EKF instance.
 * @param gyro Pointer to 3D gyroscope measurement vector (rad/s).
 * @param timestampNs Current timestamp in nanoseconds.
 */
void OrientationEKF_UpdateGyro(OrientationEKF *ekf, const Vector3d *gyro, uint64_t timestampNs) {
    const float kTimeThreshold = 0.016f;
    const float kdTdefault = 0.004f;

    if (ekf->sensorTimeStampGyro != 0) {
        float dT = (timestampNs - ekf->sensorTimeStampGyro) * 1e-9f;

        if (dT > kTimeThreshold)
            dT = ekf->gyroFilterValid ? ekf->filteredGyroTimestep : kdTdefault;
        else
            filterGyroTimestep(dT);

        Vector3d_set(&ekf->mu, gyro->x, gyro->y, gyro->z);
        Vector3d_scale(&ekf->mu, -dT);

        So3Helper_sO3FromMu(&ekf->mu, &ekf->so3LastMotion);
        ekf->processGyroTempM1 = ekf->so3SensorFromWorld;
        Matrix3x3d_mult(&ekf->so3LastMotion, &ekf->so3SensorFromWorld, &ekf->processGyroTempM1);
        ekf->so3SensorFromWorld = ekf->processGyroTempM1;

        ekf->Twb = ekf->so3SensorFromWorld;
        Matrix3x3d_transpose(&ekf->Twb);

        updateCovariancesAfterMotion();

        ekf->processGyroTempM2 = ekf->mQ;  
        Matrix3x3d_scale(&ekf->processGyroTempM2, dT * dT);   
        Matrix3x3d_plusEquals(&ekf->mP, &ekf->processGyroTempM2);
    }

    ekf->sensorTimeStampGyro = timestampNs;
    Vector3d_set(&ekf->lastGyro, gyro->x, gyro->y, gyro->z);
}

/**
 * @brief EKF update step using accelerometer data.
 *
 * Uses accelerometer measurements to correct the orientation estimate,
 * aligning the estimated gravity direction with the measured acceleration vector.
 *
 * @param ekf Pointer to EKF instance.
 * @param acc Pointer to accelerometer measurement vector (m/s^2).
 * @param timestampNs Current timestamp in nanoseconds.
 */
void OrientationEKF_UpdateAccel(OrientationEKF *ekf, const Vector3d *acc, uint64_t timestampNs) {
    // Normalize acceleration vector (gravity direction)
    Vector3d_set(&ekf->mz, acc->x, acc->y, acc->z);
    float norm = sqrt(acc->x*acc->x + acc->y*acc->y + acc->z*acc->z);
    updateAccelCovariance(norm);

    if (ekf->alignedToGravity) {
        // Compute numerical Jacobian
        accObservationFunctionForNumericalJacobian(&ekf->so3SensorFromWorld, &ekf->mNu);
        for (int dof = 0; dof < 3; dof++) {
            Vector3d_setZero(&ekf->processAccVDelta);
            Vector3d_setComponent(&ekf->processAccVDelta, dof, 1E-7f);
            So3Helper_sO3FromMu(&ekf->processAccVDelta, &ekf->processAccTempM1);
            Matrix3x3d_mult(&ekf->processAccTempM1, &ekf->so3SensorFromWorld, &ekf->processAccTempM2);
            accObservationFunctionForNumericalJacobian(&ekf->processAccTempM2, &ekf->processAccTempV1);
            Vector3d_sub(&ekf->mNu, &ekf->processAccTempV1, &ekf->processAccTempV2);
            Vector3d_scale(&ekf->processAccTempV2, 1.0f / EPSILON);
            Matrix3x3d_setColumn(&ekf->mH, dof, &ekf->processAccTempV2);
        }

        // Standard EKF update equations
        Matrix3x3d_transposeTo(&ekf->mH, &ekf->processAccTempM3);
        Matrix3x3d_mult(&ekf->mP, &ekf->processAccTempM3, &ekf->processAccTempM4);
        Matrix3x3d_mult(&ekf->mH, &ekf->processAccTempM4, &ekf->processAccTempM5);
        Matrix3x3d_add(&ekf->processAccTempM5, &ekf->mRaccel, &ekf->mS);
        Matrix3x3d_invert(&ekf->mS, &ekf->processAccTempM3);
        Matrix3x3d_transposeTo(&ekf->mH, &ekf->processAccTempM4);
        Matrix3x3d_mult(&ekf->processAccTempM4, &ekf->processAccTempM3, &ekf->processAccTempM5);
        Matrix3x3d_mult(&ekf->mP, &ekf->processAccTempM5, &ekf->mK);

        Matrix3x3d_multVec(&ekf->mK, &ekf->mNu, &ekf->mx);
        Matrix3x3d_mult(&ekf->mK, &ekf->mH, &ekf->processAccTempM3);
        Matrix3x3d_setIdentity(&ekf->processAccTempM4);
        Matrix3x3d_minusEquals(&ekf->processAccTempM4, &ekf->processAccTempM3);
        Matrix3x3d_mult(&ekf->processAccTempM4, &ekf->mP, &ekf->processAccTempM3);
        ekf->mP = ekf->processAccTempM3;

        So3Helper_sO3FromMu(&ekf->mx, &ekf->so3LastMotion);
        Matrix3x3d_mult(&ekf->so3LastMotion, &ekf->so3SensorFromWorld, &ekf->so3SensorFromWorld);
    } else {
        So3Helper_sO3FromTwoVec(&ekf->down, &ekf->mz, &ekf->so3SensorFromWorld);
        ekf->alignedToGravity = 1;
    }

    ekf->Twb = ekf->so3SensorFromWorld;
    Matrix3x3d_transpose(&ekf->Twb);
}

/**
 * @brief EKF update step using magnetometer data.
 *
 * Uses magnetometer measurements to correct the orientation estimate,
 * aligning the estimated magnetic north direction with the measured magnetic field vector.
 *
 * @param ekf Pointer to EKF instance.
 * @param mag Pointer to magnetometer measurement vector (microtesla).
 * @param timestampNs Current timestamp in nanoseconds.
 */
void OrientationEKF_UpdateMag(OrientationEKF *ekf, const Vector3d *mag, uint64_t timestampNs) {
    if (ekf->alignedToGravity)
    {
        Vector3d_set(&ekf->mz, mag->x, mag->y, mag->z);
        Vector3d_normalize(&ekf->mz);

        Vector3d downInSensorFrame;
        Matrix3x3d_getColumn(&ekf->so3SensorFromWorld, 2, &downInSensorFrame);
        Vector3d_cross(&ekf->mz, &downInSensorFrame, &ekf->processMagTempV1);
        Vector3d_normalize(&ekf->processMagTempV1);
        Vector3d_cross(&downInSensorFrame, &ekf->processMagTempV1, &ekf->processMagTempV2);
        Vector3d_normalize(&ekf->processMagTempV2);
        Vector3d_set(&ekf->mz, ekf->processMagTempV2.x, ekf->processMagTempV2.y, ekf->processMagTempV2.z);

        if (ekf->alignedToNorth)
        {
            magObservationFunctionForNumericalJacobian(&ekf->so3SensorFromWorld, &ekf->mNu);
            const float eps = 1E-7f;
            for (int dof = 0; dof < 3; ++dof) {
                Vector3d_setZero(&ekf->processMagTempV3);
                Vector3d_setComponent(&ekf->processMagTempV3, dof, eps);
                So3Helper_sO3FromMu(&ekf->processMagTempV3, &ekf->processMagTempM1);
                Matrix3x3d_mult(&ekf->processMagTempM1, &ekf->so3SensorFromWorld, &ekf->processMagTempM2);
                magObservationFunctionForNumericalJacobian(&ekf->processMagTempM2, &ekf->processMagTempV4);
                Vector3d_sub(&ekf->mNu, &ekf->processMagTempV4, &ekf->processMagTempV5);
                Vector3d_scale(&ekf->processMagTempV5, 1.0f / eps);
                Matrix3x3d_setColumn(&ekf->mH, dof, &ekf->processMagTempV5);
            }
            Matrix3x3d_transposeTo(&ekf->mH, &ekf->processMagTempM4);
            Matrix3x3d_mult(&ekf->mP, &ekf->processMagTempM4, &ekf->processMagTempM5);
            Matrix3x3d_mult(&ekf->mH, &ekf->processMagTempM5, &ekf->processMagTempM6);
            Matrix3x3d_add(&ekf->processMagTempM6, &ekf->mR, &ekf->mS);

            Matrix3x3d_invert(&ekf->mS, &ekf->processMagTempM4);
            Matrix3x3d_transposeTo(&ekf->mH, &ekf->processMagTempM5);
            Matrix3x3d_mult(&ekf->processMagTempM5, &ekf->processMagTempM4, &ekf->processMagTempM6);
            Matrix3x3d_mult(&ekf->mP, &ekf->processMagTempM6, &ekf->mK);
            Matrix3x3d_multVec(&ekf->mK, &ekf->mNu, &ekf->mx);
            Matrix3x3d_mult(&ekf->mK, &ekf->mH, &ekf->processMagTempM4);
            Matrix3x3d_setIdentity(&ekf->processMagTempM5);
            Matrix3x3d_minusEquals(&ekf->processMagTempM5, &ekf->processMagTempM4);
            Matrix3x3d_mult(&ekf->processMagTempM5, &ekf->mP, &ekf->processMagTempM4);
            ekf->mP = ekf->processMagTempM4;

            So3Helper_sO3FromMu(&ekf->mx, &ekf->so3LastMotion);
            Matrix3x3d_mult(&ekf->so3LastMotion, &ekf->so3SensorFromWorld, &ekf->processMagTempM4);
            ekf->so3SensorFromWorld = ekf->processMagTempM4;
            updateCovariancesAfterMotion();
        }else
        {
            magObservationFunctionForNumericalJacobian(&ekf->so3SensorFromWorld, &ekf->mNu);
            So3Helper_sO3FromMu(&ekf->mNu, &ekf->so3LastMotion);
            Matrix3x3d_mult(&ekf->so3LastMotion, &ekf->so3SensorFromWorld, &ekf->processMagTempM4);
            ekf->so3SensorFromWorld = ekf->processMagTempM4;
            updateCovariancesAfterMotion();
            ekf->alignedToNorth = 1;
        }
    }
}

/**
 * @brief Convert quaternion to roll, pitch, yaw angles.
 *
 * @param quanterion Input quaternion array [qw, qx, qy, qz].
 * @param rpy Output array for roll, pitch, yaw angles (in radians).
 */
void OrientationEKF_GetRPY(const float* quanterion, float *rpy) {
    float qw = quanterion[0];
    float qx = quanterion[1];
    float qy = quanterion[2];
    float qz = quanterion[3];
    float *roll = rpy;
    float *pitch = rpy + 1;
    float *yaw = rpy + 2;

    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qw * qy - qz * qx);
    *pitch = asin(sinp);

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief Extract quaternion from rotation matrix.
 *
 * @param ekf Pointer to EKF instance.
 * @param qw,qx,qy,qz Output quaternion components.
 */
void OrientationEKF_GetQuaternion(OrientationEKF *ekf, float * quaternion) {
    Matrix3x3d R; 
    Matrix3x3d_transposeTo(&ekf->so3SensorFromWorld, &R);
    float* qw = quaternion;
    float* qx = quaternion + 1;
    float* qy = quaternion + 2;
    float* qz = quaternion + 3;
    float trace = R.m[0][0] + R.m[1][1] + R.m[2][2];
    if(trace > 0.0f){
        float s = 0.5f / sqrt(trace + 1.0f);
        *qw = 0.25f / s;
        *qx = (R.m[2][1] - R.m[1][2]) * s;
        *qy = (R.m[0][2] - R.m[2][0]) * s;
        *qz = (R.m[1][0] - R.m[0][1]) * s;
    } else if(R.m[0][0] > R.m[1][1] && R.m[0][0] > R.m[2][2]){
        float s = 2.0f * sqrt(1.0f + R.m[0][0] - R.m[1][1] - R.m[2][2]);
        *qw = (R.m[2][1] - R.m[1][2]) / s;
        *qx = 0.25f * s;
        *qy = (R.m[0][1] + R.m[1][0]) / s;
        *qz = (R.m[0][2] + R.m[2][0]) / s;
    } else if(R.m[1][1] > R.m[2][2]){
        float s = 2.0f * sqrt(1.0f + R.m[1][1] - R.m[0][0] - R.m[2][2]);
        *qw = (R.m[0][2] - R.m[2][0]) / s;
        *qx = (R.m[0][1] + R.m[1][0]) / s;
        *qy = 0.25f * s;
        *qz = (R.m[1][2] + R.m[2][1]) / s;
    } else {
        float s = 2.0f * sqrt(1.0f + R.m[2][2] - R.m[0][0] - R.m[1][1]);
        *qw = (R.m[1][0] - R.m[0][1]) / s;
        *qx = (R.m[0][2] + R.m[2][0]) / s;
        *qy = (R.m[1][2] + R.m[2][1]) / s;
        *qz = 0.25f * s;
    }

    // Normalize quaternion to avoid numerical drift
    float qnorm = sqrt((*qw)*(*qw) + (*qx)*(*qx) + (*qy)*(*qy) + (*qz)*(*qz));
    if (qnorm > 1e-6f) {
        *qw /= qnorm; *qx /= qnorm; *qy /= qnorm; *qz /= qnorm;
    } else {
        *qw = 1.0f; *qx = *qy = *qz = 0.0f;
    }

}
