#ifndef ORIENTATION_EKF_H
#define ORIENTATION_EKF_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "matrix3x3d.h"
#include "vector3d.h"
#include "so3_helper.h"

#define MAX_ACCEL_NOISE_SIGMA 7e0f
#define MIN_ACCEL_NOISE_SIGMA 1e0f

typedef struct {
    Vector3d mNu;
    Vector3d mz;
    Vector3d mh;
    Vector3d mu;
    Vector3d mx;
    Vector3d down;
    Vector3d north;
    Vector3d lastGyro;
    Vector3d processAccTempV1;
    Vector3d processAccTempV2;
    Vector3d processAccVDelta;
    Vector3d processMagTempV1;
    Vector3d processMagTempV2;
    Vector3d processMagTempV3;
    Vector3d processMagTempV4;
    Vector3d processMagTempV5;

    Matrix3x3d so3SensorFromWorld;
    Matrix3x3d Twb;
    Matrix3x3d so3LastMotion;
    Matrix3x3d mP;
    Matrix3x3d mQ;
    Matrix3x3d mR;
    Matrix3x3d mRaccel;
    Matrix3x3d mS;
    Matrix3x3d mH;
    Matrix3x3d mK;

    Matrix3x3d processGyroTempM1;
    Matrix3x3d processGyroTempM2;
    Matrix3x3d processAccTempM1;
    Matrix3x3d processAccTempM2;
    Matrix3x3d processAccTempM3;
    Matrix3x3d processAccTempM4;
    Matrix3x3d processAccTempM5;
    Matrix3x3d processMagTempM1;
    Matrix3x3d processMagTempM2;
    Matrix3x3d processMagTempM4;
    Matrix3x3d processMagTempM5;
    Matrix3x3d processMagTempM6;
    Matrix3x3d updateCovariancesAfterMotionTempM1;
    Matrix3x3d updateCovariancesAfterMotionTempM2;
    Matrix3x3d accObservationFunctionForNumericalJacobianTempM;
    Matrix3x3d magObservationFunctionForNumericalJacobianTempM;

    uint64_t sensorTimeStampGyro;
    float previousAccelNorm;
    float movingAverageAccelNormChange;
    float filteredGyroTimestep;
    int timestepFilterInit;
    int numGyroTimestepSamples;
    int gyroFilterValid;
    int alignedToGravity;
    int alignedToNorth;
} OrientationEKF;

extern OrientationEKF ekf;

void OrientationEKF_Init(OrientationEKF *ekf);
void accObservationFunctionForNumericalJacobian(const Matrix3x3d *so3SensorFromWorldPred, Vector3d *result);
void updateAccelCovariance(float currentAccelNorm);
void OrientationEKF_UpdateGyro(OrientationEKF *ekf, const Vector3d *gyro, uint64_t timestampNs);
void OrientationEKF_UpdateAccel(OrientationEKF *ekf, const Vector3d *acc, uint64_t timestampNs);
void OrientationEKF_UpdateMag(OrientationEKF *ekf, const Vector3d *mag, uint64_t timestampNs);
void OrientationEKF_GetRPY(const float* quanterion, float *rpy);
void OrientationEKF_GetQuaternion(OrientationEKF *ekf, float * quaternion);

#endif
