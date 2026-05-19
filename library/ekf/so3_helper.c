/**
 * @file       so3_helper.c
 * @brief      Implementation of SO(3) helper functions for rotation operations.
 * @details    This module provides basic operations on 3D rotations represented in SO(3),
 *             including exponential and logarithmic maps, conversion between vector and
 *             rotation matrix, and constructing rotation matrices from two vectors.
 * @author     Chen Chen
 */

#include "so3_helper.h"
#include <string.h>
#include <math.h>

/** 
 * @brief Global SO(3) helper instance 
 */
So3Helper so3Helper;

/**
 * @brief Initialize an SO(3) helper instance.
 * @param h Pointer to a So3Helper structure to be initialized.
 */
void So3Helper_init(So3Helper *h) {
    memset(h, 0, sizeof(So3Helper));
}

/**
 * @brief Internal function to compute the Rodrigues exponential map.
 * @param w   Input rotation vector (axis * angle).
 * @param kA  Sine-based coefficient.
 * @param kB  Cosine-based coefficient.
 * @param result Pointer to the resulting 3×3 rotation matrix.
 */
static void rodriguesSo3Exp_impl(const Vector3d *w, float kA, float kB, Matrix3x3d *result) {
    float wx2 = w->x * w->x;
    float wy2 = w->y * w->y;
    float wz2 = w->z * w->z;
    Matrix3x3d_set(result,
        1.0f - kB * (wy2 + wz2), kB * (w->x * w->y) - kA * w->z, kB * (w->x * w->z) + kA * w->y,
        kB * (w->x * w->y) + kA * w->z, 1.0f - kB * (wx2 + wz2), kB * (w->y * w->z) - kA * w->x,
        kB * (w->x * w->z) - kA * w->y, kB * (w->y * w->z) + kA * w->x, 1.0f - kB * (wx2 + wy2));
}

/**
 * @brief Compute the exponential map from a 3D rotation vector to an SO(3) matrix.
 * @param w       Input rotation vector.
 * @param result  Output rotation matrix.
 */
void So3Helper_sO3FromMu(const Vector3d *w, Matrix3x3d *result) {
    float thetaSq = Vector3d_dot(w, w);
    float theta = sqrt(thetaSq);
    float kA, kB;

    if (thetaSq < 1e-8f) {  // Small angle approximation
        kA = 1.0f - 0.1666666716337204f * thetaSq;
        kB = 0.5f;
    } else if (thetaSq < 1e-6f) {
        kB = 0.5f - 0.0416666679084301f * thetaSq;
        kA = 1.0f - thetaSq * 0.1666666716337204f * (1.0f - 0.1666666716337204f * thetaSq);
    } else {
        float invTheta = 1.0f / theta;
        kA = sin(theta) * invTheta;
        kB = (1.0f - cos(theta)) * (invTheta * invTheta);
    }
    rodriguesSo3Exp_impl(w, kA, kB, result);
}

/**
 * @brief Generate a rotation matrix representing a π rotation about the given axis.
 * @param v       Rotation axis.
 * @param result  Output rotation matrix.
 */
void So3Helper_rotationPiAboutAxis(const Vector3d *v, Matrix3x3d *result) {
    Vector3d tmp = *v;
    float len = Vector3d_length(&tmp);
    if (fabsf(len) < EPSILON) {
        Matrix3x3d_setIdentity(result);
        return;
    }
    Vector3d_scale(&tmp, M_PI / len);
    rodriguesSo3Exp_impl(&tmp, 0.0f, 0.20264236728467558, result);
}

/**
 * @brief Construct an SO(3) rotation that rotates vector a to vector b.
 * @param a       Source vector.
 * @param b       Target vector.
 * @param result  Output rotation matrix.
 */
void So3Helper_sO3FromTwoVec(const Vector3d *a, const Vector3d *b, Matrix3x3d *result) {
    Vector3d_cross(a, b, &so3Helper.sO3FromTwoVecN);
    float nlen = Vector3d_length(&so3Helper.sO3FromTwoVecN);

    if (fabsf(nlen) < 10e-7f) {
        float dot = Vector3d_dot(a, b);
        if (dot >= EPSILON) {
            Matrix3x3d_setIdentity(result);
        } else {
            Vector3d_ortho(a, &so3Helper.sO3FromTwoVecRotationAxis);
            So3Helper_rotationPiAboutAxis(&so3Helper.sO3FromTwoVecRotationAxis, result);
        }
        return;
    }

    Vector3d_copy(&so3Helper.sO3FromTwoVecA, a);
    Vector3d_copy(&so3Helper.sO3FromTwoVecB, b);
    Vector3d_normalize(&so3Helper.sO3FromTwoVecN);
    Vector3d_normalize(&so3Helper.sO3FromTwoVecA);
    Vector3d_normalize(&so3Helper.sO3FromTwoVecB);

    Matrix3x3d_setColumn(&so3Helper.sO3FromTwoVec33R1, 0, &so3Helper.sO3FromTwoVecA);
    Matrix3x3d_setColumn(&so3Helper.sO3FromTwoVec33R1, 1, &so3Helper.sO3FromTwoVecN);
    Vector3d_cross(&so3Helper.sO3FromTwoVecN, &so3Helper.sO3FromTwoVecA, &so3Helper.temp31);
    Matrix3x3d_setColumn(&so3Helper.sO3FromTwoVec33R1, 2, &so3Helper.temp31);

    Matrix3x3d_setColumn(&so3Helper.sO3FromTwoVec33R2, 0, &so3Helper.sO3FromTwoVecB);
    Matrix3x3d_setColumn(&so3Helper.sO3FromTwoVec33R2, 1, &so3Helper.sO3FromTwoVecN);
    Vector3d_cross(&so3Helper.sO3FromTwoVecN, &so3Helper.sO3FromTwoVecB, &so3Helper.temp31);
    Matrix3x3d_setColumn(&so3Helper.sO3FromTwoVec33R2, 2, &so3Helper.temp31);

    Matrix3x3d_transposeTo(&so3Helper.sO3FromTwoVec33R1, &so3Helper.sO3FromTwoVec33R1);
    Matrix3x3d_mult(&so3Helper.sO3FromTwoVec33R2, &so3Helper.sO3FromTwoVec33R1, result);
}

/**
 * @brief Compute the logarithmic map from SO(3) matrix to 3D rotation vector.
 * @param so3     Input rotation matrix.
 * @param result  Output rotation vector.
 */
void So3Helper_muFromSO3(const Matrix3x3d *so3, Vector3d *result) {
    float cosAngle = (Matrix3x3d_get(so3,0,0) + Matrix3x3d_get(so3,1,1) + Matrix3x3d_get(so3,2,2) - 1.0f) * 0.5f;

    result->x = (Matrix3x3d_get(so3,2,1) - Matrix3x3d_get(so3,1,2)) / 2.0f;
    result->y = (Matrix3x3d_get(so3,0,2) - Matrix3x3d_get(so3,2,0)) / 2.0f;
    result->z = (Matrix3x3d_get(so3,1,0) - Matrix3x3d_get(so3,0,1)) / 2.0f;

    float sinAngleAbs = Vector3d_length(result);

    if (cosAngle > 0.7071067811865476f) {  // small angle
        if (sinAngleAbs > 0.0f)
            Vector3d_scale(result, asin(sinAngleAbs) / sinAngleAbs);
    } else if (cosAngle > -0.7071067811865476f) {
        float angle = acos(cosAngle);
        Vector3d_scale(result, angle / sinAngleAbs);
    } else {
        float angle = M_PI - asin(sinAngleAbs);
        float d0 = Matrix3x3d_get(so3,0,0) - cosAngle;
        float d1 = Matrix3x3d_get(so3,1,1) - cosAngle;
        float d2 = Matrix3x3d_get(so3,2,2) - cosAngle;

        if (d0*d0 > d1*d1 && d0*d0 > d2*d2) {
            Vector3d_set(&so3Helper.muFromSO3R2, d0, (Matrix3x3d_get(so3,1,0) + Matrix3x3d_get(so3,0,1)) / 2.0f,
                         (Matrix3x3d_get(so3,0,2) + Matrix3x3d_get(so3,2,0)) / 2.0f);
        } else if (d1*d1 > d2*d2) {
            Vector3d_set(&so3Helper.muFromSO3R2, (Matrix3x3d_get(so3,1,0) + Matrix3x3d_get(so3,0,1)) / 2.0f,
                         d1, (Matrix3x3d_get(so3,2,1) + Matrix3x3d_get(so3,1,2)) / 2.0f);
        } else {
            Vector3d_set(&so3Helper.muFromSO3R2, (Matrix3x3d_get(so3,0,2) + Matrix3x3d_get(so3,2,0)) / 2.0f,
                         (Matrix3x3d_get(so3,2,1) + Matrix3x3d_get(so3,1,2)) / 2.0f, d2);
        }

        if (Vector3d_dot(&so3Helper.muFromSO3R2, result) < 0.0f)
            Vector3d_scale(&so3Helper.muFromSO3R2, -1.0f);

        Vector3d_normalize(&so3Helper.muFromSO3R2);
        Vector3d_scale(&so3Helper.muFromSO3R2, angle);
        Vector3d_copy(result, &so3Helper.muFromSO3R2);
    }
}
