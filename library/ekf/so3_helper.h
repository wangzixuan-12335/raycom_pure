#pragma once
#ifndef SO3_HELPER_H
#define SO3_HELPER_H

#include "vector3d.h"
#include "matrix3x3d.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Vector3d temp31;
    Vector3d sO3FromTwoVecN;
    Vector3d sO3FromTwoVecA;
    Vector3d sO3FromTwoVecB;
    Vector3d sO3FromTwoVecRotationAxis;
    Matrix3x3d sO3FromTwoVec33R1;
    Matrix3x3d sO3FromTwoVec33R2;
    Vector3d muFromSO3R2;
    Vector3d rotationPiAboutAxisTemp;
} So3Helper;

extern So3Helper so3Helper;

void So3Helper_init(So3Helper *h);
void So3Helper_sO3FromMu(const Vector3d *w, Matrix3x3d *result);
void So3Helper_sO3FromTwoVec(const Vector3d *a, const Vector3d *b, Matrix3x3d *result);
void So3Helper_muFromSO3(const Matrix3x3d *so3, Vector3d *result);
void So3Helper_rotationPiAboutAxis( const Vector3d *v, Matrix3x3d *result);

#ifdef __cplusplus
}
#endif

#endif
