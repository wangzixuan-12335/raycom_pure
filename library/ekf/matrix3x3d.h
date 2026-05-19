#pragma once
#ifndef MATRIX3X3D_H
#define MATRIX3X3D_H

#include "vector3d.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float m[3][3]; // row-major
} Matrix3x3d;

void Matrix3x3d_set(Matrix3x3d *mat, float m00,float m01,float m02,float m10,float m11,float m12,float m20,float m21,float m22);
void Matrix3x3d_zero(Matrix3x3d *mat);
void Matrix3x3d_setIdentity(Matrix3x3d *mat);
void Matrix3x3d_setSameDiagonal(Matrix3x3d *mat, float d);
float Matrix3x3d_get(const Matrix3x3d *mat, int row, int col);
void Matrix3x3d_setElement(Matrix3x3d *mat, int row, int col, float val);
void Matrix3x3d_getColumn(const Matrix3x3d *mat, int col, Vector3d *v);
void Matrix3x3d_setColumn(Matrix3x3d *mat, int col, const Vector3d *v);
void Matrix3x3d_scale(Matrix3x3d *mat, float s);
void Matrix3x3d_add(const Matrix3x3d *a, const Matrix3x3d *b, Matrix3x3d *r);
void Matrix3x3d_mult(const Matrix3x3d *a, const Matrix3x3d *b, Matrix3x3d *r);
void Matrix3x3d_multVec(const Matrix3x3d *a, const Vector3d *v, Vector3d *r);
float Matrix3x3d_determinant(const Matrix3x3d *mat);
int Matrix3x3d_invert(const Matrix3x3d *mat, Matrix3x3d *result); // returns 1 if success
void Matrix3x3d_transpose(Matrix3x3d *mat);
void Matrix3x3d_transposeTo(const Matrix3x3d *mat, Matrix3x3d *result);
void Matrix3x3d_plusEquals(Matrix3x3d *a, const Matrix3x3d *b);
void Matrix3x3d_minusEquals(Matrix3x3d *a, const Matrix3x3d *b);

#ifdef __cplusplus
}
#endif

#endif
