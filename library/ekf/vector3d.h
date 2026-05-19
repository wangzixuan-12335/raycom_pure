#pragma once
#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <math.h>
#define M_PI    3.14159265358979323846f

#ifdef __cplusplus
extern "C" {
#endif

#define     EPSILON     1e-7f

typedef struct {
    float x;
    float y;
    float z;
} Vector3d;

void Vector3d_set(Vector3d *v, float x, float y, float z);
void Vector3d_copy(Vector3d *dst, const Vector3d *src);
void Vector3d_setZero(Vector3d *v);
void Vector3d_scale(Vector3d *v, float s);
float Vector3d_length(const Vector3d *v);
void Vector3d_normalize(Vector3d *v);
float Vector3d_dot(const Vector3d *a, const Vector3d *b);
void Vector3d_add(const Vector3d *a, const Vector3d *b, Vector3d *r);
void Vector3d_sub(const Vector3d *a, const Vector3d *b, Vector3d *r);
void Vector3d_cross(const Vector3d *a, const Vector3d *b, Vector3d *r);
int Vector3d_largestAbsComponent(const Vector3d *v);
void Vector3d_ortho(const Vector3d *v, Vector3d *r);
void Vector3d_setComponent(Vector3d *v, int i, float val);

#ifdef __cplusplus
}
#endif

#endif
