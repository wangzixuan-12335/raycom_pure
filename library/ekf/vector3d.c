/**
 * @file vector3d.c
 * @brief 3D vector operations with float precision
 * @author ChenChen
 * @note This file provides basic operations for 3D vectors using float data type.
 */

#include "vector3d.h"
#include <string.h>

/**
 * @brief Set the components of a 3D vector.
 * @param v   Pointer to the target vector.
 * @param x   X component.
 * @param y   Y component.
 * @param z   Z component.
 */
void Vector3d_set(Vector3d *v, float x, float y, float z) {
    v->x = x; 
    v->y = y; 
    v->z = z;
}

/**
 * @brief Copy the contents of one vector to another.
 * @param dst  Destination vector.
 * @param src  Source vector.
 */
void Vector3d_copy(Vector3d *dst, const Vector3d *src) {
    memcpy(dst, src, sizeof(Vector3d));
}

/**
 * @brief Set all components of the vector to zero.
 * @param v  Pointer to the vector.
 */
void Vector3d_setZero(Vector3d *v) {
    v->x = v->y = v->z = 0.0f;
}

/**
 * @brief Scale a vector by a scalar factor.
 * @param v  Pointer to the vector.
 * @param s  Scale factor.
 */
void Vector3d_scale(Vector3d *v, float s) {
    v->x *= s; 
    v->y *= s; 
    v->z *= s;
}

/**
 * @brief Compute the Euclidean length (magnitude) of a vector.
 * @param v  Pointer to the vector.
 * @return The vector’s length.
 */
float Vector3d_length(const Vector3d *v) {
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

/**
 * @brief Normalize the vector (make its length = 1).
 * @note  Does nothing if the length is smaller than EPSILON.
 * @param v  Pointer to the vector to normalize.
 */
void Vector3d_normalize(Vector3d *v) {
    float d = Vector3d_length(v);
    if (fabsf(d) > EPSILON) 
        Vector3d_scale(v, 1.0f / d);
}

/**
 * @brief Compute the dot product of two vectors.
 * @param a  First vector.
 * @param b  Second vector.
 * @return The scalar dot product result.
 */
float Vector3d_dot(const Vector3d *a, const Vector3d *b) {
    return a->x*b->x + a->y*b->y + a->z*b->z;
}

/**
 * @brief Compute the sum of two vectors.
 * @param a  First vector.
 * @param b  Second vector.
 * @param r  Resulting vector (a + b).
 */
void Vector3d_add(const Vector3d *a, const Vector3d *b, Vector3d *r) {
    r->x = a->x + b->x;
    r->y = a->y + b->y;
    r->z = a->z + b->z;
}

/**
 * @brief Compute the difference between two vectors.
 * @param a  First vector.
 * @param b  Second vector.
 * @param r  Resulting vector (a - b).
 */
void Vector3d_sub(const Vector3d *a, const Vector3d *b, Vector3d *r) {
    r->x = a->x - b->x;
    r->y = a->y - b->y;
    r->z = a->z - b->z;
}

/**
 * @brief Compute the cross product of two vectors.
 * @param a  First vector.
 * @param b  Second vector.
 * @param r  Resulting vector (a × b).
 */
void Vector3d_cross(const Vector3d *a, const Vector3d *b, Vector3d *r) {
    r->x = a->y*b->z - a->z*b->y;
    r->y = a->z*b->x - a->x*b->z;
    r->z = a->x*b->y - a->y*b->x;
}

/**
 * @brief Find the index of the component with the largest absolute value.
 * @param v  Pointer to the vector.
 * @return Index of the component (0 for x, 1 for y, 2 for z).
 */
int Vector3d_largestAbsComponent(const Vector3d *v) {
    float ax = fabsf(v->x), ay = fabsf(v->y), az = fabsf(v->z);
    if (ax > ay) return (ax > az) ? 0 : 2;
    else return (ay > az) ? 1 : 2;
}

/**
 * @brief Compute a normalized vector orthogonal to the given one.
 * @param v  Input vector.
 * @param r  Output orthogonal unit vector.
 */
void Vector3d_ortho(const Vector3d *v, Vector3d *r) {
    int k = Vector3d_largestAbsComponent(v) - 1;
    if (k < 0) k = 2;
    Vector3d_setZero(r);
    if (k == 0) r->x = 1.0f;
    else if (k == 1) r->y = 1.0f;
    else r->z = 1.0f;
    Vector3d_cross(v, r, r);
    Vector3d_normalize(r);
}

/**
 * @brief Set a specific component of the vector.
 * @param v    Pointer to the vector.
 * @param i    Component index (0=x, 1=y, 2=z).
 * @param val  Value to assign.
 */
void Vector3d_setComponent(Vector3d *v, int i, float val) {
    if (i == 0)
        v->x = val;
    else if (i == 1)
        v->y = val;
    else
        v->z = val;
}
