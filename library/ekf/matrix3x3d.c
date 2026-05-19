/**
 * @file matrix3x3d.c
 * @brief 3x3 Matrix operations with float precision
 * @author ChenChen
 * @note This file provides basic operations for 3x3 matrices using float data type.
 */

#include "matrix3x3d.h"
#include <string.h>


/**
 * @brief Set all elements of the matrix manually.
 */
void Matrix3x3d_set(Matrix3x3d *mat,
                    float m00, float m01, float m02,
                    float m10, float m11, float m12,
                    float m20, float m21, float m22)
{
    mat->m[0][0] = m00; mat->m[0][1] = m01; mat->m[0][2] = m02;
    mat->m[1][0] = m10; mat->m[1][1] = m11; mat->m[1][2] = m12;
    mat->m[2][0] = m20; mat->m[2][1] = m21; mat->m[2][2] = m22;
}

/**
 * @brief Set all matrix elements to zero.
 */
void Matrix3x3d_zero(Matrix3x3d *mat) {
    memset(mat->m, 0, sizeof(mat->m));
}

/**
 * @brief Set matrix to identity matrix.
 */
void Matrix3x3d_setIdentity(Matrix3x3d *mat) {
    Matrix3x3d_zero(mat);
    mat->m[0][0] = mat->m[1][1] = mat->m[2][2] = 1.0f;
}

/**
 * @brief Set a diagonal matrix with the same diagonal value.
 * @param d Value to set along the diagonal.
 */
void Matrix3x3d_setSameDiagonal(Matrix3x3d *mat, float d) {
    Matrix3x3d_zero(mat);
    mat->m[0][0] = mat->m[1][1] = mat->m[2][2] = d;
}

/**
 * @brief Get an element from the matrix.
 */
float Matrix3x3d_get(const Matrix3x3d *mat, int row, int col) {
    return mat->m[row][col];
}

/**
 * @brief Set an element of the matrix.
 */
void Matrix3x3d_setElement(Matrix3x3d *mat, int row, int col, float val) {
    mat->m[row][col] = val;
}

/**
 * @brief Get a column vector from the matrix.
 */
void Matrix3x3d_getColumn(const Matrix3x3d *mat, int col, Vector3d *v) {
    v->x = mat->m[0][col];
    v->y = mat->m[1][col];
    v->z = mat->m[2][col];
}

/**
 * @brief Set a column of the matrix from a 3D vector.
 */
void Matrix3x3d_setColumn(Matrix3x3d *mat, int col, const Vector3d *v) {
    mat->m[0][col] = v->x;
    mat->m[1][col] = v->y;
    mat->m[2][col] = v->z;
}

/**
 * @brief Scale the entire matrix by a scalar.
 */
void Matrix3x3d_scale(Matrix3x3d *mat, float s) {
    mat->m[0][0] *= s; mat->m[0][1] *= s; mat->m[0][2] *= s;
    mat->m[1][0] *= s; mat->m[1][1] *= s; mat->m[1][2] *= s;
    mat->m[2][0] *= s; mat->m[2][1] *= s; mat->m[2][2] *= s;
}

/**
 * @brief Add two matrices element-wise.
 */
void Matrix3x3d_add(const Matrix3x3d *a, const Matrix3x3d *b, Matrix3x3d *r) {
    r->m[0][0] = a->m[0][0] + b->m[0][0];
    r->m[0][1] = a->m[0][1] + b->m[0][1];
    r->m[0][2] = a->m[0][2] + b->m[0][2];
    r->m[1][0] = a->m[1][0] + b->m[1][0];
    r->m[1][1] = a->m[1][1] + b->m[1][1];
    r->m[1][2] = a->m[1][2] + b->m[1][2];
    r->m[2][0] = a->m[2][0] + b->m[2][0];
    r->m[2][1] = a->m[2][1] + b->m[2][1];
    r->m[2][2] = a->m[2][2] + b->m[2][2];
}

/**
 * @brief Multiply two 3x3 matrices: r = a × b.
 */
void Matrix3x3d_mult(const Matrix3x3d *a, const Matrix3x3d *b, Matrix3x3d *r) {
    Matrix3x3d temp; // Temporary to avoid data overwrite

    temp.m[0][0] = a->m[0][0]*b->m[0][0] + a->m[0][1]*b->m[1][0] + a->m[0][2]*b->m[2][0];
    temp.m[0][1] = a->m[0][0]*b->m[0][1] + a->m[0][1]*b->m[1][1] + a->m[0][2]*b->m[2][1];
    temp.m[0][2] = a->m[0][0]*b->m[0][2] + a->m[0][1]*b->m[1][2] + a->m[0][2]*b->m[2][2];

    temp.m[1][0] = a->m[1][0]*b->m[0][0] + a->m[1][1]*b->m[1][0] + a->m[1][2]*b->m[2][0];
    temp.m[1][1] = a->m[1][0]*b->m[0][1] + a->m[1][1]*b->m[1][1] + a->m[1][2]*b->m[2][1];
    temp.m[1][2] = a->m[1][0]*b->m[0][2] + a->m[1][1]*b->m[1][2] + a->m[1][2]*b->m[2][2];

    temp.m[2][0] = a->m[2][0]*b->m[0][0] + a->m[2][1]*b->m[1][0] + a->m[2][2]*b->m[2][0];
    temp.m[2][1] = a->m[2][0]*b->m[0][1] + a->m[2][1]*b->m[1][1] + a->m[2][2]*b->m[2][1];
    temp.m[2][2] = a->m[2][0]*b->m[0][2] + a->m[2][1]*b->m[1][2] + a->m[2][2]*b->m[2][2];

    *r = temp;
}

/**
 * @brief Multiply matrix by a 3D vector: r = A * v.
 */
void Matrix3x3d_multVec(const Matrix3x3d *a, const Vector3d *v, Vector3d *r) {
    r->x = a->m[0][0]*v->x + a->m[0][1]*v->y + a->m[0][2]*v->z;
    r->y = a->m[1][0]*v->x + a->m[1][1]*v->y + a->m[1][2]*v->z;
    r->z = a->m[2][0]*v->x + a->m[2][1]*v->y + a->m[2][2]*v->z;
}

/**
 * @brief Compute determinant of a 3x3 matrix.
 */
float Matrix3x3d_determinant(const Matrix3x3d *m) {
    return m->m[0][0]*(m->m[1][1]*m->m[2][2] - m->m[1][2]*m->m[2][1])
         - m->m[0][1]*(m->m[1][0]*m->m[2][2] - m->m[1][2]*m->m[2][0])
         + m->m[0][2]*(m->m[1][0]*m->m[2][1] - m->m[1][1]*m->m[2][0]);
}

/**
 * @brief Invert a 3x3 matrix.
 * @param m Input matrix.
 * @param r Output inverted matrix.
 * @return 1 if successful, 0 if matrix is singular (non-invertible).
 */
int Matrix3x3d_invert(const Matrix3x3d *m, Matrix3x3d *r) {
    float det = Matrix3x3d_determinant(m);
    if (fabsf(det) < EPSILON) return 0; // Singular matrix

    float inv = 1.0f / det;

    r->m[0][0] = (m->m[1][1]*m->m[2][2] - m->m[1][2]*m->m[2][1]) * inv;
    r->m[0][1] = -(m->m[0][1]*m->m[2][2] - m->m[0][2]*m->m[2][1]) * inv;
    r->m[0][2] = (m->m[0][1]*m->m[1][2] - m->m[0][2]*m->m[1][1]) * inv;

    r->m[1][0] = -(m->m[1][0]*m->m[2][2] - m->m[1][2]*m->m[2][0]) * inv;
    r->m[1][1] = (m->m[0][0]*m->m[2][2] - m->m[0][2]*m->m[2][0]) * inv;
    r->m[1][2] = -(m->m[0][0]*m->m[1][2] - m->m[0][2]*m->m[1][0]) * inv;

    r->m[2][0] = (m->m[1][0]*m->m[2][1] - m->m[1][1]*m->m[2][0]) * inv;
    r->m[2][1] = -(m->m[0][0]*m->m[2][1] - m->m[0][1]*m->m[2][0]) * inv;
    r->m[2][2] = (m->m[0][0]*m->m[1][1] - m->m[0][1]*m->m[1][0]) * inv;

    return 1;
}

/**
 * @brief Transpose a matrix in place.
 */
void Matrix3x3d_transpose(Matrix3x3d *m) {
    if (m == NULL) return;
    float t;
    t = m->m[0][1]; m->m[0][1] = m->m[1][0]; m->m[1][0] = t;
    t = m->m[0][2]; m->m[0][2] = m->m[2][0]; m->m[2][0] = t;
    t = m->m[1][2]; m->m[1][2] = m->m[2][1]; m->m[2][1] = t;
}

/**
 * @brief Compute transpose of a matrix into another matrix.
 */
void Matrix3x3d_transposeTo(const Matrix3x3d *m, Matrix3x3d *r) {
    Matrix3x3d temp;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            temp.m[i][j] = m->m[j][i];
    *r = temp;
}

/**
 * @brief Add matrix b to matrix a (in place): a += b.
 */
void Matrix3x3d_plusEquals(Matrix3x3d *a, const Matrix3x3d *b) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            a->m[i][j] += b->m[i][j];
}

/**
 * @brief Subtract matrix b from matrix a (in place): a -= b.
 */
void Matrix3x3d_minusEquals(Matrix3x3d *a, const Matrix3x3d *b) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            a->m[i][j] -= b->m[i][j];
}
