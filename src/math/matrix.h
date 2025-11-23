/**
 * @file matrix.h
 * @brief This file contains the definition of the 4x4 matrix structure and functions to work with those matrices.
 * 
 */
#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "vector4.h"
#include "vector3.h"

/**
 * @brief The 4x4 matrix structure.
 * 
 */
typedef fm_mat4_t Matrix4x4;

/**
 * @brief 3x3 Matrix with single precision floating point components.
 */
typedef struct {
    float m[3][3];  ///< Two-dimensional array that contains the matrix coefficients in column-major order.
} Matrix3x3;

/**
 * @brief Create a 3x3 identity matrix.
 */
inline void matrix3Identity(Matrix3x3 *out)
{
    *out = (Matrix3x3){};
    out->m[0][0] = 1;
    out->m[1][1] = 1;
    out->m[2][2] = 1;
}

/**
 * @brief Create a 4x4 identity matrix.
 */
inline void matrix4Identity(Matrix4x4 *out)
{
    *out = (Matrix4x4){};
    out->m[0][0] = 1;
    out->m[1][1] = 1;
    out->m[2][2] = 1;
    out->m[3][3] = 1;
}

/**
 * @brief Multiplies a 3x3 matrix by a 3D vector.
 *
 * This function multiplies a 3x3 matrix by a 3D vector, storing the result in a 3D vector.
 *
 * @param matrix The 3x3 matrix to be multiplied.
 * @param input The input 3D vector to be multiplied.
 * @param output The output 3D vector where the result will be stored.
 */
inline void matrix3Vec3Mul(const Matrix3x3* matrix, const Vector3* input, Vector3* output) {
    output->x = matrix->m[0][0] * input->x + matrix->m[1][0] * input->y + matrix->m[2][0] * input->z;
    output->y = matrix->m[0][1] * input->x + matrix->m[1][1] * input->y + matrix->m[2][1] * input->z;
    output->z = matrix->m[0][2] * input->x + matrix->m[1][2] * input->y + matrix->m[2][2] * input->z;
}

/**
 * @brief Multiplies a 4x4 matrix by a 3D vector.
 *
 * This function multiplies a 4x4 matrix by a 3D vector, storing the result in a 4D vector.
 *
 * @param matrix The 4x4 matrix to be multiplied.
 * @param input The input 3D vector to be multiplied.
 * @param output The output 4D vector where the result will be stored.
 */
inline void matrix4Vec3Mul(const Matrix4x4* matrix, const Vector3* input, Vector4* output) {
    output->x = matrix->m[0][0] * input->x + matrix->m[1][0] * input->y + matrix->m[2][0] * input->z + matrix->m[3][0];
    output->y = matrix->m[0][1] * input->x + matrix->m[1][1] * input->y + matrix->m[2][1] * input->z + matrix->m[3][1];
    output->z = matrix->m[0][2] * input->x + matrix->m[1][2] * input->y + matrix->m[2][2] * input->z + matrix->m[3][2];
    output->w = matrix->m[0][3] * input->x + matrix->m[1][3] * input->y + matrix->m[2][3] * input->z + matrix->m[3][3];
}

/**
 * @brief Multiplies two 4x4 matrices.
 *
 * This function multiplies two 4x4 matrices and stores the result in an output matrix.
 *
 * @param a The first 4x4 matrix to be multiplied.
 * @param b The second 4x4 matrix to be multiplied.
 * @param output The output 4x4 matrix where the result will be stored.
 */
inline void matrix4Mul(const Matrix4x4* a, const Matrix4x4* b, Matrix4x4* output) {
    Matrix4x4 tmp;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            tmp.m[i][j] = a->m[0][j] * b->m[i][0] +
                          a->m[1][j] * b->m[i][1] +
                          a->m[2][j] * b->m[i][2] +
                          a->m[3][j] * b->m[i][3];
        }
    }
    *output = tmp;
}

/**
 * @brief Multiplies two 3x3 matrices.
 *
 * This function multiplies two 3x3 matrices and stores the result in an output matrix.
 *
 * @param a The first 3x3 matrix to be multiplied.
 * @param b The second 3x3 matrix to be multiplied.
 * @param output The output 3x3 matrix where the result will be stored.
 */
inline void matrix3Mul(const Matrix3x3* a, const Matrix3x3* b, Matrix3x3* output) {
    Matrix3x3 tmp;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            tmp.m[i][j] = a->m[0][j] * b->m[i][0] +
                          a->m[1][j] * b->m[i][1] +
                          a->m[2][j] * b->m[i][2];
        }
    }
    *output = tmp;
}

/**
 * @brief Transposes a 3x3 matrix.
 *
 * @param in The input 3x3 matrix.
 * @param out The output transposed 3x3 matrix.
 */
inline void matrix3Transpose(const Matrix3x3* in, Matrix3x3* out) {
    Matrix3x3 tmp;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tmp.m[i][j] = in->m[j][i];
        }
    }
    *out = tmp;
}

float matrixNormalizedZValue(float depth, float nearPlane, float farPlane);


void matrix4FromBasis(float matrix[4][4], Vector3* origin, Vector3* x, Vector3* y, Vector3* z);

void matrix4FromPosition(float matrix[4][4], Vector3* position);

void matrix4FromScale(float matrix[4][4], float scale);

void matrix4ApplyPosition(float matrix[4][4], Vector3* position);

#endif