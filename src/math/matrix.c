/**
 * @file matrix.c
 * @brief This file contains the implementation of 4x4 matrix functions.
 * 
 */

#include "matrix.h"

/**
 * @brief Sets up a perspective projection matrix.
 *
 * This function initializes a 4x4 matrix to represent a perspective projection
 * transformation. The resulting matrix can be used to transform 3D coordinates
 * into a perspective view.
 *
 * @param matrix The 4x4 matrix to be initialized.
 * @param l The coordinate for the left vertical clipping plane.
 * @param r The coordinate for the right vertical clipping plane.
 * @param t The coordinate for the top horizontal clipping plane.
 * @param b The coordinate for the bottom horizontal clipping plane.
 * @param near The distance to the near clipping plane.
 * @param far The distance to the far clipping plane.
 */
void matrixPerspective(float matrix[4][4], float l, float r, float t, float b, float near, float far) {
    matrix[0][0] = 2.0f * near / (r - l);
    matrix[0][1] = 0.0f;
    matrix[0][2] = 0.0f;
    matrix[0][3] = 0.0f;

    matrix[1][0] = 0.0f;
    matrix[1][1] = 2.0f * near / (t - b);
    matrix[1][2] = 0.0f;
    matrix[1][3] = 0.0f;

    matrix[2][0] = (r + l) / (r - l);
    matrix[2][1] = (t + b) / (t - b);
    matrix[2][2] = -(far + near) / (far - near);
    matrix[2][3] = -1;

    matrix[3][0] = 0.0f;
    matrix[3][1] = 0.0f;
    matrix[3][2] = -2.0f * far * near / (far - near);
    matrix[3][3] = 0.0f;
}

/**
 * @brief Computes the normalized Z value for a given depth within a specified near and far plane.
 *
 * This function calculates the normalized Z value based on the depth, near, and far plane distances.
 * The normalized Z value is used to map depth values to a normalized range.
 *
 * @param depth The depth value to be normalized.
 * @param near The near plane distance.
 * @param far The far plane distance.
 * @return The normalized Z value. Returns -1.0 if the depth is greater than or equal to -near,
 *         1.0 if the depth is less than or equal to -far, otherwise returns the computed normalized Z value.
 */
float matrixNormalizedZValue(float depth, float near, float far) {
    if (depth >= -near) {
        return -1.0f;
    }

    if (depth <= -far) {
        return 1.0f;
    }

    return (far * (depth + near) + 2.0 * far * near) / (depth * (far - near));
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
void matrixVec3Mul(float matrix[4][4], Vector3* input, Vector4* output) {
    output->x = matrix[0][0] * input->x + matrix[1][0] * input->y + matrix[2][0] * input->z + matrix[3][0];
    output->y = matrix[0][1] * input->x + matrix[1][1] * input->y + matrix[2][1] * input->z + matrix[3][1];
    output->z = matrix[0][2] * input->x + matrix[1][2] * input->y + matrix[2][2] * input->z + matrix[3][2];
    output->w = matrix[0][3] * input->x + matrix[1][3] * input->y + matrix[2][3] * input->z + matrix[3][3];
}

/**
 * @brief Generates a 4x4 matrix from a basis.
 *
 * This function generates a 4x4 matrix from a basis defined by an origin and three vectors (x, y, z).
 *
 * @param matrix The 4x4 matrix to be initialized.
 * @param origin The origin of the basis.
 * @param x The x vector of the basis.
 * @param y The y vector of the basis.
 * @param z The z vector of the basis.
 */
void matrixFromBasis(float matrix[4][4], Vector3* origin, Vector3* x, Vector3* y, Vector3* z) {
    matrix[0][0] = x->x;
    matrix[0][1] = x->y;
    matrix[0][2] = x->z;
    matrix[0][3] = 0.0f;

    matrix[1][0] = y->x;
    matrix[1][1] = y->y;
    matrix[1][2] = y->z;
    matrix[1][3] = 0.0f;

    matrix[2][0] = z->x;
    matrix[2][1] = z->y;
    matrix[2][2] = z->z;
    matrix[2][3] = 0.0f;

    matrix[3][0] = origin->x;
    matrix[3][1] = origin->y;
    matrix[3][2] = origin->z;
    matrix[3][3] = 1.0f;
}

/**
 * @brief Generates a 4x4 matrix from a position.
 *
 * This function generates a 4x4 matrix from a position vector.
 *
 * @param matrix The 4x4 matrix to be initialized.
 * @param position The position vector.
 */
void matrixFromPosition(float matrix[4][4], Vector3* position) {
    matrix[0][0] = 1.0f;
    matrix[0][1] = 0.0f;
    matrix[0][2] = 0.0f;
    matrix[0][3] = 0.0f;

    matrix[1][0] = 0.0f;
    matrix[1][1] = 1.0f;
    matrix[1][2] = 0.0f;
    matrix[1][3] = 0.0f;

    matrix[2][0] = 0.0f;
    matrix[2][1] = 0.0f;
    matrix[2][2] = 1.0f;
    matrix[2][3] = 0.0f;

    matrix[3][0] = position->x;
    matrix[3][1] = position->y;
    matrix[3][2] = position->z;
    matrix[3][3] = 1.0f;
}

/**
 * @brief Generates a 4x4 matrix from a scale factor.
 *
 * This function generates a 4x4 matrix from a scale factor.
 *
 * @param matrix The 4x4 matrix to be initialized.
 * @param scale The scale factor.
 */
void matrixFromScale(float matrix[4][4], float scale) {
    matrix[0][0] = scale;
    matrix[0][1] = 0.0f;
    matrix[0][2] = 0.0f;
    matrix[0][3] = 0.0f;

    matrix[1][0] = 0.0f;
    matrix[1][1] = scale;
    matrix[1][2] = 0.0f;
    matrix[1][3] = 0.0f;

    matrix[2][0] = 0.0f;
    matrix[2][1] = 0.0f;
    matrix[2][2] = scale;
    matrix[2][3] = 0.0f;

    matrix[3][0] = 0.0f;
    matrix[3][1] = 0.0f;
    matrix[3][2] = 0.0f;
    matrix[3][3] = 1.0f;
}

/**
 * @brief Applies a position to a 4x4 matrix.
 *
 * This function applies a position to a 4x4 matrix.
 *
 * @param matrix The 4x4 matrix to be modified.
 * @param position The position vector to be applied.
 */
void matrixApplyPosition(float matrix[4][4], Vector3* position) {
    matrix[3][0] = position->x;
    matrix[3][1] = position->y;
    matrix[3][2] = position->z;
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
void matrixMul(float a[4][4], float b[4][4], float output[4][4]) {
    for (int x = 0; x < 4; ++x) {
        for (int y = 0; y < 4; ++y) {
            output[x][y] = 0.0f;
            for (int j = 0; j < 4; ++j) {
                output[x][y] += a[j][y] * b[x][j];
            }
        }
    }
}