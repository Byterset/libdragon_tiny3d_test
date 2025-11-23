/**
 * @file matrix.c
 * @brief This file contains the implementation of 4x4 matrix functions.
 * 
 */

#include "matrix.h"


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
void matrix4FromBasis(float matrix[4][4], Vector3* origin, Vector3* x, Vector3* y, Vector3* z) {
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
void matrix4FromPosition(float matrix[4][4], Vector3* position) {
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
void matrix4FromScale(float matrix[4][4], float scale) {
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
void matrix4ApplyPosition(float matrix[4][4], Vector3* position) {
    matrix[3][0] = position->x;
    matrix[3][1] = position->y;
    matrix[3][2] = position->z;
}
