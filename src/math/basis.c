/** 
 * @file basis.c
 * @brief Implementation of the Basis structure and functions for basis rotations.
 * 
*/
#include "basis.h"

#include <assert.h>

/**
 * @brief Generates a basis (orthonormal set of vectors) from a quaternion.
 *
 * This function takes a quaternion and computes the corresponding basis vectors.
 * The resulting basis vectors are stored in the provided Basis structure.
 *
 * @param basis Pointer to the Basis structure where the resulting basis vectors will be stored.
 * @param quat Pointer to the Quaternion structure used to generate the basis vectors.
 */
void basisFromQuat(struct Basis* basis, struct Quaternion* quat) {
    quatMultVector(quat, &gRight, &basis->x);
    quatMultVector(quat, &gUp, &basis->y);
    vector3Cross(&basis->x, &basis->y, &basis->z);
}


/**
 * Rotates a vector using the given basis.
 *
 * This function takes an input vector and rotates it using the provided basis,
 * storing the result in the output vector. The input and output vectors must
 * not be the same.
 *
 * @param basis A pointer to the Basis structure containing the rotation basis.
 * @param input A pointer to the input Vector3 structure to be rotated.
 * @param output A pointer to the output Vector3 structure where the result will be stored.
 */
void basisRotate(struct Basis* basis, struct Vector3* input, struct Vector3* output) {
    assert(input != output);

    vector3Scale(&basis->x, output, input->x);
    vector3AddScaled(output, &basis->y, input->y, output);
    vector3AddScaled(output, &basis->z, input->z, output);
}

/**
 * @brief Rotates a vector using the inverse of the given basis.
 *
 * This function takes an input vector and rotates it using the inverse of the provided basis.
 * The result is stored in the output vector. The input and output vectors must not be the same.
 *
 * @param basis A pointer to the Basis structure containing the basis vectors.
 * @param input A pointer to the input Vector3 structure to be rotated.
 * @param output A pointer to the output Vector3 structure to store the result.
 */
void basisUnRotate(struct Basis* basis, struct Vector3* input, struct Vector3* output) {
    assert(input != output);

    output->x = vector3Dot(&basis->x, input);
    output->y = vector3Dot(&basis->y, input);
    output->z = vector3Dot(&basis->z, input);
}