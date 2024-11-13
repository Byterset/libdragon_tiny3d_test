/**
 * @file basis.h
 * @brief Defines the Basis structure and functions for quaternion-based rotations.
 *
 * This header file contains the definition of the Basis structure, which represents
 * a 3D coordinate system using three vectors (x, y, z). It also provides functions
 * to initialize a Basis from a quaternion and to perform rotations and inverse rotations
 * using the Basis.
 *
 */
#ifndef __BASIS_H__
#define __BASIS_H__

#include "vector3.h"
#include "quaternion.h"

/**
 * @brief Represents a 3D coordinate system using three vectors.
 * 
 */
struct Basis {
    Vector3 x;
    Vector3 y;
    Vector3 z;
};

void basisFromQuat(struct Basis* basis, Quaternion* quat);

void basisRotate(struct Basis* basis, Vector3* input, Vector3* output);
void basisUnRotate(struct Basis* basis, Vector3* input, Vector3* output);

#endif