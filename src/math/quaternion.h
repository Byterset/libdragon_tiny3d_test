
#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "vector3.h"
#include "vector2.h"
#include <stdbool.h>
#include "matrix.h"

typedef fm_quat_t Quaternion;


/// @brief Create an identity Quaternion
/// @param q 
inline void quatIdent(Quaternion* q) {
    *q = (Quaternion){{0,0,0,1}};
}


/// @brief Normalizes a quaternion
/// @param q 
/// @param out 
inline void quatNormalize(const Quaternion* q, Quaternion* out) {
    float magSqr = q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;

    if (magSqr < EPSILON) {
        quatIdent(out);
        return;
    } 
    
    magSqr = 1.0f / sqrtf(magSqr);
    *out = (Quaternion){{ q->x * magSqr, q->y * magSqr, q->z * magSqr, q->w * magSqr }};    
}


/// @brief Computes the Conjugate of a quaternion.
///
/// The conjugate represents the inverse rotation if the quaternion is unit length.
/// @param in 
/// @param out 
inline void quatConjugate(const Quaternion* in, Quaternion* out) {
    out->x = -in->x;
    out->y = -in->y;
    out->z = -in->z;
    out->w = in->w;
}


/// @brief Multiply two Quaternions
///
/// Intuitively the result can be understood as "First apply rotation B, then apply rotation A."
/// @param a 
/// @param b 
/// @param out The Quaternion representing the combined Rotation of A and B
inline void quatMultiply(const Quaternion* a, const Quaternion* b, Quaternion* out) {
    *out = (Quaternion){{ a->v[3] * b->v[0] + a->v[0] * b->v[3] + a->v[1] * b->v[2] - a->v[2] * b->v[1],
                         a->v[3] * b->v[1] - a->v[0] * b->v[2] + a->v[1] * b->v[3] + a->v[2] * b->v[0],
                         a->v[3] * b->v[2] + a->v[0] * b->v[1] - a->v[1] * b->v[0] + a->v[2] * b->v[3],
                         a->v[3] * b->v[3] - a->v[0] * b->v[0] - a->v[1] * b->v[1] - a->v[2] * b->v[2] }};
}


/// @brief Adds two Quaternions together component-wise.
/// @param a 
/// @param b 
/// @param out 
inline void quatAdd(const Quaternion* a, const Quaternion* b, Quaternion* out) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
    out->w = a->w + b->w;
}


/// @brief Compute the dot product of two quaternions
/// @param a 
/// @param b 
/// @return 
inline float quatDot(const Quaternion* a, const Quaternion* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w;
}

/// @brief Checks if two quaternions are the same component-wise
/// @param a 
/// @param b 
/// @return 
inline bool quatIsIdentical(const Quaternion* a, const Quaternion* b) {
    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z) && (a->w == b->w);
}


/// @brief Constructs a Quaternion representing a rotation around an axis by a given angle.
/// @param axis Axis to rotate around (must be normalized)
/// @param angle Rotation Angle in Radians
/// @param out the resulting Quaternion
void quatAxisAngle(const Vector3* axis, float angle, Quaternion* out);


/// @brief Similarly to quatAxisAngle this will construct a Quaternion that represents
/// a rotation around an axis. But here the rotation is given by a complex number
/// @param axis Axis to rotate around (must be normalized)
/// @param complex 2D Vector encoding the rotation
/// @param out the resulting Quaternion
void quatAxisComplex(const Vector3* axis, const Vector2* complex, Quaternion* out);


/// @brief Multiplies a quaternion by a vector.
///
/// This function takes a quaternion `q` and a vector `a`, and performs the
/// quaternion-vector multiplication. The result is stored in the output vector `out`.
///
/// @param q Pointer to the quaternion to be multiplied.
/// @param a Pointer to the vector to be multiplied.
/// @param out Pointer to the vector where the result will be stored.
void quatMultVector(const Quaternion* q, const Vector3* a, Vector3* out);


/// @brief Converts an input quaternion into an equivalent 4x4 rotation matrix
/// @param q 
/// @param out 
void quatToMatrix4(const Quaternion* q, Matrix4x4* out);


/// @brief Converts an input quaternion into an equivalent 3x3 rotation matrix
/// @param q 
/// @param out 
void quatToMatrix3(const Quaternion* q, Matrix3x3* out);


/// @brief Generate a pseudo-random quaternion rotation
/// @param q 
void quatRandom(Quaternion* q);


/// @brief Converts a look direction and an up-vector into a quaternion represention the corresponding rotation.
///
/// Essentially implement a "look-at" rotation with quaternions.
/// @param lookDir 
/// @param up 
/// @param out 
void quatLook(const Vector3* lookDir, const Vector3* up, Quaternion* out);


/// @brief Create a quaternion from euler angles.
/// @param angles vector containing the euler angles in radians.
/// @param out 
void quatEulerAngles(const Vector3* angles, Quaternion* out);


/// @brief Normalized linear interpolation between two quaternions.
///
/// Faster than slerp but less accurate
/// @param a 
/// @param b 
/// @param t blending factor
/// @param out 
void quatLerp(const Quaternion* a, const Quaternion* b, float t, Quaternion* out);


/// @brief Updates a quaternion based on angular velocity over a timestep.
/// @param input 
/// @param w 
/// @param timeStep 
/// @param output 
void quatApplyAngularVelocity(const Quaternion* input, const Vector3* w, float timeStep, Quaternion* output);


/// @brief Decomposes a quaternion into an axis and an angle.
///
/// @param input Pointer to the input quaternion to be decomposed.
/// @param axis Pointer to the vector that will store the axis of rotation.
/// @param angle Pointer to the float that will store the angle of rotation in radians.
///
/// The function calculates the magnitude of the quaternion's vector part and normalizes it to obtain the axis of rotation.
/// If the magnitude is very small (less than 0.0001), it sets the axis to a default up vector and the angle to 0.
/// Otherwise, it normalizes the axis and calculates the angle using the sine of the magnitude.
void quatDecompose(const Quaternion* input, Vector3* axis, float* angle);

/// @brief Applies an additional Euler-axis rotation to an existing quaternion rotation
/// @param q 
/// @param axis 
/// @param angle 
/// @param out 
void quatRotateAxisEuler(const Quaternion* q, const Vector3* axis, float angle, Quaternion* out);


#endif