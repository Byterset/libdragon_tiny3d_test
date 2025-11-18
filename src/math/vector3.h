
#ifndef _VECTOR3_H
#define _VECTOR3_H

#include <libdragon.h>
#include "mathf.h"
#include <math.h>

typedef fm_vec3_t Vector3;

typedef union Vector3u8 {
    struct {char x, y, z;};
    char v[3];
} Vector3u8;

extern const Vector3 gRight;
extern const Vector3 gUp;
extern const Vector3 gDown;
extern const Vector3 gForward;
extern const Vector3 gZeroVec;
extern const Vector3 gOneVec;

/// @brief Copy the values of a vector to another
/// @param source 
/// @param target 
inline void vector3Copy(const Vector3* source, Vector3* target){
    target->x = source->x;
    target->y = source->y;
    target->z = source->z;
}

/// @brief Remove the sign of the components of a vector
/// @param in 
/// @param out 
inline void vector3Abs(const Vector3* in, Vector3* out) {
    out->x = fabsf(in->x);
    out->y = fabsf(in->y);
    out->z = fabsf(in->z);
}

/// @brief Negate the components of a vector
/// @param in 
/// @param out 
inline void vector3Negate(const Vector3* in, Vector3* out) {
    out->x = -in->x;
    out->y = -in->y;
    out->z = -in->z;
}

/// @brief Scale vector component-wise
/// @param in 
/// @param out 
/// @param scale 
inline void vector3Scale(const Vector3* in, Vector3* out, float scale) {
    out->x = in->x * scale;
    out->y = in->y * scale;
    out->z = in->z * scale;
}

/// @brief Add b to a and store in out
/// @param a 
/// @param b 
/// @param out 
inline void vector3Add(const Vector3* a, const Vector3* b, Vector3* out) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;   
}

/// @brief Add b to a component-wise in-place
/// @param a 
/// @param b 
inline void vector3AddToSelf(Vector3* a, const Vector3* b) {
    a->x += b->x;
    a->y += b->y;
    a->z += b->z;
}

/// @brief Add b to a component-wise and scaled
/// @param a 
/// @param b 
/// @param scale 
/// @param out 
inline void vector3AddScaled(const Vector3* a, const Vector3* b, float scale, Vector3* out) {
    out->x = a->x + b->x * scale;
    out->y = a->y + b->y * scale;
    out->z = a->z + b->z * scale;
}

/// @brief Subtract b from a component-wise and store result in 'out'
/// @param a 
/// @param b 
/// @param out 
inline void vector3Sub(const Vector3* a, const Vector3* b, Vector3* out) {
    out->x = a->x - b->x;
    out->y = a->y - b->y;
    out->z = a->z - b->z;
}


/// @brief Subtract b from a component-wise in place
/// @param a 
/// @param b 
inline void vector3SubFromSelf(Vector3* a, const Vector3* b) {
    a->x -= b->x;
    a->y -= b->y;
    a->z -= b->z;
}


/// @brief Multiply two vectors component-wise
/// @param a 
/// @param b 
/// @param out 
/// @note Also works if input and output are the same vector
inline void vector3Multiply(const Vector3* a, const Vector3* b, Vector3* out) {
    out->x = a->x * b->x;
    out->y = a->y * b->y;
    out->z = a->z * b->z;
}

/// @brief Normalize a given vector
/// @param in 
/// @param out vector that points in direction of 'in' and has length 1
/// @note This function works also if both in and out are the same pointer
inline void vector3Normalize(const Vector3* in, Vector3* out) {
    float denom = in->x * in->x + in->y * in->y + in->z * in->z;

    if (denom == 0.0f) {
        out->x = 0.0f;
        out->y = 0.0f;
        out->z = 0.0f;
    } else {
        float invSqrt = 1.0f / sqrtf(denom);
        vector3Scale(in, out, invSqrt);
    }
}

/// @brief Normalize a vector in-place
/// @param self 
inline void vector3NormalizeSelf(Vector3* self) {
    vector3Normalize(self, self);
}


/// @brief A small helper that is a bit more intuitive than calling vector3Sub in reverse order
/// @param from 
/// @param to 
/// @param out 
inline void vector3FromTo(const Vector3* from, const Vector3* to, Vector3* out) {
    vector3Sub(to, from, out);
}

/// @brief Linearly interpolate between two vectors by factor 't'
/// @param a 
/// @param b 
/// @param t factor of the interpolation (must be in [0, 1])
/// @param out 
inline void vector3Lerp(const Vector3* a, const Vector3* b, float t, Vector3* out) {
    float tFlip = 1.0f - t;
    out->x = a->x * tFlip + b->x * t;
    out->y = a->y * tFlip + b->y * t;
    out->z = a->z * tFlip + b->z * t;
}

/// @brief Calculate vector dot product
/// @param a 
/// @param b 
/// @return 
inline float vector3Dot(const Vector3* a, const Vector3* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

/// @brief Calculate the squared magnitude/length of a vector 
/// @param a 
/// @return 
inline float vector3MagSqrd(const Vector3* a) {
    return vector3Dot(a, a);
}

/// @brief Clamp the length of a vector without changing it's direction
/// @param in 
/// @param out 
/// @param max_length 
inline void vector3ClampMag(const Vector3* in, Vector3* out, float max_length) {
    float len_sq = vector3MagSqrd(in);

    if(len_sq > (max_length * max_length)) {
        float inv_len = 1.0f / sqrtf(len_sq);
        float scale = max_length * inv_len;

        out->x = in->x * scale;
        out->y = in->y * scale;
        out->z = in->z * scale;
    }
}

/// @brief Calculate the magnitute/length of a vector
/// @param a 
/// @return 
inline float vector3Mag(const Vector3* a) {
    return sqrtf(vector3MagSqrd(a));
}

/// @brief Calculate the squared distance between to vectors
/// @param a 
/// @param b 
/// @return 
inline float vector3DistSqrd(const Vector3* a, const Vector3* b) {
    float x = a->x - b->x;
    float y = a->y - b->y;
    float z = a->z - b->z;

    return x * x + y * y + z * z;
}

/// @brief Calculate the distance between two vectors
/// @param a 
/// @param b 
/// @return 
inline float vector3Dist(const Vector3* a, const Vector3* b){
    return sqrtf(vector3DistSqrd(a, b));
}

/// @brief Compute the Vecotr Cross Product
/// @param a 
/// @param b 
/// @param out 
inline void vector3Cross(const Vector3* a, const Vector3* b, Vector3* out) {
    out->x = a->y * b->z - a->z * b->y;
    out->y = a->z * b->x - a->x * b->z;
    out->z = a->x * b->y - a->y * b->x;
}

/// @brief Calculates Vector perpendicular to input
/// @param a 
/// @param out 
inline void vector3Perpendicular(const Vector3* a, Vector3* out) {
    if (fabsf(a->x) > fabsf(a->z)) {
        vector3Cross(a, &gForward, out);
    } else {
        vector3Cross(a, &gRight, out);
    }
}

/// @brief Gives the component of in parallel to normal
/// @param in 
/// @param normal must be normalized
/// @param out 
inline void vector3Project(const Vector3* in, const Vector3* normal, Vector3* out) {
    float mag = vector3Dot(in, normal);
    out->x = normal->x * mag;
    out->y = normal->y * mag;
    out->z = normal->z * mag;
}

/// @brief Gives the component of in that is perpendicular to the planes normal
/// ergo the component of the input vector that lies on the plane given by the normal
/// @param in 
/// @param normal must be normalized
/// @param out 
inline void vector3ProjectPlane(const Vector3* in, const Vector3* normal, Vector3* out) {
    float mag = vector3Dot(in, normal);
    out->x = in->x - normal->x * mag;
    out->y = in->y - normal->y * mag;
    out->z = in->z - normal->z * mag;
}

/// @brief Computes (a×b)×c=b(a⋅c)−a(b⋅c)
/// @param a 
/// @param b 
/// @param c 
/// @param output 
inline void vector3TripleProduct(const Vector3* a, const Vector3* b, const Vector3* c, Vector3* output) {
    vector3Scale(b, output, vector3Dot(a, c)); // output=b(a⋅c)
    vector3AddScaled(output, a, -vector3Dot(b, c), output); // adds −a(b⋅c)
}

/// @brief Calculates a new vector where each component is the max of the two input components
/// @param a (a.x, a.y, a.z)
/// @param b (b.x, b.y, b.z)
/// @param out (max(a.x, b.x), max(a.y, b.y), max(a.z, b.z))
inline void vector3Max(const Vector3* a, const Vector3* b, Vector3* out) {
    out->x = fmaxf(a->x, b->x);
    out->y = fmaxf(a->y, b->y);
    out->z = fmaxf(a->z, b->z);
}

/// @brief Calculates a new vector where each component is the min of the two input components
/// @param a (a.x, a.y, a.z)
/// @param b (b.x, b.y, b.z)
/// @param out (min(a.x, b.x), min(a.y, b.y), min(a.z, b.z))
inline void vector3Min(const Vector3* a, const Vector3* b, Vector3* out) {
    out->x = fminf(a->x, b->x);
    out->y = fminf(a->y, b->y);
    out->z = fminf(a->z, b->z);
}

/// @brief Determines if a given vector is equal to the zero vector
/// @param vector 
/// @return 
inline bool vector3IsZero(const Vector3* vector) {
    return vector->x == 0.0f && vector->y == 0.0f && vector->z == 0.0f;
}

/// @brief Determines if two given Vectors are the same
/// @param a 
/// @param b 
/// @return 
inline bool vector3IsIdentical(const Vector3* a, const Vector3* b){
    return a->x == b->x && a->y == b->y && a->z == b->z;
}

/// @brief Evaluates a barycentric interpolation of three scalar values
/// @param baryCoords 
/// @param a 
/// @param b 
/// @param c 
/// @return 
inline float vector3EvalBarycentric1D(const Vector3* baryCoords, float a, float b, float c) {
    return baryCoords->x * a + baryCoords->y * b + baryCoords->z * c;
}

/// @brief
/// @param from 
/// @param towards 
/// @param maxDistance 
/// @param out the moved point
/// @return true if destination reached, false otherwise
bool vector3MoveTowards(const Vector3* from, const Vector3* towards, float maxDistance, Vector3* out);

/// @brief Converts single precision floating point vector to char vector
/// @param input 
/// @param output 
void vector3ToVector3u8(const Vector3* input, Vector3u8* output);

/// @brief Compute the reflection of an incident vector off a surface.
/// @param in The incident vector
/// @param normal The surface's normal vector. Must be normalized
/// @param out The reflected vector
void vector3Reflect(const Vector3 *in, const Vector3 *normal, Vector3 *out);


/// @brief Compute the refraction of an incident vector through a surface.
/// @param in The incident vector. Must be normalized.
/// @param normal The surface's normal vector. Must be normalized.
/// @param eta The ratio of indices of refraction (indicent/transmitted)
/// @param out Will contain the refracted vector.
/// @return True if refraction occurs; false if total internal reflection occurs.
bool vector3Refract(const Vector3 *in, const Vector3 *normal, float eta, Vector3 *out);

#endif