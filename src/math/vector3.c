
#include "vector3.h"

const Vector3 gRight = {{1.0f, 0.0f, 0.0f}};
const Vector3 gUp = {{0.0f, 1.0f, 0.0f}};
const Vector3 gDown = {{0.0f, -1.0f, 0.0f}};
const Vector3 gForward = {{0.0f, 0.0f, 1.0f}};
const Vector3 gZeroVec = {{0.0f, 0.0f, 0.0f}};
const Vector3 gOneVec = {{1.0f, 1.0f, 1.0f}};

bool vector3MoveTowards(const Vector3* from, const Vector3* towards, float maxDistance, Vector3* out) {
    float distance = vector3DistSqrd(from, towards);

    if (distance < maxDistance * maxDistance) {
        *out = *towards;
        return true;
    } else {
        float scale = maxDistance / sqrtf(distance);
        out->x = (towards->x - from->x) * scale + from->x;
        out->y = (towards->y - from->y) * scale + from->y;
        out->z = (towards->z - from->z) * scale + from->z;
        return false;
    }
}

void vector3ToVector3u8(const Vector3* input, Vector3u8* output) {
    output->x = floatTos8norm(input->x);
    output->y = floatTos8norm(input->y);
    output->z = floatTos8norm(input->z);
}

void vector3Reflect(const Vector3 *in, const Vector3 *normal, Vector3 *out)
{
    Vector3 tmp;
    vector3Scale(normal, &tmp, 2.0f * vector3Dot(in, normal));
    vector3Sub(in, &tmp, out);
}

bool vector3Refract(const Vector3 *in, const Vector3 *normal, float eta, Vector3 *out)
{
    float ndi = vector3Dot(normal, in);
    float eni = eta * ndi;
    float k = 1.0f - eta*eta + eni*eni;

    if (k < 0.0f) {
        *out = (Vector3){};
        return false;
    }

    Vector3 tmp;
    vector3Scale(normal, &tmp, eni + sqrtf(k));
    vector3Scale(in, out, eta);
    vector3SubFromSelf(out, &tmp);
    return true;
}

/// @brief Helper function to calculate tangent vectors orthogonal to normal
void vector3CalculateTangents(const Vector3* normal, Vector3* tangent_u, Vector3* tangent_v) {

    // First tangent is normal cross axis
    vector3Perpendicular(normal, tangent_u);

    // Second tangent is normal cross first tangent
    vector3Cross(normal, tangent_u, tangent_v);
    vector3Normalize(tangent_v, tangent_v);
}