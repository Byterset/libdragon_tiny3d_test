
#include "quaternion.h"
#include <assert.h>
#include "mathf.h"
#include <math.h>

Quaternion gQuaternionZero = {{0.0f, 0.0f, 0.0f, 0.0f}};
Quaternion gQuaternionIdentity = {{0.0f, 0.0f, 0.0f, 1.0f}};

void quatIdent(Quaternion* q) {
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
    q->w = 1.0f;
}

void quatAxisAngle(Vector3* axis, float angle, Quaternion* out) {
    float sinTheta = sinf(angle * 0.5f);
    float cosTheta = cosf(angle * 0.5f);

    out->x = axis->x * sinTheta;
    out->y = axis->y * sinTheta;
    out->z = axis->z * sinTheta;
    out->w = cosTheta;
}

void quatEulerAngles(Vector3* angles, Quaternion* out) {
    Quaternion angle;
    Quaternion tmp;

    quatAxisAngle(&gRight, angles->x, &angle);
    quatAxisAngle(&gUp, angles->y, out);
    quatMultiply(out, &angle, &tmp);
    quatAxisAngle(&gForward, angles->z, &angle);
    quatMultiply(&angle, &tmp, out);
}

void quatAxisComplex(Vector3* axis, Vector2* complex, Quaternion* out) {
    float sinTheta = 0.5f - complex->x * 0.5f;

    if (sinTheta < 0.0f) {
        sinTheta = 0.0f;
    } else {
        sinTheta = sqrtf(sinTheta);

        if (complex->y < 0.0f) {
            sinTheta = -sinTheta;
        }
    }

    float cosTheta = 0.5f + complex->x * 0.5f;

    if (cosTheta < 0.0f) {
        cosTheta = 0.0f;
    } else {
        cosTheta = sqrtf(cosTheta);
    }

    out->x = axis->x * sinTheta;
    out->y = axis->y * sinTheta;
    out->z = axis->z * sinTheta;
    out->w = cosTheta;
}

void quatConjugate(Quaternion* in, Quaternion* out) {
    out->x = -in->x;
    out->y = -in->y;
    out->z = -in->z;
    out->w = in->w;
}

void quatNegate(Quaternion* in, Quaternion* out) {
    out->x = -in->x;
    out->y = -in->y;
    out->z = -in->z;
    out->w = -in->w;
}

/**
 * Multiplies a quaternion by a vector.
 *
 * This function takes a quaternion `q` and a vector `a`, and performs the
 * quaternion-vector multiplication. The result is stored in the output vector `out`.
 *
 * @param q Pointer to the quaternion to be multiplied.
 * @param a Pointer to the vector to be multiplied.
 * @param out Pointer to the vector where the result will be stored.
 */
void quatMultVector(Quaternion* q, Vector3* a, Vector3* out) {

    Quaternion tmp;
    Quaternion conj;
    quatConjugate(q, &conj);
    tmp.x = q->w*a->x + q->y*a->z - q->z*a->y;
    tmp.y = q->w*a->y + q->z*a->x - q->x*a->z;
    tmp.z = q->w*a->z + q->x*a->y - q->y*a->x;
    tmp.w = - q->x*a->x - q->y*a->y - q->z*a->z;

    out->x = tmp.w*conj.x + tmp.x*q->w + tmp.y*conj.z - tmp.z*conj.y;
    out->y = tmp.w*conj.y + tmp.y*q->w + tmp.z*conj.x - tmp.x*conj.z;
    out->z = tmp.w*conj.z + tmp.z*q->w + tmp.x*conj.y - tmp.y*conj.x;

}

void quatRotatedBoundingBoxSize(Quaternion* q, Vector3* halfBoxSize, Vector3* out) {
    float xx = q->x*q->x;
    float yy = q->y*q->y;
    float zz = q->z*q->z;

    float xy = q->x*q->y;
    float yz = q->y*q->z;
    float xz = q->x*q->z;

    float xw = q->x*q->w;
    float yw = q->y*q->w;
    float zw = q->z*q->w;

    out->x = fabsf(1.0f - 2.0f * (yy + zz)) * halfBoxSize->x +
        fabsf(2.0f * (xy - zw)) * halfBoxSize->y +
        fabsf(2.0f * (xz + yw)) * halfBoxSize->z;

    out->y = fabsf(2.0f * (xy + zw)) * halfBoxSize->x +
        fabsf(1.0f - 2.0f * (xx + zz)) * halfBoxSize->y +
        fabsf(2.0f * (yz - xw)) * halfBoxSize->z;

    out->z = fabsf(2.0f * (xz - yw)) * halfBoxSize->x +
        fabsf(2.0f * (yz + xw)) * halfBoxSize->y +
        fabsf(1.0f - 2.0f * (xx + yy)) * halfBoxSize->z;
}

void quatMultiply(Quaternion* a, Quaternion* b, Quaternion* out) {
    assert(a != out && b != out);
    out->x = a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y;
    out->y = a->w*b->y + a->y*b->w + a->z*b->x - a->x*b->z;
    out->z = a->w*b->z + a->z*b->w + a->x*b->y - a->y*b->x;
    out->w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
}

void quatAdd(Quaternion* a, Quaternion* b, Quaternion* out) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
    out->z = a->z + b->z;
    out->w = a->w + b->w;
}

void quatToMatrix(Quaternion* q, float out[4][4]) {
    float xx = q->x*q->x;
    float yy = q->y*q->y;
    float zz = q->z*q->z;

    float xy = q->x*q->y;
    float yz = q->y*q->z;
    float xz = q->x*q->z;

    float xw = q->x*q->w;
    float yw = q->y*q->w;
    float zw = q->z*q->w;

    out[0][0] = 1.0f - 2.0f * (yy + zz);
    out[0][1] = 2.0f * (xy + zw);
    out[0][2] = 2.0f * (xz - yw);
    out[0][3] = 0.0f;
    out[1][0] = 2.0f * (xy - zw);
    out[1][1] = 1.0f - 2.0f * (xx + zz);
    out[1][2] = 2.0f * (yz + xw);
    out[1][3] = 0.0f;
    out[2][0] = 2.0f * (xz + yw);
    out[2][1] = 2.0f * (yz - xw);
    out[2][2] = 1.0f - 2.0f * (xx + yy);
    out[2][3] = 0.0f;
    out[3][0] = 0.0f;
    out[3][1] = 0.0f;
    out[3][2] = 0.0f;
    out[3][3] = 1.0f;
}

void quatNormalize(Quaternion* q, Quaternion* out) {
    float magSqr = q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;

    if (magSqr < 0.00001f) {
        out->w = 1.0f;
        out->x = 0.0f;
        out->y = 0.0f;
        out->z = 0.0f;
    } else {
        magSqr = 1.0f / sqrtf(magSqr);

        out->x = q->x * magSqr;
        out->y = q->y * magSqr;
        out->z = q->z * magSqr;
        out->w = q->w * magSqr;
    }
}

void quatRandom(Quaternion* q) {
    q->x = mathfRandomFloat() - 0.5f;
    q->y = mathfRandomFloat() - 0.5f;
    q->z = mathfRandomFloat() - 0.5f;
    q->w = mathfRandomFloat() - 0.5f;
    quatNormalize(q, q);
}

void quatLook(Vector3* lookDir, Vector3* up, Quaternion* out) {
    // Build orthonormal basis
    Vector3 zDir;
    vector3Normalize(lookDir, &zDir);
    vector3Negate(&zDir, &zDir);

    Vector3 xDir;
    vector3Cross(up, &zDir, &xDir);
    vector3Normalize(&xDir, &xDir);

    Vector3 yDir;
    vector3Cross(&zDir, &xDir, &yDir);

    // Matrix-to-quaternion conversion
    float trace = xDir.x + yDir.y + zDir.z;
    float s;
    
    if (trace > 0) { 
        s = sqrtf(trace + 1.0f) * 2.0f;
        out->w = 0.25f * s;
        out->x = (yDir.z - zDir.y) / s;
        out->y = (zDir.x - xDir.z) / s; 
        out->z = (xDir.y - yDir.x) / s; 
    } else if (xDir.x > yDir.y && xDir.x > zDir.z) { 
        s = sqrtf(1.0f + xDir.x - yDir.y - zDir.z) * 2.0f;
        out->w = (yDir.z - zDir.y) / s;
        out->x = 0.25f * s;
        out->y = (yDir.x + xDir.y) / s; 
        out->z = (zDir.x + xDir.z) / s; 
    } else if (yDir.y > zDir.z) { 
        s = sqrtf(1.0f + yDir.y - xDir.x - zDir.z) * 2.0f;
        out->w = (zDir.x - xDir.z) / s;
        out->x = (yDir.x + xDir.y) / s; 
        out->y = 0.25f * s;
        out->z = (zDir.y + yDir.z) / s; 
    } else { 
        s = sqrtf(1.0f + zDir.z - xDir.x - yDir.y) * 2.0f;
        out->w = (xDir.y - yDir.x) / s;
        out->x = (zDir.x + xDir.z) / s;
        out->y = (zDir.y + yDir.z) / s;
        out->z = 0.25f * s;
    }
}

void quatLerp(Quaternion* a, Quaternion* b, float t, Quaternion* out) {
    float tInv = 1.0f - t;

    if (quatDot(a, b) < 0) {
        out->x = tInv * a->x - t * b->x;
        out->y = tInv * a->y - t * b->y;
        out->z = tInv * a->z - t * b->z;
        out->w = tInv * a->w - t * b->w;
    } else {
        out->x = tInv * a->x + t * b->x;
        out->y = tInv * a->y + t * b->y;
        out->z = tInv * a->z + t * b->z;
        out->w = tInv * a->w + t * b->w;
    }

    quatNormalize(out, out);
}

void quatApplyAngularVelocity(Quaternion* input, Vector3* w, float timeStep, Quaternion* output) {
    Quaternion velocityAsQuat;
    velocityAsQuat.w = 0.0f;
    velocityAsQuat.x = w->x * timeStep * 0.5f;
    velocityAsQuat.y = w->y * timeStep * 0.5f;
    velocityAsQuat.z = w->z * timeStep * 0.5f;

    Quaternion intermediate;
    quatMultiply(&velocityAsQuat, input, &intermediate);

    quatAdd(&intermediate, input, output);
    quatNormalize(output, output);
}


/**
 * Decomposes a quaternion into an axis and an angle.
 *
 * @param input Pointer to the input quaternion to be decomposed.
 * @param axis Pointer to the vector that will store the axis of rotation.
 * @param angle Pointer to the float that will store the angle of rotation in radians.
 *
 * The function calculates the magnitude of the quaternion's vector part and normalizes it to obtain the axis of rotation.
 * If the magnitude is very small (less than 0.0001), it sets the axis to a default up vector and the angle to 0.
 * Otherwise, it normalizes the axis and calculates the angle using the sine of the magnitude.
 */
void quatDecompose(Quaternion* input, Vector3* axis, float* angle) {
    float axisMag = sqrtf(input->x * input->x + input->y * input->y + input->z * input->z);

    if (axisMag < 0.0001f) {
        *axis = gUp;
        *angle = 0.0f;
        return;
    }

    float magInv = 1.0f / axisMag;

    axis->x = input->x * magInv;
    axis->y = input->y * magInv;
    axis->z = input->z * magInv;
    *angle = sinf(axisMag) * 2.0f;
}

float quatDot(Quaternion* a, Quaternion* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w;
}

void quatRotateAxisEuler(Quaternion *q, Vector3 *axis, float angleRad, Quaternion *out)
{
    Quaternion tmp, quatRot;
    quatAxisAngle(axis, angleRad, &quatRot);
    quatMultiply(q, &quatRot, &tmp);
    *out = tmp;
}

bool quatIsIdentical(Quaternion* a, Quaternion* b) {
    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z) && (a->w == b->w);
}
