
#include "quaternion.h"
#include <assert.h>
#include "mathf.h"
#include <math.h>


void quatAxisAngle(const Vector3* axis, float angle, Quaternion* out) {
    float sinTheta = sinf(angle * 0.5f);
    float cosTheta = cosf(angle * 0.5f);

    out->x = axis->x * sinTheta;
    out->y = axis->y * sinTheta;
    out->z = axis->z * sinTheta;
    out->w = cosTheta;
}


void quatAxisComplex(const Vector3* axis, const Vector2* complex, Quaternion* out) {
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


void quatEulerAngles(const Vector3* angles, Quaternion* out) {
    float c1, c2, c3, s1, s2, s3;
    fm_sincosf(angles->x * 0.5f, &s1, &c1);
    fm_sincosf(angles->y * 0.5f, &s2, &c2);
    fm_sincosf(angles->z * 0.5f, &s3, &c3);

    *out = (Quaternion){{ c1 * c2 * c3 + s1 * s2 * s3,
                         s1 * c2 * c3 - c1 * s2 * s3,
                         c1 * s2 * c3 + s1 * c2 * s3,
                         c1 * c2 * s3 - s1 * s2 * c3 }};
}


void quatMultVector(const Quaternion* q, const Vector3* a, Vector3* out) {

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


void quatToMatrix4(const Quaternion* q, Matrix4x4* out) {
    float xx = q->x*q->x;
    float yy = q->y*q->y;
    float zz = q->z*q->z;

    float xy = q->x*q->y;
    float yz = q->y*q->z;
    float xz = q->x*q->z;

    float xw = q->x*q->w;
    float yw = q->y*q->w;
    float zw = q->z*q->w;

    Matrix4x4 tmp;

    tmp.m[0][0] = 1.0f - 2.0f * (yy + zz);
    tmp.m[0][1] = 2.0f * (xy + zw);
    tmp.m[0][2] = 2.0f * (xz - yw);
    tmp.m[0][3] = 0.0f;
    tmp.m[1][0] = 2.0f * (xy - zw);
    tmp.m[1][1] = 1.0f - 2.0f * (xx + zz);
    tmp.m[1][2] = 2.0f * (yz + xw);
    tmp.m[1][3] = 0.0f;
    tmp.m[2][0] = 2.0f * (xz + yw);
    tmp.m[2][1] = 2.0f * (yz - xw);
    tmp.m[2][2] = 1.0f - 2.0f * (xx + yy);
    tmp.m[2][3] = 0.0f;
    tmp.m[3][0] = 0.0f;
    tmp.m[3][1] = 0.0f;
    tmp.m[3][2] = 0.0f;
    tmp.m[3][3] = 1.0f;

    *out = tmp;
}

void quatToMatrix3(const Quaternion* q, Matrix3x3* out) {
    float xx = q->x*q->x;
    float yy = q->y*q->y;
    float zz = q->z*q->z;

    float xy = q->x*q->y;
    float yz = q->y*q->z;
    float xz = q->x*q->z;

    float xw = q->x*q->w;
    float yw = q->y*q->w;
    float zw = q->z*q->w;

    Matrix3x3 tmp;

    tmp.m[0][0] = 1.0f - 2.0f * (yy + zz);
    tmp.m[0][1] = 2.0f * (xy + zw);
    tmp.m[0][2] = 2.0f * (xz - yw);

    tmp.m[1][0] = 2.0f * (xy - zw);
    tmp.m[1][1] = 1.0f - 2.0f * (xx + zz);
    tmp.m[1][2] = 2.0f * (yz + xw);

    tmp.m[2][0] = 2.0f * (xz + yw);
    tmp.m[2][1] = 2.0f * (yz - xw);
    tmp.m[2][2] = 1.0f - 2.0f * (xx + yy);

    *out = tmp;

}


void quatRandom(Quaternion* q) {
    q->x = mathfRandomFloat() - 0.5f;
    q->y = mathfRandomFloat() - 0.5f;
    q->z = mathfRandomFloat() - 0.5f;
    q->w = mathfRandomFloat() - 0.5f;
    quatNormalize(q, q);
}


void quatLook(const Vector3* lookDir, const Vector3* up, Quaternion* out) {
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


void quatLerp(const Quaternion* a, const Quaternion* b, float t, Quaternion* out) {
    float tInv = 1.0f - t;

    if (quatDot(a, b) < 0) {
        tInv = -tInv;
    } 
    *out = (Quaternion){{tInv * a->x + t * b->x,
                        tInv * a->y + t * b->y,
                        tInv * a->z + t * b->z,
                        tInv * a->w + t * b->w}};

    quatNormalize(out, out);
}


void quatApplyAngularVelocity(const Quaternion* input, const Vector3* w, float timeStep, Quaternion* output) {
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


void quatDecompose(const Quaternion* input, Vector3* axis, float* angle) {
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


void quatRotateAxisEuler(const Quaternion *q, const Vector3 *axis, float angleRad, Quaternion *out)
{
    Quaternion tmp, quatRot;
    quatAxisAngle(axis, angleRad, &quatRot);
    quatMultiply(q, &quatRot, &tmp);
    *out = tmp;
}
