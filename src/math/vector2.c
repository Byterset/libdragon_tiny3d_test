
#include "vector2.h"
#include "vector3.h"
#include "mathf.h"

Vector2 gRight2 = {1.0f, 0.0f};
Vector2 gUp2 = {0.0f, 1.0f};
Vector2 gZeroVec2 = {0.0f, 0.0f};
Vector2 gOneVec2 = {1.0f, 1.0f};

void vector2ComplexMul(Vector2* a, Vector2* b, Vector2* out) {
    float x = a->x * b->x - a->y * b->y;
    out->y = a->x * b->y + a->y * b->x;
    out->x = x;
}

void vector2ComplexConj(Vector2* a, Vector2* out) {
    out->x = a->x;
    out->y = -a->y;
}

void vector2ComplexFromAngleRad(float radians, Vector2* out) {
    out->x = cosf(radians);
    out->y = sinf(radians);
}

int vector2RotateTowards(Vector2* from, Vector2* towards, Vector2* max, Vector2* out) {
    Vector2 fromInv = {from->x, -from->y};
    Vector2 diff;
    vector2ComplexMul(&fromInv, towards, &diff);

    if (diff.x > max->x) {
        *out = *towards;
        return 1;
    } else {
        if (diff.y < 0) {
            diff.x = max->x;
            diff.y = -max->y;
        } else {
            diff = *max;
        }
        vector2ComplexMul(from, &diff, out);

        return 0;
    }
}

void vector2ComplexFromAngleDeg(float degrees, Vector2* out){
    vector2ComplexFromAngleRad(degrees * PI / 180.0f, out);
}

void vector2Rotate90(Vector2* input, Vector2* out) {
    // in case input == out
    float tmp = input->x;
    out->x = -input->y;
    out->y = tmp;
}

float vector2Cross(Vector2* a, Vector2* b) {
    return a->x * b->y - a->y * b->x;
}

float vector2Dot(Vector2* a, Vector2* b) {
    return a->x * b->x + a->y * b->y;
}

void vector2Add(Vector2* a, Vector2* b, Vector2* out) {
    out->x = a->x + b->x;
    out->y = a->y + b->y;
}

void vector2Sub(Vector2* a, Vector2* b, Vector2* out) {
    out->x = a->x - b->x;
    out->y = a->y - b->y;
}

void vector2Scale(Vector2* a, float scale, Vector2* out) {
    out->x = a->x * scale;
    out->y = a->y * scale;
}

float vector2MagSqr(Vector2* a)  {
    return a->x * a->x + a->y * a->y;
}

float vector2DistSqr(Vector2* a, Vector2* b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;

    return dx * dx + dy * dy;
}

void vector2Normalize(Vector2* a, Vector2* out) {
    if (a->x == 0.0f && a->y == 0.0f) {
        *out = *a;
        return;
    }

    float denom = sqrtf(a->x * a->x + a->y * a->y);

    if (denom < 0.0000001f) {
        *out = *a;
        return;
    }

    float scale = 1.0f / denom;
    out->x = a->x * scale;
    out->y = a->y * scale;
}

void vector2Negate(Vector2* a, Vector2* out) {
    out->x = -a->x;
    out->y = -a->y;
}

void vector2Min(Vector2* a, Vector2* b, Vector2* out) {
    out->x = minf(a->x, b->x);
    out->y = minf(a->y, b->y);
}

void vector2Max(Vector2* a, Vector2* b, Vector2* out) {
    out->x = maxf(a->x, b->x);
    out->y = maxf(a->y, b->y);
}

void vector2Lerp(Vector2* a, Vector2* b, float lerp, Vector2* out) {
    out->x = (b->x - a->x) * lerp + a->x;
    out->y = (b->y - a->y) * lerp + a->y;
}

void vector2RandomUnitCircle(Vector2* result) {
    float xSqrd = randomInRangef(0.0f, 1.0f);

    result->x = sqrtf(xSqrd);
    result->y = sqrtf(1.0f - xSqrd);

    if (randomInt() & 0x800) {
        result->x = -result->x;
    }

    if (randomInt() & 0x800) {
        result->y = -result->y;
    }
}

void vector2LookDir(Vector2* result, Vector3* direction) {
    result->x = direction->z;
    result->y = -direction->x;
    vector2Normalize(result, result);
}

void vector3RotatedSpeed(Vector2* rotation, Vector3* result, float speed) {
    result->x = -rotation->y * speed;
    result->y = 0.0f;
    result->z = rotation->x * speed;
}