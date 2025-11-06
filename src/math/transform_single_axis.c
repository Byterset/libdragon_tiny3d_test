#include "transform_single_axis.h"
#include "../render/defs.h"

void transformSAToMatrix(TransformSingleAxis* transform, float mtx[4][4]) {
    mtx[0][0] = transform->rotation.x * transform->scale.x * INV_MODEL_SCALE;
    mtx[0][1] = 0.0f;
    mtx[0][2] = transform->rotation.y * transform->scale.z * INV_MODEL_SCALE;
    mtx[0][3] = 0.0f;

    mtx[1][0] = 0.0f;
    mtx[1][1] = transform->scale.y * INV_MODEL_SCALE;
    mtx[1][2] = 0.0f;
    mtx[1][3] = 0.0f;

    mtx[2][0] = -transform->rotation.y * transform->scale.x * INV_MODEL_SCALE;
    mtx[2][1] = 0.0f;
    mtx[2][2] = transform->rotation.x * transform->scale.z * INV_MODEL_SCALE;
    mtx[2][3] = 0.0f;

    mtx[3][0] = transform->position.x;
    mtx[3][1] = transform->position.y;
    mtx[3][2] = transform->position.z;
    mtx[3][3] = 1.0f;
}