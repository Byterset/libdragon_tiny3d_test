#include "transform_single_axis.h"
#include "../render/defs.h"

void transformSAToMatrix(TransformSingleAxis* transform, mat4x4 matrix) {
    matrix[0][0] = transform->rotation.x * transform->scale.x * INV_MODEL_SCALE;
    matrix[0][1] = 0.0f;
    matrix[0][2] = transform->rotation.y * transform->scale.z * INV_MODEL_SCALE;
    matrix[0][3] = 0.0f;

    matrix[1][0] = 0.0f;
    matrix[1][1] = transform->scale.y * INV_MODEL_SCALE;
    matrix[1][2] = 0.0f;
    matrix[1][3] = 0.0f;

    matrix[2][0] = -transform->rotation.y * transform->scale.x * INV_MODEL_SCALE;
    matrix[2][1] = 0.0f;
    matrix[2][2] = transform->rotation.x * transform->scale.z * INV_MODEL_SCALE;
    matrix[2][3] = 0.0f;

    matrix[3][0] = transform->position.x;
    matrix[3][1] = transform->position.y;
    matrix[3][2] = transform->position.z;
    matrix[3][3] = 1.0f;
}