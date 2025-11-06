/**
 * @file matrix.h
 * @brief This file contains the definition of the 4x4 matrix structure and functions to work with those matrices.
 * 
 */
#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "vector4.h"
#include "vector3.h"

/**
 * @brief The 4x4 matrix structure.
 * 
 */
typedef fm_mat4_t Matrix4x4;

void matrixPerspective(float matrix[4][4], float l, float r, float top, float b, float near, float far);

float matrixNormalizedZValue(float depth, float nearPlane, float farPlane);

void matrixVec3Mul(float matrix[4][4], Vector3* input, Vector4* output);

void matrixMul(float a[4][4], float b[4][4], float output[4][4]);

void matrixFromBasis(float matrix[4][4], Vector3* origin, Vector3* x, Vector3* y, Vector3* z);

void matrixFromPosition(float matrix[4][4], Vector3* position);

void matrixFromScale(float matrix[4][4], float scale);

void matrixApplyPosition(float matrix[4][4], Vector3* position);

#endif