#include "mesh_collider.h"

#include <math.h>
#include <stdio.h>
#include "../math/minmax.h"

#define MAX_INDEX_SET_SIZE 64

void mesh_triangle_gjk_support_function(void* data, Vector3* direction, Vector3* output) {
    struct mesh_triangle* triangle = (struct mesh_triangle*)data;

    int idx = 0;
    float distance = vector3Dot(&triangle->vertices[triangle->triangle.indices[0]], direction);

    float check = vector3Dot(&triangle->vertices[triangle->triangle.indices[1]], direction);

    if (check > distance) {
        idx = 1;
        distance = check;
    }

    check = vector3Dot(&triangle->vertices[triangle->triangle.indices[2]], direction);

    if (check > distance) {
        idx = 2;
    }

    *output = triangle->vertices[triangle->triangle.indices[idx]];
}

/// @brief Check if a point is in front of or behind a triangle
///
/// if result > 0: point is in front of triangle
///
/// if result = 0: point is coplanar with triangle
///
/// if result < 0: point is behind triangle
/// @param triangle 
/// @param point 
/// @return 
float mesh_triangle_comparePoint(struct mesh_triangle *triangle, Vector3* point){
    Vector3 toPoint = *point;
    vector3SubFromSelf(&toPoint, &triangle->vertices[triangle->triangle.indices[0]]);
    return vector3Dot(&triangle->normal, &toPoint);
}

bool is_inf(float value) {
    return value == infinityf() || value == -infinityf();
}

