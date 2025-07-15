#ifndef __COLLISION_SHAPE_RAY_TRIANGLE_INTERSECTION_H__
#define __COLLISION_SHAPE_RAY_TRIANGLE_INTERSECTION_H__

#include "../../math/vector3.h"
#include "../../math/quaternion.h"
#include "../raycast.h"
#include "../mesh_collider.h"

/// @brief Möller–Trumbore Ray-Triangle intersection algorithm
/// @param ray 
/// @param hit the resulting hit object
/// @param triangle the mesh triangle to be tested
/// @return 
bool ray_triangle_intersection(
    raycast *ray, 
    raycast_hit* hit, 
    struct mesh_triangle *triangle
);

#endif // __COLLISION_SHAPE_RAY_TRIANGLE_INTERSECTION_H__