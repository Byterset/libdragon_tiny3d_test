#include "ray_triangle_intersection.h"
#include "../../math/mathf.h"
#include <math.h>
#include <float.h>

bool ray_triangle_intersection(raycast *ray, raycast_hit* hit, struct mesh_triangle *triangle){

    Vector3 v0 = triangle->vertices[triangle->triangle.indices[0]];
    Vector3 v1 = triangle->vertices[triangle->triangle.indices[1]];
    Vector3 v2 = triangle->vertices[triangle->triangle.indices[2]];

    // Calculate triangle edges
    Vector3 edge1;
    vector3Sub(&v1, &v0, &edge1);
    Vector3 edge2;
    vector3Sub(&v2, &v0, &edge2);
    
    // Check for degenerate triangle (zero area)
    Vector3 triangle_normal;
    vector3Cross(&edge1, &edge2, &triangle_normal);
    float triangle_area_sq = vector3MagSqrd(&triangle_normal);
    if (triangle_area_sq < EPSILON * EPSILON) {
        return false;  // Degenerate triangle
    }
    
    // Begin calculating determinant - also used to calculate u parameter
    Vector3 h;
    vector3Cross(&ray->dir, &edge2, &h);
    float a = vector3Dot(&edge1, &h);

    // Use a more robust parallel check
    // If a is close to zero, the ray is parallel to the triangle
    if (fabsf(a) < EPSILON) {
        return false;  // Ray is parallel to triangle
    }

    // Make the algorithm winding order independent
    // We'll handle both positive and negative determinants
    float f = 1.0f / a;
    Vector3 s;
    vector3Sub(&ray->origin, &v0, &s);
    float u = f * vector3Dot(&s, &h);

    // Check if the intersection is outside the triangle (u parameter)
    // Use slightly more permissive bounds to handle floating point precision issues
    if (u < -EPSILON || u > 1.0f + EPSILON) {
        return false;
    }

    Vector3 q;
    vector3Cross(&s, &edge1, &q);
    float v = f * vector3Dot(&ray->dir, &q);

    // Check if the intersection is outside the triangle (v parameter)
    // Also check the third barycentric coordinate (w = 1 - u - v)
    if (v < -EPSILON || u + v > 1.0f + EPSILON) {
        return false;
    }

    // At this stage we can compute t to find out where the intersection point is on the line
    float t = f * vector3Dot(&edge2, &q);

    // Check if the intersection is within the ray's valid range
    // Use a smaller epsilon for the minimum distance to avoid rejecting valid close intersections
    const float MIN_T = 1e-6f;  // Smaller than EPSILON to catch very close intersections
    
    if (t > MIN_T && t <= ray->maxDistance) {
        // Ray intersects the triangle
        hit->distance = t;
        
        // Use the precomputed triangle normal from the mesh data
        hit->normal = triangle->normal;
        
        // Handle winding order: if determinant is negative, we hit the back face
        // For collision detection, we typically want to flip the normal to point toward the ray
        if (a < 0.0f) {
            // Back face hit - flip the normal to point toward the ray origin
            vector3Scale(&hit->normal, &hit->normal, -1.0f);
        }
        
        // Ensure the normal is properly normalized (defensive programming)
        float normal_length_sq = vector3MagSqrd(&hit->normal);
        if (normal_length_sq < EPSILON * EPSILON) {
            // Fallback: compute normal from triangle vertices if mesh normal is invalid
            vector3Normalize(&triangle_normal, &hit->normal);
            // Apply the same winding order correction to the computed normal
            if (a < 0.0f) {
                vector3Scale(&hit->normal, &hit->normal, -1.0f);
            }
        } else if (fabsf(normal_length_sq - 1.0f) > EPSILON) {
            // Normalize if not already normalized
            vector3Normalize(&hit->normal, &hit->normal);
        }
        
        hit->hit_entity_id = 0;
        
        // Calculate intersection point
        vector3AddScaled(&ray->origin, &ray->dir, t, &hit->point);
        
        return true;
    }

    return false;  // Intersection is behind the ray or beyond max distance
}
