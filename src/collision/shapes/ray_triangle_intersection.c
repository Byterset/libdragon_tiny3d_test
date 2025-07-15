#include "ray_triangle_intersection.h"
#include "../../math/mathf.h"
#include <math.h>
#include <float.h>

bool ray_triangle_intersection(raycast *ray, raycast_hit* hit, struct mesh_triangle *triangle){

    Vector3 v0 = triangle->vertices[triangle->triangle.indices[0]];
    Vector3 v1 = triangle->vertices[triangle->triangle.indices[1]];
    Vector3 v2 = triangle->vertices[triangle->triangle.indices[2]];

    Vector3 edge1;
    vector3Sub(&v1, &v0, &edge1);
    Vector3 edge2;
    vector3Sub(&v2, &v0, &edge2);
    Vector3 h;
    vector3Cross(&ray->dir, &edge2, &h);
    float a = vector3Dot(&edge1, &h);

    if (fabs(a) < EPSILON)
        return false;  // The ray is parallel to the triangle

    float f = 1.0 / a;
    Vector3 s;
    vector3Sub(&ray->origin, &v0, &s);
    float u = f * vector3Dot(&s, &h);

    // Check if the intersection is outside the triangle
    if (u < -EPSILON || u > 1.0 + EPSILON)
        return false;

    Vector3 q;
    vector3Cross(&s, &edge1, &q);
    float v = f * vector3Dot(&ray->dir, &q);

    if (v < -EPSILON || u + v > 1.0 + EPSILON)
        return false;

    float t = f * vector3Dot(&edge2, &q);

    if (t > EPSILON) { // Ray intersects the triangle, t<0 means the triangle is behind the ray
        hit->distance = t;
        hit->normal = triangle->normal;
        hit->hit_entity_id = 0;
        vector3Scale(&ray->dir, &hit->point, t);
        vector3Add(&ray->origin, &hit->point, &hit->point);
        return true;
    }

    return false;

}