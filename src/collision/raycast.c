#include "raycast.h"
#include "../collision/collision_scene.h"
#include "../collision/shapes/lineSegment.h"
#include <math.h>
#include "epa.h"

raycast raycast_init(Vector3 origin, Vector3 dir, float maxDistance, raycast_collision_scene_mask mask, bool interact_trigger, uint16_t collision_layer_filter) {
    raycast ray = {
        .origin = origin,
        .dir = dir,
        .maxDistance = maxDistance,
        .mask = mask,
        .interact_trigger = interact_trigger,
        .collision_layer_filter = collision_layer_filter
    };
    vector3Normalize(&ray.dir, &ray.dir);
    ray._invDir.x = safeInvert(dir.x);
    ray._invDir.y = safeInvert(dir.y);
    ray._invDir.z = safeInvert(dir.z);
    return ray;
}

void raycast_transform(Transform* transform, raycast* ray, raycast* output) {
    transformPoint(transform, &ray->origin, &output->origin);
    quatMultVector(&transform->rotation, &ray->dir, &output->dir);
}

float raycast_calc_distance_to_point(raycast* ray, Vector3* point) {
    Vector3 relative;
    vector3Sub(point, &ray->origin, &relative);
    return vector3Dot(&relative, &ray->dir);
}

/// @brief Möller–Trumbore Ray-Triangle intersection algorithm
/// @param ray 
/// @param hit the resulting hit object
/// @param mesh 
/// @param triangle_index 
/// @return 
bool raycast_triangle_intersection(raycast *ray, raycast_hit* hit, struct mesh_collider *mesh, int triangle_index){

    struct mesh_triangle triangle;
    triangle.triangle = mesh->triangles[triangle_index];
    triangle.normal = mesh->normals[triangle_index];
    triangle.vertices = mesh->vertices;

    Vector3 v0 = triangle.vertices[triangle.triangle.indices[0]];
    Vector3 v1 = triangle.vertices[triangle.triangle.indices[1]];
    Vector3 v2 = triangle.vertices[triangle.triangle.indices[2]];

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
        hit->normal = triangle.normal;
        hit->hit_entity_id = 0;
        vector3Scale(&ray->dir, &hit->point, t);
        vector3Add(&ray->origin, &hit->point, &hit->point);
        return true;
    }

    return false;

}


/// @brief Tests for intersection between a ray and a physics object by treating the ray as a line segment and the object as a convex shape performing GJK/EPA
/// @param ray 
/// @param hit 
/// @param object 
/// @return true if the ray intersects the object collider, false otherwise
bool raycast_object_intersection(raycast* ray, raycast_hit* hit, struct physics_object* object){
    line_segment segment;
    segment.segment_start = ray->origin;
    vector3AddScaled(&ray->origin, &ray->dir, ray->maxDistance, &segment.segment_end);
    struct Simplex simplex;
    if (!gjkCheckForOverlap(&simplex, &segment, lineSegment_support_function, object, physics_object_gjk_support_function, &gRight))
    {
        return false;
    }
    struct EpaResult result;
    if (epaSolve(
            &simplex,
            &segment,
            lineSegment_support_function,
            object,
            physics_object_gjk_support_function,
            &result))
    {
        hit->point = result.contactB;
        hit->distance = vector3Dist(&ray->origin, &result.contactB);
        hit->normal = result.normal;
        hit->hit_entity_id = object->entity_id;
        return true;
    }

    return false;
}


/// @brief cast a ray into the existing collision scene and return a hit object if one is found
///
/// The hit object will contain the raycast hit information of the intersection with the least distance
/// @param ray pointer to the ray to be cast
/// @param hit pointer to the resulting hit object
/// @return true if the raycast has hit anything, false otherwise
bool raycast_cast(raycast* ray, raycast_hit* hit){
    struct collision_scene* collision_scene = collision_scene_get();
    // prepare result structures for AABBTree BVH queries
    int max_results = 8;
    NodeProxy results[max_results];
    int result_count = 0;
    raycast_hit current_hit;
    hit->distance = INFINITY;
    current_hit.distance = INFINITY;
    bool has_intersection = false;

    // check for intersection with the static collision scene if the mask allows it
    if(ray->mask & RAYCAST_COLLISION_SCENE_MASK_STATIC_COLLISION && collision_scene->mesh_collider != NULL){
        // query the BVH of mesh triangles for potential intersection candidates
        AABBTree_queryRay(&collision_scene->mesh_collider->aabbtree, ray, results, &result_count, max_results);

        //iterate over the results and perform the ray-triangle intersection test, update the hit object if the current result is closer
        for (size_t i = 0; i < result_count; i++)
        {
            current_hit.distance = INFINITY;
            int triangle_index = (int)AABBTreeNode_getData(&collision_scene->mesh_collider->aabbtree, results[i]);
            has_intersection = has_intersection | raycast_triangle_intersection(ray, &current_hit, collision_scene->mesh_collider, triangle_index);
            if(current_hit.distance < hit->distance){
                *hit = current_hit;
            }
        }
    }
    result_count = 0;
    
    // check for intersection with physics objects if the mask allows it
    if(ray->mask & RAYCAST_COLLISION_SCENE_MASK_PHYSICS_OBJECTS && collision_scene->objectCount > 0){

        // query the BVH of physics objects for potential intersection candidates
        AABBTree_queryRay(&collision_scene->object_aabbtree, ray, results, &result_count, max_results);

        //iterate over the results and perform the ray-object intersection test, update the hit object if the current result is closer
        for (size_t i = 0; i < result_count; i++)
        {
            struct physics_object *object = (struct physics_object *)AABBTreeNode_getData(&collision_scene->object_aabbtree, results[i]);
            // skip if the node does not contain a physics object or if the object is a trigger and the ray does not interact with triggers
            if (!object || object->collision_layers & ray->collision_layer_filter || (object->is_trigger && !ray->interact_trigger)) continue;
            current_hit.distance = INFINITY;
            has_intersection = has_intersection | raycast_object_intersection(ray, &current_hit, object);
            if(current_hit.distance < hit->distance){
                *hit = current_hit;
            }
            
        }
    }

    return has_intersection;

}


