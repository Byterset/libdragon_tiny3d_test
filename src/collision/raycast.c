#include "raycast.h"
#include "../collision/collision_scene.h"
#include "../collision/shapes/ray_shape_intersection.h"
#include "../collision/shapes/ray_triangle_intersection.h"
#include <math.h>

raycast raycast_init(Vector3 origin, Vector3 dir, float maxDistance, raycast_collision_scene_mask mask, bool interact_trigger, uint16_t collision_layer_filter) {
    assertf(maxDistance > 0.0f, "raycast maxDistance must be positive");
    raycast ray = {
        .origin = origin,
        .dir = dir,
        .maxDistance = maxDistance > RAYCAST_MAX_DISTANCE ? RAYCAST_MAX_DISTANCE : maxDistance,
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

bool raycast_cast(raycast* ray, raycast_hit* hit){
    struct collision_scene* collision_scene = collision_scene_get();
    // prepare result structures for AABBTree BVH queries
    NodeProxy results[RAYCAST_MAX_OBJECT_TESTS > RAYCAST_MAX_TRIANGLE_TESTS ? RAYCAST_MAX_OBJECT_TESTS : RAYCAST_MAX_TRIANGLE_TESTS];
    int result_count = 0;
    raycast_hit current_hit;
    hit->distance = INFINITY;
    current_hit.distance = INFINITY;
    bool has_intersection = false;

    // check for intersection with the static collision scene if the mask allows it
    if(ray->mask & RAYCAST_COLLISION_SCENE_MASK_STATIC_COLLISION && collision_scene->mesh_collider != NULL){
        // query the BVH of mesh triangles for potential intersection candidates
        AABBTree_queryRay(&collision_scene->mesh_collider->aabbtree, ray, results, &result_count, RAYCAST_MAX_TRIANGLE_TESTS);

        //iterate over the results and perform the ray-triangle intersection test, update the hit object if the current result is closer
        for (size_t i = 0; i < result_count; i++)
        {
            current_hit.distance = INFINITY;
            int triangle_index = (int)AABBTreeNode_getData(&collision_scene->mesh_collider->aabbtree, results[i]);
            struct mesh_triangle triangle;
            triangle.triangle = collision_scene->mesh_collider->triangles[triangle_index];
            triangle.normal = collision_scene->mesh_collider->normals[triangle_index];
            triangle.vertices = collision_scene->mesh_collider->vertices;

            has_intersection = has_intersection | ray_triangle_intersection(ray, &current_hit, &triangle);
            if(current_hit.distance < hit->distance && current_hit.distance <= ray->maxDistance){
                *hit = current_hit;
            }
        }
    }
    result_count = 0;
    
    // check for intersection with physics objects if the mask allows it
    if(ray->mask & RAYCAST_COLLISION_SCENE_MASK_PHYSICS_OBJECTS && collision_scene->objectCount > 0){

        // query the BVH of physics objects for potential intersection candidates
        AABBTree_queryRay(&collision_scene->object_aabbtree, ray, results, &result_count, RAYCAST_MAX_OBJECT_TESTS);

        //iterate over the results and perform the ray-object intersection test, update the hit object if the current result is closer
        for (size_t i = 0; i < result_count; i++)
        {
            struct physics_object *object = (struct physics_object *)AABBTreeNode_getData(&collision_scene->object_aabbtree, results[i]);
            // skip if the node does not contain a physics object or if the object is a trigger and the ray does not interact with triggers
            if (!object || object->collision_layers & ray->collision_layer_filter || (object->is_trigger && !ray->interact_trigger)) 
                continue;
            current_hit.distance = INFINITY;
            has_intersection = has_intersection | ray_physics_object_intersection(ray, object, &current_hit);
            if(current_hit.distance < hit->distance && current_hit.distance <= ray->maxDistance){
                *hit = current_hit;
            }
            
        }
    }

    return has_intersection;

}
