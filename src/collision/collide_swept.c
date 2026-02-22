#include "collide_swept.h"

#include <math.h>
#include <float.h>
#include "collision_scene.h"
#include "collide.h"
#include "gjk.h"
#include "epa.h"
#include "raycast.h"
#include "shapes/ray_triangle_intersection.h"
#include <stdio.h>

/// @brief Internal structure to represent a swept physics object for GJK support.
struct swept_physics_object {
    physics_object* object;
    Vector3 offset;
};

struct swept_hit_candidate {
    bool has_hit;
    float t;
    Vector3 position;
    struct EpaResult result;
};

struct swept_triangle_check_ctx {
    struct object_mesh_collide_data* collide_data;
    Vector3 sweep_start;
    Vector3 sweep_end;
    Vector3 sweep_dir;
    float sweep_dist;
    struct swept_hit_candidate best;
};

#define COLLIDE_SWEPT_MAX_ITERATIONS 4
#define COLLIDE_SWEPT_EPSILON 0.0001f
#define COLLIDE_SWEPT_BACKOFF 0.01f

/// @brief GJK support function for a swept physics object.
/// The support point is the support point of the object at the current position,
/// extended by the sweep offset if the direction aligns with the sweep.
static void collide_swept_gjk_support_function(const void* data, const Vector3* direction, Vector3* output) {
    struct swept_physics_object* obj = (struct swept_physics_object*)data;
    Vector3 norm_dir;
    vector3Normalize(direction, &norm_dir);
    physics_object_gjk_support_function(obj->object, direction, output);

    if (vector3Dot(&obj->offset, &norm_dir) > 0.0f) {
        vector3Add(output, &obj->offset, output);
    }
}

/// @brief Initializes the collision data structure.
static void collide_swept_data_init(
    struct object_mesh_collide_data* data,
    Vector3* prev_pos,
    struct mesh_collider* mesh,
    physics_object* object
) {
    data->prev_pos = prev_pos;
    data->mesh = mesh;
    data->object = object;
}

/// @brief Checks for collision between a swept object and a single triangle.
static bool collide_swept_triangle_check(void* data, int triangle_index) {
    struct swept_triangle_check_ctx* ctx = (struct swept_triangle_check_ctx*)data;
    struct object_mesh_collide_data* collide_data = ctx->collide_data;
    
    // Use raycast for high-speed tunneling prevention
    // This is more robust than EPA on swept hull for thin walls
    Vector3 dir = ctx->sweep_dir;
    float dist = ctx->sweep_dist;
    
    if (dist > COLLIDE_SWEPT_EPSILON) {
        raycast ray;
        ray.origin = ctx->sweep_start;
        ray.dir = dir;
        vector3Scale(&ray.dir, &ray.dir, 1.0f / dist);
        ray.maxDistance = dist;
        
        struct mesh_triangle triangle;
        triangle.vertices = collide_data->mesh->vertices;
        triangle.triangle = collide_data->mesh->triangles[triangle_index];
        triangle.normal = collide_data->mesh->normals[triangle_index];
        
        raycast_hit hit;
        if (ray_triangle_intersection(&ray, &hit, &triangle)) {
            float t = hit.distance / dist;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;

            // Construct a result that looks like EPA result
            struct EpaResult ray_result;
            ray_result.normal = hit.normal;
            ray_result.penetration = 0.0f;
            ray_result.contactA = hit.point; // On triangle
            ray_result.contactB = hit.point; // On object (approx)

            // Candidate position is the hit point minus a small back-off along travel dir
            Vector3 back_off;
            Vector3 candidate_pos;
            vector3Scale(&ray.dir, &back_off, COLLIDE_SWEPT_BACKOFF);
            vector3Sub(&hit.point, &back_off, &candidate_pos);

            if (!ctx->best.has_hit || t < ctx->best.t) {
                ctx->best.has_hit = true;
                ctx->best.t = t;
                ctx->best.position = candidate_pos;
                ctx->best.result = ray_result;
            }
        }
    }

    struct swept_physics_object swept;
    swept.object = collide_data->object;
    vector3Sub(&ctx->sweep_start, &ctx->sweep_end, &swept.offset);

    struct mesh_triangle triangle;
    triangle.vertices = collide_data->mesh->vertices;
    triangle.triangle = collide_data->mesh->triangles[triangle_index];
    triangle.normal = collide_data->mesh->normals[triangle_index];

    struct Simplex simplex;
    Vector3 firstDir = gRight;
    if (!gjkCheckForOverlap(&simplex, &triangle, mesh_triangle_gjk_support_function, &swept, collide_swept_gjk_support_function, &firstDir)) {
        return false;
    }

    struct EpaResult result;
    Vector3 swept_end_candidate = ctx->sweep_end;
    if (epaSolveSwept(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            &swept,
            collide_swept_gjk_support_function,
            &ctx->sweep_start,
            &swept_end_candidate,
            &result))
    {
        float t = 0.0f;
        if (ctx->sweep_dist > COLLIDE_SWEPT_EPSILON) {
            t = vector3Dist(&ctx->sweep_start, &swept_end_candidate) / ctx->sweep_dist;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
        }

        if (!ctx->best.has_hit || t < ctx->best.t) {
            ctx->best.has_hit = true;
            ctx->best.t = t;
            ctx->best.position = swept_end_candidate;
            ctx->best.result = result;
        }
        return true;
    }

    // Fallback: Check static collision at previous position if swept failed but overlap exists
    Vector3 final_pos = *collide_data->object->position;
    *collide_data->object->position = ctx->sweep_start;

    if (epaSolve(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            collide_data->object,
            physics_object_gjk_support_function,
            &result))
    {
        bool ignore = false;
        contact* c = collide_data->object->active_contacts;
        while (c) {
            if (c->other_object == NULL && vector3Dot(&result.normal, &c->constraint->normal) > 0.9f) {
                ignore = true;
                break;
            }
            c = c->next;
        }

        if (!ignore) {
            if (!ctx->best.has_hit || 0.0f < ctx->best.t) {
                ctx->best.has_hit = true;
                ctx->best.t = 0.0f;
                ctx->best.position = ctx->sweep_start;
                ctx->best.result = result;
            }
            *collide_data->object->position = final_pos; // Restore position
            return true;
        }
    }
    *collide_data->object->position = final_pos;

    return false;
}

/// @brief Resolves the collision by bouncing the object off the surface.
static void collide_swept_resolve_bounce(
    physics_object* object, 
    struct object_mesh_collide_data* collide_data,
    Vector3* start_pos
) {
    // this is the new prev position when iterating
    // over multiple swept collisions
    *collide_data->prev_pos = *object->position;

    Vector3 move_amount;
    vector3Sub(start_pos, object->position, &move_amount);

    // split the move amount due to the collision into normal and tangent components
    Vector3 move_amount_normal;
    vector3Project(&move_amount, &collide_data->hit_result.normal, &move_amount_normal);
    Vector3 move_amount_tangent;
    vector3Sub(&move_amount, &move_amount_normal, &move_amount_tangent);

    vector3Scale(&move_amount_normal, &move_amount_normal, -object->collision->bounce);


    vector3Add(object->position, &move_amount_normal, object->position);
    vector3Add(object->position, &move_amount_tangent, object->position);

    // don't include friction on a bounce
    collide_correct_velocity(object, &collide_data->hit_result, 0.0f, object->collision->bounce);

    vector3Sub(object->position, start_pos, &move_amount);
    vector3Add(&move_amount, &object->bounding_box.min, &object->bounding_box.min);
    vector3Add(&move_amount, &object->bounding_box.max, &object->bounding_box.max);

    //Add new contact to object (object is contact Point B in the case of mesh collision)
    // Cache the contact (entity_a = 0 for static mesh)
    contact_constraint *constraint = collide_cache_contact_constraint(NULL, object, &collide_data->hit_result, 0, object->collision->bounce, false);
    if (constraint) {
        constraint->is_active = false;
        // Still add to old contact list for ground detection logic
        collide_add_contact(object, constraint, NULL);
    }
}


bool collide_object_to_mesh_swept(physics_object* object, struct mesh_collider* mesh, Vector3* prev_pos){
    if (object->is_trigger) {
        return false;
    }

    struct object_mesh_collide_data collide_data;
    collide_swept_data_init(&collide_data, prev_pos, mesh, object);

    bool did_any_hit = false;

    for (int iteration = 0; iteration < COLLIDE_SWEPT_MAX_ITERATIONS; ++iteration) {
        Vector3 sweep_start = *prev_pos;
        Vector3 sweep_end = *object->position;
        Vector3 sweep_dir;
        vector3FromTo(&sweep_start, &sweep_end, &sweep_dir);
        float sweep_dist = vector3Mag(&sweep_dir);

        if (sweep_dist <= COLLIDE_SWEPT_EPSILON) {
            break;
        }

        Vector3 box_extent;
        vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &box_extent);
        vector3Scale(&box_extent, &box_extent, 0.5f);

        AABB sweep_start_box;
        vector3Sub(&sweep_start, &box_extent, &sweep_start_box.min);
        vector3Add(&sweep_start, &box_extent, &sweep_start_box.max);

        AABB sweep_end_box;
        vector3Sub(&sweep_end, &box_extent, &sweep_end_box.min);
        vector3Add(&sweep_end, &box_extent, &sweep_end_box.max);

        // Span a box from sweep start to sweep end to catch all possible triangle collisions
        AABB expanded_box = AABBUnion(&sweep_start_box, &sweep_end_box);

        int result_count = 0;
        int max_results = 20;
        node_proxy results[max_results];
        AABB_tree_query_bounds(&mesh->aabbtree, &expanded_box, results, &result_count, max_results);

        struct swept_triangle_check_ctx check_ctx;
        check_ctx.collide_data = &collide_data;
        check_ctx.sweep_start = sweep_start;
        check_ctx.sweep_end = sweep_end;
        check_ctx.sweep_dir = sweep_dir;
        check_ctx.sweep_dist = sweep_dist;
        check_ctx.best.has_hit = false;
        check_ctx.best.t = FLT_MAX;

        for (size_t j = 0; j < result_count; j++) {
            int triangle_index = (int)AABB_tree_get_node_data(&mesh->aabbtree, results[j]);
            collide_swept_triangle_check(&check_ctx, triangle_index);
        }

        if (!check_ctx.best.has_hit) {
            break;
        }

        did_any_hit = true;
        *object->position = check_ctx.best.position;
        collide_data.hit_result = check_ctx.best.result;

        // Resolve using the motion target from this iteration and continue with remaining motion
        collide_swept_resolve_bounce(object, &collide_data, &sweep_end);
    }

    return did_any_hit;
}