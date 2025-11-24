#include "collision_scene.h"
#include "collide.h"

#include "epa.h"
#include "../util/flags.h"
#include "../math/matrix.h"
#include "../math/mathf.h"
#include "../time/time.h"
#include <stdio.h>
#include <math.h>

const float baumgarteFactor = 0.3f;
const float steeringConstant = 0.5f;
const float maxCorrection = 0.04f;
const float slop = 0.005;



void collide_add_contact(physics_object* object, contact_constraint* constraint, physics_object* other_object) {
    contact* contact = collision_scene_new_contact();

    if (!contact) {
        return;
    }

    contact->constraint = constraint;
    contact->other_object = other_object;

    contact->next = object->active_contacts;
    object->active_contacts = contact;
}


// ============================================================================
// NEW: DETECTION-ONLY FUNCTIONS FOR ITERATIVE CONSTRAINT SOLVER
// ============================================================================

contact_constraint* cache_contact_constraint(physics_object* a, physics_object* b, const struct EpaResult* result,
                               float combined_friction, float combined_bounce, bool is_trigger) {
    struct collision_scene* scene = collision_scene_get();

    contact_pair_id pid = contact_pair_id_get(a ? a->entity_id : (entity_id)0, b ? b->entity_id : (entity_id)0);

    // Find existing constraint for this entity pair with similar normal
    contact_constraint* cont_constraint = NULL;
    float best_normal_dot = 0.90f; // Threshold for matching normals (approx 25 degrees)

    // OPTIMIZATION: Use hash map to find constraints with same PID
    intptr_t idx_plus_1 = (intptr_t)hash_map_get(&scene->contact_map, pid);
    int idx = (int)idx_plus_1 - 1;

    while (idx >= 0) {
        contact_constraint* c = &scene->cached_contact_constraints[idx];
        // Verify PID (should match if map is correct)
        if (c->pid == pid)
        {
            float dot = vector3Dot(&c->normal, &result->normal);
            if (dot > best_normal_dot)
            {
                cont_constraint = c;
                best_normal_dot = dot;
                break;
            }
        }
        idx = c->next_same_pid_index;
    }

    // If no existing constraint, create a new one
    if (!cont_constraint) {
        if (scene->cached_contact_constraint_count >= MAX_CACHED_CONTACTS) {
            return NULL; // No more room in cache
        }

        int new_idx = scene->cached_contact_constraint_count;
        cont_constraint = &scene->cached_contact_constraints[new_idx];
        scene->cached_contact_constraint_count++;
        cont_constraint->pid = pid;
        cont_constraint->point_count = 0;

        // Add to map/list
        cont_constraint->next_same_pid_index = -1;
        intptr_t existing_head_plus_1 = (intptr_t)hash_map_get(&scene->contact_map, pid);
        if (existing_head_plus_1 != 0) {
            cont_constraint->next_same_pid_index = (int)existing_head_plus_1 - 1;
        }
        hash_map_set(&scene->contact_map, pid, (void*)(intptr_t)(new_idx + 1));
    }

    // Update shared constraint data
    cont_constraint->objectA = a;
    cont_constraint->objectB = b;
    cont_constraint->normal = result->normal;
    cont_constraint->combined_friction = combined_friction;
    cont_constraint->combined_bounce = combined_bounce;
    cont_constraint->is_trigger = is_trigger;
    cont_constraint->is_active = true;

    // Calculate local points for the new contact
    Vector3 localA = result->contactA;
    Vector3 localB = result->contactB;

    
    if (a) {
        vector3Sub(&localA, a->position, &localA);
        if (a->rotation)
        {
            Quaternion invRotA;
            quatConjugate(a->rotation, &invRotA);
            quatMultVector(&invRotA, &localA, &localA);
        }
    } else {
        localA = result->contactA;
    }

    if (b) {
        vector3Sub(&localB, b->position, &localB);
        if (b->rotation)
        {
            Quaternion invRotB;
            quatConjugate(b->rotation, &invRotB);
            quatMultVector(&invRotB, &localB, &localB);
        }
    } else {
        localB = result->contactB;
    }


    // Try to match this contact point with an existing point by proximity
    const float match_distance_sq = 0.05f; // sqrt(x) units are considered the same
    int matched_point_index = -1;
    float best_dist_sq = match_distance_sq;


    for (int i = 0; i < cont_constraint->point_count; i++) {
        float dist_a = vector3DistSqrd(&cont_constraint->points[i].contactA, &result->contactA);
        float dist_b = vector3DistSqrd(&cont_constraint->points[i].contactB, &result->contactB);
        float min_dist = fminf(dist_a, dist_b);

        if (min_dist < best_dist_sq) {
            best_dist_sq = min_dist;
            matched_point_index = i;
        }
    }

    contact_point* cont_point;
    if (matched_point_index >= 0) {
        // Reuse existing contact point (preserve accumulated impulses for warm starting)
        cont_point = &cont_constraint->points[matched_point_index];
    } else {
        // Add new contact point if we have room
        if (cont_constraint->point_count >= MAX_CONTACT_POINTS_PER_PAIR) {
            // Replace the point with smallest penetration (least important)
            int min_pen_index = 0;
            float min_pen = cont_constraint->points[0].penetration;
            for (int i = 1; i < cont_constraint->point_count; i++) {
                if (cont_constraint->points[i].penetration < min_pen) {
                    min_pen = cont_constraint->points[i].penetration;
                    min_pen_index = i;
                }
            }
            // Only replace if new point has deeper penetration
            if (result->penetration > min_pen) {
                cont_point = &cont_constraint->points[min_pen_index];
                // Reset accumulated impulses for new point
                cont_point->accumulated_normal_impulse = 0.0f;
                cont_point->accumulated_tangent_impulse_u = 0.0f;
                cont_point->accumulated_tangent_impulse_v = 0.0f;
            } else {
                // Don't add this point, but we still need to validate others!
                goto validate_others;
            }
        } else {
            // Add new point
            cont_point = &cont_constraint->points[cont_constraint->point_count];
            cont_constraint->point_count++;
            // Initialize accumulated impulses
            cont_point->accumulated_normal_impulse = 0.0f;
            cont_point->accumulated_tangent_impulse_u = 0.0f;
            cont_point->accumulated_tangent_impulse_v = 0.0f;
        }
    }

    // Update contact point data
    cont_point->point = result->contactA;
    cont_point->contactA = result->contactA;
    cont_point->contactB = result->contactB;
    cont_point->localPointA = localA;
    cont_point->localPointB = localB;
    cont_point->penetration = result->penetration;
    cont_point->active = true;

validate_others:
    // Validate other points against the new normal
    for (int i = 0; i < cont_constraint->point_count; i++) {
        contact_point* cp = &cont_constraint->points[i];
        if (cp->active) continue; // Already processed

        // Calculate penetration depth along the new normal
        // Normal points B -> A
        // diff = A - B
        // If overlapping, A is "behind" B relative to normal, so dot(diff, normal) is negative
        // We want penetration to be positive for overlap, so we negate the dot product
        Vector3 diff;
        vector3Sub(&cp->contactA, &cp->contactB, &diff);
        float pen = -vector3Dot(&diff, &cont_constraint->normal);

        // Keep point if it's still penetrating or very close (allow small separation)
        // pen > -0.05f means "separation < 0.05"
        if (pen > -0.05f) {
            cp->penetration = pen;
            cp->active = true;
        }
    }

    return cont_constraint;
}

bool detect_contact_object_to_triangle(physics_object* object, const struct mesh_collider* mesh, int triangle_index) {
    struct mesh_triangle triangle;
    triangle.triangle = mesh->triangles[triangle_index];
    triangle.normal = mesh->normals[triangle_index];
    triangle.vertices = mesh->vertices;

    struct Simplex simplex;
    Vector3 firstDir = gRight;
    if (!gjkCheckForOverlap(&simplex, &triangle, mesh_triangle_gjk_support_function, object, physics_object_gjk_support_function, &firstDir))
    {
        return false;
    }

    struct EpaResult result;
    if (epaSolve(
            &simplex,
            &triangle,
            mesh_triangle_gjk_support_function,
            object,
            physics_object_gjk_support_function,
            &result))
    {
        // Cache the contact (entity_a = 0 for static mesh)
        contact_constraint* constraint = cache_contact_constraint(NULL, object, &result, object->collision->friction, object->collision->bounce, false);

        // Still add to old contact list for ground detection logic
        collide_add_contact(object, constraint, NULL);

        return true;
    }

    return false;
}

void detect_contacts_object_to_mesh(physics_object* object, const struct mesh_collider* mesh) {
    int result_count = 0;
    int max_results = 64; // Increased from 20 to handle complex geometry with multiple simultaneous contacts
    node_proxy results[max_results];

    AABB_tree_query_bounds(&mesh->aabbtree, &object->bounding_box, results, &result_count, max_results);
    for (size_t j = 0; j < result_count; j++)
    {
        int triangle_index = (int)AABB_tree_get_node_data(&mesh->aabbtree, results[j]);
        detect_contact_object_to_triangle(object, mesh, triangle_index);
    }
}

void detect_contact_object_to_object(physics_object* a, physics_object* b) {
    // If the Objects don't share any collision layers, don't collide
    if (!(a->collision_layers & b->collision_layers)) {
        return;
    }

    // If the objects are in the same collision group, don't collide
    if (a->collision_group && (a->collision_group == b->collision_group)) {
        return;
    }

    // triggers can't collide with each other
    if (a->is_trigger && b->is_trigger) {
        return;
    }

    struct Simplex simplex;
    struct EpaResult result;

    Vector3 delta;
    float dist_sq;
    float radii_sum;
    bool is_sphere_sphere = a->collision->shape_type == COLLISION_SHAPE_SPHERE && b->collision->shape_type == COLLISION_SHAPE_SPHERE;

    // Sphere-Sphere Optimization
    if(is_sphere_sphere){
        vector3FromTo(b->position, a->position, &delta);
        dist_sq = vector3MagSqrd(&delta);
        radii_sum = a->collision->shape_data.sphere.radius + b->collision->shape_data.sphere.radius;
        float radii_sum_sq = radii_sum * radii_sum;
        if(dist_sq >= radii_sum_sq)
            return;
    }
    else{
        Vector3 firstDir = gRight;
        if (!gjkCheckForOverlap(&simplex, a, physics_object_gjk_support_function, b, physics_object_gjk_support_function, &firstDir))
        {
            return;
        }
    }

    // Handle trigger contacts
    if (a->is_trigger || b->is_trigger) {
        struct EpaResult dummy_result;
        dummy_result.normal = gZeroVec;
        dummy_result.contactA = *a->position;
        dummy_result.contactB = *b->position;
        dummy_result.penetration = 0.0f;

        contact_constraint* constraint = cache_contact_constraint(a, b, &dummy_result, 0.0f, 0.0f, true);

        if (constraint) {
            if (a->is_trigger) {
                collide_add_contact(b, constraint, a);
            } else {
                collide_add_contact(a, constraint, b);
            }
        }

        return;
    }

    // Compute EPA result
    if(is_sphere_sphere){
        float dist = sqrtf(dist_sq);
        result.penetration = radii_sum - dist;
        if (dist > EPSILON)
            vector3Normalize(&delta, &result.normal);
        else
            result.normal = gUp;

        vector3AddScaled(a->position, &result.normal, -a->collision->shape_data.sphere.radius, &result.contactA);
        vector3AddScaled(b->position, &result.normal, b->collision->shape_data.sphere.radius, &result.contactB);
    }
    else{
        bool epa_success = epaSolve(
            &simplex,
            a,
            physics_object_gjk_support_function,
            b,
            physics_object_gjk_support_function,
            &result);

        if (!epa_success)
        {
            return; // Skip if EPA fails
        }
    }

    // Wake up sleeping objects only if the collision is energetic enough
    // This allows stacked objects to sleep
    Vector3 velA = gZeroVec;
    Vector3 velB = gZeroVec;

    if (a && !a->is_kinematic) {
        velA = a->velocity;
        if (a->rotation) {
            Vector3 centerOfMassA;
            Vector3 rotatedOffset;
            quatMultVector(a->rotation, &a->center_offset, &rotatedOffset);
            vector3Add(a->position, &rotatedOffset, &centerOfMassA);
            
            Vector3 rA;
            vector3Sub(&result.contactA, &centerOfMassA, &rA);
            Vector3 angularPart;
            vector3Cross(&a->angular_velocity, &rA, &angularPart);
            vector3Add(&velA, &angularPart, &velA);
        }
    }

    if (b && !b->is_kinematic) {
        velB = b->velocity;
        if (b->rotation) {
            Vector3 centerOfMassB;
            Vector3 rotatedOffset;
            quatMultVector(b->rotation, &b->center_offset, &rotatedOffset);
            vector3Add(b->position, &rotatedOffset, &centerOfMassB);
            
            Vector3 rB;
            vector3Sub(&result.contactB, &centerOfMassB, &rB);
            Vector3 angularPart;
            vector3Cross(&b->angular_velocity, &rB, &angularPart);
            vector3Add(&velB, &angularPart, &velB);
        }
    }

    Vector3 relVel;
    vector3Sub(&velA, &velB, &relVel);
    float impactSpeedSq = vector3MagSqrd(&relVel);
    
    // Use a threshold slightly higher than the sleep threshold to ensure stability.
    // If objects are moving slower than the sleep threshold, they shouldn't wake each other up.
    const float wake_threshold_sq = PHYS_OBJECT_SPEED_SLEEP_THRESHOLD_SQ * 1.5f;

    if (impactSpeedSq > wake_threshold_sq) {
        if (a && !a->is_kinematic) physics_object_wake(a);
        if (b && !b->is_kinematic) physics_object_wake(b);
    } 
    // The solver will transfer momentum if necessary, and if the resulting velocity
    // is high enough, the object will wake up in the next update cycle.

    // Combined friction and bounce
    float combined_friction = minf(a->collision->friction, b->collision->friction);
    float combined_bounce = a->collision->bounce * b->collision->bounce;//minf(a->collision->bounce, b->collision->bounce);

    // Cache the contact constraint
    contact_constraint* constraint = cache_contact_constraint(a, b, &result, combined_friction, combined_bounce, false);

    // Still add to old contact lists for compatibility
    if (constraint) {
        collide_add_contact(a, constraint, b);
        collide_add_contact(b, constraint, a);
    }
}