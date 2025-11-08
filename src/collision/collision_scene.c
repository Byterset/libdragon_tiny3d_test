#include "collision_scene.h"

#include <malloc.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <libdragon.h>

#include "mesh_collider.h"
#include "collide.h"
#include "collide_swept.h"
#include "../util/hash_map.h"

#include "../render/defs.h"
#include "../game/gamestate.h"

#ifdef DEBUG_COLLIDERS_RAYLIB
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#endif


struct collision_scene g_scene;

void collision_scene_reset() {
    free(g_scene.elements);
    free(g_scene.all_contacts);
    AABBTree_free(&g_scene.object_aabbtree);
    hash_map_destroy(&g_scene.entity_mapping);

    hash_map_init(&g_scene.entity_mapping, MIN_PHYSICS_OBJECTS);
    AABBTree_create(&g_scene.object_aabbtree, MIN_PHYSICS_OBJECTS);
    g_scene.elements = malloc(sizeof(struct collision_scene_element) * MIN_PHYSICS_OBJECTS);
    g_scene.capacity = MIN_PHYSICS_OBJECTS;
    g_scene.objectCount = 0;
    AABBTree_free(&g_scene.mesh_collider->aabbtree);
    g_scene.mesh_collider = NULL;
    g_scene.all_contacts = malloc(sizeof(struct contact) * MAX_ACTIVE_CONTACTS);
    g_scene.next_free_contact = &g_scene.all_contacts[0];

    for (int i = 0; i + 1 < MAX_ACTIVE_CONTACTS; ++i) {
        g_scene.all_contacts[i].next = &g_scene.all_contacts[i + 1];
    }

    g_scene.all_contacts[MAX_ACTIVE_CONTACTS - 1].next = NULL;
}

struct collision_scene* collision_scene_get() {
    return &g_scene;
}

void collision_scene_add(struct physics_object* object) {
    if (g_scene.objectCount >= g_scene.capacity) {
        g_scene.capacity *= 2;
        g_scene.elements = realloc(g_scene.elements, sizeof(struct collision_scene_element) * g_scene.capacity);
    }

    struct collision_scene_element* next = &g_scene.elements[g_scene.objectCount];

    next->object = object;

    g_scene.objectCount += 1;

    hash_map_set(&g_scene.entity_mapping, object->entity_id, object);
    object->_aabb_tree_node_id = AABBTreeNode_createNode(&g_scene.object_aabbtree, object->bounding_box, object);
}

/// @brief Returns the physics object associated with the given entity id if it exists in the collision scene.
struct physics_object* collision_scene_find_object(entity_id id) {
    if (!id) {
        return 0;
    }

    return hash_map_get(&g_scene.entity_mapping, id);
}

/**
 * @brief Returns the active contacts of a physics object to the global scene's free contact list.
 *
 * This function iterates through the active contacts of the given physics object and appends
 * them to the global scene's free contact list. It ensures that the active contacts of the
 * object are properly returned and the object's active contact list is cleared.
 *
 * @param object Pointer to the physics object whose active contacts are to be returned.
 */
void collision_scene_release_object_contacts(struct physics_object* object) {
    struct contact* last_contact = object->active_contacts;

    while (last_contact && last_contact->next) {
        last_contact = last_contact->next;
    }

    if (last_contact) {
        last_contact->next = g_scene.next_free_contact;
        g_scene.next_free_contact = object->active_contacts;
        object->active_contacts = NULL;
    }
}

/// @brief Removes a physics object from the collision scene. 
///
/// The removed object will no longer be updated in the phys loop or considered for collision.
/// All contacts associated with the object will be released.
/// @param object 
void collision_scene_remove(struct physics_object* object) {
    bool has_found = false;

    for (int i = 0; i < g_scene.objectCount; ++i) {
        if (object == g_scene.elements[i].object) {
            collision_scene_release_object_contacts(object);
            has_found = true;
        }

        if (has_found) {
            g_scene.elements[i] = g_scene.elements[i + 1];
        }
    }

    if (has_found) {
        g_scene.objectCount -= 1;
    }
    AABBTree_removeLeaf(&g_scene.object_aabbtree, object->_aabb_tree_node_id, true);
    hash_map_delete(&g_scene.entity_mapping, object->entity_id);
}


void collision_scene_use_static_collision(struct mesh_collider* mesh_collider) {
    g_scene.mesh_collider = mesh_collider;
}


/// @brief Removes the current static collision mesh from the scene.
void collision_scene_remove_static_collision() {
    AABBTree_free(&g_scene.mesh_collider->aabbtree);
    g_scene.mesh_collider = NULL;
}


/// @brief Performs collision detection for a single physics object agains other objects in the scene.
///
/// First a broad phase detection using a AABB BVH tree is performed to find potential collision candidates.
/// Then a narrow phase detection (GJK/EPA) is performed to check for and resolve actual collisions.
/// @param element the scene element (phys object) to check for collisions
void collision_scene_collide_phys_object(struct collision_scene_element* element) {

    //perform a broad phase check to find potential collision candidates by traversing the AABB BVH tree 
    //and collecting leaf nodes that overlap with the object's bounding box
    int result_count = 0;
    int max_results = 4;
    NodeProxy results[max_results];
    AABBTree_queryBounds(&g_scene.object_aabbtree, &element->object->bounding_box, results, &result_count, max_results);

    //iterate over the list of candidates and perform detailed collision detection
    for (size_t i = 0; i < result_count; i++)
    {
        struct physics_object *other = (struct physics_object *)AABBTreeNode_getData(&g_scene.object_aabbtree, results[i]);
        // skip narrow phase if there is no physics object associated with the result node
        // or if it is the same object as the one queried
        if (!other || other == element->object)
        {
            continue;
        }
        collide_object_to_object(element->object, other);

    }
}

#define MAX_SWEPT_ITERATIONS    5

void collision_scene_collide_object_to_static(struct physics_object* object, Vector3* prev_pos) {

    for (int i = 0; i < MAX_SWEPT_ITERATIONS; i += 1)
    {
        Vector3 offset;
        vector3Sub(object->position, prev_pos, &offset);
        Vector3 bounding_box_size;
        vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &bounding_box_size);
        vector3Scale(&bounding_box_size, &bounding_box_size, 0.5f);

        // if the object has moved more than the bounding box size perform a swept collision check
        if (fabs(offset.x) > bounding_box_size.x ||
            fabs(offset.y) > bounding_box_size.y ||
            fabs(offset.z) > bounding_box_size.z)
        {
            if (!collide_object_to_mesh_swept(object, g_scene.mesh_collider, prev_pos))
            {
                return;
            }
        }
        // otherwise just do a normal collision check
        else
        {
            collide_object_to_mesh(object, g_scene.mesh_collider);

            return;
        }
    }

    // too many swept iterations
    // to prevent tunneling just move the object back 
    // to the previous known valid location
    *object->position = *prev_pos;
}

/// @brief performs a physics step on all objects in the scene, updating their positions, velocities, Bounding Boxes and performing collision detection
void collision_scene_step() {
    struct collision_scene_element* element;
    bool moved_flags[g_scene.objectCount];
    bool rotated_flags[g_scene.objectCount];

    // First loop: Position updates and AABB maintenance
    for (int i = 0; i < g_scene.objectCount; ++i) {
        element = &g_scene.elements[i];
        struct physics_object* obj = element->object;

        if (!obj->_is_sleeping) {
            if (obj->has_gravity && !obj->is_kinematic) {
                // Check if object has ground contact
                bool hasGroundContact = false;
                if (obj->active_contacts) {
                    struct contact* c = obj->active_contacts;
                    while (c) {
                        // If contact normal points upward (more lenient threshold)
                        // 0.5 = surfaces up to 60° from horizontal
                        // This prevents issues with rotating platforms or numerical precision
                        if (c->normal.y > 0.5f) {
                            hasGroundContact = true;
                            break;
                        }
                        c = c->next;
                    }
                }

                // Only apply full gravity if not grounded, or apply reduced gravity for stability
                if (!hasGroundContact) {
                    obj->acceleration.y += PHYS_GRAVITY_CONSTANT * obj->gravity_scalar;
                } else {
                    // Apply minimal gravity to grounded objects (maintains contact pressure)
                    // but not enough to cause jitter
                    obj->acceleration.y += PHYS_GRAVITY_CONSTANT * obj->gravity_scalar * 0.10f;
                }
            }

            collision_scene_release_object_contacts(obj);
        }
        physics_object_update_velocity_verlet(obj);
        physics_object_update_angular_velocity(obj);

        // Track movement for AABB updates
        const bool has_moved = !vector3Equals(&obj->_prev_step_pos, obj->position);
        const bool has_rotated = obj->rotation ? !quatIsIdentical(obj->rotation, &obj->_prev_step_rot) : false;
        moved_flags[i] = has_moved;
        rotated_flags[i] = has_rotated;

        if (!obj->_is_sleeping && (has_moved || has_rotated)) {
            // would be technically correct to do this after all objects have been updated but this saves a loop
            collision_scene_collide_phys_object(element);


            physics_object_recalculate_aabb(obj);
            Vector3 displacement;
            vector3Sub(obj->position, &obj->_prev_step_pos, &displacement);
            AABBTree_moveNode(&g_scene.object_aabbtree, obj->_aabb_tree_node_id,
                            obj->bounding_box, &displacement);
        }
    }

    // Second loop: Mesh Collision, Constraints and Sleep State Update
    for (int i = 0; i < g_scene.objectCount; ++i) {
        element = &g_scene.elements[i];
        struct physics_object* obj = element->object;

        // Only do mesh collision if the object is not sleeping, not fixed in place, is tangible and has moved or rotated previously 
        if (g_scene.mesh_collider && !obj->_is_sleeping && !obj->is_kinematic && 
            (obj->collision_layers & (COLLISION_LAYER_TANGIBLE)) && (moved_flags[i] || rotated_flags[i])) {
            collision_scene_collide_object_to_static(obj, &obj->_prev_step_pos);
        }

        // Apply physical constraints to the object
        physics_object_apply_constraints(obj);

        // Update sleep state using comprehensive checks
        const bool insignificant_movement = vector3DistSqrd(obj->position, &obj->_prev_step_pos) < PHYS_OBJECT_SLEEP_THRESHOLD_SQ;
        const bool no_rotation = !rotated_flags[i];

        if (insignificant_movement && no_rotation) {
            if (obj->_sleep_counter < PHYS_OBJECT_SLEEP_STEPS) {
                obj->_sleep_counter += 1;
            }
            else obj->_is_sleeping = true;
        } else {
            obj->_is_sleeping = false;
            obj->_sleep_counter = 0;
        }

        // Update previous state with current state
        obj->_prev_step_pos = *obj->position;
        if (obj->rotation)
        {
            obj->_prev_step_rot = *obj->rotation;
        }
    }
}

/// @brief Returns a new contact from the global scene's free contact list, NULL if none are available.
struct contact* collision_scene_new_contact() {
    if (!g_scene.next_free_contact) {
        return NULL;
    }

    struct contact* result = g_scene.next_free_contact;
    g_scene.next_free_contact = result->next;
    return result;
}

#ifdef DEBUG_COLLIDERS_RAYLIB
/// @brief Renders the collision scene colliders in debug mode using raylib.
void collision_scene_render_debug_raylib(){
    // if(g_scene.mesh_collider){ 
    //     DrawModelWires(g_scene.mesh_collider->raylib_mesh_model, (Raylib_Vector3){0, 0, 0}, 1, YELLOW);   
    // }
    for (int i = 0; i < g_scene.objectCount; ++i) {
        struct collision_scene_element* element = &g_scene.elements[i];
        struct physics_object* object = element->object;

        Vector3 center_offset_rotated;

        DrawBoundingBox(
            (BoundingBox){
                (Raylib_Vector3){object->bounding_box.min.x, object->bounding_box.min.y, object->bounding_box.min.z},
                (Raylib_Vector3){object->bounding_box.max.x, object->bounding_box.max.y, object->bounding_box.max.z}},
            RED);
        if (object->rotation)
            quatMultVector(object->rotation, &object->center_offset, &center_offset_rotated);
        else
            vector3Copy(&object->center_offset, &center_offset_rotated);

        if(object->collision->shape_type == COLLISION_SHAPE_CAPSULE){
            // Get capsule dimensions
            float half_height = object->collision->shape_data.capsule.inner_half_height;
            float radius = object->collision->shape_data.capsule.radius;

            // Define the capsule's central axis in local space
            Vector3 local_axis = {{0.0f, half_height, 0.0f}};

            // Rotate the central axis by the given rotation to get its orientation in world space
            Vector3 world_axis;
            
            if (object->rotation)
                quatMultVector(object->rotation, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            Vector3 start;
            Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &center_offset_rotated, &end);
            vector3Add(&start, &center_offset_rotated, &start);

            DrawCapsuleWires(
                (Raylib_Vector3){start.x, start.y, start.z},
                (Raylib_Vector3){end.x, end.y, end.z},
                radius,
                4,
                2,
                PINK
            );
        }
        else if(object->collision->shape_type == COLLISION_SHAPE_BOX){
            // Get capsule dimensions
            Vector3 half_size = object->collision->shape_data.box.half_size;
            Vector3 worldPos;

            Raylib_Mesh cubeMesh = GenMeshCube(half_size.x * 2, half_size.y * 2, half_size.z * 2);

            Raylib_Model cubeModel = LoadModelFromMesh(cubeMesh);

            Raylib_Quaternion q = {.x = object->rotation->x, .y = object->rotation->y, .z = object->rotation->z, .w = object->rotation->w};
            Matrix m = QuaternionToMatrix(q);
            cubeModel.transform = MatrixMultiply(m, cubeModel.transform);
            vector3Add(object->position, &center_offset_rotated, &worldPos);
            
            Raylib_Vector3 pos = {worldPos.x, worldPos.y, worldPos.z};

            DrawModelWires(cubeModel, pos, 1, GREEN);
            UnloadModel(cubeModel);
        }        
        else if(object->collision->shape_type == COLLISION_SHAPE_SPHERE){
            float radius = object->collision->shape_data.sphere.radius;
            Vector3 worldPos;
            vector3Add(object->position, &center_offset_rotated, &worldPos);
            Raylib_Vector3 pos = {worldPos.x, worldPos.y, worldPos.z};
            DrawSphereWires(pos, radius, 5, 5, PINK);
        }
        else if(object->collision->shape_type == COLLISION_SHAPE_CYLINDER){
            // Get capsule dimensions
            float half_height = object->collision->shape_data.cylinder.half_height;
            float radius = object->collision->shape_data.cylinder.radius;

            // Define the capsule's central axis in local space
            Vector3 local_axis = {{0.0f, half_height, 0.0f}};

            // Rotate the central axis by the given rotation to get its orientation in world space
            Vector3 world_axis;
            
            if (object->rotation)
                quatMultVector(object->rotation, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            Vector3 start;
            Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &center_offset_rotated, &end);
            vector3Add(&start, &center_offset_rotated, &start);

            DrawCylinderWiresEx(
                (Raylib_Vector3){start.x, start.y, start.z},
                (Raylib_Vector3){end.x, end.y, end.z},
                radius,
                radius,
                8,
                BLUE
            );
        }
        else if(object->collision->shape_type == COLLISION_SHAPE_CONE){
            // Get capsule dimensions
            float half_height = object->collision->shape_data.cone.half_height;
            float radius = object->collision->shape_data.cone.radius;

            // Define the capsule's central axis in local space
            Vector3 local_axis = {{0.0f, half_height, 0.0f}};

            // Rotate the central axis by the given rotation to get its orientation in world space
            Vector3 world_axis;
            
            if (object->rotation)
                quatMultVector(object->rotation, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            Vector3 start;
            Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &center_offset_rotated, &end);
            vector3Add(&start, &center_offset_rotated, &start);

            DrawLine3D(
                (Raylib_Vector3){start.x, start.y, start.z},
                (Raylib_Vector3){end.x, end.y, end.z},
                VIOLET
            );
        }
    }
}
#endif