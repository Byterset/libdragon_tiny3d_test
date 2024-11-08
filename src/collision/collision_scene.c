#include "collision_scene.h"

#include <malloc.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <libdragon.h>

#include "mesh_collider.h"
#include "collide.h"
#include "collide_swept.h"
#include "contact.h"
#include "../util/hash_map.h"

#include "../render/defs.h"
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include "../game/gamestate.h"


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
    g_scene.count = 0;
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
    if (g_scene.count >= g_scene.capacity) {
        g_scene.capacity *= 2;
        g_scene.elements = realloc(g_scene.elements, sizeof(struct collision_scene_element) * g_scene.capacity);
    }

    struct collision_scene_element* next = &g_scene.elements[g_scene.count];

    next->object = object;

    g_scene.count += 1;

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

    for (int i = 0; i < g_scene.count; ++i) {
        if (object == g_scene.elements[i].object) {
            collision_scene_release_object_contacts(object);
            has_found = true;
        }

        if (has_found) {
            g_scene.elements[i] = g_scene.elements[i + 1];
        }
    }

    if (has_found) {
        g_scene.count -= 1;
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

    int result_count = 0;
    int aabbCheck_count = 0;
    int max_results = 5;
    NodeProxy results[max_results];

    AABBTree_queryBounds(&g_scene.object_aabbtree, &element->object->bounding_box, results, &result_count, &aabbCheck_count, max_results);
    for (size_t j = 0; j < result_count; j++)
    {
        struct physics_object *other = (struct physics_object *)AABBTreeNode_getData(&g_scene.object_aabbtree, results[j]);
        // skip narrow phase if there is no physics object associated with the result node
        // or if it is the same object as the one queried
        if (!other || other == element->object)
        {
            continue;
        }
        // only do detailed collision calculation if the bounding boxes overlap
        if (AABBHasOverlap(&element->object->bounding_box, &other->bounding_box))
            collide_object_to_object(element->object, other);
    }
}

#define MAX_SWEPT_ITERATIONS    5

void collision_scene_collide_single(struct physics_object* object, struct Vector3* prev_pos) {

    for (int i = 0; i < MAX_SWEPT_ITERATIONS; i += 1)
    {
        struct Vector3 offset;
        vector3Sub(object->position, prev_pos, &offset);
        struct Vector3 bounding_box_size;
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

#define SLEEP_THRESHOLD 0.0001f
#define SLEEP_THRESHOLD_SQ (SLEEP_THRESHOLD * SLEEP_THRESHOLD)
#define SLEEP_STEPS 20

/// @brief performs a physics step on all objects in the scene, updating their positions and velocities and performing collision detection
void collision_scene_step() {

    struct collision_scene_element* element;

    // Update the positions of the objects and update the AABB object tree
    for (int i = 0; i < g_scene.count; ++i) {
        element = &g_scene.elements[i];

        collision_scene_release_object_contacts(element->object);

        if (element->object->has_gravity && !element->object->is_sleeping) // don't apply gravity to sleeping objects
        {
            element->object->acceleration.y += GRAVITY_CONSTANT * element->object->gravity_scalar;
        }
        physics_object_update_velocity_verlet_simple(element->object);

        physics_object_recalculate_aabb(element->object);

        struct Vector3 displacement;
        vector3Sub(element->object->position, &element->object->_prev_step_pos, &displacement);
        AABBTree_moveNode(&g_scene.object_aabbtree, element->object->_aabb_tree_node_id, element->object->bounding_box, &displacement);

    }

    // Perform collision detection and resolution for both objects and the static mesh
    for (int i = 0; i < g_scene.count; ++i)
    {
        element = &g_scene.elements[i];
        if(element->object->is_sleeping){
            continue;
        }

        collision_scene_collide_phys_object(element);

    }

    // Update the sleep state of the objects so they can be put to sleep if they are not moving for a while
    for (int i = 0; i < g_scene.count; i++)
    {
        element = &g_scene.elements[i];

        if (g_scene.mesh_collider && element->object->collision_layers & COLLISION_LAYER_TANGIBLE)
        {
            collision_scene_collide_single(element->object, &element->object->_prev_step_pos);
        }

        physics_object_apply_constraints(element->object);
        struct Vector3 displacement_after_collision;
        vector3Sub(element->object->position, &element->object->_prev_step_pos, &displacement_after_collision);
        float displacement_dist = sqrtf(vector3MagSqrd(&displacement_after_collision));
        int rot_same = 1;

        element->object->_prev_step_pos = *element->object->position;
        if(element->object->rotation){
            rot_same = quatIsIdentical(element->object->rotation, &element->object->_prev_step_rot);
            element->object->_prev_step_rot = *element->object->rotation;
        }

        if (displacement_dist < SLEEP_THRESHOLD && rot_same)
        {
            element->object->_sleep_counter += 1;
            if (element->object->_sleep_counter > SLEEP_STEPS)
            {
                element->object->_sleep_counter = SLEEP_STEPS;
                element->object->is_sleeping = 1;
                
            }

            continue;
        }
        element->object->is_sleeping = 0;
        element->object->_sleep_counter = 0;

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
    if(g_scene.mesh_collider){ 
        DrawModelWires(g_scene.mesh_collider->raylib_mesh_model, (Raylib_Vector3){0, 0, 0}, SCENE_SCALE, YELLOW);   
    }
    for (int i = 0; i < g_scene.count; ++i) {
        struct collision_scene_element* element = &g_scene.elements[i];
        struct physics_object* object = element->object;

        struct Vector3 center_offset_rotated;

        DrawBoundingBox(
            (BoundingBox){
                (Raylib_Vector3){object->bounding_box.min.x * SCENE_SCALE, object->bounding_box.min.y * SCENE_SCALE, object->bounding_box.min.z * SCENE_SCALE},
                (Raylib_Vector3){object->bounding_box.max.x * SCENE_SCALE, object->bounding_box.max.y * SCENE_SCALE, object->bounding_box.max.z * SCENE_SCALE}},
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
            struct Vector3 local_axis = {0.0f, half_height, 0.0f};

            // Rotate the central axis by the given rotation to get its orientation in world space
            struct Vector3 world_axis;
            
            if (object->rotation)
                quatMultVector(object->rotation, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            struct Vector3 start;
            struct Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &center_offset_rotated, &end);
            vector3Add(&start, &center_offset_rotated, &start);

            DrawCapsuleWires(
                (Raylib_Vector3){start.x * SCENE_SCALE, start.y * SCENE_SCALE, start.z * SCENE_SCALE},
                (Raylib_Vector3){end.x * SCENE_SCALE, end.y * SCENE_SCALE, end.z * SCENE_SCALE},
                radius * SCENE_SCALE,
                4,
                2,
                RED
            );
        }
        else if(object->collision->shape_type == COLLISION_SHAPE_BOX){
            // Get capsule dimensions
            struct Vector3 half_size = object->collision->shape_data.box.half_size;
            struct Vector3 worldPos;

            Raylib_Mesh cubeMesh = GenMeshCube(half_size.x * 2, half_size.y * 2, half_size.z * 2);

            Raylib_Model cubeModel = LoadModelFromMesh(cubeMesh);

            Raylib_Quaternion q = {.x = object->rotation->x, .y = object->rotation->y, .z = object->rotation->z, .w = object->rotation->w};
            Matrix m = QuaternionToMatrix(q);
            cubeModel.transform = MatrixMultiply(m, cubeModel.transform);
            vector3Add(object->position, &center_offset_rotated, &worldPos);
            
            Raylib_Vector3 pos = {worldPos.x * SCENE_SCALE, worldPos.y * SCENE_SCALE, worldPos.z * SCENE_SCALE};

            DrawModelWires(cubeModel, pos, SCENE_SCALE, GREEN);
            UnloadModel(cubeModel);
        }        
        else if(object->collision->shape_type == COLLISION_SHAPE_SPHERE){
            float radius = object->collision->shape_data.sphere.radius;
            struct Vector3 worldPos;
            vector3Add(object->position, &center_offset_rotated, &worldPos);
            Raylib_Vector3 pos = {worldPos.x * SCENE_SCALE, worldPos.y * SCENE_SCALE, worldPos.z * SCENE_SCALE};
            DrawSphereWires(pos, radius * SCENE_SCALE, 5, 5, PINK);
        }
        else if(object->collision->shape_type == COLLISION_SHAPE_CYLINDER){
            // Get capsule dimensions
            float half_height = object->collision->shape_data.cylinder.half_height;
            float radius = object->collision->shape_data.cylinder.radius;

            // Define the capsule's central axis in local space
            struct Vector3 local_axis = {0.0f, half_height, 0.0f};

            // Rotate the central axis by the given rotation to get its orientation in world space
            struct Vector3 world_axis;
            
            if (object->rotation)
                quatMultVector(object->rotation, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            struct Vector3 start;
            struct Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &center_offset_rotated, &end);
            vector3Add(&start, &center_offset_rotated, &start);

            DrawCylinderWiresEx(
                (Raylib_Vector3){start.x * SCENE_SCALE, start.y * SCENE_SCALE, start.z * SCENE_SCALE},
                (Raylib_Vector3){end.x * SCENE_SCALE, end.y * SCENE_SCALE, end.z * SCENE_SCALE},
                radius * SCENE_SCALE,
                radius * SCENE_SCALE,
                10,
                BLUE
            );
        }
        else if(object->collision->shape_type == COLLISION_SHAPE_CONE){
            // Get capsule dimensions
            float half_height = object->collision->shape_data.cone.half_height;
            float radius = object->collision->shape_data.cone.radius;

            // Define the capsule's central axis in local space
            struct Vector3 local_axis = {0.0f, half_height, 0.0f};

            // Rotate the central axis by the given rotation to get its orientation in world space
            struct Vector3 world_axis;
            
            if (object->rotation)
                quatMultVector(object->rotation, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            struct Vector3 start;
            struct Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &center_offset_rotated, &end);
            vector3Add(&start, &center_offset_rotated, &start);

            DrawLine3D(
                (Raylib_Vector3){start.x * SCENE_SCALE, start.y * SCENE_SCALE, start.z * SCENE_SCALE},
                (Raylib_Vector3){end.x * SCENE_SCALE, end.y * SCENE_SCALE, end.z * SCENE_SCALE},
                VIOLET
            );
            // float angle = 0.0f;
            // struct Vector3 axis;
            // quatDecompose(object->rotation, &axis, &angle);
            // DrawCircle3D(
            //     (Raylib_Vector3){end.x * SCENE_SCALE, end.y * SCENE_SCALE, end.z * SCENE_SCALE},
            //     radius * SCENE_SCALE,
            //     (Raylib_Vector3){axis.x, axis.y, axis.z},
            //     angle,
            //     BLUE
            // );

        }
    }
}
#endif