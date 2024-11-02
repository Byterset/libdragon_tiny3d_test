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

    hash_map_init(&g_scene.entity_mapping, MIN_DYNAMIC_OBJECTS);
    AABBTree_create(&g_scene.object_aabbtree, MIN_DYNAMIC_OBJECTS);
    g_scene.elements = malloc(sizeof(struct collision_scene_element) * MIN_DYNAMIC_OBJECTS);
    g_scene.capacity = MIN_DYNAMIC_OBJECTS;
    g_scene.count = 0;
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

void collision_scene_add(struct dynamic_object* object) {
    if (g_scene.count >= g_scene.capacity) {
        g_scene.capacity *= 2;
        g_scene.elements = realloc(g_scene.elements, sizeof(struct collision_scene_element) * g_scene.capacity);
    }

    struct collision_scene_element* next = &g_scene.elements[g_scene.count];

    next->object = object;

    g_scene.count += 1;

    hash_map_set(&g_scene.entity_mapping, object->entity_id, object);
    object->aabb_tree_node = AABBTreeNode_createNode(&g_scene.object_aabbtree, object->bounding_box, object);
}


struct dynamic_object* collision_scene_find_object(entity_id id) {
    if (!id) {
        return 0;
    }

    return hash_map_get(&g_scene.entity_mapping, id);
}

void collision_scene_return_contacts(struct dynamic_object* object) {
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

void collision_scene_remove(struct dynamic_object* object) {
    bool has_found = false;

    for (int i = 0; i < g_scene.count; ++i) {
        if (object == g_scene.elements[i].object) {
            collision_scene_return_contacts(object);
            has_found = true;
        }

        if (has_found) {
            g_scene.elements[i] = g_scene.elements[i + 1];
        }
    }

    if (has_found) {
        g_scene.count -= 1;
    }

    hash_map_delete(&g_scene.entity_mapping, object->entity_id);
}

void collision_scene_use_static_collision(struct mesh_collider* collider) {
    g_scene.mesh_collider = collider;
}

void collision_scene_remove_static_collision(struct mesh_collider* collider) {
    if (collider == g_scene.mesh_collider) {
        g_scene.mesh_collider = NULL;
    }
}

struct collide_edge {
    uint16_t is_start_edge: 1;
    uint16_t object_index: 15;
    short x;
};

int collide_edge_compare(struct collide_edge a, struct collide_edge b) {
    if (a.x == b.x) {
        return b.is_start_edge - a.is_start_edge;
    }

    return a.x - b.x;
}

void collide_edge_sort(struct collide_edge* edges, struct collide_edge* tmp, int start, int end) {
    if (start + 1 >= end) {
        return;
    }

    int mid = (start + end) >> 1;

    collide_edge_sort(edges, tmp, start, mid);
    collide_edge_sort(edges, tmp, mid, end);

    int a = start;
    int b = mid;
    int output = start;

    while (a < mid || b < end) {
        if (b >= end || (a < mid && collide_edge_compare(edges[a], edges[b]) < 0)) {
            tmp[output] = edges[a];
            ++output;
            ++a;
        } else {
            tmp[output] = edges[b];
            ++output;
            ++b;
        }
    }

    for (int i = start; i < end; ++i) {
        edges[i] = tmp[i];
    }
}

void collision_scene_collide_dynamic_sweep_and_prune() {
    int edge_count = g_scene.count * 2;

    struct collide_edge collide_edges[edge_count];

    struct collide_edge* curr_edge = &collide_edges[0];

    for (int i = 0; i < g_scene.count; ++i) {
        struct collision_scene_element* element = &g_scene.elements[i];

        curr_edge->is_start_edge = 1;
        curr_edge->object_index = i;
        curr_edge->x = (short)(element->object->bounding_box.min.x * 32.0f);

        curr_edge += 1;

        curr_edge->is_start_edge = 0;
        curr_edge->object_index = i;
        curr_edge->x = (short)(element->object->bounding_box.max.x * 32.0f);

        curr_edge += 1;
    }

    struct collide_edge tmp[edge_count];
    collide_edge_sort(collide_edges, tmp, 0, edge_count);

    uint16_t active_objects[g_scene.count];
    int active_object_count = 0;

    for (int edge_index = 0; edge_index < edge_count; edge_index += 1) {
        struct collide_edge edge = collide_edges[edge_index];

        if (edge.is_start_edge) {
            struct dynamic_object* a = g_scene.elements[edge.object_index].object;

            for (int active_index = 0; active_index < active_object_count; active_index += 1) {
                struct dynamic_object* b = g_scene.elements[active_objects[active_index]].object;

                if (AABBHasOverlap(&a->bounding_box, &b->bounding_box)) {
                    collide_object_to_object(a, b);
                }
            }

            active_objects[active_object_count] = edge.object_index;
            active_object_count += 1;
            
        } else {
            int found_index = -1;

            for (int active_index = 0; active_index < active_object_count; active_index += 1) {
                if (active_objects[active_index] == edge.object_index) {
                    found_index = active_index;
                    break;
                }
            }

            assert(found_index != -1);

            // remove item by replacing it with the last one
            active_objects[found_index] = active_objects[active_object_count - 1];
            active_object_count -= 1;
        }
    }
}

void collision_scene_collide_dynamic_aabbtree(struct collision_scene_element* element) {

    int result_count = 0;
    int aabbCheck_count = 0;
    int max_results = 20;
    NodeProxy results[max_results];

    AABBTree_queryBounds(&g_scene.object_aabbtree, &element->object->bounding_box, results, &result_count, &aabbCheck_count, max_results);
    for (size_t j = 0; j < result_count; j++)
    {
        struct dynamic_object *other = (struct dynamic_object *)AABBTreeNode_getData(&g_scene.object_aabbtree, results[j]);
        // skip narrow phase if there is no dynamic object associated with the result node
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

void collision_scene_collide_single(struct dynamic_object* object, struct Vector3* prev_pos) {
    for (int i = 0; i < MAX_SWEPT_ITERATIONS; i += 1) {
        struct Vector3 offset;
        vector3Sub(object->position, prev_pos, &offset);
        struct Vector3 bbSize;
        vector3Sub(&object->bounding_box.max, &object->bounding_box.min, &bbSize);
        vector3Scale(&bbSize, &bbSize, 0.5f);

        if (fabs(offset.x) > bbSize.x ||
            fabs(offset.y) > bbSize.y ||
            fabs(offset.z) > bbSize.z
        ) {
            if (!collide_object_to_mesh_swept(object, g_scene.mesh_collider, prev_pos)) {
                return;
            }
        } else {
            collide_object_to_mesh(object, g_scene.mesh_collider);
            return;
        }
    }

    // too many swept iterations
    // to prevent tunneling just move 
    // the object back to the previous known
    // valid location
    *object->position = *prev_pos;
}

void collision_scene_collide() {
    // struct Vector3 prev_pos[g_scene.count];

    struct collision_scene_element* element;

    for (int i = 0; i < g_scene.count; ++i) {
        element = &g_scene.elements[i];
        // prev_pos[i] = *element->object->position;

        collision_scene_return_contacts(element->object);

        dynamic_object_update(element->object);


        dynamic_object_recalc_bb(element->object);

        struct Vector3 displacement;
        vector3Sub(element->object->position, &element->object->prev_position, &displacement);
        AABBTree_moveNode(&g_scene.object_aabbtree, element->object->aabb_tree_node, element->object->bounding_box, &displacement);

        if (g_scene.mesh_collider)
        {

            collision_scene_collide_single(element->object, &element->object->prev_position);

            element->object->is_out_of_bounds = mesh_index_is_contained(&g_scene.mesh_collider->index, element->object->position);
        }
    }

    // collision_scene_collide_dynamic_sweep_and_prune();
    for (int i = 0; i < g_scene.count; ++i)
    {
        element = &g_scene.elements[i];

        collision_scene_collide_dynamic_aabbtree(element);
        
    }
    for (int i = 0; i < g_scene.count; ++i)
    {
        element = &g_scene.elements[i];

        dynamic_object_apply_constraints(element->object);
    }
}

struct contact* collision_scene_new_contact() {
    if (!g_scene.next_free_contact) {
        return NULL;
    }

    struct contact* result = g_scene.next_free_contact;
    g_scene.next_free_contact = result->next;
    return result;
}

struct positioned_shape {
    struct dynamic_object_type* type;
    struct Vector3* center;
};

void positioned_shape_mink_sum(void* data, struct Vector3* direction, struct Vector3* output) {
    struct positioned_shape* shape = (struct positioned_shape*)data;
    shape->type->minkowski_sum(&shape->type->data, direction, output);
    vector3Add(output, shape->center, output);
}

void collision_scene_query(struct dynamic_object_type* shape, struct Vector3* center, int collision_layers, collision_scene_query_callback callback, void* callback_data) {
    struct AABB bounding_box;
    shape->bounding_box(&shape->data, NULL, &bounding_box);
    vector3Add(&bounding_box.min, center, &bounding_box.min);
    vector3Add(&bounding_box.max, center, &bounding_box.max);

    struct positioned_shape positioned_shape;

    positioned_shape.type = shape;
    positioned_shape.center = center;

    for (int i = 0; i < g_scene.count; ++i) {
        struct collision_scene_element* element = &g_scene.elements[i];

        if (!(element->object->collision_layers & collision_layers)) {
            continue;
        }

        if (!AABBHasOverlap(&bounding_box, &element->object->bounding_box)) {
            continue;
        }

        struct Simplex simplex;

        struct Vector3 first_dir;
        vector3Sub(center, element->object->position, &first_dir);

        if (!gjkCheckForOverlap(&simplex, &positioned_shape, positioned_shape_mink_sum, element->object, dynamic_object_minkowski_sum, &first_dir)) {
            continue;;
        }

        callback(callback_data, element->object);
    }
}

void collision_scene_render_debug_raylib(){
    for (int i = 0; i < g_scene.count; ++i) {
        struct collision_scene_element* element = &g_scene.elements[i];
        struct dynamic_object* object = element->object;

        struct Vector3 world_center;

        if (object->rotation_quat)
            quatMultVector(object->rotation_quat, &object->center, &world_center);
        else
            vector3Copy(&object->center, &world_center);

        if(object->type->type == DYNAMIC_OBJECT_TYPE_CAPSULE){
            // Get capsule dimensions
            float half_height = object->type->data.capsule.inner_half_height;
            float radius = object->type->data.capsule.radius;

            // Define the capsule's central axis in local space
            struct Vector3 local_axis = {0.0f, half_height, 0.0f};

            // Rotate the central axis by the given rotation to get its orientation in world space
            struct Vector3 world_axis;
            
            if (object->rotation_quat)
                quatMultVector(object->rotation_quat, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            struct Vector3 start;
            struct Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &world_center, &end);
            vector3Add(&start, &world_center, &start);

            DrawCapsuleWires(
                (Raylib_Vector3){start.x * SCENE_SCALE, start.y * SCENE_SCALE, start.z * SCENE_SCALE},
                (Raylib_Vector3){end.x * SCENE_SCALE, end.y * SCENE_SCALE, end.z * SCENE_SCALE},
                radius * SCENE_SCALE,
                4,
                2,
                RED
            );
        }
        else if(object->type->type == DYNAMIC_OBJECT_TYPE_BOX){
            // Get capsule dimensions
            struct Vector3 half_size = object->type->data.box.half_size;
            struct Vector3 worldPos;

            Raylib_Mesh cubeMesh = GenMeshCube(half_size.x * 2, half_size.y * 2, half_size.z * 2);

            Raylib_Model cubeModel = LoadModelFromMesh(cubeMesh);

            Raylib_Quaternion q = {.x = object->rotation_quat->x, .y = object->rotation_quat->y, .z = object->rotation_quat->z, .w = object->rotation_quat->w};
            Matrix m = QuaternionToMatrix(q);
            cubeModel.transform = MatrixMultiply(m, cubeModel.transform); // something like that
            vector3Add(object->position, &world_center, &worldPos);
            
            Raylib_Vector3 pos = {worldPos.x * SCENE_SCALE, worldPos.y * SCENE_SCALE, worldPos.z * SCENE_SCALE};

            DrawModelWires(cubeModel, pos, SCENE_SCALE, GREEN);
            UnloadModel(cubeModel);
        }        
        else if(object->type->type == DYNAMIC_OBJECT_TYPE_SPHERE){
            float radius = object->type->data.sphere.radius;
            struct Vector3 worldPos;
            vector3Add(object->position, &world_center, &worldPos);
            Raylib_Vector3 pos = {worldPos.x * SCENE_SCALE, worldPos.y * SCENE_SCALE, worldPos.z * SCENE_SCALE};
            DrawSphereWires(pos, radius * SCENE_SCALE, 3, 3, YELLOW);
        }
        else if(object->type->type == DYNAMIC_OBJECT_TYPE_CYLINDER){
            // Get capsule dimensions
            float half_height = object->type->data.cylinder.half_height;
            float radius = object->type->data.cylinder.radius;

            // Define the capsule's central axis in local space
            struct Vector3 local_axis = {0.0f, half_height, 0.0f};

            // Rotate the central axis by the given rotation to get its orientation in world space
            struct Vector3 world_axis;
            
            if (object->rotation_quat)
                quatMultVector(object->rotation_quat, &local_axis, &world_axis);
            else
                vector3Copy(&local_axis, &world_axis);

            struct Vector3 start;
            struct Vector3 end;
            vector3Copy(&world_axis, &start);
            vector3Copy(&world_axis, &end);
            vector3Scale(&end, &end, -1.0f);
            vector3Add(&start, object->position, &start);
            vector3Add(&end, object->position, &end);
            vector3Add(&end, &world_center, &end);
            vector3Add(&start, &world_center, &start);

            DrawCylinderWiresEx(
                (Raylib_Vector3){start.x * SCENE_SCALE, start.y * SCENE_SCALE, start.z * SCENE_SCALE},
                (Raylib_Vector3){end.x * SCENE_SCALE, end.y * SCENE_SCALE, end.z * SCENE_SCALE},
                radius * SCENE_SCALE,
                radius * SCENE_SCALE,
                5,
                BLUE
            );
        }
    }
}