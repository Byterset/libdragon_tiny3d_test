#include "ray_shape_intersection.h"
#include "../../math/mathf.h"
#include <math.h>
#include <float.h>

/**
 * @brief Transforms a ray from world space to local space
 * 
 * @param ray The ray in world space
 * @param center The center of the object
 * @param rotation The rotation of the object
 * @param local_ray The ray in local space (output)
 */
static void transform_ray_to_local_space(
    raycast* ray, 
    Vector3* center, 
    Quaternion* rotation, 
    raycast* local_ray
) {
    // Copy the ray
    *local_ray = *ray;
    
    // Translate ray origin relative to object center
    Vector3 translated_origin;
    vector3Sub(&ray->origin, center, &translated_origin);
    
    // If there's a rotation, apply inverse rotation to both origin and direction
    if (rotation) {
        Quaternion inverse_rotation;
        quatConjugate(rotation, &inverse_rotation);
        
        // Rotate origin
        quatMultVector(&inverse_rotation, &translated_origin, &local_ray->origin);
        
        // Rotate direction
        quatMultVector(&inverse_rotation, &ray->dir, &local_ray->dir);
    } else {
        local_ray->origin = translated_origin;
    }
    
    // Recalculate inverse direction
    local_ray->_invDir.x = safeInvert(local_ray->dir.x);
    local_ray->_invDir.y = safeInvert(local_ray->dir.y);
    local_ray->_invDir.z = safeInvert(local_ray->dir.z);
}

/**
 * @brief Transforms a point from local space to world space
 * 
 * @param local_point The point in local space
 * @param center The center of the object
 * @param rotation The rotation of the object
 * @param world_point The point in world space (output)
 */
static void transform_point_to_world_space(
    Vector3* local_point, 
    Vector3* center, 
    Quaternion* rotation, 
    Vector3* world_point
) {
    if (rotation) {
        // Apply rotation
        quatMultVector(rotation, local_point, world_point);
        
        // Apply translation
        vector3Add(world_point, center, world_point);
    } else {
        // Just apply translation
        vector3Add(local_point, center, world_point);
    }
}

/**
 * @brief Transforms a normal from local space to world space
 * 
 * @param local_normal The normal in local space
 * @param rotation The rotation of the object
 * @param world_normal The normal in world space (output)
 */
static void transform_normal_to_world_space(
    Vector3* local_normal, 
    Quaternion* rotation, 
    Vector3* world_normal
) {
    if (rotation) {
        // Apply rotation to normal
        quatMultVector(rotation, local_normal, world_normal);
    } else {
        *world_normal = *local_normal;
    }
}

bool ray_sphere_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    raycast_hit* hit,
    entity_id entity_id
) {
    // Calculate vector from ray origin to sphere center
    Vector3 oc;
    vector3Sub(&ray->origin, center, &oc);
    
    // Calculate quadratic coefficients
    float a = vector3Dot(&ray->dir, &ray->dir);  // Should be 1.0 if ray direction is normalized
    float b = 2.0f * vector3Dot(&oc, &ray->dir);
    float c = vector3Dot(&oc, &oc) - radius * radius;
    
    // Calculate discriminant
    float discriminant = b * b - 4.0f * a * c;
    
    if (discriminant < 0.0f) {
        // No intersection
        return false;
    }
    
    // Calculate the two intersection points
    float sqrt_discriminant = sqrtf(discriminant);
    float t1 = (-b - sqrt_discriminant) / (2.0f * a);
    float t2 = (-b + sqrt_discriminant) / (2.0f * a);
    
    // Find the nearest intersection point that is within the ray's range
    float t = t1;
    if (t < 0.0f) {
        t = t2;  // If t1 is behind the ray, try t2
        if (t < 0.0f) {
            return false;  // Both intersections are behind the ray
        }
    }
    
    // Check if the intersection is within the ray's max distance
    if (t > ray->maxDistance) {
        return false;
    }
    
    // Calculate the intersection point
    hit->distance = t;
    vector3AddScaled(&ray->origin, &ray->dir, t, &hit->point);
    
    // Calculate the normal at the intersection point (points outward from sphere center)
    Vector3 normal;
    vector3Sub(&hit->point, center, &normal);
    vector3Normalize(&normal, &hit->normal);
    
    // Set the entity ID
    hit->hit_entity_id = entity_id;
    
    return true;
}

bool ray_box_intersection(
    raycast* ray, 
    Vector3* center, 
    Vector3* half_size, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
) {
    // Transform ray to local space of the box
    raycast local_ray;
    transform_ray_to_local_space(ray, center, rotation, &local_ray);
    
    // In local space, the box is axis-aligned with min = -half_size and max = half_size
    AABB local_box = {
        .min = { {-half_size->x, -half_size->y, -half_size->z} },
        .max = { {half_size->x, half_size->y, half_size->z} }
    };
    
    // Check for intersection with the AABB in local space
    float tmin = -INFINITY;
    float tmax = INFINITY;
    int hit_face = -1;  // 0=x-, 1=x+, 2=y-, 3=y+, 4=z-, 5=z+
    
    // Check intersection with each pair of planes
    for (int i = 0; i < 3; i++) {
        if (fabsf(local_ray.dir.data[i]) < EPSILON) {
            // Ray is parallel to the slab, check if ray origin is within the slab
            if (local_ray.origin.data[i] < local_box.min.data[i] || 
                local_ray.origin.data[i] > local_box.max.data[i]) {
                return false;
            }
        } else {
            // Compute intersection with the two planes
            float ood = 1.0f / local_ray.dir.data[i];
            float t1 = (local_box.min.data[i] - local_ray.origin.data[i]) * ood;
            float t2 = (local_box.max.data[i] - local_ray.origin.data[i]) * ood;
            
            // Ensure t1 <= t2
            if (t1 > t2) {
                float temp = t1;
                t1 = t2;
                t2 = temp;
                
                // Adjust hit face index
                if (t1 > tmin) hit_face = i * 2 + 1;
            } else {
                // Adjust hit face index
                if (t1 > tmin) hit_face = i * 2;
            }
            
            // Update tmin and tmax
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            
            // Exit early if there's no intersection
            if (tmin > tmax) return false;
        }
    }
    
    // Check if intersection is within ray bounds
    if (tmin < 0.0f) {
        if (tmax < 0.0f || tmax > local_ray.maxDistance) {
            return false;
        }
        // We're inside the box, use tmax and adjust hit face
        tmin = tmax;
        hit_face = (hit_face & ~1) | 1;  // Flip to the opposite face
    } else if (tmin > local_ray.maxDistance) {
        return false;
    }
    
    // Calculate local hit point
    Vector3 local_hit_point;
    vector3AddScaled(&local_ray.origin, &local_ray.dir, tmin, &local_hit_point);
    
    // Calculate local normal based on hit face
    Vector3 local_normal = {{0.0f, 0.0f, 0.0f}};
    switch (hit_face) {
        case 0: local_normal.x = -1.0f; break;  // -X face
        case 1: local_normal.x = 1.0f; break;   // +X face
        case 2: local_normal.y = -1.0f; break;  // -Y face
        case 3: local_normal.y = 1.0f; break;   // +Y face
        case 4: local_normal.z = -1.0f; break;  // -Z face
        case 5: local_normal.z = 1.0f; break;   // +Z face
    }
    
    // Transform hit point and normal to world space
    transform_point_to_world_space(&local_hit_point, center, rotation, &hit->point);
    transform_normal_to_world_space(&local_normal, rotation, &hit->normal);
    
    // Set hit data
    hit->distance = tmin;
    hit->hit_entity_id = entity_id;
    
    return true;
}

bool ray_capsule_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    float half_height, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
) {
    // Transform ray to local space of the capsule
    raycast local_ray;
    transform_ray_to_local_space(ray, center, rotation, &local_ray);
    
    // In local space, the capsule is aligned with the Y axis
    // The capsule consists of a cylinder and two hemispheres
    
    // First, check if the ray intersects the infinite cylinder
    Vector3 ray_origin_xz = {{local_ray.origin.x, 0.0f, local_ray.origin.z}};
    Vector3 ray_dir_xz = {{local_ray.dir.x, 0.0f, local_ray.dir.z}};
    
    float a = vector3Dot(&ray_dir_xz, &ray_dir_xz);
    float b = 2.0f * vector3Dot(&ray_origin_xz, &ray_dir_xz);
    float c = vector3Dot(&ray_origin_xz, &ray_origin_xz) - radius * radius;
    
    float discriminant = b * b - 4.0f * a * c;
    
    float cylinder_t = FLT_MAX;
    Vector3 cylinder_normal;
    
    if (discriminant >= 0.0f && a > EPSILON) {
        // Ray intersects the infinite cylinder
        float sqrt_discriminant = sqrtf(discriminant);
        float t1 = (-b - sqrt_discriminant) / (2.0f * a);
        float t2 = (-b + sqrt_discriminant) / (2.0f * a);
        
        // Find the nearest valid intersection
        float t = t1 > 0.0f ? t1 : t2;
        
        if (t > 0.0f && t < local_ray.maxDistance) {
            // Check if the intersection is within the cylinder part (not the hemispheres)
            Vector3 hit_point;
            vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &hit_point);
            
            if (fabsf(hit_point.y) <= half_height) {
                cylinder_t = t;
                
                // Calculate normal at cylinder intersection (points outward from cylinder axis)
                cylinder_normal = (Vector3){{hit_point.x, 0.0f, hit_point.z}};
                vector3Normalize(&cylinder_normal, &cylinder_normal);
            }
        }
    }
    
    // Check intersection with the two hemispheres (sphere caps)
    Vector3 top_sphere_center = {{0.0f, half_height, 0.0f}};
    Vector3 bottom_sphere_center = {{0.0f, -half_height, 0.0f}};
    
    raycast_hit top_hit, bottom_hit;
    bool hit_top = ray_sphere_intersection(&local_ray, &top_sphere_center, radius, &top_hit, 0);
    bool hit_bottom = ray_sphere_intersection(&local_ray, &bottom_sphere_center, radius, &bottom_hit, 0);
    
    // Find the closest valid intersection
    float sphere_t = FLT_MAX;
    Vector3 sphere_normal;
    
    if (hit_top && top_hit.distance < sphere_t) {
        // Check if the intersection is within the hemisphere (not the cylinder part)
        Vector3 hit_to_center;
        vector3Sub(&top_hit.point, &top_sphere_center, &hit_to_center);
        if (hit_to_center.y > 0.0f) {
            sphere_t = top_hit.distance;
            sphere_normal = top_hit.normal;
        }
    }
    
    if (hit_bottom && bottom_hit.distance < sphere_t) {
        // Check if the intersection is within the hemisphere (not the cylinder part)
        Vector3 hit_to_center;
        vector3Sub(&bottom_hit.point, &bottom_sphere_center, &hit_to_center);
        if (hit_to_center.y < 0.0f) {
            sphere_t = bottom_hit.distance;
            sphere_normal = bottom_hit.normal;
        }
    }
    
    // Determine the closest intersection between cylinder and hemispheres
    if (cylinder_t == FLT_MAX && sphere_t == FLT_MAX) {
        return false;  // No intersection
    }
    
    Vector3 local_hit_point;
    Vector3 local_normal;
    float t;
    
    if (cylinder_t <= sphere_t) {
        t = cylinder_t;
        vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &local_hit_point);
        local_normal = cylinder_normal;
    } else {
        t = sphere_t;
        vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &local_hit_point);
        local_normal = sphere_normal;
    }
    
    // Transform hit point and normal to world space
    transform_point_to_world_space(&local_hit_point, center, rotation, &hit->point);
    transform_normal_to_world_space(&local_normal, rotation, &hit->normal);
    
    // Set hit data
    hit->distance = t;
    hit->hit_entity_id = entity_id;
    
    return true;
}

bool ray_cylinder_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    float half_height, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
) {
    // Transform ray to local space of the cylinder
    raycast local_ray;
    transform_ray_to_local_space(ray, center, rotation, &local_ray);
    
    // In local space, the cylinder is aligned with the Y axis
    
    // Check intersection with the infinite cylinder (side)
    Vector3 ray_origin_xz = {{local_ray.origin.x, 0.0f, local_ray.origin.z}};
    Vector3 ray_dir_xz = {{local_ray.dir.x, 0.0f, local_ray.dir.z}};
    
    float a = vector3Dot(&ray_dir_xz, &ray_dir_xz);
    float b = 2.0f * vector3Dot(&ray_origin_xz, &ray_dir_xz);
    float c = vector3Dot(&ray_origin_xz, &ray_origin_xz) - radius * radius;
    
    float discriminant = b * b - 4.0f * a * c;
    
    float t_side = FLT_MAX;
    Vector3 side_normal;
    
    if (discriminant >= 0.0f && a > EPSILON) {
        // Ray intersects the infinite cylinder
        float sqrt_discriminant = sqrtf(discriminant);
        float t1 = (-b - sqrt_discriminant) / (2.0f * a);
        float t2 = (-b + sqrt_discriminant) / (2.0f * a);
        
        // Check both intersection points
        for (int i = 0; i < 2; i++) {
            float t = (i == 0) ? t1 : t2;
            
            if (t > 0.0f && t < local_ray.maxDistance && t < t_side) {
                // Check if the intersection is within the finite cylinder
                Vector3 hit_point;
                vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &hit_point);
                
                if (fabsf(hit_point.y) <= half_height) {
                    t_side = t;
                    
                    // Calculate normal at cylinder intersection (points outward from cylinder axis)
                    side_normal = (Vector3){{hit_point.x, 0.0f, hit_point.z}};
                    vector3Normalize(&side_normal, &side_normal);
                }
            }
        }
    }
    
    // Check intersection with the two end caps (discs)
    float t_cap = FLT_MAX;
    Vector3 cap_normal;
    
    for (int i = 0; i < 2; i++) {
        float cap_y = (i == 0) ? half_height : -half_height;
        Vector3 cap_normal_local = {{0.0f, (i == 0) ? 1.0f : -1.0f, 0.0f}};
        
        // Check if ray is parallel to the cap
        if (fabsf(local_ray.dir.y) < EPSILON) {
            continue;
        }
        
        // Calculate intersection with the cap plane
        float t = (cap_y - local_ray.origin.y) / local_ray.dir.y;
        
        if (t > 0.0f && t < local_ray.maxDistance && t < t_cap) {
            // Check if the intersection is within the disc
            Vector3 hit_point;
            vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &hit_point);
            
            float dist_sq = hit_point.x * hit_point.x + hit_point.z * hit_point.z;
            
            if (dist_sq <= radius * radius) {
                t_cap = t;
                cap_normal = cap_normal_local;
            }
        }
    }
    
    // Determine the closest intersection between side and caps
    if (t_side == FLT_MAX && t_cap == FLT_MAX) {
        return false;  // No intersection
    }
    
    Vector3 local_hit_point;
    Vector3 local_normal;
    float t;
    
    if (t_side <= t_cap) {
        t = t_side;
        local_normal = side_normal;
    } else {
        t = t_cap;
        local_normal = cap_normal;
    }
    
    // Calculate the hit point
    vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &local_hit_point);
    
    // Transform hit point and normal to world space
    transform_point_to_world_space(&local_hit_point, center, rotation, &hit->point);
    transform_normal_to_world_space(&local_normal, rotation, &hit->normal);
    
    // Set hit data
    hit->distance = t;
    hit->hit_entity_id = entity_id;
    
    return true;
}

bool ray_cone_intersection(
    raycast* ray, 
    Vector3* center, 
    float radius, 
    float half_height, 
    Quaternion* rotation, 
    raycast_hit* hit,
    entity_id entity_id
) {
    // Transform ray to local space of the cone
    raycast local_ray;
    transform_ray_to_local_space(ray, center, rotation, &local_ray);
    
    // In local space, the cone is aligned with the Y axis
    // The apex is at (0, half_height, 0) and the base is at y = -half_height
    
    // Calculate cone parameters
    float h = 2.0f * half_height;  // Total height
    float tan_theta_sq = (radius * radius) / (h * h);  // Squared tangent of cone angle
    
    // Apex of the cone
    Vector3 apex = {{0.0f, half_height, 0.0f}};
    
    // Check intersection with the cone side
    Vector3 co;
    vector3Sub(&local_ray.origin, &apex, &co);
    
    float a = local_ray.dir.x * local_ray.dir.x + local_ray.dir.z * local_ray.dir.z - 
              tan_theta_sq * local_ray.dir.y * local_ray.dir.y;
    float b = 2.0f * (co.x * local_ray.dir.x + co.z * local_ray.dir.z - 
              tan_theta_sq * co.y * local_ray.dir.y);
    float c = co.x * co.x + co.z * co.z - tan_theta_sq * co.y * co.y;
    
    float discriminant = b * b - 4.0f * a * c;
    
    float t_side = FLT_MAX;
    Vector3 side_normal;
    
    if (fabsf(a) > EPSILON && discriminant >= 0.0f) {
        // Ray intersects the infinite cone
        float sqrt_discriminant = sqrtf(discriminant);
        float t1 = (-b - sqrt_discriminant) / (2.0f * a);
        float t2 = (-b + sqrt_discriminant) / (2.0f * a);
        
        // Check both intersection points
        for (int i = 0; i < 2; i++) {
            float t = (i == 0) ? t1 : t2;
            
            if (t > 0.0f && t < local_ray.maxDistance && t < t_side) {
                // Check if the intersection is within the finite cone
                Vector3 hit_point;
                vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &hit_point);
                
                if (hit_point.y <= half_height && hit_point.y >= -half_height) {
                    t_side = t;
                    
                    // Calculate normal at cone intersection
                    // The normal is perpendicular to both the cone axis and the vector from axis to hit point
                    Vector3 axis_to_hit = {{hit_point.x, 0.0f, hit_point.z}};
                    float axis_to_hit_len = sqrtf(vector3MagSqrd(&axis_to_hit));
                    
                    if (axis_to_hit_len > EPSILON) {
                        // Normalize the axis_to_hit vector
                        vector3Scale(&axis_to_hit, &axis_to_hit, 1.0f / axis_to_hit_len);
                        
                        // Calculate the angle between the cone axis and the normal
                        float cone_angle = atanf(radius / h);
                        
                        // Construct the normal
                        side_normal.x = axis_to_hit.x;
                        side_normal.y = sinf(cone_angle);
                        side_normal.z = axis_to_hit.z;
                        vector3Normalize(&side_normal, &side_normal);
                    } else {
                        // Hit point is on the cone axis, use a default normal
                        side_normal = (Vector3){{0.0f, 1.0f, 0.0f}};
                    }
                }
            }
        }
    }
    
    // Check intersection with the base (disc)
    float t_base = FLT_MAX;
    Vector3 base_normal = {{0.0f, -1.0f, 0.0f}};
    
    // Check if ray is not parallel to the base
    if (fabsf(local_ray.dir.y) > EPSILON) {
        // Calculate intersection with the base plane
        float t = (-half_height - local_ray.origin.y) / local_ray.dir.y;
        
        if (t > 0.0f && t < local_ray.maxDistance && t < t_base) {
            // Check if the intersection is within the base disc
            Vector3 hit_point;
            vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &hit_point);
            
            float dist_sq = hit_point.x * hit_point.x + hit_point.z * hit_point.z;
            
            if (dist_sq <= radius * radius) {
                t_base = t;
            }
        }
    }
    
    // Determine the closest intersection between side and base
    if (t_side == FLT_MAX && t_base == FLT_MAX) {
        return false;  // No intersection
    }
    
    Vector3 local_hit_point;
    Vector3 local_normal;
    float t;
    
    if (t_side <= t_base) {
        t = t_side;
        local_normal = side_normal;
    } else {
        t = t_base;
        local_normal = base_normal;
    }
    
    // Calculate the hit point
    vector3AddScaled(&local_ray.origin, &local_ray.dir, t, &local_hit_point);
    
    // Transform hit point and normal to world space
    transform_point_to_world_space(&local_hit_point, center, rotation, &hit->point);
    transform_normal_to_world_space(&local_normal, rotation, &hit->normal);
    
    // Set hit data
    hit->distance = t;
    hit->hit_entity_id = entity_id;
    
    return true;
}

bool ray_physics_object_intersection(
    raycast* ray, 
    struct physics_object* object, 
    raycast_hit* hit
) {
    // Get the object's collision data
    struct physics_object_collision_data* collision = object->collision;
    
    // Calculate the world center of the collider
    Vector3 collider_center;
    vector3Add(object->position, &object->center_offset, &collider_center);
    
    // Dispatch to the appropriate shape-specific intersection function
    switch (collision->shape_type) {
        case COLLISION_SHAPE_SPHERE:
            return ray_sphere_intersection(
                ray, 
                &collider_center, 
                collision->shape_data.sphere.radius, 
                hit,
                object->entity_id
            );
            
        case COLLISION_SHAPE_BOX:
            return ray_box_intersection(
                ray, 
                &collider_center, 
                &collision->shape_data.box.half_size, 
                object->rotation, 
                hit,
                object->entity_id
            );
            
        case COLLISION_SHAPE_CAPSULE:
            return ray_capsule_intersection(
                ray, 
                &collider_center, 
                collision->shape_data.capsule.radius, 
                collision->shape_data.capsule.inner_half_height, 
                object->rotation, 
                hit,
                object->entity_id
            );
            
        case COLLISION_SHAPE_CYLINDER:
            return ray_cylinder_intersection(
                ray, 
                &collider_center, 
                collision->shape_data.cylinder.radius, 
                collision->shape_data.cylinder.half_height, 
                object->rotation, 
                hit,
                object->entity_id
            );
            
        case COLLISION_SHAPE_CONE:
            return ray_cone_intersection(
                ray, 
                &collider_center, 
                collision->shape_data.cone.radius, 
                collision->shape_data.cone.half_height, 
                object->rotation, 
                hit,
                object->entity_id
            );
            
        case COLLISION_SHAPE_SWEEP:
            // For sweep shapes, we could implement a specialized ray-sweep intersection
            // For now, we'll just return false as it's not implemented yet
            return false;
            
        default:
            // Unknown shape type
            return false;
    }
}
