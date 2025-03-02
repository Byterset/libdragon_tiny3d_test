# n64 Game Engine
A In-Progress 3D Game & Physics Engine based on [Libdragon](https://github.com/DragonMinded/libdragon) and [Tiny3D](https://github.com/HailToDodongo/tiny3d)

# Features:
- Game Object based Entity System
    - Objects can register tasks for rendering, physics/collision, or update callbacks
    - Easy creation of new Entities and Behaviours
- Unity Engine-like Loop Execution Order with framerate independent physics model
    - Fixed Update (registerable callbacks)
    - Physics Simulation Step (simplified verlet integration)
    - Detection & Resolution of Collisions
    - Update (registerable callbacks)
    - Batch based Rendering
- Dynamic Physics System featuring BVH accelerated static level collision handling and dynamic Object collision
    - Built-In variety of collider shapes (Sphere, AABB, OBB, Capsule, Cylinder, Cone)
    - Collider Shapes are easily expandable by just defining a GJK Support function for the shape
    - All collider shapes are inherently compatible with each other
    - Colliders support full optional Quaternion based rotation
    - Trigger Colliders are supported
    - Raycasting
- Export of Collision Meshes from Blender in simple expandable binary format
- 2D fast approximated Skyboxes


- Features due to implementation with T3D & Libdragon
    - 3D Model (.gltf Format) Support with Fast64 Materials
    - Skeletal Animation Support
    - Particle Systems
    - Comprehensive 3D API
    - Low level api access rdpq
    - File System Support
    - Support of multiple modern audio formats
    - Video Support
    - (for full Feature list visit the repos above, those people are crazy good and helpful!)


# How to build

> Libdragon and Tiny3D are included as submodules in this repository and pointing to commit versions that are known to be compatible. This Repository relies on Libdragon Preview Features.

1) Init git Submodules
2) Install Libdragon according to the [installation guide](https://github.com/DragonMinded/libdragon/wiki/Installing-libdragon) (make sure to also read the README if you are not familiar with libdragon already)
3) Install Tiny3D, should be as easy as running the `build.sh` in the tiny3d submodule
4) run the `make` command from the project root

This should build the code and convert the assets as well as assemble the filesystem and the final rom. If something is not working as expected try building the libdragon or tiny3d examples first according to the official instructions.