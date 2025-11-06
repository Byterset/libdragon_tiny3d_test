import bpy
import struct
import os

# Scene file format constants
SCENE_FILE_MAGIC = int.from_bytes(b'SCNE', 'big')
SCENE_FILE_VERSION = 1

SCENE_CHUNK_TYPE_HEADER = int.from_bytes(b'HDR ', 'big')
SCENE_CHUNK_TYPE_OBJECTS = int.from_bytes(b'OBJS', 'big')
SCENE_CHUNK_TYPE_COLLISION = int.from_bytes(b'COLL', 'big')

def write_header_chunk(file):
    # Write chunk header
    file.write(struct.pack('>I', SCENE_CHUNK_TYPE_HEADER))
    file.write(struct.pack('>I', 8))  # Chunk size

    # Write chunk data
    file.write(struct.pack('>I', SCENE_FILE_MAGIC))
    file.write(struct.pack('>I', SCENE_FILE_VERSION))

def write_objects_chunk(file, context):
    objects = []
    for obj in bpy.context.scene.objects:
        if "type" in obj:
            objects.append(obj)

    # Write chunk header
    file.write(struct.pack('>I', SCENE_CHUNK_TYPE_OBJECTS))
    chunk_size = 4 + len(objects) * 44 # 4 for num_objects, 44 for each object entry (32 for name, 12 for pos)
    file.write(struct.pack('>I', chunk_size))

    # Write chunk data
    file.write(struct.pack('>I', len(objects)))

    for obj in objects:
        type_name = obj["type"].ljust(32, '\\0').encode('utf-8')
        pos = obj.location
        rot = obj.rotation_quaternion

        file.write(type_name)
        file.write(struct.pack('>fff', pos.x, pos.y, pos.z))
        file.write(struct.pack('>ffff', rot.x, rot.y, rot.z, rot.w))


def write_collision_chunk(file, context):
    collision_object = None
    for obj in bpy.context.scene.objects:
        if obj.get("is_collision_mesh"):
            collision_object = obj
            break
    
    if collision_object and "collision_path" in collision_object:
        collision_path = collision_object["collision_path"]
        
        # Write chunk header
        file.write(struct.pack('>I', SCENE_CHUNK_TYPE_COLLISION))
        file.write(struct.pack('>I', len(collision_path)))

        # Write chunk data
        file.write(collision_path.encode('utf-8'))


def export_scene(context, filepath):
    with open(filepath, 'wb') as file:
        write_header_chunk(file)
        write_objects_chunk(file, context)
        write_collision_chunk(file, context)
    return {'FINISHED'}

# Blender UI Panel
class SCENE_PT_export(bpy.types.Panel):
    bl_label = "Export Scene"
    bl_idname = "SCENE_PT_export"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.operator("scene.export")

class SCENE_OT_export(bpy.types.Operator):
    bl_idname = "scene.export"
    bl_label = "Export"
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        export_scene(context, self.filepath)
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

def register():
    bpy.utils.register_class(SCENE_OT_export)
    bpy.utils.register_class(SCENE_PT_export)

def unregister():
    bpy.utils.unregister_class(SCENE_OT_export)
    bpy.utils.unregister_class(SCENE_PT_export)

if __name__ == "__main__":
    register()
