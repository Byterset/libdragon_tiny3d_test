import bpy
import struct
import sys

def write_collision_data(output_path, base_scale):
    # Ensure the collection exists
    collection = bpy.data.collections.get("collision")
    if not collection:
        print("Collection 'collision' not found.")
        return
    
    vertices = []
    triangles = []
    normals = []

    for obj in collection.objects:
        if obj.type != 'MESH':
            continue  # Skip non-mesh objects

        mesh = obj.data
        mesh.calc_normals_split()  # Ensure the normals are calculated
        vert_offset = len(vertices)  # Keep track of vertex indices offset for each object


        # Collect and scale vertices
        # vertices.extend([tuple(vert.co * scale) for vert in mesh.vertices])
        vertices.extend([(vert.co.x * base_scale, vert.co.z * base_scale, -vert.co.y * base_scale) for vert in mesh.vertices])

        # Collect triangles and normals
        for poly in mesh.polygons:
            if len(poly.vertices) != 3:
                continue  # Skip non-triangular faces
            
            triangles.append([vert_offset + poly.vertices[i] for i in range(3)])
            normal = poly.normal
            normals.append((normal.x, normal.y, normal.z))

    print(f"Vertices: {len(vertices)}")
    print(f"Triangles: {len(triangles)}")

    # Write data to the binary file
    with open(output_path, 'wb') as f:
        # Write header
        f.write(b"CMSH")
        
        # Write vertex count
        vertex_count = len(vertices)
        f.write(struct.pack('>H', vertex_count))
        
        # Write vertices
        for vert in vertices:
            f.write(struct.pack('>fff', *vert))
        
        # Write triangle count
        triangle_count = len(triangles)
        f.write(struct.pack('>H', triangle_count))
        
        # Write triangle indices
        for tri in triangles:
            f.write(struct.pack('>HHH', *tri))
        
        # Write normals
        for normal in normals:
            f.write(struct.pack('>fff', *normal))
    
    print(f"Collision data successfully written to {output_path}")

# Entry point for the script
if __name__ == "__main__":
    # Retrieve arguments
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]  # Get all arguments after "--"
    else:
        argv = []  # No arguments provided

    if len(argv) < 1:
        print(bpy.data.filepath)
        print("Usage: blender -b <source_file> --python <script.py> -- <output_path> [base_scale]")
        sys.exit(1)

    source_file = bpy.data.filepath
    output_path = argv[0]
    base_scale = int(argv[1]) if len(argv) > 1 else 1  # Default base_scale to 1 if not provided
    
    # Print out the arguments
    print(f"Source file: {source_file}")
    print(f"Output path: {output_path}")
    print(f"Base scale: {base_scale}")
    
    # Write the collision data
    write_collision_data(output_path, base_scale)