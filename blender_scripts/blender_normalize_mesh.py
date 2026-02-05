"""
Blender script to normalize a mesh to match Hunyuan3D's preprocessing.

Usage:
1. Select your mesh in Blender
2. Run this script (Alt+P in Scripting workspace)

The mesh will be:
- Centered at origin
- Scaled so the bounding sphere diameter = 1.15 (matching Hunyuan3D)
"""

import bpy
from mathutils import Vector

# Mesh normalization parameter (matches Hunyuan3D's load_mesh default)
MESH_SCALE_FACTOR = 1.15


def normalize_mesh():
    obj = bpy.context.active_object

    if obj is None:
        print("Error: No object selected. Please select a mesh.")
        return

    if obj.type != 'MESH':
        print(f"Error: Selected object '{obj.name}' is not a mesh (type: {obj.type})")
        return

    # Get world-space vertex coordinates
    mesh = obj.data
    world_matrix = obj.matrix_world

    vertices = [world_matrix @ v.co for v in mesh.vertices]

    if not vertices:
        print("Error: Mesh has no vertices")
        return

    # Calculate bounding box center
    min_co = Vector((
        min(v.x for v in vertices),
        min(v.y for v in vertices),
        min(v.z for v in vertices)
    ))
    max_co = Vector((
        max(v.x for v in vertices),
        max(v.y for v in vertices),
        max(v.z for v in vertices)
    ))
    center = (min_co + max_co) / 2

    # Calculate bounding sphere radius (max distance from center)
    max_dist = max((v - center).length for v in vertices)
    current_diameter = max_dist * 2.0

    if current_diameter == 0:
        print("Error: Mesh has zero size")
        return

    # Calculate scale to achieve target diameter
    target_scale = MESH_SCALE_FACTOR / current_diameter

    # Apply transformations
    # First, move to origin
    obj.location = obj.location - center

    # Then scale uniformly
    obj.scale = obj.scale * target_scale

    # Apply transformations to mesh data
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=True)

    print("=" * 50)
    print(f"Normalized mesh: '{obj.name}'")
    print("=" * 50)
    print(f"  Original bounding sphere diameter: {current_diameter:.4f}")
    print(f"  Target diameter (scale_factor):    {MESH_SCALE_FACTOR}")
    print(f"  Applied scale:                     {target_scale:.4f}")
    print("")
    print("  Mesh is now centered at origin")
    print(f"  Mesh fits in sphere of diameter {MESH_SCALE_FACTOR}")
    print("=" * 50)


# Run automatically when script is executed
if __name__ == "__main__":
    normalize_mesh()
