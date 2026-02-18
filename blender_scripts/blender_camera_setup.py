"""
Blender script to reproduce Hunyuan3D MultiView camera setup.

This script creates 6 cameras matching the exact viewpoints used by:
- Hy3DRenderMultiView
- Hy3DRenderMultiViewDepth

Usage in Blender:
1. Open Blender
2. Go to Scripting workspace
3. Open or paste this script
4. Run the script (Alt+P or click "Run Script")

IMPORTANT:
- Your mesh should be centered at origin
- Hunyuan3D transforms meshes internally: (X, Y, Z) → (-X, Z, -Y)
- If importing from GLB/GLTF (Y-up), the mesh will be auto-converted by Blender
- For best match, ensure your mesh in Blender matches the orientation you see in ComfyUI
"""

import bpy
import math
from mathutils import Vector


# ============================================================================
# PARAMETERS - Match these to ComfyUI node settings
# ============================================================================

# Default camera viewpoints (same as Hy3DRenderMultiView defaults)
# Note: The names are what the view SHOWS in Blender, not Hunyuan3D's internal naming
CAMERA_AZIMS = [0, 90, 180, 270, 0, 180, 270, 90]
CAMERA_ELEVS = [0, 0, 0, 0, 90, -90, -45, -45]

# Corrected view names based on what each camera actually shows in Blender
# (After coordinate system transformation from Hunyuan3D)
VIEW_NAMES = ["top", "right", "bottom", "left", "back", "front", "front_left", "front_right"]

# Custom up axis for specific views to match Hunyuan3D orientation
# Most views use UP_Y, but top/bottom need special handling
VIEW_UP_AXIS = {
    "top": 'UP_Z',      # Looking down, use Z as up reference
    "bottom": 'UP_X',   # Use X as base, then add 90° rotation
}
DEFAULT_UP_AXIS = 'UP_Y'

# Additional Z rotation (in radians) to apply after constraint
# Used for views that need rotation correction
VIEW_Z_ROTATION_OFFSET = {
    "bottom": math.pi / 2,  # 90° rotation (UP_X gives 90°, this adds another 90° = 180° total)
}

# Camera parameters (match MeshRender defaults)
CAMERA_DISTANCE = 1.45
ORTHO_SCALE = 1.2  # Must be >= 1.15 to fit a normalized mesh

# Render resolution
RENDER_SIZE = 1024


# ============================================================================
# COORDINATE SYSTEM NOTES
# ============================================================================
"""
Hunyuan3D internal coordinate system (after mesh transform):
- Uses Z-up convention
- Camera position calculated with spherical coordinates
- azim=0 means looking at the mesh from +Y axis (after +90° offset applied internally)

Blender coordinate system:
- Z-up (same as Hunyuan3D internal)
- Front view (Numpad 1) = looking along -Y
- Right view (Numpad 3) = looking along -X
- Top view (Numpad 7) = looking along -Z
- Cameras look down their LOCAL -Z axis

Hunyuan3D camera calculation (from camera_utils.py):
    elev = -elev
    azim += 90
    camera_position = [
        distance * cos(elev_rad) * cos(azim_rad),
        distance * cos(elev_rad) * sin(azim_rad),
        distance * sin(elev_rad)
    ]
"""


# ============================================================================
# CAMERA POSITION CALCULATION
# ============================================================================

def get_camera_position_hunyuan(elev, azim, camera_distance):
    """
    Calculate camera position exactly as Hunyuan3D does.
    Returns position in Hunyuan3D's coordinate system.
    """
    # Apply Hunyuan3D's transformations
    elev_transformed = -elev
    azim_transformed = azim + 90

    elev_rad = math.radians(elev_transformed)
    azim_rad = math.radians(azim_transformed)

    x = camera_distance * math.cos(elev_rad) * math.cos(azim_rad)
    y = camera_distance * math.cos(elev_rad) * math.sin(azim_rad)
    z = camera_distance * math.sin(elev_rad)

    return Vector((x, y, z))


def hunyuan_to_blender_position(pos):
    """
    Convert Hunyuan3D camera position to Blender coordinates.

    Hunyuan3D applies this mesh transform: (X, Y, Z) → (-X, Z, -Y)
    To get equivalent view in Blender (where mesh is NOT transformed),
    we need to apply the INVERSE camera transform.

    If mesh transform is M, and camera is at position P looking at origin,
    the equivalent in untransformed space is M^(-1) applied to the camera setup.

    Mesh transform: (-X, Z, -Y)
    Inverse: (-X, -Z, Y) ... let's verify:
      If (X,Y,Z) → (-X, Z, -Y) = (X', Y', Z')
      Then X' = -X, Y' = Z, Z' = -Y
      So X = -X', Z = Y', Y = -Z'
      Original = (-X', -Z', Y')

    So to convert Hunyuan camera pos to Blender: (X, Y, Z) → (-X, -Z, Y)
    """
    return Vector((-pos.x, -pos.z, pos.y))


# ============================================================================
# MAIN SCRIPT
# ============================================================================

def create_hunyuan3d_cameras(use_constraints=True):
    """
    Create all 6 cameras matching Hunyuan3D MultiView setup.

    Args:
        use_constraints: If True, use Track To constraint (easier to adjust).
                        If False, set rotation directly.
    """

    # Create a collection for the cameras
    collection_name = "Hunyuan3D_Cameras"
    if collection_name in bpy.data.collections:
        collection = bpy.data.collections[collection_name]
        # Remove existing objects in collection
        for obj in list(collection.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
    else:
        collection = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(collection)

    # Create target empty at origin for Track To constraints
    if use_constraints:
        target_name = "Hy3D_Camera_Target"
        if target_name in bpy.data.objects:
            target = bpy.data.objects[target_name]
        else:
            target = bpy.data.objects.new(target_name, None)
            collection.objects.link(target)
        target.location = (0, 0, 0)
        target.empty_display_type = 'SPHERE'
        target.empty_display_size = 0.1

    cameras = []

    print("\nCamera positions:")
    print("-" * 70)

    for i, (azim, elev, name) in enumerate(zip(CAMERA_AZIMS, CAMERA_ELEVS, VIEW_NAMES)):
        # Calculate camera position in Hunyuan3D coordinates
        pos_hunyuan = get_camera_position_hunyuan(elev, azim, CAMERA_DISTANCE)

        # Convert to Blender coordinates
        pos_blender = hunyuan_to_blender_position(pos_hunyuan)

        # Create camera data
        cam_data = bpy.data.cameras.new(name=f"Hy3D_Camera_{name}")
        cam_data.type = 'ORTHO'
        cam_data.ortho_scale = ORTHO_SCALE
        cam_data.clip_start = 0.1
        cam_data.clip_end = 100

        # Create camera object
        cam_obj = bpy.data.objects.new(f"Hy3D_Camera_{name}", cam_data)
        collection.objects.link(cam_obj)

        # Set position
        cam_obj.location = pos_blender

        # Check if this view needs a Z rotation offset
        z_offset = VIEW_Z_ROTATION_OFFSET.get(name, 0)

        if use_constraints and z_offset == 0:
            # Use Track To constraint - most reliable method
            constraint = cam_obj.constraints.new(type='TRACK_TO')
            constraint.target = target
            constraint.track_axis = 'TRACK_NEGATIVE_Z'  # Camera looks down -Z

            # Set up axis based on view (most use UP_Y, top/bottom are special)
            constraint.up_axis = VIEW_UP_AXIS.get(name, DEFAULT_UP_AXIS)
        else:
            # Calculate rotation directly (required for views with Z rotation offset)
            direction = (Vector((0, 0, 0)) - pos_blender).normalized()
            # Map constraint up axis to track_quat parameter
            up_axis = VIEW_UP_AXIS.get(name, DEFAULT_UP_AXIS)
            up_ref = up_axis.replace('UP_', '')  # 'UP_Y' -> 'Y'
            rot = direction.to_track_quat('-Z', up_ref).to_euler()

            # Apply Z rotation offset if needed
            if z_offset != 0:
                rot.z += z_offset

            cam_obj.rotation_euler = rot

        cameras.append(cam_obj)

        print(f"  {name:8s}: azim={azim:4d}°, elev={elev:4d}° → "
              f"Hy3D pos: ({pos_hunyuan.x:6.2f}, {pos_hunyuan.y:6.2f}, {pos_hunyuan.z:6.2f}) → "
              f"Blender: ({pos_blender.x:6.2f}, {pos_blender.y:6.2f}, {pos_blender.z:6.2f})")

    print("-" * 70)

    return cameras


def setup_render_settings():
    """Configure render settings to match Hunyuan3D output."""
    scene = bpy.context.scene

    scene.render.resolution_x = RENDER_SIZE
    scene.render.resolution_y = RENDER_SIZE
    scene.render.resolution_percentage = 100

    # Set transparent background (like Hunyuan3D's mask)
    scene.render.film_transparent = True

    print(f"\nRender settings: {RENDER_SIZE}x{RENDER_SIZE}, transparent background")


def create_test_cube():
    """Create a test cube to verify camera alignment."""
    # Delete existing test cube if present
    if "Hy3D_TestCube" in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects["Hy3D_TestCube"], do_unlink=True)

    bpy.ops.mesh.primitive_cube_add(size=0.5, location=(0, 0, 0))
    cube = bpy.context.active_object
    cube.name = "Hy3D_TestCube"

    # Add different colored materials to each face for orientation testing
    # This helps verify the camera views match

    print("\nCreated test cube at origin (size=0.5)")
    print("Use this to verify camera orientations match ComfyUI output")


def render_all_views(output_dir="/tmp/hunyuan3d_renders"):
    """Render all camera views."""
    import os
    os.makedirs(output_dir, exist_ok=True)

    scene = bpy.context.scene

    for obj in bpy.data.objects:
        if obj.name.startswith("Hy3D_Camera_") and obj.type == 'CAMERA':
            view_name = obj.name.replace("Hy3D_Camera_", "")
            scene.camera = obj

            scene.render.filepath = os.path.join(output_dir, f"{view_name}.png")
            bpy.ops.render.render(write_still=True)
            print(f"Rendered: {scene.render.filepath}")


def print_expected_views():
    """Print what each view should show for verification."""
    print("""
Camera mapping (Hunyuan3D params → Blender view name):

  Hy3D (azim=0°,   elev=0°)   → "top"         : Looking down at top of model
  Hy3D (azim=90°,  elev=0°)   → "right"        : Looking at right side of model
  Hy3D (azim=180°, elev=0°)   → "bottom"       : Looking up at bottom of model
  Hy3D (azim=270°, elev=0°)   → "left"         : Looking at left side of model
  Hy3D (azim=0°,   elev=90°)  → "back"         : Looking at back of model
  Hy3D (azim=180°, elev=-90°) → "front"        : Looking at front of model
  Hy3D (azim=270°, elev=-45°) → "front_left"   : Front view rotated -45° around Z
  Hy3D (azim=90°,  elev=-45°) → "front_right"  : Front view rotated +45° around Z

Note: front_right and front_left are derived from the front view by rotating
the camera position ±45° around the world Z axis (not the Y axis as before).
Both cameras remain at the same height as the front camera (elev=-45° in
Hunyuan3D params = horizontal plane in Blender space).

To verify: Render in both ComfyUI and Blender, compare the views.
""")


# ============================================================================
# RUN
# ============================================================================

if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("Creating Hunyuan3D MultiView Cameras for Blender")
    print("=" * 70)

    cameras = create_hunyuan3d_cameras(use_constraints=True)
    setup_render_settings()

    # Set the first camera as active
    if cameras:
        bpy.context.scene.camera = cameras[0]

    print("\n" + "=" * 70)
    print("DONE! Created 6 cameras in 'Hunyuan3D_Cameras' collection")
    print("=" * 70)
    print(f"""
Camera parameters:
  - Type: Orthographic
  - Ortho Scale: {ORTHO_SCALE}
  - Distance: {CAMERA_DISTANCE}
  - Clip: 0.1 - 100
  - Resolution: {RENDER_SIZE}x{RENDER_SIZE}

Tips:
  - Cameras use Track To constraint pointing at origin
  - Your mesh should be centered at (0, 0, 0)
  - Run blender_normalize_mesh.py to normalize your mesh (select mesh first)
  - Run create_test_cube() to add a reference cube
  - Run render_all_views('/path/to/output') to batch render

Mesh normalization:
  - Hunyuan3D normalizes meshes to fit in a sphere of diameter 1.15
  - Run blender_normalize_mesh.py after selecting your mesh
  - Or manually scale your mesh to fit within ~0.575 radius from origin

View correspondence with ComfyUI output order [0,1,2,3,4,5,6,7]:
  - ComfyUI index 0 (azim=0°,   elev=0°)   → Blender "top"
  - ComfyUI index 1 (azim=90°,  elev=0°)   → Blender "right"
  - ComfyUI index 2 (azim=180°, elev=0°)   → Blender "bottom"
  - ComfyUI index 3 (azim=270°, elev=0°)   → Blender "left"
  - ComfyUI index 4 (azim=0°,   elev=90°)  → Blender "back"
  - ComfyUI index 5 (azim=180°, elev=-90°) → Blender "front"
  - ComfyUI index 6 (azim=270°, elev=-45°) → Blender "front_left"  (front view rotated -45° around Z)
  - ComfyUI index 7 (azim=90°,  elev=-45°) → Blender "front_right" (front view rotated +45° around Z)
""")
    print("=" * 70 + "\n")

    print_expected_views()
