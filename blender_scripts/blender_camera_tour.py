"""
Seamless single-camera trajectory through Hy3D multiview positions, with NO roll flips.

Core ideas:
- Camera position: SLERP between unit direction vectors -> fixed radius exactly.
- Camera orientation: build a continuous local frame using parallel transport of the "up" vector.
  This eliminates 90° roll jumps at waypoint boundaries and near poles.

Trajectory:
front -> front_left -> left -> back -> right -> front_right -> front -> top -> back -> bottom

Usage:
- Paste in Blender Scripting, Run.
- Scrub timeline. You should see completely seamless motion.

Notes:
- Assumes mesh centered at origin.
- Uses same Hy3D spherical camera convention + Hy3D->Blender camera position conversion
  as your validated multiview setup.
"""

import bpy
import math
from mathutils import Vector, Matrix, Quaternion

# =========================
# PARAMETERS
# =========================
CAMERA_DISTANCE = 1.45
ORTHO_SCALE = 1.2
DURATION_SECONDS = 5.0

START_AT_CURRENT_FRAME = False

# Bake density: keep at 1 for maximum smoothness & stability
KEY_EVERY_N_FRAMES = 1

# =========================
# Hy3D view definitions (matching your tested mapping)
# =========================
VIEW_TO_AZIM_ELEV = {
    "top":         (0,    0),
    "right":       (90,   0),
    "bottom":      (180,  0),
    "left":        (270,  0),
    "back":        (0,   90),
    "front":       (180, -90),
    "front_left":  (270, -45),
    "front_right": (90,  -45),
}

TRAJECTORY = [
    "front",
    "front_left",
    "left",
    "back",
    "right",
    "front_right",
    "front",
    "top",
    "back",
    "bottom",
]

# =========================
# Hy3D camera position math
# =========================
def get_camera_position_hunyuan(elev, azim, camera_distance):
    elev_transformed = -elev
    azim_transformed = azim + 90

    elev_rad = math.radians(elev_transformed)
    azim_rad = math.radians(azim_transformed)

    x = camera_distance * math.cos(elev_rad) * math.cos(azim_rad)
    y = camera_distance * math.cos(elev_rad) * math.sin(azim_rad)
    z = camera_distance * math.sin(elev_rad)

    return Vector((x, y, z))

def hunyuan_to_blender_position(pos):
    # Inverse camera transform for Hy3D mesh transform (-X, Z, -Y)
    return Vector((-pos.x, -pos.z, pos.y))

def blender_pos_for_view(view_name):
    azim, elev = VIEW_TO_AZIM_ELEV[view_name]
    pos_hy = get_camera_position_hunyuan(elev, azim, CAMERA_DISTANCE)
    return hunyuan_to_blender_position(pos_hy)

# =========================
# Math helpers
# =========================
def clamp(x, a, b):
    return max(a, min(b, x))

def slerp(u: Vector, v: Vector, t: float) -> Vector:
    """Spherical linear interpolation between unit vectors u and v."""
    dot = clamp(u.dot(v), -1.0, 1.0)
    omega = math.acos(dot)
    if omega < 1e-9:
        return u.copy()
    so = math.sin(omega)
    a = math.sin((1.0 - t) * omega) / so
    b = math.sin(t * omega) / so
    return (a * u + b * v).normalized()

def rotation_from_forward_up(forward: Vector, up: Vector) -> Quaternion:
    """
    Build a camera rotation such that:
      - Camera local -Z points along 'forward' (towards origin).
      - Camera local +Y aligns with 'up' as much as possible.
    Returns quaternion rotation.
    """
    f = forward.normalized()
    u = up.normalized()

    # Orthonormalize (Gram-Schmidt)
    # right = f x u  (we want right to be perpendicular)
    r = f.cross(u)
    if r.length < 1e-8:
        # Degenerate: up parallel to forward. Caller should avoid, but be safe.
        # Pick an arbitrary axis not parallel to forward:
        alt = Vector((1, 0, 0)) if abs(f.dot(Vector((1, 0, 0)))) < 0.9 else Vector((0, 1, 0))
        r = f.cross(alt)
    r.normalize()
    u = r.cross(f).normalized()

    # Camera basis in world:
    # local +X = right, local +Y = up, local +Z = backward
    # because camera looks along local -Z, so local +Z points opposite forward
    backward = (-f).normalized()

    # Columns are basis vectors for X,Y,Z in world space
    m = Matrix((r, u, backward)).transposed()
    return m.to_quaternion()

def parallel_transport_up(prev_forward: Vector, forward: Vector, prev_up: Vector) -> Vector:
    """
    Parallel transport of 'up' along changing forward direction:
    - Project prev_up onto plane perpendicular to new forward, renormalize.
    This produces a smooth, minimal-twist frame (no roll flips).
    """
    f = forward.normalized()
    u = prev_up - f * prev_up.dot(f)  # remove component along forward
    if u.length < 1e-8:
        # If we hit degeneracy (rare), reconstruct using prev_forward to keep continuity:
        # Use the previous right vector if possible.
        pf = prev_forward.normalized()
        prev_right = pf.cross(prev_up)
        if prev_right.length < 1e-8:
            # absolute fallback
            prev_right = Vector((1, 0, 0))
        u = prev_right.cross(f)
    return u.normalized()

# =========================
# Scene / object creation
# =========================
def ensure_collection(name: str):
    if name in bpy.data.collections:
        col = bpy.data.collections[name]
        for obj in list(col.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        return col
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col

def create_camera(col, name="Hy3D_PathCam"):
    cam_data = bpy.data.cameras.new(name=name)
    cam_data.type = 'ORTHO'
    cam_data.ortho_scale = ORTHO_SCALE
    cam_data.clip_start = 0.1
    cam_data.clip_end = 100.0

    cam_obj = bpy.data.objects.new(name, cam_data)
    col.objects.link(cam_obj)
    bpy.context.scene.camera = cam_obj
    return cam_obj

def create_target_empty(col, name="Hy3D_PathTarget"):
    target = bpy.data.objects.new(name, None)
    target.empty_display_type = 'SPHERE'
    target.empty_display_size = 0.08
    target.location = (0, 0, 0)
    col.objects.link(target)
    return target

# =========================
# Animation baking
# =========================
def bake_camera_motion(cam_obj, trajectory_names, duration_seconds):
    scene = bpy.context.scene
    fps = scene.render.fps if scene.render.fps > 0 else 24

    total_frames = max(2, int(round(duration_seconds * fps)))
    start_frame = scene.frame_current if START_AT_CURRENT_FRAME else 1

    # Compute waypoint unit directions
    waypoint_positions = [blender_pos_for_view(v) for v in trajectory_names]
    waypoint_dirs = [p.normalized() for p in waypoint_positions]

    # Segment angles for timing
    angles = []
    for i in range(len(waypoint_dirs) - 1):
        dot = clamp(waypoint_dirs[i].dot(waypoint_dirs[i+1]), -1.0, 1.0)
        angles.append(math.acos(dot))

    total_angle = sum(angles) if sum(angles) > 1e-9 else 1.0

    seg_frames = []
    remaining = total_frames
    for i, ang in enumerate(angles):
        if i == len(angles) - 1:
            n = remaining
        else:
            n = max(1, int(round(total_frames * (ang / total_angle))))
            remaining -= n
        seg_frames.append(n)

    # Clear existing animation
    cam_obj.animation_data_clear()

    # Initialize at first waypoint
    frame = start_frame
    pos = waypoint_dirs[0] * CAMERA_DISTANCE

    # forward points towards origin
    forward = (-pos).normalized()

    # Choose a stable initial up reference:
    # Use world Z as "up" unless too close to forward, then fall back to world Y.
    world_up_z = Vector((0, 0, 1))
    world_up_y = Vector((0, 1, 0))
    init_up = world_up_z
    if abs(forward.dot(init_up)) > 0.95:
        init_up = world_up_y
    up = (init_up - forward * init_up.dot(forward)).normalized()

    q = rotation_from_forward_up(forward, up)

    cam_obj.location = pos
    cam_obj.rotation_mode = 'QUATERNION'
    cam_obj.rotation_quaternion = q

    cam_obj.keyframe_insert(data_path="location", frame=frame)
    cam_obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)

    prev_forward = forward.copy()
    prev_up = up.copy()

    # Bake each segment
    for i in range(len(waypoint_dirs) - 1):
        u = waypoint_dirs[i]
        v = waypoint_dirs[i + 1]
        nframes = seg_frames[i]

        for step in range(1, nframes + 1):
            if (step % KEY_EVERY_N_FRAMES) != 0 and step != nframes:
                continue

            t = step / nframes
            d = slerp(u, v, t)
            pos = d * CAMERA_DISTANCE

            forward = (-pos).normalized()
            up = parallel_transport_up(prev_forward, forward, prev_up)

            q = rotation_from_forward_up(forward, up)

            frame += 1
            cam_obj.location = pos
            cam_obj.rotation_quaternion = q

            cam_obj.keyframe_insert(data_path="location", frame=frame)
            cam_obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)

            prev_forward = forward
            prev_up = up

    # Timeline bounds
    scene.frame_start = start_frame
    scene.frame_end = frame

    # Make interpolation perfectly smooth and non-overshooting:
    # - Linear location prevents spherical overshoot artifacts.
    # - Linear quaternion keys avoids spline weirdness in rotation.
    if cam_obj.animation_data and cam_obj.animation_data.action:
        for fcu in cam_obj.animation_data.action.fcurves:
            for kp in fcu.keyframe_points:
                kp.interpolation = 'LINEAR'

    # Optional: transparent background like your multiview
    scene.render.film_transparent = True

    print("=" * 70)
    print("DONE: Seamless Hy3D path camera baked (no roll flips).")
    print(f"FPS: {fps}, Duration: {duration_seconds:.2f}s (~{total_frames} frames)")
    print(f"Frames: {start_frame} -> {frame}")
    print("Trajectory:", " -> ".join(trajectory_names))
    print("=" * 70)

# =========================
# RUN
# =========================
def main():
    col = ensure_collection("Hunyuan3D_PathCamera_Seamless")
    cam = create_camera(col, "Hy3D_PathCam")

    # Optional target at origin (not used for constraints, but nice to see)
    create_target_empty(col, "Hy3D_PathTarget")

    bake_camera_motion(cam, TRAJECTORY, DURATION_SECONDS)

if __name__ == "__main__":
    main()
