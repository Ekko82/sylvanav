"""Build an independent, synchronized point-cloud visualization scene.

Run with the existing portable Blender in background mode. Optional --preview
renders only frame 350 at half resolution; the saved scene retains 1920x1080.
Only cloud.blend and cloud-preview.jpg are written; woodland.blend is untouched.
"""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))

from pathlib import Path
import json
import math
import sys
import time

import bpy
from mathutils import Quaternion, Vector


from common import OUTPUT_ROOT, PROJECT_ROOT
ROOT = OUTPUT_ROOT
ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
cloud = json.loads((ROOT / "pointcloud-data.json").read_text(encoding="utf-8"))
animation = json.loads((ROOT / "animation-data.json").read_text(encoding="utf-8"))
all_points = [point for batch in cloud["batches"] for point in batch["points"]]
if not all_points:
    raise RuntimeError("No point cloud data")

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.name = "Synchronized environment scan"
scene.frame_start = 1
scene.frame_end = 600
scene.render.fps = 30
scene.render.resolution_x = 1920
scene.render.resolution_y = 1080
scene.render.resolution_percentage = 100
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.eevee.taa_render_samples = 32
scene.render.image_settings.file_format = "JPEG"
scene.render.image_settings.quality = 95
scene.render.film_transparent = False
scene.render.use_motion_blur = False
scene.view_settings.view_transform = "Standard"
scene.view_settings.look = "Medium High Contrast"
scene.view_settings.exposure = 0
scene.view_settings.gamma = 1
scene.world = bpy.data.worlds.new("Deep navy scan background")
scene.world.use_nodes = True
scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (.0012, .003, .009, 1)
scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 1


def emission_material(name, color, strength=1):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    shader = nodes.new("ShaderNodeEmission")
    shader.inputs["Color"].default_value = (*color, 1)
    shader.inputs["Strength"].default_value = strength
    output = nodes.new("ShaderNodeOutputMaterial")
    material.node_tree.links.new(shader.outputs[0], output.inputs["Surface"])
    return material


palette = [
    (.015, .15, .40), (.015, .31, .62), (.015, .54, .78),
    (.025, .77, .76), (.06, .86, .56), (.16, .90, .42),
    (.13, .65, .91), (.22, .40, .98),
]
point_materials = [emission_material("Height band %d" % i, color, 1.15)
                   for i, color in enumerate(palette)]
grid_material = emission_material("Quiet grid", (.006, .035, .073), .65)
floor_material = emission_material("Dark base plane", (.002, .006, .013), .75)
route_material = emission_material("Golden recorded route", (1, .42, .025), 1.6)
marker_material = emission_material("Recorded attitude cyan", (.11, .83, 1), 1.5)
frustum_material = emission_material("Mounted camera frustum", (.42, .66, .85), .65)


def poly_curve(name, lines, material, radius):
    data = bpy.data.curves.new(name, "CURVE")
    data.dimensions = "3D"
    data.resolution_u = 1
    data.bevel_resolution = 1
    data.bevel_depth = radius
    for points in lines:
        spline = data.splines.new("POLY")
        spline.points.add(len(points) - 1)
        for vertex, point in zip(spline.points, points):
            vertex.co = (*point, 1)
    obj = bpy.data.objects.new(name, data)
    scene.collection.objects.link(obj)
    data.materials.append(material)
    return obj


# Each batch is one mesh. Its 6-vertex, 8-face octahedra have no instances,
# modifiers or geometry nodes, so appearance remains deterministic per frame.
radius = .017
offsets = [(radius, 0, 0), (-radius, 0, 0), (0, radius, 0),
           (0, -radius, 0), (0, 0, radius), (0, 0, -radius)]
octa_faces = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4),
              (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]
z_max = max(point[2] for point in all_points)
batch_objects = []
for batch_index, batch in enumerate(cloud["batches"]):
    vertices, faces, material_indices = [], [], []
    for x, y, z in batch["points"]:
        start = len(vertices)
        vertices.extend([(x + a, y + b, z + c) for a, b, c in offsets])
        faces.extend([tuple(start + index for index in face) for face in octa_faces])
        color = min(7, max(0, int(z / max(z_max, 1e-6) * 8)))
        material_indices.extend([color] * 8)
    mesh = bpy.data.meshes.new("Scan geometry %02d" % batch_index)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    for material in point_materials:
        mesh.materials.append(material)
    mesh.polygons.foreach_set("material_index", material_indices)
    obj = bpy.data.objects.new("Scan batch frame %03d" % batch["frame"], mesh)
    scene.collection.objects.link(obj)
    obj["scan_frame"] = batch["frame"]
    obj["point_count"] = len(batch["points"])
    for frame, hidden in ((1, True), (batch["frame"] - 1, True), (batch["frame"], False)):
        obj.hide_render = hidden
        obj.hide_viewport = hidden
        obj.keyframe_insert("hide_render", frame=frame)
        obj.keyframe_insert("hide_viewport", frame=frame)
    for curve in obj.animation_data.action.fcurves:
        for key in curve.keyframe_points:
            key.interpolation = "CONSTANT"
    batch_objects.append(obj)

bpy.ops.mesh.primitive_plane_add(size=70, location=(6, 0, -.04))
floor = bpy.context.object
floor.name = "Subtle navy grid plane"
floor.data.materials.append(floor_material)
grid_lines = []
for index in range(-17, 23):
    grid_lines.append([(index, -17, -.021), (index, 17, -.021)])
for index in range(-17, 18):
    grid_lines.append([(-17, index, -.021), (22, index, -.021)])
poly_curve("Metre grid", grid_lines, grid_material, .0026)

ground_z = animation["metadata"]["coordinates"]["ground_z"]
route_translation = Vector(cloud["route_translation"])
sensor_config = cloud["sensor_config"]
route_points = [Vector(row["position_enu"]) + route_translation + Vector((0, 0, .22))
                for row in animation["frames"]]
route = poly_curve("Recorded route reveal", [route_points], route_material, .015)
route.data.bevel_factor_mapping_start = "SPLINE"
route.data.bevel_factor_mapping_end = "SPLINE"
lengths = [0.0]
for a, b in zip(route_points, route_points[1:]):
    lengths.append(lengths[-1] + (b - a).length)
for row, length in zip(animation["frames"], lengths):
    route.data.bevel_factor_end = length / lengths[-1]
    route.data.keyframe_insert("bevel_factor_end", frame=row["frame"])

marker = bpy.data.objects.new("Recorded FLU attitude", None)
scene.collection.objects.link(marker)
marker.rotation_mode = "QUATERNION"
arrow = poly_curve("Body forward arrow", [[(-.10, 0, 0), (.47, 0, 0)],
                    [(.34, -.075, 0), (.47, 0, 0), (.34, .075, 0)],
                    [(-.10, -.14, 0), (-.10, .14, 0)]], marker_material, .014)
arrow.parent = marker

# The frustum uses exactly the same local camera mounting as woodland.
sensor = bpy.data.objects.new("Synchronized sensor pose", None)
scene.collection.objects.link(sensor)
sensor.rotation_mode = "QUATERNION"
corner_points = [Vector((x * sensor_config["tan_half_fov"][0], y * sensor_config["tan_half_fov"][1], -1)) * .52
                 for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
frustum_lines = [[(0, 0, 0), corner] for corner in corner_points]
frustum_lines.append(corner_points + [corner_points[0]])
frustum = poly_curve("Mounted camera view cone", frustum_lines, frustum_material, .006)
frustum.parent = sensor
sensor_local_q = Quaternion(sensor_config["local_quaternion_wxyz"])
for row, position in zip(animation["frames"], route_points):
    frame = row["frame"]
    x, y, z, w = row["orientation_xyzw"]
    marker.location = position
    marker.rotation_quaternion = Quaternion((w, x, y, z))
    marker.keyframe_insert("location", frame=frame)
    marker.keyframe_insert("rotation_quaternion", frame=frame)
    x, y, z, w = row["orientation_inspire_xyzw"]
    inspire_q = Quaternion((w, x, y, z))
    sensor.location = (Vector(row["position_enu"]) + route_translation
                       + inspire_q @ Vector(sensor_config["local_position"]))
    sensor.rotation_quaternion = inspire_q @ sensor_local_q
    sensor.keyframe_insert("location", frame=frame)
    sensor.keyframe_insert("rotation_quaternion", frame=frame)

camera_data = bpy.data.cameras.new("Cloud three-quarter camera")
camera_data.type = "ORTHO"
camera_data.clip_start = .01
camera_data.clip_end = 100
camera = bpy.data.objects.new("CloudCamera", camera_data)
scene.collection.objects.link(camera)
scene.camera = camera
camera.rotation_mode = "QUATERNION"
target = Vector((1, 1, 1.8))
base_offset = Vector((6, -8, 6)) - target


def camera_pose(frame):
    time_s = (frame - 1) / 30
    u = max(0, min(1, (time_s - 6) / 10))
    u = u * u * (3 - 2 * u)
    angle = math.radians(7) * u
    offset = Vector((base_offset.x * math.cos(angle) - base_offset.y * math.sin(angle),
                     base_offset.x * math.sin(angle) + base_offset.y * math.cos(angle),
                     base_offset.z))
    position = target + offset
    q = (target - position).to_track_quat("-Z", "Y")
    return position, q


def projected_bounds(frame):
    position, q = camera_pose(frame)
    right = q @ Vector((1, 0, 0))
    up = q @ Vector((0, 1, 0))
    xs = [right.dot(Vector(point) - position) for point in all_points]
    ys = [up.dot(Vector(point) - position) for point in all_points]
    return min(xs), max(xs), min(ys), max(ys)


# Fit the actual visible environment samples, not only the 2 m body-centre route.
# Camera shift preserves the requested viewing direction while centring the trees.
camera_data.ortho_scale = 1
frame_view = camera_data.view_frame(scene=scene)
unit_width = max(v.x for v in frame_view) - min(v.x for v in frame_view)
unit_height = max(v.y for v in frame_view) - min(v.y for v in frame_view)
fit_bounds = [projected_bounds(frame) for frame in (1, 350, 481)]
camera_data.ortho_scale = 1.08 * max(
    max((xmax - xmin) / unit_width, (ymax - ymin) / unit_height)
    for xmin, xmax, ymin, ymax in fit_bounds)
camera_data.shift_x = 1
shift_width = sum(v.x for v in camera_data.view_frame(scene=scene)) / 4
camera_data.shift_x = 0
camera_data.shift_y = 1
shift_height = sum(v.y for v in camera_data.view_frame(scene=scene)) / 4
camera_data.shift_y = 0
for frame in (1, 181, 350, 481, 600):
    camera.location, camera.rotation_quaternion = camera_pose(frame)
    xmin, xmax, ymin, ymax = projected_bounds(frame)
    camera_data.shift_x = (xmin + xmax) / 2 / shift_width
    camera_data.shift_y = (ymin + ymax) / 2 / shift_height
    camera.keyframe_insert("location", frame=frame)
    camera.keyframe_insert("rotation_quaternion", frame=frame)
    camera_data.keyframe_insert("shift_x", frame=frame)
    camera_data.keyframe_insert("shift_y", frame=frame)

for owner in (marker, sensor, route.data, camera, camera_data):
    for curve in owner.animation_data.action.fcurves:
        for key in curve.keyframe_points:
            key.interpolation = "LINEAR"

# Verify batch visibility boundaries and the camera/sensor contract without rendering.
for batch, obj in zip(cloud["batches"], batch_objects):
    scene.frame_set(batch["frame"] - 1)
    assert obj.hide_render
    scene.frame_set(batch["frame"])
    assert not obj.hide_render
max_sensor_error = 0.0
for batch in cloud["batches"]:
    scene.frame_set(batch["frame"])
    max_sensor_error = max(max_sensor_error,
                           (sensor.location - Vector(batch["sensor_origin"])).length)
assert max_sensor_error < .00001, max_sensor_error
scene["point_cloud_method"] = cloud["method"]
scene["point_count"] = len(all_points)
scene["source_batch_count"] = len(cloud["batches"])
scene["point_radius_m"] = radius
scene["no_text_overlays"] = True
scene.frame_set(1)
bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT / "cloud.blend"))
print("CLOUD_READY", json.dumps({"points": len(all_points), "batches": len(batch_objects),
      "sensor_max_error_m": max_sensor_error, "ortho_scale": camera_data.ortho_scale,
      "blend": str(ROOT / "cloud.blend")}), flush=True)

if "--preview" in ARGS:
    scene.frame_set(350)
    scene.render.resolution_percentage = 50
    scene.eevee.taa_render_samples = 8
    scene.render.image_settings.quality = 92
    scene.render.filepath = str(ROOT / "cloud-preview.jpg")
    start = time.perf_counter()
    bpy.ops.render.render(write_still=True)
    print("CLOUD_PREVIEW", round(time.perf_counter() - start, 3),
          scene.render.filepath, flush=True)
