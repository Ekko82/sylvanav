"""Geometrically calibrated camera mount for the downloaded Inspire model.

Only the optical camera assembly tilts; the aircraft and gimbal suspension
remain unchanged. Intended for an unanimated, normalized initial body pose.
"""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import math
import bpy
import numpy as np
from mathutils import Vector, Matrix

# Mesh 72 is the side pitch bearing. Mesh 94 is the surrounding U cradle;
# 89/90/91/92 belong to the roll/yaw suspension above and behind the camera.
CAMERA_MESH_NAMES = ('100', '101', '102', '103', '68_01', '69', '70', '71',
                     '73', '74', '75', '76', '77', '93', '95', '96', '97', '98', '99')
STATIONARY_SUPPORT_NAMES = ('72', '89', '90', '91', '92', '94')


def _world_vertices(obj):
    return np.asarray([list(obj.matrix_world @ vertex.co) for vertex in obj.data.vertices], dtype=np.float64)


def _centroid(obj):
    return Vector(tuple(_world_vertices(obj).mean(axis=0)))


def _lens_geometry():
    lens = bpy.data.objects['98']
    rear_barrel = bpy.data.objects['102']
    vertices = _world_vertices(lens)
    centre = vertices.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov((vertices - centre).T))
    normal = eigenvectors[:, 0]
    # The smallest covariance axis has arbitrary sign. The lens is on the
    # outward side of the rear barrel, giving a physically directed optical axis.
    outward = centre - np.asarray(_centroid(rear_barrel))
    if np.dot(normal, outward) < 0:
        normal = -normal
    return Vector(tuple(centre)), Vector(tuple(normal)).normalized()


def configure_sensor(scene, drone):
    """Return (SensorCamera, extrinsic_metadata) in the Drone local frame.

    The visible camera's pitch is calibrated to six degrees down from the body
    forward axis. The camera mount is measured from the real mesh 98 aperture;
    it is not a free-floating body-front offset. Both rendering and downstream
    ray sampling should use returned tan_half_fov or camera.data.view_frame.
    """
    if bpy.data.objects.get('Sensor optical pitch') is not None:
        raise RuntimeError('configure_sensor must be called only once on the source scene')
    required = set(CAMERA_MESH_NAMES + STATIONARY_SUPPORT_NAMES + ('DL_mount',))
    missing = required - set(bpy.data.objects.keys())
    if missing:
        raise RuntimeError('Missing source camera parts: ' + ', '.join(sorted(missing)))
    if max(abs(float(s) - 1.) for s in drone.scale) > 1e-5:
        raise RuntimeError('Normalize scale into Model alignment before configuring the sensor')

    # Source showcase uses a portrait render aspect. Establish final aspect
    # before reading view_frame, so the FOV matches the eventual 1920x1080 RGB.
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    scene.render.pixel_aspect_x = 1
    scene.render.pixel_aspect_y = 1
    bpy.context.view_layer.update()
    body_to_world = drone.matrix_world.copy()
    world_to_body = body_to_world.inverted()
    centre_before, optical_before_world = _lens_geometry()
    lens_before_local = world_to_body @ centre_before
    before = (world_to_body.to_3x3() @ optical_before_world).normalized()
    pitch_down = math.radians(6.)
    target = Vector((0, -math.cos(pitch_down), -math.sin(pitch_down)))
    pitch_change = before.rotation_difference(target)

    # The bearing's X axis crosses the camera housing. Its X coordinate does
    # not change the pitch-axis line, and the side cap remains stationary.
    bearing_world = _centroid(bpy.data.objects['72'])
    pivot_local = world_to_body @ bearing_world
    pivot = bpy.data.objects.new('Sensor optical pitch', None)
    scene.collection.objects.link(pivot)
    pivot.parent = drone
    pivot.matrix_parent_inverse = Matrix.Identity(4)
    pivot.location = pivot_local
    pivot.rotation_mode = 'QUATERNION'
    bpy.context.view_layer.update()
    moved = [bpy.data.objects[name] for name in CAMERA_MESH_NAMES] + [bpy.data.objects['DL_mount']]
    for obj in moved:
        old_world = obj.matrix_world.copy()
        obj.parent = pivot
        obj.matrix_world = old_world
    bpy.context.view_layer.update()
    pivot.rotation_quaternion = pitch_change
    bpy.context.view_layer.update()

    lens_world, optical_world = _lens_geometry()
    lens_local = world_to_body @ lens_world
    expected_lens_local = pivot_local + pitch_change @ (lens_before_local - pivot_local)
    pivot_transform_error = (lens_local - expected_lens_local).length
    assert pivot_transform_error < 1e-5, pivot_transform_error
    forward_local = (world_to_body.to_3x3() @ optical_world).normalized()
    # Move just beyond the front glass/rim surfaces so wide-angle camera rays
    # cannot hit the model's own optical mesh. It remains within ~1–2cm of glass.
    front_vertices = np.concatenate([_world_vertices(bpy.data.objects[name])
                                     for name in ('98', '99', '100')])
    max_front = float(np.max((front_vertices - np.asarray(lens_world)) @ np.asarray(optical_world)))
    clearance = .006
    front_offset = max(0., max_front) + clearance
    camera_world = lens_world + optical_world * front_offset
    camera_local = world_to_body @ camera_world

    camera_data = bpy.data.cameras.new('SensorCamera')
    camera_data.lens = 17
    camera_data.sensor_width = 36
    camera_data.sensor_fit = 'HORIZONTAL'
    camera_data.clip_start = .002
    camera_data.clip_end = 30
    sensor = bpy.data.objects.new('SensorCamera', camera_data)
    scene.collection.objects.link(sensor)
    sensor.parent = drone
    sensor.matrix_parent_inverse = Matrix.Identity(4)
    sensor.location = camera_local
    sensor.rotation_mode = 'QUATERNION'
    sensor.rotation_quaternion = forward_local.to_track_quat('-Z', 'Y')
    bpy.context.view_layer.update()
    camera_forward_world = (sensor.matrix_world.to_3x3() @ Vector((0, 0, -1))).normalized()
    optical_dot = float(camera_forward_world.dot(optical_world))
    target_dot = float(forward_local.dot(target))
    assert optical_dot > .99999, optical_dot
    assert target_dot > .99999, target_dot
    corners = camera_data.view_frame(scene=scene)
    tan_half = [max(abs(v.x / v.z) for v in corners), max(abs(v.y / v.z) for v in corners)]
    metadata = {
        'coordinate_frame': 'Drone body local; metres; -Y forward, +Z up',
        'local_position': list(sensor.location),
        'local_quaternion_wxyz': list(sensor.rotation_quaternion),
        'local_forward': list(forward_local),
        'tan_half_fov': tan_half,
        'camera_lens_mm': camera_data.lens,
        'camera_sensor_width_mm': camera_data.sensor_width,
        'image_size': [1920, 1080],
        'physical_lens_object': '98',
        'normal_sign_reference': 'from rear barrel mesh 102 toward physical lens mesh 98',
        'physical_lens_center_local': list(lens_local),
        'physical_lens_forward_local': list(forward_local),
        'physical_lens_to_sensor_forward_dot': optical_dot,
        'physical_lens_to_target_forward_dot': target_dot,
        'source_physical_lens_center_local': list(lens_before_local),
        'source_optical_forward_local': list(before),
        'lens_center_displacement_m': (lens_local - lens_before_local).length,
        'lens_center_pivot_rotation_error_m': pivot_transform_error,
        'target_pitch_down_deg': 6.,
        'applied_pitch_change_deg': math.degrees(pitch_change.angle),
        'pitch_pivot_local': list(pivot_local),
        'front_surface_offset_m': front_offset,
        'front_surface_clearance_m': clearance,
        'camera_meshes_rotated': list(CAMERA_MESH_NAMES),
        'camera_marking_group_rotated': 'DL_mount',
        'stationary_gimbal_supports': list(STATIONARY_SUPPORT_NAMES),
    }
    return sensor, metadata
