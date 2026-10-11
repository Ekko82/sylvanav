"""CPU-only replay of complete_understory.py sampling against saved meshes.

The original scanner and the complete render-visible environment are evaluated
separately, so successful reproduction cannot conceal a geometry-scope omission.
This module does not render or save any Blender scene.
"""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))

import bisect
import math
import random

from mathutils import Vector
from mathutils.bvhtree import BVHTree


def replay_scan(scene, sensor, cloud_data, visible_environment, full_bvh,
                sampling_scope='name_or_collection'):
    import bpy

    if sampling_scope not in {'name_only', 'name_or_collection'}:
        raise ValueError('Unsupported sampler geometry scope: ' + sampling_scope)

    lo, hi = Vector((-1.8, -1.8, -.05)), Vector((1.8, 1.8, 2.7))

    def reachable(obj):
        corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        low = [min(point[axis] for point in corners) for axis in range(3)]
        high = [max(point[axis] for point in corners) for axis in range(3)]
        return sum(max(0, low[axis] - hi[axis], lo[axis] - high[axis]) ** 2
                   for axis in range(3)) <= 18 ** 2

    def included_by_scope(obj):
        return obj.name.startswith('ENV2_') or (
            sampling_scope == 'name_or_collection'
            and any(col.name.startswith('ENV2_') for col in obj.users_collection))

    # Match the saved scanner's selection and scene.objects packing order.
    # name_only remains available to reproduce the archived pre-fix evidence.
    # The full visible BVH is a separate check, rather than an assumption that
    # successful replay proves the scanner included every rendered surface.
    objects = [obj for obj in scene.objects
               if included_by_scope(obj) and obj.type == 'MESH'
               and not obj.hide_render
               and not any(col.hide_render for col in obj.users_collection)
               and reachable(obj)]
    excluded_reachable = [obj for obj in visible_environment if obj not in objects and reachable(obj)]
    vertices, triangles, triangle_ends = [], [], []
    for obj in objects:
        mesh = obj.data
        mesh.calc_loop_triangles()
        first = len(vertices)
        vertices.extend(tuple(obj.matrix_world @ vertex.co) for vertex in mesh.vertices)
        triangles.extend(tuple(first + index for index in tri.vertices)
                         for tri in mesh.loop_triangles)
        triangle_ends.append(len(triangles))
    triangle_count, vertex_count = len(triangles), len(vertices)
    sampling_bvh = BVHTree.FromPolygons(vertices, triangles, all_triangles=True)
    del vertices, triangles

    def scanner_hit_object(index):
        return objects[bisect.bisect_right(triangle_ends, index)].name if index is not None else None

    view = sensor.data.view_frame(scene=scene)
    xmin, xmax = min(v.x / -v.z for v in view), max(v.x / -v.z for v in view)
    ymin, ymax = min(v.y / -v.z for v in view), max(v.y / -v.z for v in view)
    recorded = {batch['frame']: batch for batch in cloud_data['batches']}
    rng = random.Random(481)
    voxels, full_voxels = set(), set()
    batch_reports, full_batch_reports, redirected_examples = [], [], []
    original_to_rounded_errors, redirected_errors = [], []
    accepted_occlusion_count = 0
    accepted_occlusion_max = 0.0
    occlusion_examples = []
    full_ray_hit_differences = 0
    full_ray_closer_hit_count = 0
    replay_points_all, full_points_all, stored_points_all = [], [], []
    origin_errors = []
    source_origins_inside_padded_volume = True
    ray_count = 0
    for frame in range(91, 481, 15):
        scene.frame_set(frame)
        bpy.context.view_layer.update()
        world = sensor.matrix_world.copy()
        origin = world.translation
        rotation = world.to_3x3()
        batch = recorded[frame]
        stored = batch['points']
        origin_errors.append((origin - Vector(batch['sensor_origin'])).length)
        source_origins_inside_padded_volume &= all(lo[axis] <= origin[axis] <= hi[axis]
                                                    for axis in range(3))
        points, full_points = [], []
        wanted = set(list(range(0, len(stored), max(1, len(stored) // 24)))[:24])
        batch_occlusions = 0
        for y in range(64):
            for x in range(112):
                nx = (x + rng.random()) / 112
                ny = (y + rng.random()) / 64
                local = Vector((xmin + nx * (xmax - xmin),
                                ymin + ny * (ymax - ymin), -1)).normalized()
                direction = (rotation @ local).normalized()
                ray_count += 1
                loc, normal, index, distance = sampling_bvh.ray_cast(origin, direction, 18)
                full_loc, full_normal, full_index, full_distance = full_bvh.ray_cast(origin, direction, 18)
                raw_difference = ((loc - full_loc).length if loc is not None and full_loc is not None else None)
                if ((loc is None) != (full_loc is None)
                        or (raw_difference is not None and raw_difference > .01)):
                    full_ray_hit_differences += 1
                if (loc is not None and full_loc is not None
                        and full_distance + .01 < distance):
                    full_ray_closer_hit_count += 1
                if full_loc is not None and full_distance >= .15:
                    full_key = tuple(round(value / .07) for value in full_loc)
                    if full_key not in full_voxels:
                        full_voxels.add(full_key)
                        full_points.append([round(value, 5) for value in full_loc])
                if loc is None or distance < .15:
                    continue
                key = tuple(round(value / .07) for value in loc)
                if key in voxels:
                    continue
                voxels.add(key)
                point_index = len(points)
                rounded = [round(value, 5) for value in loc]
                points.append(rounded)
                blocked = (full_loc is not None and full_distance + .01 < distance)
                if blocked:
                    accepted_occlusion_count += 1
                    batch_occlusions += 1
                    accepted_occlusion_max = max(accepted_occlusion_max, distance - full_distance)
                    if len(occlusion_examples) < 12:
                        occlusion_examples.append({'frame': frame, 'point_index': point_index,
                            'pixel': [x, y], 'recorded_sampler_hit': rounded,
                            'sampler_hit_object': scanner_hit_object(index),
                            'full_environment_hit': list(full_loc),
                            'sampler_hit_distance_m': distance,
                            'full_environment_hit_distance_m': full_distance,
                            'distance_gap_m': distance - full_distance})
                if point_index in wanted:
                    target = Vector(rounded)
                    original_to_rounded_errors.append((loc - target).length)
                    redirected = (target - origin).normalized()
                    redirect_loc, _, redirect_index, _ = sampling_bvh.ray_cast(origin, redirected, 18)
                    redirect_error = (redirect_loc - target).length if redirect_loc is not None else None
                    redirected_errors.append(redirect_error)
                    if redirect_error is not None and redirect_error > .01 and len(redirected_examples) < 12:
                        redirected_examples.append({'frame': frame, 'point_index': point_index,
                            'pixel': [x, y], 'stored_point': rounded,
                            'original_ray_hit': list(loc),
                            'original_hit_object': scanner_hit_object(index),
                            'rounded_target_ray_hit': list(redirect_loc),
                            'rounded_target_hit_object': scanner_hit_object(redirect_index),
                            'rounded_ray_endpoint_error_m': redirect_error})
        exact = points == stored
        mismatches = [(index, point, saved) for index, (point, saved)
                      in enumerate(zip(points, stored)) if point != saved]
        batch_reports.append({'frame': frame, 'stored_points': len(stored),
            'replayed_points': len(points), 'identical_point_list': exact,
            'coordinate_mismatches_at_same_index': len(mismatches),
            'first_mismatches': [{'index': index, 'replayed': point, 'stored': saved}
                                  for index, point, saved in mismatches[:3]],
            'original_ray_accepted_points_occluded_by_full_render_environment': batch_occlusions})
        full_batch_reports.append({'frame': frame, 'stored_points': len(stored),
            'replayed_points': len(full_points), 'identical_point_list': full_points == stored})
        replay_points_all.extend(points)
        full_points_all.extend(full_points)
        stored_points_all.extend(stored)
        print('EXACT_SCAN_AUDIT', frame, len(points), len(stored), exact,
              'full_environment', len(full_points), 'accepted_occlusions', batch_occlusions,
              flush=True)

    match = all(batch['identical_point_list'] for batch in batch_reports)
    redirected_numeric = [error for error in redirected_errors if error is not None]
    stored_set = set(map(tuple, stored_points_all))
    full_set = set(map(tuple, full_points_all))
    return {
        'seed': 481, 'image_rays_per_batch': [112, 64],
        'frames': list(range(91, 481, 15)), 'rays_replayed': ray_count,
        'sampling_geometry_scope': sampling_scope,
        'source_algorithm': 'complete_understory.py: original jittered ray direction, 18m range, reject <0.15m, unrounded 7cm voxel de-duplication, round coordinates to 5 decimals',
        'sampler_bvh': {'objects': len(objects), 'vertices': vertex_count,
            'triangles': triangle_count, 'stored_triangle_count': cloud_data.get('sampled_mesh_triangles'),
            'padded_sensor_volume_min': list(lo), 'padded_sensor_volume_max': list(hi),
            'whole_object_aabb_max_distance_m': 18,
            'actual_sensor_origins_inside_padded_volume': source_origins_inside_padded_volume},
        'original_sampler_reproduction': {'stored_point_count': len(stored_points_all),
            'replayed_point_count': len(replay_points_all),
            'all_batches_and_points_identical': match,
            'maximum_saved_sensor_origin_error_m': max(origin_errors),
            'batches': batch_reports},
        'render_geometry_scope_audit': {
            'potentially_reachable_render_meshes_excluded_by_original_scanner': [obj.name for obj in excluded_reachable],
            'excluded_reachable_mesh_count': len(excluded_reachable),
            'all_render_visible_environment_replay_point_count': len(full_points_all),
            'all_render_visible_environment_matches_recorded_point_lists': all(batch['identical_point_list'] for batch in full_batch_reports),
            'recorded_rounded_points_not_present_in_full_environment_replay': len(stored_set - full_set),
            'additional_rounded_points_in_full_environment_replay': len(full_set - stored_set),
            'all_original_rays_with_hit_difference_over_1cm_or_hit_presence_change': full_ray_hit_differences,
            'all_original_rays_hitting_a_closer_full_environment_surface_over_1cm': full_ray_closer_hit_count,
            'accepted_recorded_sampler_points_original_ray_occluded_over_1cm': accepted_occlusion_count,
            'accepted_original_ray_occlusion_max_distance_gap_m': accepted_occlusion_max,
            'examples': occlusion_examples, 'batches': full_batch_reports},
        'five_decimal_endpoint_reprojection_audit': {
            'sample_count': len(redirected_errors),
            'original_hit_to_rounded_coordinate_max_error_m': max(original_to_rounded_errors, default=0),
            'recast_toward_rounded_endpoint_error_over_1cm_count': sum(error > .01 for error in redirected_numeric),
            'recast_toward_rounded_endpoint_max_error_m': max(redirected_numeric, default=None),
            'recast_toward_rounded_endpoint_no_hit_count': sum(error is None for error in redirected_errors),
            'endpoint_rounding_can_change_first_hit_within_original_sampler': match and any(error > .01 for error in redirected_numeric),
            'examples': redirected_examples},
        'interpretation': 'Original-scanner reproduction, full rendered-geometry visibility, and rounded-endpoint reprojection are separate checks. Successful reproduction alone does not prove all rendered objects participated in sampling.'
    }
