"""Read-only audit of saved scenes and actual environment visibility."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import json, math, sys, hashlib
import bpy
import numpy as np
from mathutils import Vector, Quaternion
from mathutils.bvhtree import BVHTree
from bpy_extras.object_utils import world_to_camera_view

from common import OUTPUT_ROOT, PROJECT_ROOT
ROOT = OUTPUT_ROOT
sys.path.insert(0, str(ROOT.parent))
sys.path.insert(0, str(ROOT))
from sensor_rig import _lens_geometry
from exact_scan_replay import replay_scan
animation = json.loads((ROOT / 'animation-data.json').read_text(encoding='utf-8'))
cloud_data = json.loads((ROOT / 'pointcloud-data.json').read_text(encoding='utf-8'))
config_file = json.loads((ROOT / 'sensor-config.json').read_text(encoding='utf-8'))
config = cloud_data['sensor_config']
rows = {row['frame']: row for row in animation['frames']}
translation = Vector(cloud_data['route_translation'])

def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()

audit_source_names=['woodland.blend','cloud.blend','animation-data.json','pointcloud-data.json','sensor-config.json']
audit_source_hashes={name:file_hash(ROOT/name) for name in audit_source_names}
def quaternion(row, key='orientation_inspire_xyzw'):
    x,y,z,w = row[key]
    return Quaternion((w,x,y,z)).normalized()
def angle_degrees(a, b):
    return math.degrees(math.acos(max(-1., min(1., a.normalized().dot(b.normalized())))))
def owner_drone(obj):
    while obj:
        if obj.name == 'Drone': return True
        obj = obj.parent
    return False

report = {'files_read': ['woodland.blend','cloud.blend','animation-data.json',
                        'pointcloud-data.json','sensor-config.json'],
          'formal_scenes_modified': False,
          'sensor_configs_identical': config == config_file,
          'method': 'saved Blender poses; directed lens PCA normal from rear barrel 102 to aperture 98; actual ENV2 mesh BVH; geometric visibility rays'}
bpy.ops.wm.open_mainfile(filepath=str(ROOT / 'woodland.blend'))
scene = bpy.context.scene
bpy.context.view_layer.update()
drone = bpy.data.objects['Drone']
sensor = bpy.data.objects['SensorCamera']
beauty = bpy.data.objects['BeautyCamera']
focus = bpy.data.objects['Camera focus']
environment_objects = [obj for obj in scene.objects if obj.type == 'MESH' and not obj.hide_render
                       and not any(col.hide_render for col in obj.users_collection)
                       and (obj.name.startswith('ENV2_') or any(col.name.startswith('ENV2_') for col in obj.users_collection))
                       and not any(col.name.startswith('ENV2_Prototypes') for col in obj.users_collection)]
model_objects = [obj for obj in scene.objects if obj.type == 'MESH' and owner_drone(obj)]
model_vertex_cache={obj.data.name:np.asarray([(*v.co,1.) for v in obj.data.vertices],dtype=np.float64)
                    for obj in model_objects}
propeller_names={'63','63_2','63_3','63_4','64','64_2','64_3','64_4'}
vertices, triangles = [], []
for obj in environment_objects:
    mesh = obj.data
    mesh.calc_loop_triangles()
    start = len(vertices)
    vertices.extend([obj.matrix_world @ v.co for v in mesh.vertices])
    triangles.extend([tuple(start + index for index in tri.vertices) for tri in mesh.loop_triangles])
report['environment'] = {'objects': len(environment_objects), 'vertices': len(vertices),
                         'triangles': len(triangles),
                         'mesh_collections':sorted({col.name for obj in environment_objects for col in obj.users_collection}),
                         'scan_geometry_scope': 'all render-visible ENV2_ meshes or meshes in ENV2_ collections, including understory; hidden prototype meshes excluded; HDRI has no finite geometric depth'}
report['scene_flags']={'composition_polished':bool(scene.get('composition_polished',False))}
environment_bvh = BVHTree.FromPolygons(vertices, triangles, all_triangles=True)
del vertices, triangles
view = sensor.data.view_frame(scene=scene)
tx,ty = max(abs(p.x/p.z) for p in view), max(abs(p.y/p.z) for p in view)
report['fov'] = {'render_size':[scene.render.resolution_x,scene.render.resolution_y],
                 'render_percentage':scene.render.resolution_percentage,
                 'sensor_lens_mm':sensor.data.lens,
                 'actual_tan_half_fov':[tx,ty],
                 'stored_tan_half_fov':config['tan_half_fov'],
                 'max_tan_difference':max(abs(tx-config['tan_half_fov'][0]),abs(ty-config['tan_half_fov'][1]))}
actual_poses = {}
lens_tests = []
def set_frame(frame):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    matrix = sensor.matrix_world.copy()
    actual_poses[frame] = (matrix.translation.copy(), matrix.to_quaternion())
    return matrix
test_frames = [1,90,170,181,220,241,255,270,285,300,340,360,391,450,490,540,590]
for frame in test_frames:
    matrix = set_frame(frame)
    lens_centre, lens_forward = _lens_geometry()
    camera_forward = (matrix.to_3x3() @ Vector((0,0,-1))).normalized()
    body_inverse = drone.matrix_world.inverted()
    lens_tests.append({'frame':frame,
                       'physical_lens_to_camera_forward_dot':float(lens_forward.dot(camera_forward)),
                       'physical_lens_to_camera_angle_deg':angle_degrees(lens_forward,camera_forward),
                       'lens_center_local_error_m':(body_inverse@lens_centre-Vector(config['physical_lens_center_local'])).length,
                       'sensor_local_position_error_m':(body_inverse@matrix.translation-Vector(config['local_position'])).length,
                       'drone_position_from_record_error_m':(drone.matrix_world.translation-Vector(rows[frame]['position_enu'])-translation).length,
                       'drone_quaternion_record_abs_dot':abs(drone.matrix_world.to_quaternion().normalized().dot(quaternion(rows[frame])))})
report['physical_lens_and_body_pose'] = lens_tests

# Check every FPV frame, not only the existing preview at frame 270.
fpv_rows=[]
for frame in range(241,301):
    matrix=set_frame(frame)
    row=rows[frame]
    velocity=(Vector(rows[frame+1]['position_enu'])-Vector(rows[frame-1]['position_enu']))*15
    body_forward=quaternion(row,'orientation_xyzw')@Vector((1,0,0))
    dot=velocity.normalized().dot(body_forward) if velocity.length>1e-6 else None
    near_hits=[]
    depsgraph=bpy.context.evaluated_depsgraph_get()
    for y in (-1,-.5,0,.5,1):
        for x in (-1,-.5,0,.5,1):
            ray=(matrix.to_3x3()@Vector((x*tx,y*ty,-1))).normalized()
            hit,location,normal,index,obj,matrix_hit=scene.ray_cast(depsgraph,matrix.translation,ray,distance=.20)
            if hit: near_hits.append({'object':obj.name,'distance_m':(location-matrix.translation).length,'is_drone':owner_drone(obj)})
    fpv_rows.append({'frame':frame,'velocity_body_forward_dot':float(dot) if dot is not None else None,
                     'display_speed_m_per_s':velocity.length,'near_frustum_hits_20cm':near_hits})
positions=[Vector(rows[f]['position_enu']) for f in range(241,301)]
report['fpv_241_300']={'encoded_duration_s':2.,'first_t_s':rows[241]['t_s'],
                      'last_t_s':rows[300]['t_s'],
                      'source_time_first_s':rows[241].get('source_time_s'),
                      'source_time_last_s':rows[300].get('source_time_s'),
                      'body_forward_dot_min':min(r['velocity_body_forward_dot'] for r in fpv_rows),
                      'body_forward_dot_mean':sum(r['velocity_body_forward_dot'] for r in fpv_rows)/60,
                      'frames_dot_gt_point5':sum(r['velocity_body_forward_dot']>.5 for r in fpv_rows),
                      'path_length_m':sum((b-a).length for a,b in zip(positions,positions[1:])),
                      'net_displacement_m':(positions[-1]-positions[0]).length,
                      'min_display_speed_m_per_s':min(r['display_speed_m_per_s'] for r in fpv_rows),
                      'near_frustum_hits_total':sum(len(r['near_frustum_hits_20cm']) for r in fpv_rows),
                      'frames':fpv_rows}

beauty_visibility=[]
fpv_visibility=[]
for frame in test_frames:
    matrix=set_frame(frame)
    origin=beauty.matrix_world.translation.copy()
    target=focus.matrix_world.translation.copy()
    distance=(target-origin).length
    hit,normal,index,hit_distance=environment_bvh.ray_cast(origin,(target-origin).normalized(),distance)
    visible_hit,visible_location,visible_normal,visible_index,visible_object,visible_matrix=scene.ray_cast(
        bpy.context.evaluated_depsgraph_get(),origin,(target-origin).normalized(),distance=distance)
    model_bounds=[]
    exact_projected=[]
    body_projected=[]
    projection=beauty.calc_matrix_camera(bpy.context.evaluated_depsgraph_get(),
                                        x=scene.render.resolution_x,y=scene.render.resolution_y,
                                        scale_x=scene.render.pixel_aspect_x,scale_y=scene.render.pixel_aspect_y)
    for obj in model_objects:
        model_bounds.extend(world_to_camera_view(scene,beauty,obj.matrix_world@Vector(corner)) for corner in obj.bound_box)
        clip_matrix=np.asarray(projection@beauty.matrix_world.inverted()@obj.matrix_world,dtype=np.float64)
        clips=model_vertex_cache[obj.data.name]@clip_matrix.T
        in_front=clips[:,3]>1e-8
        xy=clips[in_front,:2]/clips[in_front,3:4]*.5+.5
        exact_projected.append(xy)
        if obj.name not in propeller_names: body_projected.append(xy)
    exact_xy=np.concatenate(exact_projected)
    body_xy=np.concatenate(body_projected)
    beauty_visibility.append({'frame':frame,'camera_to_focus_distance_m':distance,
                              'environment_blocks_focus_ray':hit is not None,
                              'environment_distance_m':hit_distance,
                              'actual_first_visible_object':visible_object.name if visible_hit else None,
                              'actual_first_visible_object_is_drone':owner_drone(visible_object) if visible_hit else False,
                              'actual_first_visible_hit_distance_m':(visible_location-origin).length if visible_hit else None,
                              'model_projected_xy_bounds':[min(p.x for p in model_bounds),max(p.x for p in model_bounds),
                                                           min(p.y for p in model_bounds),max(p.y for p in model_bounds)],
                              'exact_mesh_projected_xy_bounds':[float(exact_xy[:,0].min()),float(exact_xy[:,0].max()),
                                                               float(exact_xy[:,1].min()),float(exact_xy[:,1].max())],
                              'exact_body_without_propellers_projected_xy_bounds':[float(body_xy[:,0].min()),float(body_xy[:,0].max()),
                                                                                   float(body_xy[:,1].min()),float(body_xy[:,1].max())],
                              'exact_body_vertices_outside_image':int(np.any((body_xy<0)|(body_xy>1),axis=1).sum()),
                              'exact_model_vertices_outside_image':int(np.any((exact_xy<0)|(exact_xy>1),axis=1).sum()),
                              'camera_clipping_start_m':beauty.data.clip_start})
    hits,sky,distances=0,0,[]
    for y in (-1,-.5,0,.5,1):
        for x in (-1,-.75,-.5,-.25,0,.25,.5,.75,1):
            ray=(matrix.to_3x3()@Vector((x*tx,y*ty,-1))).normalized()
            hit,normal,index,hit_distance=environment_bvh.ray_cast(matrix.translation,ray,18)
            if hit is None: sky+=1
            else: hits+=1; distances.append(hit_distance)
    center_ray=(matrix.to_3x3()@Vector((0,0,-1))).normalized()
    center_hit,normal,index,center_distance=environment_bvh.ray_cast(matrix.translation,center_ray,18)
    fpv_visibility.append({'frame':frame,'environment_hits_45_rays':hits,'no_finite_surface_within_18m':sky,
                           'nearest_environment_distance_m':min(distances) if distances else None,
                           'center_environment_distance_m':center_distance})
report['beauty_visibility']=beauty_visibility
report['fpv_environment_visibility']=fpv_visibility

# Check every pose in the 6–16 s tracking section against actual blade
# vertices, rather than using expanded per-mesh bounding boxes.
rotor_objects=[obj for obj in model_objects if obj.name in propeller_names]
rotor_bound_rows=[]
for frame in range(181,481):
    set_frame(frame)
    projection=beauty.calc_matrix_camera(bpy.context.evaluated_depsgraph_get(),
                                        x=scene.render.resolution_x,y=scene.render.resolution_y,
                                        scale_x=scene.render.pixel_aspect_x,scale_y=scene.render.pixel_aspect_y)
    projected=[]
    for obj in rotor_objects:
        clip_matrix=np.asarray(projection@beauty.matrix_world.inverted()@obj.matrix_world,dtype=np.float64)
        clips=model_vertex_cache[obj.data.name]@clip_matrix.T
        in_front=clips[:,3]>1e-8
        projected.append(clips[in_front,:2]/clips[in_front,3:4]*.5+.5)
    xy=np.concatenate(projected)
    rotor_bound_rows.append({'frame':frame,'displayed_as_beauty':not 241<=frame<=300,
                             'lens_mm':beauty.data.lens,'shift_x':beauty.data.shift_x,
                             'actual_blade_projected_xy_bounds':[float(xy[:,0].min()),float(xy[:,0].max()),
                                                                float(xy[:,1].min()),float(xy[:,1].max())],
                             'vertices_outside_image':int(np.any((xy<0)|(xy>1),axis=1).sum())})
displayed_rotor_rows=[row for row in rotor_bound_rows if row['displayed_as_beauty']]
clipped_rotor_rows=[row for row in displayed_rotor_rows if row['vertices_outside_image']>0]
report['full_tracking_rotor_bounds']={'frames_checked':[181,480],
                                    'frames_showing_fpv_instead_of_beauty':[241,300],
                                    'displayed_beauty_frames_checked':len(displayed_rotor_rows),
                                    'displayed_beauty_frames_with_blade_vertices_outside':len(clipped_rotor_rows),
                                    'clipped_frame_numbers':[row['frame'] for row in clipped_rotor_rows],
                                    'minimum_left_boundary':min(row['actual_blade_projected_xy_bounds'][0] for row in displayed_rotor_rows),
                                    'maximum_right_boundary':max(row['actual_blade_projected_xy_bounds'][1] for row in displayed_rotor_rows),
                                    'minimum_bottom_boundary':min(row['actual_blade_projected_xy_bounds'][2] for row in displayed_rotor_rows),
                                    'maximum_top_boundary':max(row['actual_blade_projected_xy_bounds'][3] for row in displayed_rotor_rows),
                                    'frames':rotor_bound_rows}

batch_tests=[]
for batch in cloud_data['batches']:
    frame=batch['frame']
    matrix=set_frame(frame)
    stored_origin=Vector(batch['sensor_origin'])
    stored_q=Quaternion(batch['sensor_quaternion'])
    q_inverse=stored_q.inverted()
    offsets=[Vector(p)-stored_origin for p in batch['points']]
    locals=[q_inverse@v for v in offsets]
    sampled_geometry=[]
    sampled_surface_distances=[]
    step=max(1,len(offsets)//24)
    for offset in offsets[::step][:24]:
        hit,normal,index,distance=environment_bvh.ray_cast(stored_origin,offset.normalized(),18)
        sampled_geometry.append((hit-(stored_origin+offset)).length if hit is not None else None)
        nearest,normal,index,surface_distance=environment_bvh.find_nearest(stored_origin+offset)
        sampled_surface_distances.append(surface_distance)
    batch_tests.append({'frame':frame,'points':len(offsets),
                        'rgb_origin_vs_batch_error_m':(matrix.translation-stored_origin).length,
                        'rgb_quaternion_vs_batch_abs_dot':abs(matrix.to_quaternion().normalized().dot(stored_q.normalized())),
                        'points_behind_sensor':sum(v.z>=0 for v in locals),
                        'points_outside_fov':sum(v.z>=0 or abs(v.x/-v.z)>tx+1e-4 or abs(v.y/-v.z)>ty+1e-4 for v in locals),
                        'max_point_range_m':max((v.length for v in offsets),default=0),
                        'sampled_geometry_first_hit_errors_m':sampled_geometry,
                        'sampled_geometry_nearest_surface_errors_m':sampled_surface_distances})
report['pointcloud_batches']=batch_tests
report['exact_scan_replay']=replay_scan(scene,sensor,cloud_data,environment_objects,environment_bvh)

# The independent cloud scene is evaluated against actual saved RGB poses.
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'cloud.blend'))
cloud_scene=bpy.context.scene
cloud_sensor=bpy.data.objects['Synchronized sensor pose']
cone=bpy.data.objects['Mounted camera view cone']
cone_points=[Vector(p.co[:3]) for spline in cone.data.splines for p in spline.points]
report['cloud_frustum_geometry']={'parent':cone.parent.name,
                                 'max_cone_vertex_x_per_depth':max(abs(p.x/-p.z) for p in cone_points if p.z<-.01),
                                 'max_cone_vertex_y_per_depth':max(abs(p.y/-p.z) for p in cone_points if p.z<-.01),
                                 'local_position':list(cone.location),
                                 'local_rotation_euler':list(cone.rotation_euler)}
cloud_pose_tests=[]
for frame,(origin,q) in sorted(actual_poses.items()):
    cloud_scene.frame_set(frame)
    bpy.context.view_layer.update()
    cloud_pose_tests.append({'frame':frame,'rgb_vs_cloud_origin_error_m':(origin-cloud_sensor.matrix_world.translation).length,
                             'rgb_vs_cloud_quaternion_abs_dot':abs(q.normalized().dot(cloud_sensor.matrix_world.to_quaternion().normalized()))})
report['cloud_sensor_pose']=cloud_pose_tests

# Verify that cloud.blend contains the new point data, not just the right
# sensor animation. Each saved batch uses six vertices per point octahedron.
cloud_geometry_tests=[]
for batch in cloud_data['batches']:
    frame=batch['frame']
    obj=bpy.data.objects.get('Scan batch frame %03d' % frame)
    test={'frame':frame,'expected_points':len(batch['points']),
          'saved_batch_object_exists':obj is not None}
    if obj is not None:
        coords=np.asarray([tuple(vertex.co) for vertex in obj.data.vertices],dtype=np.float64)
        expected=len(batch['points'])*6
        test.update({'saved_vertex_count':len(coords),'expected_vertex_count':expected,
                     'saved_polygon_count':len(obj.data.polygons),
                     'expected_polygon_count':len(batch['points'])*8,
                     'saved_point_count_property':obj.get('point_count')})
        if len(coords)==expected:
            centers=coords.reshape(-1,6,3).mean(axis=1)
            matrix=np.asarray(obj.matrix_world,dtype=np.float64)
            world_centers=np.column_stack((centers,np.ones(len(centers))))@matrix.T
            errors=np.linalg.norm(world_centers[:,:3]-np.asarray(batch['points']),axis=1)
            test['maximum_saved_point_center_error_m']=float(errors.max(initial=0))
            test['all_saved_point_centers_match_json_within_20um']=bool(np.all(errors<.00002))
        else:
            test['maximum_saved_point_center_error_m']=None
            test['all_saved_point_centers_match_json_within_20um']=False
        states={}
        for check_frame in (1,frame-1,frame,600):
            cloud_scene.frame_set(check_frame)
            bpy.context.view_layer.update()
            states[str(check_frame)]=bool(obj.hide_render)
        test['hide_render_at_checked_frames']=states
        test['reveals_on_batch_frame_and_remains_visible']=(states['1'] and states[str(frame-1)]
                                                          and not states[str(frame)] and not states['600'])
    cloud_geometry_tests.append(test)
report['saved_cloud_point_geometry']={'scene_point_count':cloud_scene.get('point_count'),
                                     'expected_point_count':sum(len(batch['points']) for batch in cloud_data['batches']),
                                     'batches':cloud_geometry_tests}
geometry_errors=[error for test in batch_tests for error in test['sampled_geometry_first_hit_errors_m'] if error is not None]
surface_errors=[error for test in batch_tests for error in test['sampled_geometry_nearest_surface_errors_m'] if error is not None]
report['summary']={'min_lens_camera_dot':min(test['physical_lens_to_camera_forward_dot'] for test in lens_tests),
                    'max_rgb_batch_origin_error_m':max(test['rgb_origin_vs_batch_error_m'] for test in batch_tests),
                    'min_rgb_batch_q_abs_dot':min(test['rgb_quaternion_vs_batch_abs_dot'] for test in batch_tests),
                    'pointcloud_points_behind_sensor':sum(test['points_behind_sensor'] for test in batch_tests),
                    'pointcloud_points_outside_fov':sum(test['points_outside_fov'] for test in batch_tests),
                    'max_rgb_cloud_origin_error_m':max(test['rgb_vs_cloud_origin_error_m'] for test in cloud_pose_tests),
                    'min_rgb_cloud_q_abs_dot':min(test['rgb_vs_cloud_quaternion_abs_dot'] for test in cloud_pose_tests),
                    'sampled_pointcloud_geometry_first_hit_max_error_m':max(geometry_errors,default=None),
                    'sampled_pointcloud_geometry_first_hit_median_error_m':float(np.median(geometry_errors)) if geometry_errors else None,
                    'sampled_pointcloud_geometry_first_hit_count_over_1cm':sum(error>.01 for error in geometry_errors),
                    'sampled_pointcloud_geometry_nearest_surface_max_error_m':max(surface_errors,default=None),
                    'sampled_beauty_frames_with_environment_focus_occlusion':[test['frame'] for test in beauty_visibility if test['environment_blocks_focus_ray']],
                    'sampled_beauty_frames_environment_first_visible':[test['frame'] for test in beauty_visibility if test['actual_first_visible_object'] and not test['actual_first_visible_object_is_drone']],
                    'sampled_beauty_frames_body_outside_image':[test['frame'] for test in beauty_visibility if test['exact_body_vertices_outside_image']>0],
                    'sampled_beauty_frames_only_propellers_outside_image':[test['frame'] for test in beauty_visibility if test['exact_body_vertices_outside_image']==0 and test['exact_model_vertices_outside_image']>0],
                    'fpv_forward_frames_above_point5':report['fpv_241_300']['frames_dot_gt_point5'],
                    'fpv_near_ray_hits':report['fpv_241_300']['near_frustum_hits_total']}
report['summary']['full_tracking_displayed_beauty_frames_with_blade_vertices_outside']=len(clipped_rotor_rows)
report['summary']['full_tracking_minimum_left_rotor_boundary']=report['full_tracking_rotor_bounds']['minimum_left_boundary']
exact_replay=report['exact_scan_replay']
report['summary']['exact_original_scan_all_points_match']=exact_replay['original_sampler_reproduction']['all_batches_and_points_identical']
report['summary']['exact_original_scan_replayed_points']=exact_replay['original_sampler_reproduction']['replayed_point_count']
report['summary']['reachable_visible_meshes_omitted_from_original_scanner']=exact_replay['render_geometry_scope_audit']['excluded_reachable_mesh_count']
report['summary']['accepted_recorded_points_occluded_on_original_rays']=exact_replay['render_geometry_scope_audit']['accepted_recorded_sampler_points_original_ray_occluded_over_1cm']
report['summary']['full_render_visible_environment_replay_matches_recorded']=exact_replay['render_geometry_scope_audit']['all_render_visible_environment_matches_recorded_point_lists']
report['summary']['rounded_endpoint_recast_is_not_authoritative']=exact_replay['five_decimal_endpoint_reprojection_audit']['endpoint_rounding_can_change_first_hit_within_original_sampler']
report['summary']['cloud_saved_point_geometry_matches_json']=all(
    test.get('all_saved_point_centers_match_json_within_20um',False)
    and test['saved_polygon_count']==test['expected_polygon_count']
    and test['saved_point_count_property']==test['expected_points'] for test in cloud_geometry_tests)
report['summary']['cloud_saved_batches_reveal_on_sampling_frame']=all(
    test.get('reveals_on_batch_frame_and_remains_visible',False) for test in cloud_geometry_tests)
report['limitations']=['Point cloud is idealized synthetic ray casting of virtual meshes, not real SLAM or obstacle avoidance.',
                      'HDRI/background has no finite geometric depth and is not included in point cloud.',
                      '25-ray near-field test per FPV frame and focus rays in selected Beauty frames cannot prove every pixel is clear.',
                      'Projected model bounds are conservative object bounding boxes; edge overlap is not exact silhouette clipping.',
                      'Exact mesh vertex bounds describe integer-frame poses, not the complete motion-blur shutter sweep. Opening frame 1 is an intentional camera close-up; Beauty bounds during 241–300 are not the displayed FPV image.',
                      'Scene frame_set was used only in memory; neither formal .blend was saved.']
report['limitations'].append('Five-decimal endpoints do not preserve the original jittered ray direction. Endpoint re-projection is reported separately from exact RNG replay. Full render-visible geometry membership is audited independently of original sampler reproduction.')
report['source_file_integrity']={name:{'sha256_before':value,'sha256_after':file_hash(ROOT/name)}
                                for name,value in audit_source_hashes.items()}
report['source_files_unchanged_during_audit']=all(record['sha256_before']==record['sha256_after']
                                              for record in report['source_file_integrity'].values())
report['existing_preview_visual_review']={
    'beauty_files':['frame_0001.jpg','frame_0170.jpg','frame_0220.jpg','frame_0340.jpg','frame_0450.jpg','frame_0540.jpg'],
    'fpv_files':['frame_0270.jpg'],
    'notes':['No visible lens or environment penetration in the inspected existing stills.',
             'Tree trunks overlap behind the drone silhouette at 170/220/450/540 and reduce separation; this is background overlap, not confirmed foreground obstruction.',
             'FPV frame 270 shows an open center passage with trees/shrubs at right; only this FPV still existed for the new two-second interval.',
             'Cloud preview is readable; empty lower navy band occupies about one fifth of image.']}
(ROOT/'direction-verification.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
print('DIRECTION_VERIFICATION_SUMMARY',json.dumps(report['summary']))
print('FPV_FORWARD',json.dumps({k:v for k,v in report['fpv_241_300'].items() if k!='frames'}))
