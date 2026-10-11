"""Build the shared scene, trajectory cameras, rotor rig and synthetic scans."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import sys
import json
import math
import random
import bpy
from mathutils import Vector, Quaternion, Matrix
from mathutils.bvhtree import BVHTree

from common import PROJECT_ROOT, OUTPUT_ROOT
ROOT = OUTPUT_ROOT
BASE = OUTPUT_ROOT
from woodland_environment_v2 import build_environment, apply_pbr_textures
from sensor_rig import configure_sensor

bpy.ops.wm.open_mainfile(filepath=str(BASE / 'showcase.blend'))
scene = bpy.context.scene
scene.name = 'Woodland'
drone = bpy.data.objects['Drone']
alignment = bpy.data.objects['Model alignment']
base_scale = drone.scale.x
base_location = drone.location.copy()
drone.scale = (1, 1, 1)
drone.location = (0, 0, 0)
alignment.scale = (base_scale,) * 3
alignment.location = base_location
bpy.context.view_layer.update()

# Move the static model normalization below the animated body origin.
model_objects = [o for o in scene.objects if o.type == 'MESH' and not o.name.startswith('ENV_')
                 and o.name not in ('Studio floor', 'Display plinth', 'Plinth luminous rim')]
for obj in list(scene.objects):
    if obj.type == 'LIGHT' or obj.name in ('Studio floor', 'Display plinth', 'Plinth luminous rim'):
        bpy.data.objects.remove(obj, do_unlink=True)
camera = scene.camera
camera.name = 'BeautyCamera'
camera.data.dof.use_dof = True
camera.data.dof.aperture_fstop = 5.6
camera.data.clip_start = .01

audit = json.loads((BASE / 'structure-audit.json').read_text(encoding='utf-8'))
rotors = []
for i, rotor in enumerate(audit['rotors']):
    pivot = bpy.data.objects.new('Rotor_' + str(i), None)
    scene.collection.objects.link(pivot)
    # The audit centres are in unscaled Blender world coordinates.
    raw = Vector(rotor['blender_pivot'])
    centre = alignment.matrix_world @ raw
    axis = (alignment.matrix_world.to_3x3() @ Vector(rotor['blender_axis_up'])).normalized()
    pivot.matrix_world = Matrix.Translation(centre) @ axis.to_track_quat('Z', 'Y').to_matrix().to_4x4()
    world = pivot.matrix_world.copy()
    pivot.parent = drone
    pivot.matrix_world = world
    for name in rotor['blade_names']:
        blade = bpy.data.objects.get(name)
        if blade is None:
            raise RuntimeError('Missing blade ' + name)
        mat = blade.matrix_world.copy()
        blade.parent = pivot
        blade.matrix_world = mat
    pivot.rotation_mode = 'XYZ'
    # Local XYZ rotation keeps the pivot Z aligned to the source motor axis.
    rotors.append((pivot, pivot.rotation_euler.copy(), 1 if i in (0,3) else -1))

environment = build_environment(scene)
realism = PROJECT_ROOT / 'assets/realism'
apply_pbr_textures(environment, {
    'ground': {key: str(realism / 'forest_floor' / ('forest_floor_' + suffix + '_2k.png'))
               for key,suffix in [('albedo','diff'),('roughness','rough'),('normal','nor_gl')]},
    'bark': {key: str(realism / 'bark_brown_01' / ('bark_brown_01_' + suffix + '_2k.png'))
             for key,suffix in [('albedo','diff'),('roughness','rough'),('normal','nor_gl')]}
}, {'ground':2.14,'bark':1.0})
for mat in bpy.data.materials:
    if mat.name=='PBR' and mat.use_nodes:
        shader=mat.node_tree.nodes.get('Principled BSDF')
        shader.inputs['Base Color'].default_value=(.006,.018,.026,1)
        shader.inputs['Alpha'].default_value=1
        shader.inputs['Metallic'].default_value=.55
        shader.inputs['Roughness'].default_value=.075
        shader.inputs['Coat Weight'].default_value=.45
glass=bpy.data.materials['PBR'].copy()
glass.name='Optical lens coating'
shader=glass.node_tree.nodes.get('Principled BSDF')
shader.inputs['Metallic'].default_value=0
shader.inputs['Roughness'].default_value=.25
shader.inputs['Coat Weight'].default_value=0
shader.inputs['Coat Roughness'].default_value=.18
shader.inputs['Specular IOR Level'].default_value=0
bpy.data.objects['98'].data.materials.clear()
bpy.data.objects['98'].data.materials.append(glass)
bpy.context.view_layer.update()
scene.world = bpy.data.worlds.new('Morning sky')
scene.world.use_nodes = True
nodes, links = scene.world.node_tree.nodes, scene.world.node_tree.links
sky = nodes.new('ShaderNodeTexEnvironment')
sky.image = bpy.data.images.load(str(realism/'rainforest_trail/rainforest_trail_2k.hdr'), check_existing=True)
mapping = nodes.new('ShaderNodeMapping')
coord = nodes.new('ShaderNodeTexCoord')
mapping.inputs['Rotation'].default_value[2] = math.radians(75)
links.new(coord.outputs['Generated'], mapping.inputs['Vector'])
links.new(mapping.outputs['Vector'], sky.inputs['Vector'])
links.new(sky.outputs['Color'], nodes.get('Background').inputs['Color'])
nodes.get('Background').inputs['Strength'].default_value = .24

def aim(obj, target):
    obj.rotation_euler = (Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()

sun_data = bpy.data.lights.new('Morning sun', 'SUN')
sun_data.energy = 1.65
sun_data.angle = math.radians(.8)
sun_data.color = (1, .83, .62)
sun = bpy.data.objects.new('Morning sun', sun_data)
scene.collection.objects.link(sun)
sun.rotation_euler = (math.radians(53), math.radians(-18), math.radians(-35))
fill_data = bpy.data.lights.new('Camera softbox', 'AREA')
fill_data.energy = 25
fill_data.shape = 'DISK'
fill_data.size = 3
fill_data.color = (.78, .88, 1)
fill = bpy.data.objects.new('Camera softbox', fill_data)
scene.collection.objects.link(fill)
fill.visible_glossy = False
fill.location = (1, -3, 4)
aim(fill, (0,0,.8))

focus = bpy.data.objects.new('Camera focus', None)
scene.collection.objects.link(focus)
camera.data.dof.focus_object = focus
fpv, sensor_config = configure_sensor(scene, drone)
(ROOT/'sensor-config.json').write_text(json.dumps(sensor_config, indent=2), encoding='utf-8')
view = fpv.data.view_frame(scene=scene)
xmin,xmax=min(p.x/-p.z for p in view),max(p.x/-p.z for p in view)
ymin,ymax=min(p.y/-p.z for p in view),max(p.y/-p.z for p in view)
animation = json.loads((ROOT / 'animation-data.json').read_text(encoding='utf-8'))
ground_z = animation['metadata']['coordinates']['ground_z']
route_translation = Vector((-1,-1,.012-ground_z))

def smooth(a,b,u):
    u = max(0,min(1,u)); u = u*u*(3-2*u)
    return Vector(a).lerp(Vector(b),u)

for row in animation['frames']:
    f, t = row['frame'], row['t_s']
    qx,qy,qz,qw = row['orientation_inspire_xyzw']
    q = Quaternion((qw,qx,qy,qz))
    pos = Vector(row['position_enu']) + route_translation
    drone.location = pos
    drone.rotation_mode = 'QUATERNION'
    drone.rotation_quaternion = q
    drone.keyframe_insert('location', frame=f)
    drone.keyframe_insert('rotation_quaternion', frame=f)
    if t < 3:
        offset = smooth((.39,-.72,.22),(.72,-1.3,.48), t/3)
        target = smooth((0,-.15,.13),(0,0,.22),t/3)
        lens = 57 - 7*t/3
    elif t < 6:
        offset = smooth((.72,-1.3,.48),(1.4,-2.35,.70),(t-3)/3)
        target = Vector((0,0,.22)); lens=50-7*(t-3)/3
    elif t < 16:
        u=(t-6)/10
        close=Vector((.78,-1.24,.44))
        wide=Vector((1.4,-2.35,.70))
        offset=wide.lerp(close, min(1,(t-6)/1.4)**2*(3-2*min(1,(t-6)/1.4)))
        if t >= 14: offset=smooth(close,(1.4,-2.35,.85),(t-14)/2)
        target=Vector((0,0,.22)); lens=43
    else:
        theta=(t-16)/4*math.radians(80)
        offset=Vector((1.4*math.cos(theta)-2.35*math.sin(theta),
                       -1.4*math.sin(theta)-2.35*math.cos(theta),.85))
        target=Vector((0,0,.22)); lens=43+3*(t-16)/4
    focus.location = pos + q @ target
    focus.keyframe_insert('location', frame=f)
    camera.location = pos + q @ offset
    aim(camera, focus.location)
    camera.data.lens = lens
    camera.data.shift_x = .075 * max(0,min(1,(14-t)/.2)) if 6 <= t < 14 else 0
    camera.keyframe_insert('location', frame=f)
    camera.keyframe_insert('rotation_euler', frame=f)
    camera.data.keyframe_insert('lens', frame=f)
    camera.data.keyframe_insert('shift_x', frame=f)
    # Display rotation, not recorded motor RPM. Ramp up during the reveal.
    speed=0 if t<1.5 else min(1,(t-1.5)/1.5)
    phase=0 if t<1.5 else 90*max(0,t-1.5)**2/3 if t<3 else 67.5+90*(t-3)
    for pivot, rotation, direction in rotors:
        pivot.rotation_euler=rotation.copy()
        pivot.rotation_euler.z += direction*phase
        pivot.keyframe_insert('rotation_euler',frame=f)

for owner in list(scene.objects)+[camera.data]:
    if owner.animation_data and owner.animation_data.action:
        for curve in owner.animation_data.action.fcurves:
            for point in curve.keyframe_points:
                point.interpolation='LINEAR'

scene.render.engine='CYCLES'
scene.cycles.samples=64
scene.cycles.use_denoising=True
scene.cycles.device='GPU'
scene.cycles.use_adaptive_sampling=True
scene.cycles.adaptive_threshold=.018
scene.cycles.max_bounces=6
scene.cycles.diffuse_bounces=3
scene.cycles.glossy_bounces=3
scene.cycles.transparent_max_bounces=8
scene.cycles.denoiser='OPENIMAGEDENOISE'
scene.render.use_motion_blur=True
scene.render.motion_blur_shutter=.45
scene.render.resolution_x=1920
scene.render.resolution_y=1080
scene.render.resolution_percentage=100
scene.render.fps=30
scene.render.image_settings.file_format='JPEG'
scene.render.image_settings.quality=95
scene.render.use_file_extension=True
scene.render.use_persistent_data=True
scene.view_settings.view_transform='AgX'
scene.view_settings.look='AgX - Medium High Contrast'
scene.view_settings.exposure=0
scene.frame_start=1
scene.frame_end=600
scene.camera=camera
scene.frame_set(1)
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT / 'woodland.blend'))

