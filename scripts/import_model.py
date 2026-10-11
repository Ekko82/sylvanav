
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import bpy
import json
import math
from mathutils import Vector

from common import PROJECT_ROOT, OUTPUT_ROOT, seed_data
ROOT = OUTPUT_ROOT
seed_data()
MODEL = next((PROJECT_ROOT / 'assets/source').glob('*.glb'))
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=str(MODEL))
scene = bpy.context.scene
objects = list(scene.objects)
meshes = [o for o in objects if o.type == 'MESH']
alignment = bpy.data.objects.new('Model alignment', None)
scene.collection.objects.link(alignment)
for obj in objects:
    if obj.parent is None:
        mat = obj.matrix_world.copy()
        obj.parent = alignment
        obj.matrix_world = mat
alignment.rotation_euler.x = math.radians(-21.1)
bpy.context.view_layer.update()
points = [o.matrix_world @ Vector(c) for o in meshes for c in o.bound_box]
lo = Vector(tuple(min(p[i] for p in points) for i in range(3)))
hi = Vector(tuple(max(p[i] for p in points) for i in range(3)))
center = (lo + hi) / 2
scale = 0.92 / max(hi.x - lo.x, hi.y - lo.y)
drone = bpy.data.objects.new('Drone', None)
scene.collection.objects.link(drone)
alignment.parent = drone
drone.scale = (scale,) * 3
drone.location = (-center.x * scale, -center.y * scale, -lo.z * scale + 0.065)
bpy.context.view_layer.update()

report = {'original_bounds': [list(lo), list(hi)], 'scale': scale,
          'objects': [{'name': o.name, 'type': o.type, 'parent': o.parent.name if o.parent else None,
                       'location': list(o.matrix_world.translation),
                       'dimensions': list(o.dimensions), 'verts': len(o.data.vertices) if o.type == 'MESH' else None}
                      for o in objects]}
(ROOT / 'blender-import.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

def material(name, color, metal=0, rough=.4, emission=None):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Base Color'].default_value = (*color, 1)
    shader.inputs['Metallic'].default_value = metal
    shader.inputs['Roughness'].default_value = rough
    if emission:
        shader.inputs['Emission Color'].default_value = (*emission, 1)
        shader.inputs['Emission Strength'].default_value = 2.0
    return mat

floor_mat = material('Studio graphite', (.022, .034, .044), .5, .36)
metal_mat = material('Plinth', (.025, .043, .052), .65, .32)
line_mat = material('Cyan light', (.08, .38, .46), .4, .3, (.04, .36, .46))
bpy.ops.mesh.primitive_plane_add(size=200, location=(0,0,-.10))
bpy.context.object.name='Studio floor'
bpy.context.object.data.materials.append(floor_mat)
bpy.ops.mesh.primitive_cylinder_add(vertices=128, radius=.67, depth=.12, location=(0,0,-.045))
stage = bpy.context.object
stage.name = 'Display plinth'
stage.data.materials.append(metal_mat)
bevel = stage.modifiers.new('Edge bevel','BEVEL')
bevel.width=.015
bevel.segments=3
stage.modifiers.new('Weighted normals','WEIGHTED_NORMAL')
bpy.ops.mesh.primitive_torus_add(major_radius=.625,minor_radius=.0025,major_segments=128,minor_segments=8,location=(0,0,.019))
bpy.context.object.name='Plinth luminous rim'
bpy.context.object.data.materials.append(line_mat)

def aim(obj,target):
    obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()

def area(name, loc, power, color, size, target=(0,0,.3), size_y=None):
    data=bpy.data.lights.new(name, 'AREA')
    data.energy=power
    data.color=color
    data.shape='RECTANGLE'
    data.size=size
    data.size_y=size_y or size
    obj=bpy.data.objects.new(name,data)
    scene.collection.objects.link(obj)
    obj.location=loc
    aim(obj,target)
    return obj

area('White key', (1,-1.4,2), 180, (.78,.9,1), 1.5)
area('Soft fill', (-1.2,-.4,.8), 70, (.32,.65,1), 1.2)
area('Rim', (0,1.4,1.7), 230, (.45,.8,1), 1.1, size_y=.2)
area('Warm edge', (1.4,.4,.6), 55, (1,.6,.27), .7, size_y=.2)
scene.world=bpy.data.worlds.new('Dark studio')
scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.025,.032,.04,1)
scene.world.node_tree.nodes['Background'].inputs[1].default_value=.35
camera_data=bpy.data.cameras.new('Camera')
camera=bpy.data.objects.new('Camera',camera_data)
scene.collection.objects.link(camera)
camera.location=(1.1,-1.45,.95)
aim(camera,(0,0,.27))
camera_data.lens=52
camera_data.sensor_width=36
scene.camera=camera
scene.render.engine='CYCLES'
scene.cycles.samples=32
scene.cycles.use_denoising=True
scene.cycles.device='GPU'
scene.cycles.device='CPU'
scene.cycles.denoiser='OPENIMAGEDENOISE'
scene.render.resolution_x=1080
scene.render.resolution_y=1440
scene.render.resolution_percentage=70
scene.render.image_settings.file_format='PNG'
scene.render.film_transparent=False
scene.render.fps=30
scene.frame_start=1
scene.frame_end=600
scene.view_settings.view_transform='AgX'
scene.view_settings.look='AgX - Medium High Contrast'
scene.view_settings.exposure=.2
scene.render.filepath=str(ROOT/'preview.png')
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'showcase.blend'))

