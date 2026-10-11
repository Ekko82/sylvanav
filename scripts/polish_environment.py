"""Blend scanned summer ground assets and use a leafy woodland HDRI."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import math
import json
import bpy

from common import OUTPUT_ROOT, PROJECT_ROOT
root = OUTPUT_ROOT
assets = PROJECT_ROOT/'assets/realism'
bpy.ops.wm.open_mainfile(filepath=str(root/'woodland.blend'))
scene = bpy.context.scene
material = bpy.data.materials['ENV2_ForestFloor']
nodes, links = material.node_tree.nodes, material.node_tree.links
nodes.clear()
out = nodes.new('ShaderNodeOutputMaterial')
shader = nodes.new('ShaderNodeBsdfPrincipled')
shader.inputs['Specular IOR Level'].default_value = .25
links.new(shader.outputs['BSDF'], out.inputs['Surface'])
coord = nodes.new('ShaderNodeTexCoord')
patches = nodes.new('ShaderNodeTexNoise')
patches.inputs['Scale'].default_value = .24
patches.inputs['Detail'].default_value = 3
patches.inputs['Roughness'].default_value = .72
links.new(coord.outputs['UV'], patches.inputs['Vector'])
ramp = nodes.new('ShaderNodeValToRGB')
ramp.color_ramp.elements[0].position = .29
ramp.color_ramp.elements[1].position = .52
links.new(patches.outputs['Fac'], ramp.inputs['Fac'])

layers = []
for identifier, tile, extension in [('forest_floor',2.14,'png'),('leafy_grass',2.0,'jpg')]:
    mapping = nodes.new('ShaderNodeVectorMath')
    mapping.operation = 'MULTIPLY'
    mapping.inputs[1].default_value = (1/tile,1/tile,1/tile)
    links.new(coord.outputs['UV'], mapping.inputs[0])
    outputs = {}
    for channel, suffix, ext in [('color','diff',extension),('roughness','rough','png'),('normal','nor_gl','png')]:
        tex = nodes.new('ShaderNodeTexImage')
        tex.label = identifier+' '+channel
        tex.image = bpy.data.images.load(str(assets/identifier/(identifier+'_'+suffix+'_2k.'+ext)),check_existing=True)
        tex.image.colorspace_settings.name = 'sRGB' if channel=='color' else 'Non-Color'
        tex.extension = 'REPEAT'
        links.new(mapping.outputs[0], tex.inputs['Vector'])
        outputs[channel] = tex.outputs['Color']
    layers.append(outputs)
for channel, input_name in [('color','Base Color'),('roughness','Roughness'),('normal',None)]:
    mix = nodes.new('ShaderNodeMixRGB')
    mix.blend_type = 'MIX'
    links.new(ramp.outputs['Color'], mix.inputs[0])
    links.new(layers[0][channel], mix.inputs[1])
    links.new(layers[1][channel], mix.inputs[2])
    if input_name:
        links.new(mix.outputs[0], shader.inputs[input_name])
    else:
        normal = nodes.new('ShaderNodeNormalMap')
        normal.uv_map = 'Surface metres'
        normal.inputs['Strength'].default_value = .8
        links.new(mix.outputs[0], normal.inputs['Color'])
        links.new(normal.outputs['Normal'], shader.inputs['Normal'])
world_nodes = scene.world.node_tree.nodes
for node in world_nodes:
    if node.type=='TEX_ENVIRONMENT':
        node.image=bpy.data.images.load(str(assets/'rainforest_trail/rainforest_trail_2k.hdr'),check_existing=True)
    elif node.type=='MAPPING':
        node.inputs['Rotation'].default_value[2]=math.radians(75)
world_nodes.get('Background').inputs['Strength'].default_value=.55
scene.frame_set(1)
scene.camera=bpy.data.objects['BeautyCamera']
scene.render.resolution_percentage=100
scene['revision']='2: calibrated physical lens, forward FPV, scanned green forest ground'
bpy.ops.wm.save_as_mainfile(filepath=str(root/'woodland.blend'))
(root/'environment-revision.json').write_text(json.dumps({
    'geometry_changed_since_pointcloud':False,
    'ground':['forest_floor','leafy_grass'],
    'ground_tile_m':[2.14,2],
    'ground_blend':'Noise patch mask in physical UV metres; fine normal maps affect shading only',
    'bark':'bark_brown_01',
    'world':'rainforest_trail',
    'world_rotation_degrees':75,
    'world_strength':.55,
    'exposure':.35
},indent=2),encoding='utf-8')

