"""Improve subject exposure with a localized follow light; retain sensor geometry."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import bpy

from common import OUTPUT_ROOT, PROJECT_ROOT
root = OUTPUT_ROOT
bpy.ops.wm.open_mainfile(filepath=str(root/'woodland.blend'))
scene = bpy.context.scene
scene.world.node_tree.nodes.get('Background').inputs['Strength'].default_value=.55
bpy.data.lights['Morning sun'].energy=1.3
fill=bpy.data.objects['Camera softbox']
fill.parent=bpy.data.objects['BeautyCamera']
fill.location=(.45,.55,.3)
fill.rotation_euler=(0,0,0)
fill.data.energy=125
fill.data.size=2.4
fill.data.color=(.88,.94,1.)
fill.visible_glossy=False
scene.view_settings.exposure=.35
bpy.ops.wm.save_as_mainfile(filepath=str(root/'woodland.blend'))
