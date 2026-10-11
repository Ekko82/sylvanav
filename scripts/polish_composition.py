"""Keep all rotors in tracking shots without changing the sensor camera."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import bpy

from common import OUTPUT_ROOT, PROJECT_ROOT
root=OUTPUT_ROOT
bpy.ops.wm.open_mainfile(filepath=str(root/'woodland.blend'))
scene=bpy.context.scene
camera=bpy.data.objects['BeautyCamera']
def ease(x):
    x=max(0,min(1,x)); return x*x*(3-2*x)
previous_strength=scene.get('composition_polish_strength',.17 if scene.get('composition_polished',False) else 0.)
lenses=[]
for frame in range(1,601):
    scene.frame_set(frame)
    lenses.append(camera.data.lens)
for frame,lens in enumerate(lenses,1):
    scene.frame_set(frame)
    t=(frame-1)/30
    amount=ease((t-6)/.35)*ease((16-t)/1.5)
    camera.data.lens=lens*(1-.20*amount)/(1-previous_strength*amount)
    camera.data.keyframe_insert('lens',frame=frame)
scene['composition_polished']=True
scene['composition_polish_strength']=.20
scene.frame_set(1)
scene.camera=camera
scene.render.resolution_percentage=100
bpy.ops.wm.save_as_mainfile(filepath=str(root/'woodland.blend'))
