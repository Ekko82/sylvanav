"""Render synchronized external and forward sensor shots with Cycles."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import argparse
from pathlib import Path
import sys
import time
import bpy
from common import OUTPUT_ROOT, configure_cycles

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--start', type=int, default=1)
parser.add_argument('--end', type=int, default=600)
parser.add_argument('--backend', default='AUTO', choices=['AUTO','CPU','OPTIX','CUDA','HIP','METAL','ONEAPI'])
parser.add_argument('--samples', type=int, default=48)
parser.add_argument('--scale', type=int, default=100)
parser.add_argument('--overwrite', action='store_true')
args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
if not 1 <= args.start <= args.end <= 600 or not 1 <= args.scale <= 100 or args.samples < 1:
    parser.error('Invalid frame range, resolution scale or sample count')
bpy.ops.wm.open_mainfile(filepath=str(OUTPUT_ROOT/'woodland.blend'))
scene = bpy.context.scene
configure_cycles(scene, args.backend)
scene.render.resolution_x, scene.render.resolution_y = 1920, 1080
scene.render.resolution_percentage = args.scale
scene.cycles.samples = args.samples
scene.cycles.adaptive_threshold = .02
scene.cycles.use_auto_tile = False
scene.render.use_persistent_data = True
scene.render.image_settings.file_format = 'JPEG'
scene.render.image_settings.quality = 95
directory = OUTPUT_ROOT/'frames-main'
directory.mkdir(exist_ok=True)
for frame in range(args.start, args.end+1):
    scene.frame_set(frame)
    scene.camera = bpy.data.objects['SensorCamera'] if 241 <= frame <= 300 else bpy.data.objects['BeautyCamera']
    scene.render.use_motion_blur = frame != 181
    target = directory/f'frame_{frame:04d}.jpg'
    if target.exists() and not args.overwrite:
        continue
    scene.render.filepath = str(target)
    start = time.monotonic()
    bpy.ops.render.render(write_still=True)
    print('FRAME_DONE', frame, round(time.monotonic()-start,2), flush=True)
