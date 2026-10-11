"""Render point cloud frames 181–480 with Eevee."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import argparse
import sys
import bpy
from common import OUTPUT_ROOT

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--start',type=int,default=181)
parser.add_argument('--end',type=int,default=480)
parser.add_argument('--scale',type=int,default=100)
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
if not 181 <= args.start <= args.end <= 480 or not 1 <= args.scale <= 100:
    parser.error('Invalid cloud frame range or scale')
bpy.ops.wm.open_mainfile(filepath=str(OUTPUT_ROOT/'cloud.blend'))
scene=bpy.context.scene
scene.render.resolution_percentage=args.scale
scene.render.image_settings.file_format='JPEG'
scene.render.image_settings.quality=95
scene.eevee.taa_render_samples=16
directory=OUTPUT_ROOT/'frames-cloud'
directory.mkdir(exist_ok=True)
for frame in range(args.start,args.end+1):
    target=directory/f'frame_{frame:04d}.jpg'
    if target.exists():continue
    scene.frame_set(frame)
    scene.render.filepath=str(target)
    bpy.ops.render.render(write_still=True)
    print('CLOUD_FRAME_DONE',frame,flush=True)
