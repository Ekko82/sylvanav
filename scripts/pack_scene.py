"""Pack textures and remove workstation-specific path metadata."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import argparse
from pathlib import Path
import sys
import bpy
from common import OUTPUT_ROOT

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input',type=Path,default=OUTPUT_ROOT/'woodland.blend')
parser.add_argument('--output',type=Path,default=OUTPUT_ROOT/'woodland.blend')
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
bpy.ops.wm.open_mainfile(filepath=str(args.input))
image_names={image.name:Path(image.filepath.replace('\\','/')).name for image in bpy.data.images}
bpy.ops.file.pack_all()
for image in bpy.data.images:
    if image.source=='FILE':
        name=image_names[image.name] or (image.name if Path(image.name).suffix else image.name+'.png')
        image.filepath='//textures/'+name
        for packed in image.packed_files:
            packed.filepath='//textures/'+name
        if not image.packed_file:raise RuntimeError('Image was not packed: '+image.name)
for screen in bpy.data.screens:
    for area in screen.areas:
        for space in area.spaces:
            if space.type=='FILE_BROWSER' and space.params:
                # Blender serializes a fixed-size directory buffer. Overwrite its
                # old bytes before assigning a shorter path, so stale path text
                # does not remain after the terminating NUL in the saved file.
                space.params.directory=b'/'*1023
                space.params.directory=b'//'
for scene in bpy.data.scenes:
    scene.render.filepath='//renders/'
    for key in list(scene.keys()):
        value=scene[key]
        if isinstance(value,str) and (':/' in value or ':\\' in value):del scene[key]
if 'BeautyCamera' in bpy.data.objects:
    bpy.context.scene.camera=bpy.data.objects['BeautyCamera']
bpy.context.scene.frame_set(1)
bpy.context.preferences.filepaths.save_version=0
args.output.parent.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=str(args.output))
print('PORTABLE_SCENE_SAVED',len([image for image in bpy.data.images if image.packed_file]),flush=True)
