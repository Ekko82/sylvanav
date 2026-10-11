"""Check sensor direction, rear follow positions and packed texture portability."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import json
import math
import bpy
from mathutils import Vector
from common import OUTPUT_ROOT
from sensor_rig import _lens_geometry

bpy.ops.wm.open_mainfile(filepath=str(OUTPUT_ROOT/'woodland.blend'))
scene=bpy.context.scene
drone=bpy.data.objects['Drone']
sensor=bpy.data.objects['SensorCamera']
camera=bpy.data.objects['BeautyCamera']
sensor_dots=[]
tail_dots=[]
for frame in range(1,601):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    body_forward=(drone.matrix_world.to_3x3()@Vector((0,-1,0))).normalized()
    sensor_forward=(sensor.matrix_world.to_3x3()@Vector((0,0,-1))).normalized()
    sensor_dots.append(body_forward.dot(sensor_forward))
    if 181<=frame<=480:
        observer=(camera.matrix_world.translation-drone.matrix_world.translation).normalized()
        tail_dots.append(observer.dot(body_forward))
assert min(sensor_dots)>math.cos(math.radians(6.1))
assert max(tail_dots)<-.9
images=[image for image in bpy.data.images if image.source=='FILE']
assert all(image.packed_file and image.filepath.startswith('//') for image in images)
assert scene.render.resolution_x==1920 and scene.render.resolution_y==1080
data=json.loads((OUTPUT_ROOT/'pointcloud-data.json').read_text(encoding='utf-8'))
count=sum(len(batch['points']) for batch in data['batches'])
assert len(data['batches'])==26 and count==47386
report={'checked_frames':600,'minimum_sensor_body_forward_dot':min(sensor_dots),
        'maximum_tail_observer_body_forward_dot':max(tail_dots),
        'packed_image_count':len(images),'point_count':count,'checks_passed':True,
        'scope':'Pose, framing-side, data count and portable image checks; no collision or SLAM validation.'}
(OUTPUT_ROOT/'scene-check.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report),flush=True)
