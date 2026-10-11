"""Change only the external filming camera to an aircraft-tail follow view."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import json
import math
import bpy
from mathutils import Vector,Quaternion

from common import OUTPUT_ROOT, PROJECT_ROOT
root=OUTPUT_ROOT
bpy.ops.wm.open_mainfile(filepath=str(root/'woodland.blend'))
scene=bpy.context.scene
camera=bpy.data.objects['BeautyCamera']
focus=bpy.data.objects['Camera focus']
sensor=bpy.data.objects['SensorCamera']
drone=bpy.data.objects['Drone']
animation=json.loads((root/'animation-data.json').read_text(encoding='utf-8'))
cloud=json.loads((root/'pointcloud-data.json').read_text(encoding='utf-8'))
translation=Vector(cloud['route_translation'])

def ease(t):
    t=max(0,min(1,t));return t*t*(3-2*t)

before={}
for row in animation['frames']:
    f=row['frame'];scene.frame_set(f);bpy.context.view_layer.update()
    before[f]={'sensor_world':list(sum([list(r) for r in sensor.matrix_world],[])),
               'drone_world':list(sum([list(r) for r in drone.matrix_world],[]))}

report=[]
for row in animation['frames']:
    f,t=row['frame'],row['t_s']
    if f<181:continue
    scene.frame_set(f)
    x,y,z,w=row['orientation_inspire_xyzw']
    q=Quaternion((w,x,y,z))
    pos=Vector(row['position_enu'])+translation
    # Model front is local -Y, so an observer at local +Y sees the aircraft tail.
    if t<14:
        offset=Vector((.40,1.85,.74))
        target=Vector((0,-.08,.24))
        lens=35
    elif t<16:
        u=ease((t-14)/2)
        offset=Vector((.40,1.85,.74)).lerp(Vector((.62,2.35,.90)),u)
        target=Vector((0,-.08,.24));lens=35+5*u
    else:
        theta=(t-16)/4*math.radians(60)
        offset=Vector((.62*math.cos(theta)-2.35*math.sin(theta),
                       .62*math.sin(theta)+2.35*math.cos(theta),.90))
        target=Vector((0,-.08,.24));lens=40+2*(t-16)/4
    focus.location=pos+q@target
    focus.keyframe_insert('location',frame=f)
    camera.location=pos+q@offset
    camera.rotation_euler=(focus.location-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.lens=lens
    camera.data.shift_x=.075 if t<14 else .075*(1-ease((t-14)/.2))
    for key in ['location','rotation_euler']:camera.keyframe_insert(key,frame=f)
    for key in ['lens','shift_x']:camera.data.keyframe_insert(key,frame=f)
    bpy.context.view_layer.update()
    now_s=list(sum([list(r) for r in sensor.matrix_world],[]))
    now_d=list(sum([list(r) for r in drone.matrix_world],[]))
    assert max(abs(a-b) for a,b in zip(now_s,before[f]['sensor_world']))<1e-7
    assert max(abs(a-b) for a,b in zip(now_d,before[f]['drone_world']))<1e-7
    forward=(q@Vector((0,-1,0))).normalized()
    report.append({'frame':f,'time_s':t,'camera_body_offset':list(offset),
                   'external_camera_side_forward_dot':float((camera.location-pos).normalized().dot(forward)),
                   'lens_mm':lens,'sensor_and_aircraft_unchanged':True})

for owner in [camera,focus,camera.data]:
    if owner.animation_data and owner.animation_data.action:
        for curve in owner.animation_data.action.fcurves:
            for k in curve.keyframe_points:
                if k.co.x==180:k.interpolation='CONSTANT'
                elif k.co.x>=181:k.interpolation='LINEAR'
scene['external_camera_style']='tail-follow from 6s; tail-side orbit from 16s'
scene['tail_camera_cut_frame']=181
scene.frame_set(1)
scene.camera=camera
scene.render.resolution_percentage=100
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(root/'woodland.blend'))
(root/'tail-camera-change.json').write_text(json.dumps({
    'revision':3,'change':'Only external BeautyCamera/focus and its camera-follow light position change; sensor, cloud, forest and aircraft poses remain identical.',
    'cut_at_s':6,'tail_follow_s':[6,16],'tail_orbit_s':[16,20],
    'model_body_forward':[0,-1,0],
    'all_sensor_and_aircraft_matrices_unchanged':all(r['sensor_and_aircraft_unchanged'] for r in report),
    'follow_observer_forward_dot_max':max(r['external_camera_side_forward_dot'] for r in report if r['time_s']<16),
    'frames':report},indent=2),encoding='utf-8')
print('TAIL_CAMERA_READY',flush=True)
