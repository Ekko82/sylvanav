"""Append actual understory meshes, then rescan those same scene surfaces."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import sys
import json
import random
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

from common import OUTPUT_ROOT, PROJECT_ROOT
root=OUTPUT_ROOT
sys.path.insert(0,str(root.parent))
from append_understory import append_understory
bpy.ops.wm.open_mainfile(filepath=str(root/'woodland.blend'))
scene=bpy.context.scene
result=append_understory(scene)
bpy.context.view_layer.update()
scene.frame_set(1)
scene.camera=bpy.data.objects['BeautyCamera']
scene.render.resolution_percentage=100
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(root/'woodland.blend'))
print('UNDERSTORY_SAVED',flush=True)

lo=Vector((-1.8,-1.8,-.05)); hi=Vector((1.8,1.8,2.7))
def reachable(obj):
    corners=[obj.matrix_world@Vector(c) for c in obj.bound_box]
    low=[min(p[a] for p in corners) for a in range(3)]
    high=[max(p[a] for p in corners) for a in range(3)]
    return sum(max(0,low[a]-hi[a],lo[a]-high[a])**2 for a in range(3))<=18**2
objects=[o for o in scene.objects if (o.name.startswith('ENV2_') or any(c.name.startswith('ENV2_') for c in o.users_collection)) and o.type=='MESH'
         and not o.hide_render and not any(c.hide_render for c in o.users_collection) and reachable(o)]
vertices,triangles=[],[]
for obj in objects:
    mesh=obj.data; mesh.calc_loop_triangles(); first=len(vertices)
    vertices.extend(tuple(obj.matrix_world@v.co) for v in mesh.vertices)
    triangles.extend(tuple(first+j for j in t.vertices) for t in mesh.loop_triangles)
triangle_count=len(triangles)
bvh=BVHTree.FromPolygons(vertices,triangles,all_triangles=True)
del vertices,triangles
fpv=bpy.data.objects['SensorCamera']
frame_view=fpv.data.view_frame(scene=scene)
xmin=min(v.x/-v.z for v in frame_view); xmax=max(v.x/-v.z for v in frame_view)
ymin=min(v.y/-v.z for v in frame_view); ymax=max(v.y/-v.z for v in frame_view)
data=json.loads((root/'pointcloud-data.json').read_text())
batches=[]; voxels=set(); rng=random.Random(481)
for f in range(91,481,15):
    scene.frame_set(f); bpy.context.view_layer.update()
    world=fpv.matrix_world.copy(); origin=world.translation; points=[]
    for y in range(64):
        for x in range(112):
            nx=(x+rng.random())/112; ny=(y+rng.random())/64
            local=Vector((xmin+nx*(xmax-xmin),ymin+ny*(ymax-ymin),-1)).normalized()
            direction=(world.to_3x3()@local).normalized()
            loc,normal,index,distance=bvh.ray_cast(origin,direction,18)
            if loc is None or distance<.15: continue
            key=tuple(round(v/.07) for v in loc)
            if key in voxels: continue
            voxels.add(key); points.append([round(v,5) for v in loc])
    batches.append({'frame':f,'points':points,'sensor_origin':list(origin),'sensor_quaternion':list(world.to_quaternion())})
    print('SCAN',f,len(points),len(voxels),flush=True)
data['batches']=batches
data['sampled_mesh_triangles']=triangle_count
data['understory_included']=True
(root/'pointcloud-data.json').write_text(json.dumps(data,separators=(',',':')),encoding='utf-8')
revision=json.loads((root/'environment-revision.json').read_text())
revision['understory_geometry_added']=True
revision['pointcloud_rescanned_after_geometry_change']=True
revision['point_count']=len(voxels)
(root/'environment-revision.json').write_text(json.dumps(revision,indent=2),encoding='utf-8')
print('RESCAN_READY',len(voxels),triangle_count,flush=True)

