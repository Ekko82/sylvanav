"""Build the woodland, mounted sensor, tail camera and geometric scan scenes."""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
from common import PROJECT_ROOT, OUTPUT_ROOT, seed_data

STAGES=['import_model.py','create_scene.py','polish_environment.py','complete_understory.py',
        'polish_lighting.py','polish_composition.py','tail_camera.py','pack_scene.py','build_cloud_scene.py',
        'pack_cloud.py','check_scene.py']

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blender',default='blender')
    parser.add_argument('--from-stage',choices=STAGES,default=STAGES[0])
    args=parser.parse_args()
    executable=shutil.which(args.blender)
    if executable is None:parser.error('Blender is unavailable; pass --blender /path/to/blender')
    model=list((PROJECT_ROOT/'assets/source').glob('*.glb'))
    if len(model)!=1:parser.error('Put exactly one downloaded source .glb in assets/source/')
    metadata=json.loads((PROJECT_ROOT/'assets/model-source.json').read_text(encoding='utf-8'))
    if hashlib.sha256(model[0].read_bytes()).hexdigest()!=metadata['model_sha256']:
        parser.error('This rig expects the original GLB variant specified in assets/model-source.json')
    assets=json.loads((PROJECT_ROOT/'assets/textures.json').read_text(encoding='utf-8'))['assets']
    for asset in assets:
        path=PROJECT_ROOT/asset['path']
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest()!=asset['sha256']:
            parser.error('Missing or modified texture; run scripts/fetch_textures.py: '+asset['path'])
    seed_data()
    for stage in STAGES[STAGES.index(args.from_stage):]:
        print('BUILD_STAGE',stage,flush=True)
        subprocess.run([executable,'--background','--python-exit-code','1','--python',str(PROJECT_ROOT/'scripts'/stage)],
                       cwd=PROJECT_ROOT,check=True,env={**os.environ,'SHOWCASE_OUTPUT':str(OUTPUT_ROOT)})
    print('BUILD_COMPLETE',OUTPUT_ROOT,flush=True)

if __name__=='__main__':main()
