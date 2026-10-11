"""Paths shared by host Python and Blender Python."""

import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from pathlib import Path
import os
import shutil

PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = Path(os.environ.get('SHOWCASE_OUTPUT', PROJECT_ROOT / 'build')).resolve()
OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)

def seed_data():
    for name in ('animation-data.json', 'pointcloud-data.json', 'sensor-config.json', 'structure-audit.json'):
        target = OUTPUT_ROOT / name
        if not target.exists():
            shutil.copyfile(PROJECT_ROOT / 'data' / name, target)

def configure_cycles(scene, backend='AUTO'):
    import bpy
    scene.render.engine = 'CYCLES'
    prefs = bpy.context.preferences.addons['cycles'].preferences
    candidates = ['OPTIX', 'CUDA', 'HIP', 'METAL', 'ONEAPI'] if backend == 'AUTO' else [backend]
    selected = None
    if backend != 'CPU':
        for candidate in candidates:
            try:
                prefs.compute_device_type = candidate
                prefs.get_devices()
                available = [device for device in prefs.devices if device.type == candidate]
                if available:
                    for device in prefs.devices:
                        device.use = device.type == candidate
                    selected = candidate
                    break
            except (TypeError, RuntimeError):
                continue
    if backend not in ('AUTO', 'CPU') and selected is None:
        raise RuntimeError('Requested Cycles backend is unavailable: ' + backend)
    scene.cycles.device = 'GPU' if selected else 'CPU'
    scene.cycles.denoiser = 'OPENIMAGEDENOISE'
    print('CYCLES_BACKEND', selected or 'CPU', flush=True)
    return selected or 'CPU'
