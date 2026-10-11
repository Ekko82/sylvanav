"""Remove workstation paths from the cloud scene after building it."""
from pathlib import Path
import runpy
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from common import OUTPUT_ROOT
sys.argv=['pack_scene.py','--','--input',str(OUTPUT_ROOT/'cloud.blend'),
          '--output',str(OUTPUT_ROOT/'cloud.blend')]
runpy.run_path(str(Path(__file__).with_name('pack_scene.py')),run_name='__main__')
