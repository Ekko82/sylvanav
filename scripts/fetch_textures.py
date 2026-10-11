"""Download the selected CC0 texture assets and verify SHA-256 hashes."""
import hashlib
import json
from pathlib import Path
import urllib.request
from common import PROJECT_ROOT

def main():
    manifest=json.loads((PROJECT_ROOT/'assets/textures.json').read_text(encoding='utf-8'))
    for asset in manifest['assets']:
        target=PROJECT_ROOT/asset['path']
        target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest()==asset['sha256']:
            print('VERIFIED',asset['path']);continue
        request=urllib.request.Request(asset['download_url'],headers={'User-Agent':'woodland-drone-showcase/1.0'})
        with urllib.request.urlopen(request,timeout=120) as response:
            content=response.read()
        if len(content)!=asset['bytes'] or hashlib.sha256(content).hexdigest()!=asset['sha256']:
            raise RuntimeError('Downloaded asset checksum mismatch: '+asset['path'])
        target.write_bytes(content)
        print('DOWNLOADED',asset['path'])

if __name__=='__main__':main()
