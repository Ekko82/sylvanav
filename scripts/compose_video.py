"""Compose the 20 second showcase from native 1080p frame sequences."""
import argparse
import json
import re
import shutil
import subprocess
import wave
from pathlib import Path
import numpy as np
from common import OUTPUT_ROOT

def make_sound(path):
    rate=48000
    t=np.arange(rate*20)/rate
    envelope=np.clip((t-1.5)/1.5,0,1)*np.clip((20-t)/.6,0,1)
    frequency=145+55*np.clip((t-3)/3,0,1)
    phase=np.cumsum(2*np.pi*frequency/rate)
    rng=np.random.default_rng(17)
    noise=rng.normal(size=len(t))
    spectrum=np.fft.rfft(noise)
    freq=np.fft.rfftfreq(len(t),1/rate)
    spectrum *= (1/(1+(freq/1700)**4))*np.clip(freq/150,0,1)
    noise=np.fft.irfft(spectrum,n=len(t))
    noise /= max(1,float(np.max(np.abs(noise))))
    sound=envelope*(.013*np.sin(phase)+.006*np.sin(phase*2)+.006*noise)
    stereo=np.column_stack((sound,sound*.97))
    with wave.open(str(path),'wb') as wav:
        wav.setnchannels(2);wav.setsampwidth(2);wav.setframerate(rate)
        wav.writeframes((stereo*32767).astype('<i2').tobytes())

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ffmpeg',default='ffmpeg')
    parser.add_argument('--crf',type=int,default=17,choices=range(52))
    parser.add_argument('--verify-only',action='store_true')
    parser.add_argument('--output',default='inspire3-woodland-20s.mp4')
    args=parser.parse_args()
    executable=shutil.which(args.ffmpeg)
    if executable is None:parser.error('FFmpeg is unavailable; install it or pass --ffmpeg /path/to/ffmpeg')
    if Path(args.output).name != args.output or Path(args.output).suffix.lower() != '.mp4':
        parser.error('--output must be a plain .mp4 filename')
    output=OUTPUT_ROOT/args.output
    filters=(
        '[0:v]split=2[base][beauty];'
        '[1:v]setpts=PTS-STARTPTS+6/TB,split=2[cs][cf];'
        '[cs]scale=620:348:flags=lanczos,pad=632:360:6:6:color=0x12232c[smallcloud];'
        "[base][smallcloud]overlay=W-w-28:28:enable='gte(t,6)*lt(t,14)':eof_action=repeat[a];"
        "[a][cf]overlay=0:0:enable='gte(t,14)*lt(t,16)':eof_action=repeat[b];"
        '[beauty]scale=620:348:flags=lanczos,pad=632:360:6:6:color=0x12232c[smallbeauty];'
        "[b][smallbeauty]overlay=W-w-28:H-h-28:enable='gte(t,14)*lt(t,16)'[out]"
    )
    if not args.verify_only:
        make_sound(OUTPUT_ROOT/'rotor-synth.wav')
        command=[executable,'-hide_banner','-y','-framerate','30','-start_number','1',
                 '-i',str(OUTPUT_ROOT/'frames-main/frame_%04d.jpg'),'-framerate','30',
                 '-start_number','181','-i',str(OUTPUT_ROOT/'frames-cloud/frame_%04d.jpg'),
                 '-i',str(OUTPUT_ROOT/'rotor-synth.wav'),'-filter_complex',filters,
                 '-map','[out]','-map','2:a:0','-t','20','-c:v','libx264','-preset','slow',
                 '-crf',str(args.crf),'-r','30','-pix_fmt','yuv420p','-c:a','aac',
                 '-b:a','128k','-movflags','+faststart',str(output)]
        with (OUTPUT_ROOT/'encode.log').open('w',encoding='utf-8') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    result=subprocess.run([executable,'-hide_banner','-nostats','-v','info','-progress','pipe:1',
                           '-i',str(output),'-map','0:v:0','-map','0:a:0','-fps_mode','passthrough',
                           '-f','null','-'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,check=True)
    log=result.stdout.decode('utf-8',errors='replace')
    (OUTPUT_ROOT/'decode-check.log').write_text(log,encoding='utf-8')
    frames=[int(value) for value in re.findall(r'^frame=(\d+)\s*$',log,re.MULTILINE)]
    assert frames and frames[-1]==600 and 'progress=end' in log
    assert '1920x1080' in log and '30 fps' in log and '00:00:20.00' in log
    assert 'Audio: aac' in log and 'Error' not in log and 'Invalid' not in log
    report={'file':output.name,'bytes':output.stat().st_size,'width':1920,'height':1080,
            'fps':30,'duration_s':20,'decoded_frames':600,'full_decode_passed':True}
    (OUTPUT_ROOT/'video-verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))

if __name__=='__main__':main()
