"""End-to-end check of an installed CLI; MP4 checked when ffmpeg is available."""
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib

cli=str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory(prefix='raymotion-smoke-') as temp:
    root=Path(temp)/'project with spaces'
    def run(*args,ok=True):
        result=subprocess.run([cli,*args],cwd=root if root.exists() else temp,capture_output=True,text=True)
        if ok and result.returncode: raise RuntimeError(result.stdout+result.stderr)
        if not ok and not result.returncode: raise RuntimeError('expected failure')
        return result
    run('init',str(root))
    (root/'assets/model.obj').write_text('v -1 -1 0\nv 1 -1 0\nv 1 1 0\nv -1 1 0\nf -4 -3 -2 -1\n')
    base='void model = object.init("assets/model.obj");\n'
    push='object.push(model, {0,0,3}, {0,0,0}, {1,1,1}, {1,0,0}, {1,0,0});\nobject.render();\n'
    (root/'main.ray').write_text(base+push)
    run('build')
    run('export','image with spaces.png','-w','32','-h','24','-s','1')
    data=(root/'image with spaces.png').read_bytes()
    assert data[:8]==b'\x89PNG\r\n\x1a\n'
    assert struct.unpack('>II',data[16:24])==(32,24)
    pos=8;compressed=b''
    while pos<len(data):
        size=struct.unpack('>I',data[pos:pos+4])[0]
        if data[pos+4:pos+8]==b'IDAT':compressed+=data[pos+8:pos+8+size]
        pos+=12+size
    raw=zlib.decompress(compressed)
    assert any(raw[y*97+1+x*3]>raw[y*97+2+x*3]+30 for y in range(24) for x in range(32)), 'model missing in PNG'
    (root/'main.ray').write_text(base+'for(int f=0;f<3;++f) {\n'+push+'}\n')
    run('export','invalid.png','-w','16','-h','16','-s','1',ok=False)
    if shutil.which('ffmpeg') and shutil.which('ffprobe'):
        (root/'main.ray').write_text(base+'for(int f=0;f<3;++f) {\n'+push.replace('{0,0,3}','{f*0.4,0,3}')+'}\n')
        run('export','movie.mp4','-w','32','-h','24','-s','1','-f','12')
        result=subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(root/'movie.mp4')])
        stream=json.loads(result)['streams'][0]
        assert (stream['width'],stream['height'],stream['nb_frames'],stream['r_frame_rate'])==(32,24,'3','12/1')
    (root/'main.ray').write_text('void bad = object.init("missing.obj");object.render();')
    run('export','missing.png','-w','16','-h','16',ok=False)
print('Installed CLI: build, PNG pixels, MP4 metadata, and failure paths passed')
