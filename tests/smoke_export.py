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
        result=subprocess.run([cli,*args],cwd=root if root.exists() else temp,capture_output=True,text=True,timeout=180)
        if ok and result.returncode: raise RuntimeError(result.stdout+result.stderr)
        if not ok and not result.returncode: raise RuntimeError('expected failure')
        return result
    run('init',str(root))
    (root/'assets/model.obj').write_text('v -1 -1 0\nv 1 -1 0\nv 1 1 0\nv -1 1 0\nf -4 -3 -2 -1\n')
    base='''#include <type_traits>
static_assert(std::is_const_v<std::remove_reference_t<decltype(u_timer)>>);
static_assert(std::is_const_v<std::remove_reference_t<decltype(u_resolution)>>);
if(u_timer!=0 || u_resolution.x!=width || u_resolution.y!=height ||
   std::abs(u_resolution.x/u_resolution.y-double(width)/height)>1e-12)
    throw std::runtime_error("render uniform initial values");
void model = object.init("assets/model.obj");
camera.set({1,2,3}, {10,20,30}, 45);
if(camera.get.position.x!=1 || camera.get.position.y!=2 || camera.get.position.z!=3 ||
   camera.get.rotation.x!=10 || camera.get.rotation.y!=20 ||
   camera.get.rotation.z!=30 || camera.get.fov!=45)
    throw std::runtime_error("camera.get values");
Camera copied = camera;
camera.x = 99;
if(camera.get.position.x!=99 || copied.get.position.x!=1)
    throw std::runtime_error("camera.get copy independence");
camera = copied;
copied.x = -1;
Vec3 position = camera.get.position;
if(position.x!=1 || position.y!=2 || position.z!=3)
    throw std::runtime_error("camera.get assignment independence");
if(camera.x!=1 || camera.y!=2 || camera.z!=3 ||
   camera.dirx!=10 || camera.diry!=20 || camera.dirz!=30 || camera.fov!=45)
    throw std::runtime_error("camera.set full arguments");
camera.set({4,5,6}, {0,90,0});
if(camera.x!=4 || camera.y!=5 || camera.z!=6 || camera.dirx!=0 ||
   camera.diry!=90 || camera.dirz!=0 || camera.fov!=60)
    throw std::runtime_error("camera.set default fov");
camera.set(Vec3{7,8,9});
if(camera.x!=7 || camera.y!=8 || camera.z!=9 ||
   camera.dirx!=0 || camera.diry!=0 || camera.dirz!=0 || camera.fov!=60)
    throw std::runtime_error("camera.set default rotation");
camera.set({}, {}, 90);
if(std::abs(camera.focal-240)>1e-9)
    throw std::runtime_error("camera.set focal update");
camera.set();
if(camera.x!=0 || camera.y!=0 || camera.z!=0 ||
   camera.dirx!=0 || camera.diry!=0 || camera.dirz!=0 || camera.fov!=60 ||
   camera.m0!=1 || camera.m4!=1 || camera.m8!=1)
    throw std::runtime_error("camera.set reset");
'''
    push='object.push(model, {0,0,3}, {0,0,0}, {1,1,1}, {1,0,0}, {1,0,0});\nobject.render();\n'
    (root/'main.ray').write_text(base+push)
    run('build')
    executable=root/'.raymotion/build/main'
    built=executable.stat().st_mtime_ns
    run('build')
    assert executable.stat().st_mtime_ns==built, 'unchanged build was recompiled'
    # Local header changes must invalidate the executable cache.
    (root/'settings.h').write_text('#define TEST_VALUE 1\n')
    original=(root/'main.ray').read_text()
    (root/'main.ray').write_text('#include "settings.h"\nstatic_assert(TEST_VALUE == 1);\n'+original)
    run('build')
    (root/'settings.h').write_text('#define TEST_VALUE 2\n')
    run('build',ok=False)
    (root/'main.ray').write_text(original)
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
    (root/'main.ray').write_text(base+'for(int f=0;f<3;++f) {\nif(std::abs(u_timer-double(f)/framerate)>1e-12) throw std::runtime_error("frame time");\n'+push+'}\n')
    run('export','invalid.png','-w','16','-h','16','-s','1',ok=False)
    if shutil.which('ffmpeg') and shutil.which('ffprobe'):
        (root/'main.ray').write_text(base+'for(int f=0;f<3;++f) {\nif(std::abs(u_timer-double(f)/framerate)>1e-12) throw std::runtime_error("frame time");\n'+push.replace('{0,0,3}','{f*0.4,0,3}')+'}\n')
        run('export','movie.mp4','-w','32','-h','24','-s','1','-f','12','-t','0.25')
        result=subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(root/'movie.mp4')])
        stream=json.loads(result)['streams'][0]
        assert (stream['width'],stream['height'],stream['nb_frames'],stream['r_frame_rate'])==(32,24,'3','12/1')
        # A single-frame program is evaluated again, with updated uniforms.
        (root/'main.ray').write_text('if(u_resolution.x!=16 || u_resolution.y!=16) throw std::runtime_error("resolution");\nstd::cout << "frame-time=" << u_timer << "\\n";\nobject.render();\n')
        result=run('export','repeated.mp4','-w','16','-h','16','-s','1','-f','20','-t','3')
        times=[float(line.split('=',1)[1]) for line in result.stdout.splitlines() if line.startswith('frame-time=')]
        assert len(times)==60 and all(abs(t-i/20)<1e-9 for i,t in enumerate(times)), times
        metadata=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(root/'repeated.mp4')]))
        assert int(metadata['streams'][0]['nb_frames'])==60
        assert abs(float(metadata['streams'][0]['duration'])-3)<1e-9
        (root/'main.ray').write_text('if(u_timer==0) object.render();')
        run('export','no-progress.mp4','-w','16','-h','16','-s','1',ok=False)
        # Infinite user loops stop successfully at the export duration limit.
        (root/'main.ray').write_text(base+'for(int f=0;;++f) {\nif(std::abs(u_timer-double(f)/framerate)>1e-12) throw std::runtime_error("frame time");\n'+push+'}\n')
        for options, fps, expected in [(('-t','0.29'),12,3), (('--time','0.5'),12,6), ((),1,10)]:
            run('export','limited.mp4','-w','16','-h','16','-s','1','-f',str(fps),*options)
            result=subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(root/'limited.mp4')])
            stream=json.loads(result)['streams'][0]
            assert int(stream['nb_frames'])==expected, (options,stream['nb_frames'])
    (root/'main.ray').write_text('void bad = object.init("missing.obj");object.render();')
    run('export','missing.png','-w','16','-h','16',ok=False)
print('Installed CLI: build, PNG pixels, MP4 metadata, and failure paths passed')
