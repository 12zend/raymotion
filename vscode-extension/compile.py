"""Editor build adapter. Compiles only; never invokes the raymotion CLI."""
import contextlib
import fcntl
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

prefix, source, output, compiler = sys.argv[1:5]
prefix, source, output = (Path(p).resolve() for p in (prefix, source, output))
if not (prefix/'include/raymotion/preview.hpp').is_file() or not any(
        candidate.is_file() for candidate in (
            prefix/'src/compiler/raymotion_compiler.py',
            prefix/'share/raymotion/compiler/raymotion_compiler.py')):
    sys.exit('Raymotion runtime not found: ' + str(prefix) +
             '\nSet raymotion.runtimePath to the updated Raymotion repository or installation prefix.')
sys.path[:0] = [str(prefix/'src/compiler'), str(prefix/'share/raymotion/compiler')]
from raymotion_compiler import transpile

output.mkdir(parents=True, exist_ok=True)
code = transpile(sys.stdin.read(), preview=True, filename=str(source))
(output/'preview.cpp').write_text(code)
# The extension supplies private persistent storage. Standalone callers retain
# their original disposable build behavior.
cache = Path(sys.argv[5]).resolve() if len(sys.argv) > 5 else output
cache.mkdir(parents=True, exist_ok=True)

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

@contextlib.contextmanager
def locked():
    with (cache/'build.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield

with locked():
    library = prefix/'lib/libraymotion_engine.a'
    if not library.is_file():
        engine = cache/('engine-' + hashlib.sha256(str(prefix).encode()).hexdigest()[:16])
        subprocess.run(['cmake', '-S', str(prefix), '-B', str(engine),
                        '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'], check=True)
        subprocess.run(['cmake', '--build', str(engine), '--parallel'], check=True)
        library = engine/'libraymotion_engine.a'
    identity = json.dumps([code, str(source), str(prefix), str(library), compiler,
        subprocess.check_output([compiler, '--version'], text=True),
        hashlib.sha256(json.dumps(dict(os.environ), sort_keys=True).encode()).hexdigest()], sort_keys=True)
    key = hashlib.sha256(json.dumps([str(source), str(prefix), compiler]).encode()).hexdigest()
    build = cache/key
    build.mkdir(exist_ok=True)
    manifest = build/'dependencies.json'
    executable = build/'preview'
    valid = False
    try:
        saved = json.loads(manifest.read_text())
        dependencies = saved['dependencies']
        valid = saved['identity'] == identity and executable.is_file() and all(digest(p) == h for p, h in dependencies.items())
    except (OSError, ValueError, KeyError):
        pass
    if not valid:
        manifest.unlink(missing_ok=True)
        cpp = build/'preview.cpp'
        cpp.write_text(code)
        depfile = build/'preview.d'
        # Include system headers so SDK and compiler updates invalidate results.
        subprocess.run([compiler, '-std=c++17', '-O2', '-pthread', '-I', str(source.parent),
                        '-I', str(prefix/'include'), '-MD', '-MF', str(depfile),
                        str(cpp), str(library), '-framework', 'Foundation',
                        '-framework', 'Metal', '-o', str(executable)], check=True)
        deps = shlex.split(depfile.read_text().replace('\\\n', '').split(':', 1)[1])
        dependencies = {str(Path(p).resolve()): digest(p) for p in deps}
        dependencies[str(library)] = digest(library)
        manifest.write_text(json.dumps({'identity': identity, 'dependencies': dependencies}))
    else:
        print('Raymotion: reusing compiled preview')
    shutil.copy2(executable, output/'preview')
