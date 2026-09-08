"""Editor build adapter. Compiles only; never invokes the raymotion CLI."""
import sys
from pathlib import Path
import subprocess

prefix, source, output, compiler = sys.argv[1:]
prefix, source, output = map(Path, (prefix, source, output))
sys.path[:0] = [str(prefix/'src/compiler'), str(prefix/'share/raymotion/compiler')]
from raymotion_compiler import transpile

output.mkdir(parents=True, exist_ok=True)
cpp = output/'preview.cpp'
cpp.write_text(transpile(sys.stdin.read(), preview=True, filename=str(source)))
library = prefix/'lib/libraymotion_engine.a'
if not library.is_file():
    engine = output/'engine'
    subprocess.run(['cmake', '-S', str(prefix), '-B', str(engine),
                    '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'], check=True)
    subprocess.run(['cmake', '--build', str(engine), '--parallel'], check=True)
    library = engine/'libraymotion_engine.a'
subprocess.run([compiler, '-std=c++17', '-O2', '-pthread', '-I', str(source.parent),
                '-I', str(prefix/'include'), str(cpp), str(library),
                '-framework', 'Foundation', '-framework', 'Metal',
                '-o', str(output/'preview')], check=True)
