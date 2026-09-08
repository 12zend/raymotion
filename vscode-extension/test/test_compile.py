"""Run after building the engine: python3 -m unittest discover -s vscode-extension/test -p 'test_*.py'."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

class CompileCacheTest(unittest.TestCase):
    def test_cache_and_header_invalidation(self):
        with tempfile.TemporaryDirectory(prefix='raymotion cache ') as directory:
            root = Path(directory)
            (root/'include').symlink_to(ROOT/'include', target_is_directory=True)
            (root/'src').symlink_to(ROOT/'src', target_is_directory=True)
            (root/'lib').mkdir()
            (root/'lib/libraymotion_engine.a').symlink_to(ROOT/'build/libraymotion_engine.a')
            header = root/'value with spaces.hpp'
            header.write_text('constexpr int value = 1;\n')
            def build(source):
                result = subprocess.run(['python3', str(ROOT/'vscode-extension/compile.py'),
                    str(root), str(root/'scene.ray'), str(root/'out'), 'c++', str(root/'cache')],
                    input=source, text=True, capture_output=True, check=True)
                return 'reusing compiled preview' in result.stdout
            code = '#include "value with spaces.hpp"\nstd::cout << value; object.render();'
            self.assertFalse(build(code))
            self.assertTrue(build(code))
            # Invalidate by content even when the timestamp and byte count match.
            stat = header.stat()
            header.write_text('constexpr int value = 2;\n')
            os.utime(header, ns=(stat.st_atime_ns, stat.st_mtime_ns))
            self.assertFalse(build(code))
            self.assertTrue(build(code))
            self.assertFalse(build(code + '\n// unsaved edit\n'))

if __name__ == '__main__':
    unittest.main()
