import importlib.machinery
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

CLI=Path(__file__).resolve().parents[1]/'src/cli/raymotion'
loader=importlib.machinery.SourceFileLoader('cli',str(CLI))
spec=importlib.util.spec_from_loader(loader.name,loader)
cli=importlib.util.module_from_spec(spec);loader.exec_module(cli)
class CompilerTests(unittest.TestCase):
    def test_literals_and_comments(self):
        source='// void a = object.init("x");\nconst char* s="void a = object.init(x)";\nvoid /*hi*/ model = object.init("a.obj");\n'
        result=cli.transpile(source)
        self.assertIn('// void a = object.init("x");',result)
        self.assertIn('"void a = object.init(x)"',result)
        self.assertIn('auto /*hi*/ model',result)
    def test_init_no_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            subprocess.run([str(CLI),'init',d],check=True,capture_output=True)
            self.assertTrue((Path(d)/'assets').is_dir())
            source=(Path(d)/'main.ray').read_text()
            self.assertNotEqual(subprocess.run([str(CLI),'init',d],capture_output=True).returncode,0)
            self.assertEqual(source,(Path(d)/'main.ray').read_text())
    def test_invalid_options(self):
        for args in [('export','out.png','-h','0'),('export','out.gif'),('export','out.mp4','-w','3')]:
            self.assertNotEqual(subprocess.run([str(CLI),*args],capture_output=True).returncode,0)
if __name__=='__main__': unittest.main()
