"""Check the app's bundled data after copying it away from the workspace."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


class StandaloneTests(unittest.TestCase):
    def test_all_tools_run_from_copied_folder_and_unrelated_working_directory(self):
        source = '''
import importlib.abc
from pathlib import Path
import sys

root = Path(sys.argv[1])
sys.path.insert(0, str(root))
class NoSiblingApp(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('suspension_lab', 'new'):
            raise AssertionError('External app import: ' + fullname)
sys.meta_path.insert(0, NoSiblingApp())

from vehicle_lab.gui import App
from vehicle_lab.models import read_project, write_project
from vehicle_lab.workflows import NAMES, defaults, run_tool
from vehicle_lab.motor import APP_ROOT, read_motor_curve
from vehicle_lab.export import write_csv

assert APP_ROOT == root
assert not (root / 'suspension_lab').exists()
assert Path(read_motor_curve('input_data/motor_speed_vs_torque.csv').source).is_relative_to(root)
vehicle, geometry, settings = read_project(root / 'examples/fsae_example.json')
for name in NAMES:
    result = run_tool(name, vehicle, geometry, {**defaults(name), **settings.get(name, {})})
    assert result['metrics'] and result['rows'] and result['plots'], name
    write_csv(Path.cwd() / (name + '.csv'), result['rows'])
write_project(Path.cwd() / 'saved.json', vehicle, geometry, settings)
assert read_project(Path.cwd() / 'saved.json')[0] == vehicle
for name, module in tuple(sys.modules.items()):
    if name.split('.')[0] == 'vehicle_lab':
        assert Path(module.__file__).resolve().is_relative_to(root), module.__file__
'''
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temporary:
            staged = Path(temporary) / "VehicleLab"
            elsewhere = Path(temporary) / "unrelated"
            staged.mkdir()
            elsewhere.mkdir()
            for folder in ("vehicle_lab", "input_data", "examples"):
                shutil.copytree(root / folder, staged / folder,
                                ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
            result = subprocess.run([sys.executable, "-I", "-B", "-c", source, str(staged)],
                                    cwd=elsewhere, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)
