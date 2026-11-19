import shutil
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest


class StandaloneTests(unittest.TestCase):
    def test_copied_workbench_starts_and_solves_without_suspension_tool(self):
        package = Path(__file__).resolve().parents[1]
        with TemporaryDirectory() as directory:
            shutil.copytree(package, Path(directory) / package.name,
                            ignore=shutil.ignore_patterns('.venv', '__pycache__', 'tests'))
            result = subprocess.run([sys.executable, '-I', '-X', 'utf8', '-c', '''
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path.cwd()))
from MultibodyGeometryLab.__main__ import Workbench
from MultibodyGeometryLab.geometry import build_vehicle
from MultibodyGeometryLab.project import default_project, read_project, save_project
from MultibodyGeometryLab.solver import solve_vehicle

project = default_project()
assert len(solve_vehicle(build_vehicle(project), project['motion']).corners) == 4

example = Path('MultibodyGeometryLab/projects/Example.json')
project, native = read_project(example)
assert native
assert len(solve_vehicle(build_vehicle(project), project['motion']).corners) == 4

source = project['geometry']
source['vehicle']['roll_stiffness_ratio'] = 1.05
source['front']['static_camber_deg'] = -0.5
source['kinematics_hardpoints'] = {'unit': 'm', 'source': 'test', 'corners': {}}
Path('suspension.json').write_text(json.dumps(source), encoding='utf-8')
imported, native = read_project('suspension.json')
assert not native and imported['geometry'] == source
assert len(solve_vehicle(build_vehicle(imported), imported['motion']).corners) == 4
save_project('converted.json', imported)
assert read_project('converted.json') == (imported, True)
assert not any(name.split('.')[0] in ('SuspensionGeometryLab', 'suspension_geometry_lab')
               for name in sys.modules)
'''], cwd=directory, capture_output=True, encoding='utf-8', timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
