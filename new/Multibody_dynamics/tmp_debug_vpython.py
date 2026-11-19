import traceback, sys
from pathlib import Path
sys.path.append(str(Path('.').resolve()))
print('cwd', Path('.').resolve())
try:
    from vpython import scene, vector
    print('vpython imported', scene, vector)
except Exception as e:
    print('vpython import failed', type(e).__name__, e)
    traceback.print_exc()

try:
    from loader import load_mechanism_data
    from Core_Engine.Multibody_engine_v3 import MechanismSolver
    from vpython_renderer import VPythonRenderer
    BASE_DIR = Path('.').resolve()
    mech = load_mechanism_data(BASE_DIR / 'data' / 'nodes.csv', BASE_DIR / 'data' / 'rods.csv', BASE_DIR / 'data' / 'wheel.csv')
    print('mechanism loaded', type(mech), 'nodes', len(mech.nodes), 'rods', len(mech.rods), 'wheels', len(mech.wheels))
    r = VPythonRenderer(mech)
    print('renderer instantiated', r)
except Exception as e:
    print('failed during load/render', type(e).__name__, e)
    traceback.print_exc()
