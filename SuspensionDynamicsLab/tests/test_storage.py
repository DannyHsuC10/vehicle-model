import copy
import json
import pickle
from pathlib import Path
import tempfile
import unittest

import numpy as np

from suspension_dynamics.project import default_project, save_project, load_project, estimate_damping
from suspension_dynamics.engine import simulate
from suspension_dynamics.storage import save_result, load_result, write_csv, write_report, import_legacy, fingerprint
from suspension_dynamics.charts import envelope


class StorageTests(unittest.TestCase):
    def setUp(self):
        self.temp_base=Path(__file__).resolve().parents[1]/'.test-tmp';self.temp_base.mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=self.temp_base);self.root=Path(self.temp.name)
        p=default_project();p['analysis']['full'].update(duration=.2,start=.01,dt=.003)
        self.result=simulate(p,'full')

    def tearDown(self):
        self.assertTrue(self.root.resolve().is_relative_to(self.temp_base.resolve()))
        self.temp.cleanup()

    def test_project_and_result_roundtrip_preserve_full_precision(self):
        r=self.result;path=self.root/'project.json';save_project(path,r.project)
        self.assertEqual(load_project(path),r.project)
        path=self.root/'result.npz';save_result(path,r);loaded=load_result(path)
        self.assertEqual(loaded.info,r.info);self.assertEqual(loaded.project,r.project)
        for name in r.signals:np.testing.assert_array_equal(r.signals[name],loaded.signals[name])

    def test_legacy_mass_fields_migrate_for_projects_and_preserve_result_snapshots(self):
        project = copy.deepcopy(self.result.project)
        project['analysis']['quarter'].pop('parameter_corner')
        full = project['parameters']['full']
        full.pop('mu_f')
        full.pop('mu_r')
        full.update(mu_FL=11., mu_FR=15., mu_RL=19., mu_RR=23.)
        path = self.root/'legacy.json'
        save_project(path, project)
        updated = load_project(path)
        self.assertEqual(updated['analysis']['quarter']['parameter_corner'], 'FL')
        self.assertEqual(updated['parameters']['full']['mu_f'], 13.)
        self.assertEqual(updated['parameters']['full']['mu_r'], 21.)
        self.assertNotIn('mu_FL', updated['parameters']['full'])
        original = simulate(project, 'full')
        save_result(self.root/'legacy.npz', original)
        loaded = load_result(self.root/'legacy.npz')
        self.assertEqual(loaded.project, original.project)
        self.assertNotEqual(loaded.signals['Normal load FL'][0], loaded.signals['Normal load FR'][0])

    def test_csv_and_report_include_units_and_input_snapshot(self):
        write_csv(self.root/'a.csv',self.result);write_report(self.root/'a.html',self.result)
        text=(self.root/'a.csv').read_text(encoding='utf-8-sig')
        self.assertIn('Normal load FL [N]',text);self.assertEqual(len(text.splitlines()),len(self.result.time)+1)
        report=(self.root/'a.html').read_text(encoding='utf-8')
        self.assertIn('180000',report);self.assertIn('body_twist_k',report)
        self.assertIn('<th>穩態平衡點</th>', report)
        self.assertIn('<th>2% 安定時間 (s)</th>', report)
        self.assertIn('<th>超越量 (%)</th>', report)

    def test_comparison_requires_identical_inputs_and_parameters(self):
        a=self.result;b=copy.deepcopy(a);b.project['analysis']['full']['model']='rigid'
        self.assertEqual(fingerprint(a),fingerprint(b))
        b.project['parameters']['full']['heave_f_k']+=1
        self.assertNotEqual(fingerprint(a),fingerprint(b))

    def test_envelope_retains_short_extreme(self):
        x=np.arange(10000);y=np.zeros(10000);y[7123]=123;y[13]=-55
        xd,yd=envelope(x,y,200)
        self.assertIn(123,yd);self.assertIn(-55,yd);self.assertLessEqual(len(xd),202)

    def test_damping_estimator_does_not_mutate_settings(self):
        p=default_project()['parameters']['quarter'];before=copy.deepcopy(p)
        estimated=estimate_damping('quarter',p)
        self.assertEqual(p,before)
        self.assertAlmostEqual(estimated['cs']*p['mr']**2,2*p['zeta']*np.sqrt(p['ks']*p['mr']**2*p['ms']))

    def test_all_supplied_legacy_archives_load_without_legacy_modules(self):
        source=Path(__file__).resolve().parents[2]/'new'/'Model_testing'/'batch_results'
        if not source.exists():self.skipTest('Original archives are optional for standalone distribution')
        files=list(source.glob('**/result.pkl'));self.assertTrue(files)
        for file in files:
            with self.subTest(file=file):
                r=import_legacy(file);self.assertTrue(r.legacy);self.assertIn('位移',r.groups)
                save_result(self.root/'old.npz',r);self.assertTrue(load_result(self.root/'old.npz').legacy)

    def test_pickle_executable_globals_are_rejected(self):
        class NotData:
            def __reduce__(self):return eval,('2+2',)
        path=self.root/'bad.pkl';path.write_bytes(pickle.dumps(NotData()))
        with self.assertRaises(ValueError):import_legacy(path)


if __name__=='__main__':unittest.main()
