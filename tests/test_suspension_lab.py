"""Contract checks for shared static provenance, constraints and exported projects."""
import copy
import csv
import gc
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from suspension_lab.project import default_project, load_project, save_project, validate, Car, SuspensionF, SuspensionR
from suspension_lab.engine import Engine, Unreachable, contact_frame, to_contact_frame, rotate2
from suspension_lab.analysis import calculate_mapping, make_plot, static_point_rows, PLOTS
from suspension_lab.export import point_rows, write_report
from suspension_lab.static.fv.wireframe_geometry import build_wireframe_scene

TEST_TMP = Path(__file__).resolve().parents[1] / ".test-tmp"
TEST_TMP.mkdir(exist_ok=True)


class ProjectTests(unittest.TestCase):
    def test_staged_app_runs_without_new_folder_or_legacy_imports(self):
        source = '''
import importlib.abc
from pathlib import Path
import runpy
import sys

root = Path.cwd()
assert not (root / 'new').exists()
sys.path.insert(0, str(root))

class NoLegacy(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('static', 'suspension_geo', 'new'):
            raise AssertionError('Legacy import: ' + fullname)
sys.meta_path.insert(0, NoLegacy())
search_path = sys.path.copy()

from suspension_lab.project import default_project, load_project, save_project
from suspension_lab.engine import Engine
from suspension_lab.analysis import calculate_mapping, make_plot
from suspension_lab.gui import App
from suspension_lab.export import write_report
from suspension_lab.static.fv.tk_wireframe import WireframeViewer

project = default_project()
project['analysis']['samples'] = 5
save_project(root / 'project.json', project)
engine = Engine(load_project(root / 'project.json'))
states = {'front': engine.fv['front'].solve(-10, 20),
          'rear': engine.fv['rear'].solve(15, -5),
          'sv': engine.solve_sv(-10, 20), 'we': engine.solve_we(20)}
for page, state in states.items():
    assert state['metrics']
    assert state['points']
    assert make_plot(page, 'motion', state, engine=engine)
    assert calculate_mapping(engine, page)['rows']
assert make_plot('static', 'iso', engine=engine)['world_lines']
assert sys.path == search_path
for name, module in tuple(sys.modules.items()):
    if name.split('.')[0] in ('suspension_lab', 'vehicle_lab'):
        assert Path(module.__file__).resolve().is_relative_to(root), module.__file__

sys.argv = ['suspension_lab', '--help']
try:
    runpy.run_module('suspension_lab', run_name='__main__')
except SystemExit as error:
    assert error.code == 0
else:
    raise AssertionError('CLI did not exit after --help')
'''
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            staged = Path(tmp)
            for package in ("suspension_lab", "vehicle_lab"):
                shutil.copytree(root / package, staged / package,
                                ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
            result = subprocess.run([sys.executable, "-I", "-B", "-c", source], cwd=staged,
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_roundtrip_and_reject_nonfinite_unknown_and_fractional_grid(self):
        project = default_project()
        project["front"]["fvsa"] = .923
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp) / "車輛.json"
            save_project(path, project)
            self.assertEqual(load_project(path), project)
            original = path.read_bytes()
            for group, key, value in (("front", "kpi_deg", math.nan), ("vehicle", "mass", True),
                                      ("analysis", "samples", 5.2), ("analysis", "roll_deg", 0)):
                bad = copy.deepcopy(project)
                bad[group][key] = value
                with self.assertRaises(ValueError):
                    save_project(path, bad)
                self.assertEqual(path.read_bytes(), original)
            project["front"]["unknown"] = 1
            with self.assertRaises(ValueError):
                validate(project)

    def test_engine_needs_no_legacy_design_defaults_or_plotting_scipy(self):
        with patch.object(Car, "__init__", side_effect=AssertionError("legacy defaults")), \
             patch.object(SuspensionF, "__init__", side_effect=AssertionError("legacy defaults")), \
             patch.object(SuspensionR, "__init__", side_effect=AssertionError("legacy defaults")):
            Engine(default_project())
        source = '''
import importlib.abc, sys
class NoPlotting(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('matplotlib', 'mpl_toolkits', 'scipy'):
            raise AssertionError(fullname)
sys.meta_path.insert(0, NoPlotting())
from suspension_lab.project import default_project
from suspension_lab.engine import Engine
e = Engine(default_project())
assert abs(e.fv['front'].solve(0,0)['metrics']['rc_y_prime_mm']-35)<1e-7
'''
        result = subprocess.run([sys.executable, "-B", "-c", source], capture_output=True, text=True,
                                cwd=Path(__file__).resolve().parents[1])
        self.assertEqual(result.returncode, 0, result.stderr)


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.engine = Engine(default_project())

    def test_default_scene_matches_authoritative_static_scene_exactly(self):
        baseline = build_wireframe_scene()
        self.assertEqual(len(baseline.lines), len(self.engine.scene.lines))
        for a, b in zip(baseline.lines, self.engine.scene.lines):
            np.testing.assert_allclose(a.points, b.points, atol=1e-12)
            self.assertEqual((a.color, a.group, a.dash), (b.color, b.group, b.dash))

    def test_fv_lengths_actual_travel_symmetry_and_reentrant_state(self):
        for axle, height in (("front", 35), ("rear", 40)):
            solver = self.engine.fv[axle]
            z = solver.zero
            zero = solver.solve(0, 0)
            self.assertAlmostEqual(zero["metrics"]["rc_y_prime_mm"], height, places=7)
            for travel in (-25, -10, 0, 10, 25):
                s = solver.side(travel)
                self.assertAlmostEqual(s["gnd"][1]*1000, travel, places=6)
                for a, b in (("cpb_ru", "kua"), ("cpb_rl", "lak"), ("kua", "lak"), ("wheel_center", "gnd")):
                    self.assertAlmostEqual(np.linalg.norm(s[a]-s[b]), np.linalg.norm(z[a]-z[b]), places=10)
                both = solver.solve(travel, travel)["metrics"]
                self.assertAlmostEqual(both["rc_x_prime_mm"], 0, places=6)
                self.assertAlmostEqual(both["camber_left_deg"], both["camber_right_deg"], places=8)
            a, b = solver.solve(-15, 20)["metrics"], solver.solve(20, -15)["metrics"]
            self.assertAlmostEqual(a["roll_deg"], -b["roll_deg"], places=7)
            self.assertAlmostEqual(a["rc_x_prime_mm"], -b["rc_x_prime_mm"], places=7)
            self.assertEqual(solver.solve(0, 0), zero)
            with self.assertRaises(Unreachable):
                solver.solve(1000, 0)

    def test_fv_pose_and_gain_partial_derivative(self):
        solver = self.engine.fv["front"]
        state = solver.pose(5, 1.2)["metrics"]
        self.assertAlmostEqual(state["heave_mm"], 5, places=7)
        self.assertAlmostEqual(state["roll_deg"], 1.2, places=6)
        s = solver.solve(10, -5)["metrics"]
        lo = solver.solve(9.9, -5, False)["metrics"]
        hi = solver.solve(10.1, -5, False)["metrics"]
        self.assertAlmostEqual(s["camber_gain_left_deg_per_mm"], (hi["camber_left_deg"]-lo["camber_left_deg"])/.2, places=6)

    def test_sv_tangent_constraints_rigidity_and_pose_roundtrip(self):
        baseline = self.engine.solve_sv(0, 0)
        for axle in ("front", "rear"):
            motion = self.engine.sv[axle]
            initial = motion.template.initial_ic.copy()
            for travel in (-25, 0, 25):
                s = motion.side(travel)
                self.assertAlmostEqual(s["contact"][1]*1000, travel, places=6)
                self.assertAlmostEqual(np.linalg.norm(s["au"]-s["al"]), motion.length, places=10)
                for k, p, d in (("au", motion.au, motion.directions[0]), ("al", motion.al, motion.directions[1])):
                    displacement = s[k]-p
                    self.assertAlmostEqual(np.dot(displacement, [-d[1], d[0]]), 0, places=10)
            np.testing.assert_array_equal(motion.template.initial_ic, initial)
        pose = self.engine.sv_pose(4, -.6)["metrics"]
        self.assertAlmostEqual(pose["heave_mm"], 4, places=7)
        self.assertAlmostEqual(pose["pitch_deg"], -.6, places=6)
        self.assertEqual(self.engine.solve_sv(0, 0), baseline)

    def test_svsa_override_remains_in_static_source(self):
        project = default_project()
        project["front"]["svsa"] = 2.5
        e = Engine(project)
        static = e.scene.geometry["side_view"].front
        self.assertAlmostEqual(abs(static.initial_ic[0]-static.contact_point[0]), 2.5)
        state = e.solve_sv(0, 0)["metrics"]
        self.assertAlmostEqual(state["anti_dive_front_pct"], 20)
        self.assertAlmostEqual(state["svsa_front_mm"], 2500)
        self.assertNotAlmostEqual(state["anti_lift_front_pct"], 0)

    def test_we_uses_static_axis_and_distinguishes_material_from_contact(self):
        for axle, corner, caster in (("front", "FR", 5), ("rear", "RR", 0)):
            initial = self.engine.solve_we(0, axle)
            np.testing.assert_allclose(initial["points"]["kingpin_ground"], self.engine.scene.geometry["kingpins"][corner]["ground"])
            for steering in (-30, 0, 30):
                s = self.engine.solve_we(steering, axle)
                self.assertAlmostEqual(s["metrics"]["caster_deg"], caster, places=10)
                self.assertAlmostEqual(s["metrics"]["material_radius_mm"], initial["metrics"]["material_radius_mm"], places=9)
                np.testing.assert_allclose(s["points"]["center"], self.engine.solve_we(steering, axle, caster)["points"]["center"], atol=1e-12)
                c, p = np.array(s["points"]["center"]), np.array(s["points"]["contact"])
                self.assertAlmostEqual(np.linalg.norm(c-p), self.engine.car.load_Radius[0 if axle == "front" else 1], places=10)
            self.assertNotAlmostEqual(initial["metrics"]["contact_radius_mm"], self.engine.solve_we(30, axle)["metrics"]["contact_radius_mm"], places=3)

    def test_mapping_masks_failures_and_has_all_requested_metrics(self):
        p = default_project()
        p["analysis"].update(samples=5, travel_mm=200)
        e = Engine(p)
        mapping = calculate_mapping(e, "front")
        self.assertEqual(mapping["total"], 25)
        self.assertTrue(any(not r["valid"] for r in mapping["rows"]))
        self.assertTrue(any(r["valid"] for r in mapping["rows"]))
        for page, mode in (("front", "pose"), ("rear", "travel"), ("sv", "travel"), ("sv", "pose"), ("we", "travel")):
            mapping = calculate_mapping(e, page, mode)
            for plot, _ in PLOTS[page]:
                if plot != "motion":
                    self.assertIsNotNone(make_plot(page, plot, mapping=mapping))
        self.assertIsNone(calculate_mapping(e, "front", cancelled=lambda: True))
        names = [r["name"] for r in static_point_rows(e)]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("FR_UBJ", names)


class ContactCoordinateTests(unittest.TestCase):
    def test_contact_coordinates_are_invariant_under_rigid_world_motion(self):
        first, second, point = np.array([-.6, .02]), np.array([.65, .08]), np.array([.12, .4])
        frame = contact_frame(first, second)
        basis = np.array([frame["x_axis"], frame["y_axis"]])
        np.testing.assert_allclose(basis@basis.T, np.eye(2), atol=1e-12)
        distance = np.linalg.norm(second-first)
        np.testing.assert_allclose(to_contact_frame(first, frame), [-distance/2, 0], atol=1e-12)
        np.testing.assert_allclose(to_contact_frame(second, frame), [distance/2, 0], atol=1e-12)
        matrix, shift = rotate2(.38), np.array([3., -2.])
        moved = contact_frame(matrix@first+shift, matrix@second+shift)
        np.testing.assert_allclose(to_contact_frame(matrix@point+shift, moved), to_contact_frame(point, frame), atol=1e-12)

    def test_rc_and_pc_use_contact_midpoint_and_perpendicular_height(self):
        e = Engine(default_project())
        for page in ("front", "rear", "sv"):
            for a, b in ((0, 0), (15, 15), (-10, 20), (20, -10)):
                with self.subTest(page=page, travel=(a, b)):
                    state = e.solve_sv(a, b) if page == "sv" else e.fv[page].solve(a, b)
                    p = state["points"]
                    first, second = (p["front_contact"], p["rear_contact"]) if page == "sv" else (p["L_gnd"], p["R_gnd"])
                    first, second = np.array(first), np.array(second)
                    u = (second-first)/np.linalg.norm(second-first)
                    n = np.array([-u[1], u[0]])
                    prefix, key = ("pc", "PC") if page == "sv" else ("rc", "RC")
                    delta = np.array(p[key])-(first+second)/2
                    m = state["metrics"]
                    self.assertAlmostEqual(m[prefix+"_x_prime_mm"], np.dot(delta, u)*1000, places=8)
                    self.assertAlmostEqual(m[prefix+"_y_prime_mm"], np.dot(delta, n)*1000, places=8)
                    np.testing.assert_allclose([m[prefix+"_world_x_mm"], m[prefix+"_world_z_mm"]], np.array(p[key])*1000)
                    zero = e.solve_sv(0, 0) if page == "sv" else e.fv[page].solve(0, 0)
                    np.testing.assert_allclose(p[key+"0"], zero["points"][key], atol=1e-12)
                    np.testing.assert_allclose(p["CG"], zero["points"]["CG"])
                    row = next(r for r in point_rows(state) if r["name"] == key)
                    self.assertAlmostEqual(row["x_prime_m"]*1000, m[prefix+"_x_prime_mm"])
                    self.assertAlmostEqual(row["y_prime_m"]*1000, m[prefix+"_y_prime_mm"])

    def test_fv_position_surfaces_and_distribution_use_rotating_frame(self):
        project = default_project()
        project["analysis"]["samples"] = 5
        e = Engine(project)
        for page in ("front", "rear"):
            self.assertIn("rc_x_prime_mm", dict(PLOTS[page]))
            self.assertIn("rc_y_prime_mm", dict(PLOTS[page]))
            mapping = calculate_mapping(e, page)
            for metric in ("rc_x_prime_mm", "rc_y_prime_mm"):
                spec = make_plot(page, metric, mapping=mapping)
                for grid_row, mesh_row in zip(mapping["grid"], spec["mesh"]):
                    self.assertEqual([r.get(metric) for r in grid_row], [p[2] for p in mesh_row])
            distribution = make_plot(page, "rc_map", mapping=mapping)
            self.assertEqual((distribution["xlabel"], distribution["ylabel"]), ("x′ (mm)", "y′ (mm)"))
            self.assertEqual(distribution["series"][0]["x"], [r["rc_x_prime_mm"] for r in mapping["grid"][0]])
            self.assertEqual(distribution["series"][0]["y"], [r["rc_y_prime_mm"] for r in mapping["grid"][0]])

    def test_undefined_rc_is_not_rendered_as_a_stale_point(self):
        e = Engine(default_project())
        with patch("suspension_lab.engine.intersection", return_value=None):
            state = e.fv["front"].solve(0, 0)
        self.assertIsNone(state["metrics"]["rc_x_prime_mm"])
        self.assertIsNone(state["metrics"]["rc_y_prime_mm"])
        self.assertNotIn("RC", state["points"])
        markers = make_plot("front", "motion", state)["markers"]
        self.assertNotIn("RC", {m["name"] for m in markers})
        self.assertIn("RC0", {m["name"] for m in markers})


class ChartScaleTests(unittest.TestCase):
    def test_equal_geometry_scale_survives_resize_zoom_and_pose_updates(self):
        from suspension_lab.charts import LabChart
        chart = object.__new__(LabChart)
        chart.canvas = Mock()
        chart.canvas.winfo_width.return_value = 1000
        chart.canvas.winfo_height.return_value = 470
        chart.spec = chart.view = None
        chart.primitives = chart.polygons = []
        chart.azimuth, chart.elevation = .65, .6
        engine = Engine(default_project())
        for page in ("front", "rear", "sv", "we", "static"):
            with self.subTest(page=page):
                state = (engine.fv[page].solve(0, 0) if page in ("front", "rear") else
                         engine.solve_sv(0, 0) if page == "sv" else engine.solve_we(0))
                chart.set(make_plot(page, "iso" if page == "static" else "motion", state, engine=engine))
                def scale():
                    l, t, r, b = chart.bounds
                    xmin, xmax, ymin, ymax = chart.view
                    xscale, yscale = (r-l)/(xmax-xmin), (b-t)/(ymax-ymin)
                    self.assertAlmostEqual(xscale, yscale, places=9)
                    return xscale
                scale()
                l, t, r, b = chart.bounds
                chart.zoom(SimpleNamespace(x=(l+r)/2, y=(t+b)/2, delta=120))
                expected_scale = scale()
                centre = ((chart.view[0]+chart.view[1])/2, (chart.view[2]+chart.view[3])/2)
                for width, height in ((650, 350), (1250, 610), (1000, 470)):
                    chart.canvas.winfo_width.return_value = width
                    chart.canvas.winfo_height.return_value = height
                    chart.draw()
                    self.assertAlmostEqual(scale(), expected_scale, places=9)
                    np.testing.assert_allclose([(chart.view[0]+chart.view[1])/2, (chart.view[2]+chart.view[3])/2], centre, atol=1e-12)
                chart.set(chart.spec, preserve_view=True)
                self.assertAlmostEqual(scale(), expected_scale, places=9)

    def test_round_point_markers_and_axes_are_present_in_svg_and_png(self):
        from io import BytesIO
        import xml.etree.ElementTree as ET
        from PIL import Image
        from suspension_lab.charts import LabChart
        chart = object.__new__(LabChart)
        chart.canvas = Mock()
        chart.canvas.winfo_width.return_value = 1200
        chart.canvas.winfo_height.return_value = 600
        chart.spec = chart.view = None
        chart.primitives = []
        chart.azimuth, chart.elevation = .65, .6
        state = Engine(default_project()).fv["front"].solve(-10, 20)
        spec = make_plot("front", "motion", state)
        # Reference axes and point labels survive hiding force lines.
        spec["world_lines"] = [p for p in spec["world_lines"] if not p["construction"]]
        chart.set(spec)
        self.assertEqual({c[-1] for c in chart.circles}, {"RC", "RC0", "CG", "IC L", "IC R"})
        labels = {p[4] for p in chart.primitives if p[0] == "text"}
        self.assertTrue({"x′", "y′"} <= labels)
        root = ET.fromstring(chart.svg())
        circles = root.findall("{http://www.w3.org/2000/svg}circle")
        self.assertEqual(len(circles), 5)
        original = next(c for c in circles if c.attrib["data-label"] == "RC0")
        self.assertEqual(original.attrib["fill"], "none")
        stream = BytesIO()
        stream.name = "points.png"
        chart.png(stream)
        cg = next(c for c in chart.circles if c[-1] == "CG")
        with Image.open(stream) as im:
            self.assertEqual(im.getpixel((round(cg[0]*2), round(cg[1]*2))), (251, 191, 36))
        view = chart.view.copy()
        chart.set(make_plot("front", "motion", Engine(default_project()).fv["front"].solve(15, -5)), preserve_view=True)
        self.assertEqual(chart.view, view)


@unittest.skipUnless(os.environ.get("SUSPENSION_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class GuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_lab.gui import App
        p = default_project()
        p["analysis"]["samples"] = 5
        self.app = App(p)
        self.app.update()

    def tearDown(self):
        self.app.dirty = False
        self.app.close()
        self.app.worker_thread.join(timeout=5)
        self.assertFalse(self.app.worker_thread.is_alive())
        self.app = None
        # Tcl variables must be finalized on the main thread between apps.
        gc.collect()

    def wait(self, panel):
        end = time.monotonic()+20
        while not getattr(panel, "ready_for_export", False) and time.monotonic() < end:
            self.app.update()
            time.sleep(.01)
        self.assertTrue(panel.ready_for_export, self.app.status.get())
        self.assertIsNone(panel.error)
        self.app.update()

    def test_every_plot_exports_real_png_svg_csv_report(self):
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            for key, panel in self.app.panels.items():
                self.app.notebook.select(panel)
                self.app.update()
                for plot, label in PLOTS[key]:
                    panel.plot.set(label)
                    panel.request()
                    self.wait(panel)
                    self.assertIsNotNone(panel.chart.spec)
                    self.assertTrue(panel.note.winfo_ismapped())
                    self.assertLessEqual(panel.note.winfo_y()+panel.note.winfo_height(), panel.note.master.winfo_height())
                    self.assertTrue(panel.metrics.winfo_ismapped())
                    svg = panel.chart.svg()
                    self.assertIn("<svg", svg)
                    if "mesh" in panel.chart.spec:
                        self.assertIn("<polygon", svg)
                    png = Path(tmp)/f"{key}_{plot}.png"
                    panel.chart.png(png)
                    self.assertGreater(png.stat().st_size, 1000)
                for kind, suffix in (("state", "csv"), ("points", "csv"), ("report", "html"), ("svg", "svg")):
                    path = Path(tmp)/f"{key}_{kind}.{suffix}"
                    self.app.perform_export(kind, path, panel)
                    self.assertTrue(path.exists())
                if key != "static":
                    path = Path(tmp)/f"{key}_mapping.csv"
                    self.app.perform_export("mapping", path, panel)
                    with path.open(encoding="utf-8-sig", newline="") as handle:
                        self.assertEqual(len(list(csv.DictReader(handle))), panel.mapping["total"])

    def test_parameter_commit_invalidates_pages_and_rejects_invalid_geometry(self):
        panel = self.app.active
        self.wait(panel)
        self.assertAlmostEqual(panel.state["metrics"]["rc_y_prime_mm"], 35)
        p = copy.deepcopy(self.app.project)
        p["front"]["rc_height"] = .048
        self.app.apply_project(p)
        self.wait(panel)
        self.assertAlmostEqual(panel.state["metrics"]["rc_y_prime_mm"], 48)
        self.assertEqual(self.app.panels["static"].entries[("front", "rc_height")].get(), "0.048")
        rev = self.app.revision
        invalid = copy.deepcopy(p)
        invalid["front"]["anti_dive"] = 0
        invalid["front"]["anti_lift"] = 0
        with self.assertRaises(ValueError):
            self.app.apply_project(invalid)
        self.assertEqual(self.app.revision, rev)
        self.assertEqual(self.app.project, p)
        from suspension_lab.gui import ParameterEditor
        editor = ParameterEditor(self.app)
        self.app.update()
        self.assertEqual(len(editor.entries), sum(map(len, __import__('suspension_lab.project', fromlist=['FIELDS']).FIELDS.values())))
        editor.destroy()

    def test_sliders_preserve_viewport_zoom_pan_and_3d_camera(self):
        for key in ("front", "rear", "sv", "we"):
            with self.subTest(page=key):
                panel = self.app.panels[key]
                self.app.notebook.select(panel)
                self.app.update()
                panel.request()
                self.wait(panel)
                chart = panel.chart
                initial_view = chart.view.copy()
                # Ordinary slider motion must not change the fitted axis limits.
                panel.controls[0].set(8.)
                panel.slider_changed()
                self.wait(panel)
                self.assertEqual(chart.view, initial_view)
                l, t, r, b = chart.bounds
                chart.zoom(SimpleNamespace(x=(l+r)/2, y=(t+b)/2, delta=120))
                chart.drag = (100, 100)
                chart.pan(SimpleNamespace(x=145, y=120))
                camera = (chart.view.copy(), chart.azimuth, chart.elevation)
                old_geometry = chart.spec["world_lines"]
                panel.controls[0].set(-10.)
                panel.slider_changed()
                self.wait(panel)
                self.assertNotEqual(chart.spec["world_lines"], old_geometry)
                self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
                # A temporary unreachable pose must not discard the camera either.
                panel.receive(None, None, "unreachable pose")
                self.assertEqual(chart.view, camera[0])
                panel.request()
                self.wait(panel)
                self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
                # Explicit reset still fits the current geometry and restores angles.
                chart.reset()
                self.assertNotEqual(chart.view, camera[0])
                self.assertEqual((chart.azimuth, chart.elevation), (.65, .6))

    def test_continuous_slider_movement_displays_intermediate_solutions(self):
        panel = self.app.panels["front"]
        self.wait(panel)
        original_solve = self.app.engine.fv["front"].solve
        def slow_solve(*args, **kwargs):
            # Even a solve slower than the input rate must produce live frames.
            time.sleep(.04)
            return original_solve(*args, **kwargs)
        seen = set()
        with patch.object(self.app.engine.fv["front"], "solve", side_effect=slow_solve):
            start = time.monotonic()
            while time.monotonic()-start < .8:
                value = -15+30*(time.monotonic()-start)/.8
                panel.sliders[0].set(value)
                self.app.update()
                seen.add(panel.state["metrics"]["travel_left_mm"])
                time.sleep(.008)
            # There has been no release or pause in the input stream yet.
            self.assertGreaterEqual(len(seen-{0.}), 2)
            panel.sliders[0].set(20.)
            self.wait(panel)
            self.assertAlmostEqual(panel.state["metrics"]["travel_left_mm"], 20.)
            self.assertFalse(panel._request_in_flight)

    def test_initial_page_fits_mapped_canvas_instead_of_hidden_canvas(self):
        for key in ("front", "rear", "sv", "we"):
            panel = self.app.panels[key]
            self.app.notebook.select(panel)
            self.app.update()
            panel.request()
            self.wait(panel)
            chart = panel.chart
            projection = chart.spec["projection"]
            points = [p for path in chart.spec["world_lines"] if not path["construction"] and path.get("fit", True)
                      for p in path["points"]]
            xs = [p[0] if len(p) == 2 else chart.project(p, projection)[0] for p in points]
            fraction = (max(xs)-min(xs))/(chart.view[1]-chart.view[0])
            # The tall 3D kingpin leaves more horizontal space than 2D views.
            self.assertGreater(fraction, .15 if key == "we" else .3, (key, fraction, chart.view))

    def test_static_reference_checkbox_hides_construction_preserving_camera(self):
        import html
        panel = self.app.panels["static"]
        self.app.notebook.select(panel)
        self.app.update()
        scene = self.app.engine.scene
        source_points = [path.points.copy() for path in scene.lines]
        for plot, label in PLOTS["static"]:
            panel.plot.set(label)
            panel.request()
            self.wait(panel)
            chart = panel.chart
            full_lines = copy.deepcopy(chart.spec["world_lines"])
            l, t, r, b = chart.bounds
            chart.zoom(SimpleNamespace(x=(l+r)/2, y=(t+b)/2, delta=120))
            camera = (chart.view.copy(), chart.azimuth, chart.elevation)
            panel.static_references_check.invoke()
            self.app.update()
            hidden = [path["points"] for path in chart.spec["world_lines"]]
            self.assertLess(len(hidden), len(full_lines))
            for wheel in scene.geometry["kingpins"].values():
                self.assertNotIn(np.array([wheel["ground"], wheel["axis_top"]]).tolist(), hidden)
                self.assertIn(np.array([wheel["LBJ"], wheel["UBJ"]]).tolist(), hidden)
            for plane in scene.geometry["arm_planes"].values():
                self.assertNotIn(np.vstack((plane["vertices"], plane["vertices"][0])).tolist(), hidden)
            self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
            if plot == "iso":
                with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
                    report = Path(tmp)/"hidden-reference.html"
                    self.app.perform_export("report", report, panel)
                    self.assertIn('"static_references": false', html.unescape(report.read_text(encoding="utf-8")))
                    chart.png(Path(tmp)/"hidden-reference.png")
            panel.static_references_check.invoke()
            self.app.update()
            self.assertEqual(chart.spec["world_lines"], full_lines)
            self.assertEqual((chart.view, chart.azimuth, chart.elevation), camera)
        for path, original in zip(scene.lines, source_points):
            np.testing.assert_array_equal(path.points, original)


if __name__ == "__main__":
    unittest.main()
