"""Geometry preservation and interaction checks for the static Tk preview."""

import math
from pathlib import Path
import subprocess
import sys
import tkinter as tk
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]

from suspension_geometry_lab.static.fv.tk_wireframe import WireframeViewer, project_points
from suspension_geometry_lab.static.fv.wireframe_geometry import build_wireframe_scene


class WireframeGeometryTests(unittest.TestCase):
    def test_preview_import_and_geometry_do_not_need_matplotlib(self):
        source = """
import importlib.abc
import sys
class NoMatplotlib(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('matplotlib', 'mpl_toolkits'):
            raise AssertionError('Preview tried to import ' + fullname)
sys.meta_path.insert(0, NoMatplotlib())
from suspension_geometry_lab.static.fv.tk_wireframe import build_wireframe_scene
scene = build_wireframe_scene()
assert len(scene.geometry['kingpins']) == 4
assert len(scene.geometry['arm_planes']) == 8
assert all(len(line.points) >= 2 for line in scene.lines)
"""
        result = subprocess.run([sys.executable, "-B", "-c", source], cwd=ROOT,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_wheel_centers_keep_fv_vehicle_coordinates(self):
        scene = build_wireframe_scene()
        for prefix, plane in (("F", scene.geometry["front"]), ("R", scene.geometry["rear"])):
            for suffix, side in (("L", plane.left_sus), ("R", plane.right_sus)):
                wheel = scene.geometry["wheel_projection_geometry"][prefix + suffix]
                np.testing.assert_allclose(wheel["center"],
                                           [plane.longitudinal_x, -side.wheel_center.x,
                                            side.wheel_center.y], atol=1e-12)
        for plane in scene.geometry["arm_planes"].values():
            self.assertTrue(np.all(plane["vertices"] >= scene.bounds[:, 0] - 1e-12))
            self.assertTrue(np.all(plane["vertices"] <= scene.bounds[:, 1] + 1e-12))

    def test_projection_keeps_metric_scale_and_expected_front_axes(self):
        points = np.eye(3)
        np.testing.assert_allclose(project_points(points, 0, 0), [[0, 0], [1, 0], [0, 1]])
        for azimuth, elevation in ((0, 0), (.5, .3), (-2.1, 1.2), (0, math.pi / 2)):
            basis = project_points(points, azimuth, elevation)
            np.testing.assert_allclose(basis.T @ basis, np.eye(2), atol=1e-12)


class WireframeInteractionTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(str(error))
        self.root.withdraw()
        self.viewer = WireframeViewer(self.root)
        self.viewer.render()

    def tearDown(self):
        if hasattr(self, "root"):
            self.root.destroy()

    def test_only_lines_and_fixed_items_during_interaction(self):
        viewer = self.viewer
        items = viewer.canvas.find_all()
        world_points = viewer._points.copy()
        self.assertEqual({viewer.canvas.type(item) for item in items}, {"line"})
        initial_coords = viewer.canvas.coords(items[0])
        viewer._begin_rotate(SimpleNamespace(x=100, y=100, state=0))
        viewer._move(SimpleNamespace(x=180, y=130))
        viewer.render()
        self.assertNotEqual(initial_coords, viewer.canvas.coords(items[0]))
        viewer._begin_pan(SimpleNamespace(x=100, y=100))
        viewer._move(SimpleNamespace(x=150, y=120))
        np.testing.assert_allclose(viewer.pan, [50, 20])
        viewer.zoom_at(1.5, 300, 240)
        for group in viewer.groups.values():
            group.set(True)
        viewer.render()
        self.assertEqual(viewer.canvas.find_all(), items)
        np.testing.assert_array_equal(viewer._points, world_points)

    def test_zoom_anchors_pointer_and_visibility_preserves_camera(self):
        viewer = self.viewer
        item = viewer.items[0]
        before = viewer.canvas.coords(item)
        viewer.zoom_at(1.5, before[0], before[1])
        viewer.render()
        np.testing.assert_allclose(viewer.canvas.coords(item)[:2], before[:2], atol=1e-8)
        camera = (viewer.azimuth, viewer.elevation, viewer.scale, viewer.center.copy(), viewer.pan.copy())
        for value in viewer.groups.values():
            value.set(False)
        viewer.render()
        self.assertEqual((viewer.azimuth, viewer.elevation, viewer.scale), camera[:3])
        np.testing.assert_array_equal(viewer.center, camera[3])
        np.testing.assert_array_equal(viewer.pan, camera[4])
        for line, item in zip(viewer.scene.lines, viewer.items):
            self.assertEqual(viewer.canvas.itemcget(item, "state"),
                             "normal" if line.group == "physical" else "hidden")

    def test_resize_round_trip_preserves_zoom(self):
        viewer = self.viewer
        viewer._last_size = (1200, 760)
        scale = viewer.scale
        for _ in range(3):
            viewer._resize(SimpleNamespace(width=600, height=760))
            viewer._resize(SimpleNamespace(width=1200, height=760))
        self.assertAlmostEqual(viewer.scale, scale)


if __name__ == "__main__":
    unittest.main()
