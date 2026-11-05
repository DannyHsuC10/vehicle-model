"""Live pose markers agree with sweep coordinates and all export projections."""
import copy
from io import BytesIO
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np

from suspension_geometry_lab.analysis import calculate_mapping, make_plot, PLOTS
from suspension_geometry_lab.charts import LabChart
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.project import default_project


def chart_for(spec):
    chart = object.__new__(LabChart)
    chart.canvas = Mock()
    chart.canvas.winfo_width.return_value = 1000
    chart.canvas.winfo_height.return_value = 600
    chart.canvas.winfo_ismapped.return_value = True
    chart.spec = chart.view = chart._plot_bounds = None
    chart.primitives, chart.polygons, chart.circles = [], [], []
    chart.azimuth, chart.elevation = .65, .6
    chart.set(spec)
    return chart


class CurrentPositionTests(unittest.TestCase):
    def setUp(self):
        p = default_project()
        p["analysis"]["samples"] = 5
        self.engine = Engine(p)

    def test_surfaces_and_center_maps_use_exact_live_pose_in_both_modes(self):
        e = self.engine
        for page in ("front", "rear", "sv"):
            for mode in ("travel", "pose"):
                if page == "sv":
                    state = e.solve_sv(-7, 13) if mode == "travel" else e.sv_pose(4, .5)
                else:
                    state = e.fv[page].solve(-7, 13) if mode == "travel" else e.fv[page].pose(4, .5)
                m = state["metrics"]
                axes = ([m["heave_mm"], m["pitch_deg" if page == "sv" else "roll_deg"]] if mode == "pose" else
                        [m["ride_height_front_mm"], m["ride_height_rear_mm"]] if page == "sv" else [-7, 13])
                mapping = calculate_mapping(e, page, mode)
                original = copy.deepcopy(mapping)
                for key, _ in PLOTS[page]:
                    if key == "motion":
                        continue
                    with self.subTest(page=page, mode=mode, plot=key):
                        spec = make_plot(page, key, state, mapping, e)
                        self.assertEqual(len(spec["markers"]), 1, spec["current_status"])
                        point = spec["markers"][0]["position"]
                        if key in ("rc_map", "pc_map"):
                            prefix = "pc" if page == "sv" else "rc"
                            np.testing.assert_allclose(point, [m[prefix+"_x_prime_mm"], m[prefix+"_y_prime_mm"]])
                        else:
                            np.testing.assert_allclose(point, [*axes, m[key]], atol=1e-9)
                self.assertEqual(mapping, original)

    def test_wheel_end_surface_curves_and_path_use_live_values_for_both_axles(self):
        e = self.engine
        for axle in ("front", "rear"):
            state = e.solve_we(7.3, axle)
            m = state["metrics"]
            mapping = calculate_mapping(e, "we", axle=axle)
            for key, _ in PLOTS["we"]:
                if key == "motion":
                    continue
                spec = make_plot("we", key, state, mapping, e)
                self.assertTrue(spec["markers"], spec["current_status"])
                if key == "caster_surface":
                    np.testing.assert_allclose(spec["markers"][0]["position"], [7.3, m["caster_deg"], m["wheel_tilt_deg"]])
                elif key == "path":
                    np.testing.assert_allclose(spec["markers"][0]["position"], [m[f"contact_{k}_mm"]/1000 for k in "xyz"])
                else:
                    self.assertEqual(len(spec["markers"]), len(spec["series"]))
                    for marker, series in zip(spec["markers"], spec["series"]):
                        np.testing.assert_allclose(marker["position"], [7.3, m[series["name"]]])

    def test_missing_undefined_and_outside_current_values_never_retain_old_point(self):
        e = self.engine
        mapping = calculate_mapping(e, "front")
        state = e.fv["front"].solve(0, 0)
        chart = chart_for(make_plot("front", "rc_y_prime_mm", state, mapping, e))
        self.assertEqual(len(chart.circles), 1)
        state["metrics"]["rc_y_prime_mm"] = None
        undefined = make_plot("front", "rc_y_prime_mm", state, mapping, e)
        self.assertEqual(undefined["markers"], [])
        self.assertIn("N/A", undefined["current_status"])
        chart.set(undefined, preserve_view=True)
        self.assertEqual(chart.circles, [])
        missing = make_plot("front", "rc_map", None, mapping, e)
        self.assertEqual(missing["markers"], [])
        self.assertIn("無有效解", missing["current_status"])
        project = default_project()
        project["analysis"].update(samples=5, caster_min_deg=10, caster_max_deg=15)
        engine = Engine(project)
        mapping = calculate_mapping(engine, "we")
        state = engine.solve_we(0)
        outside = make_plot("we", "caster_surface", state, mapping, engine)
        self.assertEqual(outside["markers"], [])
        self.assertIn("超出掃描範圍", outside["current_status"])
        # The fixed-caster steering curves are valid even outside the caster surface range.
        self.assertEqual(len(make_plot("we", "radius", state, mapping, engine)["markers"]), 2)

    def test_static_ride_height_labels_include_zero_travel_without_axis_shift(self):
        e = self.engine
        mapping = calculate_mapping(e, "sv")
        spec = make_plot("sv", "caster_front_deg", e.solve_sv(0, 0), mapping, e)
        point = spec["markers"][0]
        np.testing.assert_allclose(point["position"], [50, 50, 5], atol=1e-10)
        self.assertIn("前／後行程: 0 / 0 mm", point["label_lines"])
        self.assertIn("Front ride height (mm): 50", point["label_lines"])

    def test_surface_marker_matches_mesh_projection_and_exports_after_rotation(self):
        from PIL import Image
        e = self.engine
        mapping = calculate_mapping(e, "sv")
        spec = make_plot("sv", "caster_front_deg", e.solve_sv(0, 0), mapping, e)
        chart = chart_for(spec)
        for rotated in (False, True):
            if rotated:
                chart.azimuth += .35
                chart.draw()
            circle = chart.circles[0]
            vertices = [np.array(coords).reshape(-1, 2) for coords, _ in chart.polygons]
            self.assertTrue(any(np.linalg.norm(vertex-np.array(circle[:2])) < 1e-8 for face in vertices for vertex in face))
            root = ET.fromstring(chart.svg())
            exported = root.find("{http://www.w3.org/2000/svg}circle")
            self.assertEqual(exported.attrib["data-label"], "目前位置")
            self.assertAlmostEqual(float(exported.attrib["cx"]), circle[0], places=2)
            text = [p[4] for p in chart.primitives if p[0] == "text"]
            self.assertIn("前／後行程: 0 / 0 mm", text)
            stream = BytesIO()
            stream.name = "surface-marker.png"
            chart.png(stream)
            with Image.open(stream) as image:
                self.assertEqual(image.getpixel((round(circle[0]*2), round(circle[1]*2))), (251, 113, 133))

    def test_curve_distribution_and_3d_path_markers_render_and_follow_without_refit(self):
        e = self.engine
        for page, key in (("we", "xyz"), ("we", "path"), ("front", "rc_map")):
            mapping = calculate_mapping(e, page)
            states = [e.solve_we(x) if page == "we" else e.fv[page].solve(x, -x) for x in (0, 7)]
            chart = chart_for(make_plot(page, key, states[0], mapping, e))
            view, camera = chart.view.copy(), (chart.azimuth, chart.elevation)
            initial = [point[:2] for point in chart.circles]
            chart.set(make_plot(page, key, states[1], mapping, e), preserve_view=True)
            self.assertTrue(chart.circles)
            self.assertNotEqual(initial, [point[:2] for point in chart.circles])
            self.assertEqual(chart.view, view)
            self.assertEqual((chart.azimuth, chart.elevation), camera)
            svg = chart.svg()
            self.assertIn("<circle", svg)
            self.assertTrue(all("目前" in point[-1] for point in chart.circles))


if __name__ == "__main__":
    unittest.main()
