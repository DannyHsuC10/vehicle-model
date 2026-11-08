"""CG, anti normalization and drawing share the current road/contact frame."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab.analysis import calculate_mapping, make_plot
from suspension_geometry_lab.engine import Engine, rotate2
from suspension_geometry_lab.export import point_rows
from suspension_geometry_lab.project import default_project


def cg_marker(plot):
    return np.array(next(p["position"] for p in plot["markers"] if p["name"] == "CG"))


class PoseFrameTests(unittest.TestCase):
    def test_sv_heave_moves_body_cg_relative_to_road_and_zero_restores(self):
        engine = Engine(default_project())
        zero = engine.solve_sv(0, 0)
        original = cg_marker(make_plot("sv", "motion", zero))
        for travel in (-15, 15):
            state = engine.solve_sv(travel, travel)
            plot = make_plot("sv", "motion", state)
            self.assertAlmostEqual(cg_marker(plot)[1], original[1]-travel/1000, places=8)
            self.assertAlmostEqual(state["metrics"]["cg_height_mm"], original[1]*1000-travel, places=6)
            # The rigid body's CG must not move relative to the body itself.
            np.testing.assert_array_equal(state["points"]["CG"], zero["points"]["CG"])
        self.assertEqual(engine.solve_sv(0, 0), zero)

    def test_pitch_roll_geometry_annotations_and_csv_use_one_transform(self):
        engine = Engine(default_project())
        for page in ("front", "rear", "sv"):
            state = engine.solve_sv(-10, 20) if page == "sv" else engine.fv[page].solve(-10, 20)
            original = copy.deepcopy(state)
            plot = make_plot(page, "motion", state)
            frame = state["frame"]
            first, second = ((state["points"]["front_contact"], state["points"]["rear_contact"])
                             if page == "sv" else (state["points"]["L_gnd"], state["points"]["R_gnd"]))
            first, second = np.array(first), np.array(second)
            tangent = (second-first)/np.linalg.norm(second-first)
            normal = np.array([-tangent[1], tangent[0]])
            origin = (first+second)/2
            cg = np.array(state["points"]["CG"])
            expected = np.array([np.dot(cg-origin, tangent), np.dot(cg-origin, normal)])
            np.testing.assert_allclose(cg_marker(plot), expected, atol=1e-12)
            self.assertEqual(plot["coordinate_system"], "contact")
            self.assertEqual(state, original)
            self.assertNotEqual(frame["angle_deg"], 0)
            ground = next(path for path in plot["world_lines"] if path["name"] == "ground")
            np.testing.assert_allclose(np.array(ground["points"])[:, 1], 0, atol=1e-12)
            for annotation in plot["annotations"]:
                np.testing.assert_allclose(annotation["start"], [0, 0], atol=1e-12)
            row = next(row for row in point_rows(state) if row["name"] == "CG")
            np.testing.assert_allclose([row["x_prime_m"], row["y_prime_m"]], expected, atol=1e-12)
            self.assertAlmostEqual(state["metrics"]["cg_height_mm"], expected[1]*1000, places=8)

    def test_fv_cg_and_body_pickup_keep_rigid_distance_in_contact_view(self):
        engine = Engine(default_project())
        for axle in ("front", "rear"):
            source = engine.fv[axle].zero
            distance = np.linalg.norm(source["cg"]-source["cpb_ru"])
            for travel in ((0, 0), (-10, 20), (15, 15)):
                state = engine.fv[axle].solve(*travel)
                plot = make_plot(axle, "motion", state)
                upper = [path for path in plot["world_lines"] if path["name"] == "upper"][-1]
                self.assertAlmostEqual(np.linalg.norm(cg_marker(plot)-upper["points"][0]), distance, places=10)

    def test_anti_uses_current_cg_height_and_contact_spacing_not_nominal_values(self):
        engine = Engine(default_project())
        # Analytic pose: road Z=.02, CG Z=.52 -> h=.5 m; contacts 2 m
        # apart; CP/IC slope toward the rear axle is .2; 70% front brakes -> 56%.
        front = {"contact": np.array([-1., .02]), "center": np.array([-1., .22]),
                 "ic": np.array([-.5, .12]), "au": np.array([-.95, .35]), "al": np.array([-.97, .15]),
                 "kg": np.array([-1., .02]), "kt": np.array([-.9, .7]), "angle": 0.}
        rear = {key: value*[-1, 1] if isinstance(value, np.ndarray) else value for key, value in front.items()}
        cg = np.array([0., .52])
        for angle, shift in ((0., np.zeros(2)), (.31, np.array([2., 1.]))):
            matrix = rotate2(angle)
            moved = [{key: matrix@value+shift if isinstance(value, np.ndarray) else value
                      for key, value in state.items()} for state in (front, rear)]
            body_cg = matrix@cg+shift
            engine.car.h_cog = body_cg[1]
            engine.sv["front"].template.wheel_center[0] = body_cg[0]-engine.car.lf
            with patch.object(engine.sv["front"], "side", return_value=moved[0]), \
                 patch.object(engine.sv["rear"], "side", return_value=moved[1]):
                state = engine.solve_sv(0, 0)
            self.assertAlmostEqual(state["metrics"]["wheelbase_mm"], 2000, places=8)
            self.assertAlmostEqual(state["metrics"]["cg_height_mm"], 500, places=8)
            self.assertAlmostEqual(state["metrics"]["anti_brake_front_pct"], 56, places=8)
            np.testing.assert_allclose(cg_marker(make_plot("sv", "motion", state)), [0, .5], atol=1e-12)

    def test_nonpositive_cg_height_is_na_and_pose_sweeps_use_current_height(self):
        project = default_project()
        project["analysis"]["samples"] = 5
        engine = Engine(project)
        for mode in ("travel", "pose"):
            mapping = calculate_mapping(engine, "sv", mode)
            valid = [row for row in mapping["rows"] if row["valid"]]
            self.assertTrue(valid)
            self.assertGreater(max(row["cg_height_mm"] for row in valid)-min(row["cg_height_mm"] for row in valid), 1)
            for row in valid:
                self.assertAlmostEqual(row["cg_height_mm"], row["cg_y_prime_mm"], places=8)
        zero = engine.solve_sv(0, 0)
        contacts = [copy.deepcopy(engine.sv[axle].side(0)) for axle in ("front", "rear")]
        for side in contacts:
            for key, value in side.items():
                if isinstance(value, np.ndarray):
                    side[key] = value+[0, engine.car.h_cog+.01]
        with patch.object(engine.sv["front"], "side", return_value=contacts[0]), \
             patch.object(engine.sv["rear"], "side", return_value=contacts[1]):
            state = engine.solve_sv(0, 0)
        self.assertLess(state["metrics"]["cg_height_mm"], 0)
        self.assertIsNone(state["metrics"]["anti_brake_front_pct"])
        np.testing.assert_array_equal(state["points"]["CG"], zero["points"]["CG"])


if __name__ == "__main__":
    unittest.main()
