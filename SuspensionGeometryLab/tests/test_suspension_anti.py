"""Independent physical checks for braking, acceleration and project migration."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab.anti import anti_ratio
from suspension_geometry_lab.analysis import calculate_mapping, make_plot, PLOTS
from suspension_geometry_lab.engine import Engine, rotate2
from suspension_geometry_lab.hardpoints import from_scene
from suspension_geometry_lab.project import default_project, validate, save_project, load_project


CHANNELS = ("anti_dive_front_pct", "anti_lift_rear_pct", "anti_lift_front_pct", "anti_squat_rear_pct")


class AntiTests(unittest.TestCase):
    def test_analytic_cp_braking_wc_drive_and_rigid_frame_invariance(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .25
        e = Engine(p)
        # L=2 m, h=.5 m, CP slope=.6, WC slope=.2.
        # Brake shares .7/.3, drive shares .25/.75 -> 168/72/20/60%.
        front = {"contact": np.array([-1., .02]), "center": np.array([-1., .22]),
                 "ic": np.array([-.5, .32]), "au": np.array([-.95, .35]), "al": np.array([-.97, .15]),
                 "kg": np.array([-1., .02]), "kt": np.array([-.9, .7]), "angle": 0.}
        rear = {key: value*[-1, 1] if isinstance(value, np.ndarray) else value for key, value in front.items()}
        for angle, shift in ((0., np.zeros(2)), (.31, np.array([2., 1.]))):
            matrix = rotate2(angle)
            sides = [{key: matrix@value+shift if isinstance(value, np.ndarray) else value
                      for key, value in state.items()} for state in (front, rear)]
            cg = matrix@np.array([0., .52])+shift
            e.car.h_cog = cg[1]
            e.sv["front"].template.wheel_center[0] = cg[0]-e.car.lf
            with patch.object(e.sv["front"], "side", return_value=sides[0]), patch.object(e.sv["rear"], "side", return_value=sides[1]):
                m = e.solve_sv(0, 0)["metrics"]
            np.testing.assert_allclose([m[key] for key in CHANNELS], [168, 72, 20, 60], atol=1e-10)
            self.assertNotIn("anti_dive_rear_pct", m)

    def test_brake_and_drive_shares_change_only_their_channels_at_fixed_hardpoints(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .25
        p["hardpoints"] = {"enabled": True, "corners": from_scene(Engine(p).scene)}
        original = Engine(p)
        for variable, value, factors in (("drive_force_front", .75, [1, 1, 3, 1/3]),
                                         ("brake_bias_front", .5, [5/7, 5/3, 1, 1])):
            changed = copy.deepcopy(p)
            changed["vehicle"][variable] = value
            e = Engine(changed)
            for a, b in zip(original.scene.lines, e.scene.lines):
                np.testing.assert_array_equal(a.points, b.points)
            for travel in ((0, 0), (15, -10)):
                before, after = original.solve_sv(*travel)["metrics"], e.solve_sv(*travel)["metrics"]
                for key, factor in zip(CHANNELS, factors):
                    self.assertAlmostEqual(after[key], before[key]*factor, places=9)
            for axle in ("front", "rear"):
                for key in ("anti_dive", "anti_lift", "anti_squat"):
                    value = e.derived_parameters[axle][key]
                    if value is not None:
                        self.assertAlmostEqual(100*value, e.solve_sv(0, 0)["metrics"][f"{key}_{axle}_pct"], places=9)

    def test_drive_setting_supports_rwd_fwd_and_driven_anti_targets(self):
        for drive, inactive in ((0, "anti_lift_front_pct"), (1, "anti_squat_rear_pct"), (.4, None)):
            p = default_project()
            p["vehicle"]["drive_force_front"] = drive
            p["front"]["anti_lift"] = .02
            p["rear"]["anti_squat"] = .03
            e = Engine(p)
            m = e.solve_sv(0, 0)["metrics"]
            if drive > 0:
                self.assertAlmostEqual(m["anti_lift_front_pct"], 2)
            if drive < 1:
                self.assertAlmostEqual(m["anti_squat_rear_pct"], 3)
            if inactive:
                self.assertEqual(e.solve_sv(15, -10)["metrics"][inactive], 0)
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"drive.json"
            save_project(path, p)
            self.assertEqual(load_project(path), p)
        for value in (-.01, 1.01, True, float("nan"), "0.5"):
            p = default_project()
            p["vehicle"]["drive_force_front"] = value
            with self.assertRaises(ValueError):
                validate(p)

    def test_signed_pro_effect_and_degenerate_references(self):
        self.assertAlmostEqual(anti_ratio([-.5, .1], [0, 0], 2, .5, .3, 1), -.24)
        self.assertAlmostEqual(anti_ratio([.5, -.1], [0, 0], 2, .5, .3, 1), -.24)
        self.assertIsNone(anti_ratio([0, 1], [0, 0], 2, .5, .3, 1))
        self.assertIsNone(anti_ratio(None, [0, 0], 2, .5, .3, 1))
        self.assertEqual(anti_ratio(None, [0, 0], 2, .5, 0, 1), 0)
        self.assertIsNone(anti_ratio([.5, .1], [0, 0], 2, 0, .3, 1))

    def test_front_drive_sign_matches_force_moment_balance_with_positive_svsa(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .3
        for target in (-.05, 0., .03):
            p["front"]["anti_lift"] = target
            e = Engine(p)
            t = e.sv["front"].template
            self.assertGreater(t.initial_ic[0]-t.contact_point[0], 0)
            # Local X points rearward. Unit total tractive effort gives the
            # front axle Fx=-.3. Moment balance about IC independently gives
            # dx*Fz-dz*Fx=0. Downward body force resists front lift.
            dx, dz = t.wheel_center-t.initial_ic
            fx = -.3
            fz = dz*fx/dx
            self.assertAlmostEqual(dx*fz-dz*fx, 0)
            expected = -fz/(e.car.h_cog/e.car.l)
            m = e.solve_sv(0, 0)["metrics"]
            self.assertAlmostEqual(m["anti_lift_front_pct"], 100*expected, places=10)
            if target > 0:
                self.assertLess(fz, 0)
                self.assertGreater(t.initial_ic[1], t.wheel_center[1])
            elif target < 0:
                self.assertGreater(fz, 0)
                self.assertLess(t.initial_ic[1], t.wheel_center[1])

    def test_braking_and_acceleration_targets_set_ic_and_svsa_is_dependent(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .4
        p["front"].update(anti_dive=.3, anti_lift=.04, svsa=0.)
        p["rear"].update(anti_lift=.15, anti_squat=.05, svsa=0.)
        e = Engine(p)
        m = e.solve_sv(0, 0)["metrics"]
        self.assertAlmostEqual(m["anti_dive_front_pct"], 30)
        self.assertAlmostEqual(m["anti_lift_rear_pct"], 15)
        for i, axle, share, brake in ((0, "front", .4, .7), (1, "rear", .6, .3)):
            target = .3 if axle == "front" else .15
            acceleration = .04 if axle == "front" else .05
            distance = e.car.load_Radius[i]/(e.car.h_cog/e.car.l*(target/brake-acceleration/share))
            key = "anti_lift_front_pct" if axle == "front" else "anti_squat_rear_pct"
            self.assertAlmostEqual(m[key], 100*acceleration, places=10)
            self.assertAlmostEqual(m[f"svsa_{axle}_mm"], distance*1000, places=9)
            self.assertNotEqual(e.derived_parameters[axle]["svsa"], p[axle]["svsa"])

    def test_v2_conversion_keeps_ic_and_input_file_and_relabels_rear_braking(self):
        for old_lift in (.04, .2):
            old = default_project()
            old["schema_version"] = 2
            old["vehicle"].pop("drive_force_front")
            for axle in ("front", "rear"):
                old[axle].pop("anti_squat", None)
                old[axle].pop("svsa")
                old[axle].update(anti_dive=.2, anti_lift=old_lift)
            before = copy.deepcopy(old)
            new = validate(old)
            self.assertEqual(old, before)
            e = Engine(new)
            b, L, h = .7, e.car.l, e.car.h_cog
            expected_slope = .2*h/(b*L)
            for i, axle in enumerate(("front", "rear")):
                distance = e.car.load_Radius[i]/(expected_slope-old_lift*h/((1-b)*L))
                t = e.sv[axle].template
                self.assertAlmostEqual((t.initial_ic[0]-t.contact_point[0])*t.direction_sign, distance, places=10)
                self.assertAlmostEqual(t.initial_ic[1], distance*expected_slope, places=10)
            self.assertAlmostEqual(new["rear"]["anti_lift"], .2*.3/.7)
            self.assertEqual(new["vehicle"]["drive_force_front"], 0)
            self.assertEqual(new["front"]["anti_lift"], 0)
            self.assertNotIn("anti_dive", new["rear"])
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"old.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            original = path.read_bytes()
            self.assertEqual(load_project(path), new)
            self.assertEqual(path.read_bytes(), original)
            save_project(path, old)
            self.assertEqual(load_project(path)["schema_version"], 4)

    def test_all_sweep_channels_match_pose_and_plot_names_identify_the_load_case(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .35
        p["analysis"]["samples"] = 5
        e = Engine(p)
        plots = dict(PLOTS["sv"])
        self.assertNotIn("anti_dive_rear_pct", plots)
        for key in CHANNELS:
            self.assertIn("煞車" if key in CHANNELS[:2] else "加速", plots[key])
        for mode in ("travel", "pose"):
            mapping = calculate_mapping(e, "sv", mode)
            for row in mapping["rows"]:
                if row["valid"]:
                    state = e.solve_sv(row["travel_front_mm"], row["travel_rear_mm"])
                    for key in CHANNELS:
                        self.assertAlmostEqual(row[key], state["metrics"][key], places=9)
            for key in CHANNELS:
                spec = make_plot("sv", key, e.solve_sv(0, 0), mapping, e)
                self.assertTrue(spec["markers"])

    def test_only_zero_drive_axle_uses_svsa_and_ignores_inactive_acceleration_target(self):
        for drive, inactive, active, key in ((0., "front", "rear", "anti_squat"), (1., "rear", "front", "anti_lift")):
            p = default_project()
            p["vehicle"]["drive_force_front"] = drive
            p[inactive]["svsa"] = 2.5
            p[inactive]["anti_lift" if inactive == "front" else "anti_squat"] = 99.
            p[active][key] = .03
            first = Engine(p)
            p[active]["svsa"] = 999.
            second = Engine(p)
            for axle in ("front", "rear"):
                np.testing.assert_array_equal(first.sv[axle].template.initial_ic, second.sv[axle].template.initial_ic)
            self.assertAlmostEqual(second.derived_parameters[inactive]["svsa"], 2.5)
            self.assertEqual(second.derived_parameters[inactive]["anti_lift" if inactive == "front" else "anti_squat"], 0)
            self.assertAlmostEqual(second.derived_parameters[active][key], .03)

    def test_parallel_anti_lines_reject_instead_of_using_stored_svsa(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .35
        p["front"].update(anti_dive=.2, anti_lift=.1, svsa=2.5)
        with self.assertRaisesRegex(ValueError, "平行"):
            Engine(p)
        p["front"]["anti_lift"] = .03
        p["rear"]["anti_squat"] = p["rear"]["anti_lift"]/.3*.65
        with self.assertRaisesRegex(ValueError, "平行"):
            Engine(p)

    def test_v3_svsa_migrates_to_equivalent_targets_for_all_drive_layouts(self):
        for drive in (0., .35, 1.):
            old = default_project()
            old["schema_version"] = 3
            old["front"].pop("anti_lift")
            old["rear"].pop("anti_squat")
            old["vehicle"]["drive_force_front"] = drive
            old["front"]["svsa"], old["rear"]["svsa"] = 2.5, 3.
            original = copy.deepcopy(old)
            new = validate(old)
            self.assertEqual(old, original)
            self.assertEqual(new["schema_version"], 4)
            e = Engine(new)
            for axle, distance in (("front", 2.5), ("rear", 3.)):
                t = e.sv[axle].template
                self.assertAlmostEqual(t.direction_sign*(t.initial_ic[0]-t.contact_point[0]), distance, places=10)
                self.assertAlmostEqual(e.derived_parameters[axle]["svsa"], distance, places=10)
            root = Path(__file__).resolve().parents[1]/".test-tmp"
            with tempfile.TemporaryDirectory(dir=root) as tmp:
                path = Path(tmp)/"v3.json"
                path.write_text(json.dumps(old), encoding="utf-8")
                before = path.read_bytes()
                self.assertEqual(load_project(path), new)
                self.assertEqual(path.read_bytes(), before)
                save_project(path, new)
                self.assertEqual(load_project(path), new)


if __name__ == "__main__":
    unittest.main()
