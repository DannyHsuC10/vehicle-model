"""Independent physical checks for braking, acceleration and project migration."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab.anti import anti_ratio, axle_anti
from suspension_geometry_lab.analysis import calculate_mapping, make_plot, PLOTS
from suspension_geometry_lab.engine import Engine, rotate2
from suspension_geometry_lab.hardpoints import from_scene
from suspension_geometry_lab.project import default_project, validate, save_project, load_project


CHANNELS = ("anti_brake_front_pct", "anti_brake_rear_pct", "anti_drive_front_pct", "anti_drive_rear_pct")


class AntiTests(unittest.TestCase):
    def test_both_load_cases_use_cp_and_keep_the_signed_road_frame(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .25
        e = Engine(p)
        # L=2 m, h=.5 m, CP slope toward the other axle is +.6 at both ends.
        # WC has a different slope and must not affect either load case.
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
            np.testing.assert_allclose([m[key] for key in CHANNELS], [168, 72, 60, 180], atol=1e-10)
            np.testing.assert_allclose([m["anti_front_pct"], m["anti_rear_pct"]], [240, 240], atol=1e-10)

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
                for key in ("anti", "anti_brake", "anti_drive"):
                    value = e.derived_parameters[axle][key]
                    if value is not None:
                        self.assertAlmostEqual(100*value, e.solve_sv(0, 0)["metrics"][f"{key}_{axle}_pct"], places=9)

    def test_drive_setting_supports_rwd_fwd_and_awd(self):
        for drive, inactive in ((0, "anti_drive_front_pct"), (1, "anti_drive_rear_pct"), (.4, None)):
            p = default_project()
            p["vehicle"]["drive_force_front"] = drive
            p["front"]["anti"] = .02
            p["rear"]["anti"] = .03
            e = Engine(p)
            m = e.solve_sv(0, 0)["metrics"]
            self.assertAlmostEqual(m["anti_drive_front_pct"], 2*drive)
            self.assertAlmostEqual(m["anti_drive_rear_pct"], 3*(1-drive))
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
        self.assertAlmostEqual(anti_ratio([-.5, .1], [0, 0], 2, .5), .8)
        self.assertAlmostEqual(anti_ratio([.5, -.1], [0, 0], 2, .5), .8)
        self.assertAlmostEqual(anti_ratio([.5, .1], [0, 0], 2, .5), -.8)
        self.assertIsNone(anti_ratio([0, 1], [0, 0], 2, .5))
        self.assertIsNone(anti_ratio(None, [0, 0], 2, .5))
        self.assertEqual(axle_anti("front", None, [0, 0], 2, .5, .7, 0),
                         {"anti": None, "anti_brake": None, "anti_drive": 0.})
        self.assertIsNone(anti_ratio([.5, .1], [0, 0], 2, 0))

    def test_front_reference_matches_forward_signed_example(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .3
        p["vehicle"]["cg_height"] = .302
        target = 1.53/.302 * .35/3
        p["front"].update(anti=-target, svsa=-3.)
        e = Engine(p)
        t = e.sv["front"].template
        np.testing.assert_allclose(t.initial_ic-t.contact_point, [-3., .35], atol=1e-12)
        metrics = e.solve_sv(0, 0)["metrics"]
        self.assertAlmostEqual(metrics["anti_brake_front_pct"], -100*.7*target)
        self.assertAlmostEqual(metrics["anti_drive_front_pct"], -100*.3*target)

    def test_axle_anti_and_svsa_set_ic_on_both_axles(self):
        p = default_project()
        p["vehicle"]["drive_force_front"] = .4
        p["front"].update(anti=.3, svsa=2.5)
        p["rear"].update(anti=0., svsa=-3.)
        e = Engine(p)
        m = e.solve_sv(0, 0)["metrics"]
        self.assertAlmostEqual(m["anti_brake_front_pct"], 21)
        self.assertAlmostEqual(m["anti_drive_front_pct"], 12)
        self.assertAlmostEqual(m["anti_drive_rear_pct"], 0)
        self.assertAlmostEqual(m["svsa_front_mm"], 2500)
        self.assertAlmostEqual(m["svsa_rear_mm"], -3000)
        self.assertAlmostEqual(e.sv["front"].template.initial_ic[1], 2.5*.3*e.car.h_cog/e.car.l)
        self.assertEqual(e.sv["rear"].template.initial_ic[1], 0.)

    def test_parallel_cp_lines_keep_both_instant_centers_when_pc_is_undefined(self):
        p = default_project()
        for target in (0., -.05):
            p["front"]["anti"], p["rear"]["anti"] = -target, target
            e = Engine(p)
            self.assertIsNone(e.scene.geometry["side_view"].local_pitch_center)
            state = e.solve_sv(0, 0)
            self.assertIsNone(state["metrics"]["pc_y_prime_mm"])
            self.assertNotIn("PC0", state["points"])
            self.assertAlmostEqual(state["metrics"]["anti_front_pct"], -100*target)
            self.assertAlmostEqual(state["metrics"]["anti_rear_pct"], 100*target)

    def test_v2_conversion_keeps_ic_and_input_file_and_relabels_rear_braking(self):
        for old_lift in (.04, .2):
            old = default_project()
            old["schema_version"] = 2
            old["vehicle"].pop("drive_force_front")
            for axle in ("front", "rear"):
                old[axle].pop("anti")
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
            self.assertAlmostEqual(new["rear"]["anti"], .2/.7)
            self.assertEqual(new["vehicle"]["drive_force_front"], 0)
            self.assertAlmostEqual(new["front"]["anti"], .2/.7)
            self.assertNotIn("anti_lift", new["rear"])
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"old.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            original = path.read_bytes()
            self.assertEqual(load_project(path), new)
            self.assertEqual(path.read_bytes(), original)
            save_project(path, old)
            self.assertEqual(load_project(path)["schema_version"], 7)

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

    def test_v5_migration_preserves_explicit_and_derived_svsa(self):
        old = default_project()
        old["schema_version"] = 5
        old["vehicle"].update(brake_bias_front=0., drive_force_front=1.)
        old["front"].pop("anti")
        old["rear"].pop("anti")
        old["front"].update(anti_dive=.3, anti_lift=.25, svsa=0.)
        old["rear"].update(anti_lift=.2, anti_squat=42., svsa=-3.)
        old["svsa_design"] = {"front": False, "rear": True}
        before = copy.deepcopy(old)
        e = Engine(old)
        self.assertEqual(old, before)
        self.assertNotIn("svsa_design", e.project)
        self.assertAlmostEqual(e.project["front"]["anti"], .3)
        self.assertAlmostEqual(e.project["rear"]["anti"], .2)
        self.assertAlmostEqual(e.project["front"]["svsa"], e.car.load_Radius[0]*e.car.l/(.05*e.car.h_cog))
        self.assertEqual(e.project["rear"]["svsa"], -3.)
        m = e.solve_sv(0, 0)["metrics"]
        np.testing.assert_allclose([m[key] for key in CHANNELS], [0, 20, 30, 0], atol=1e-10)

        # Retain the active hardpoints even if the old, inactive design had no IC.
        points = from_scene(Engine(default_project()).scene)
        old["hardpoints"] = {"enabled": True, "corners": points}
        old["front"]["anti_lift"] = old["front"]["anti_dive"]
        self.assertEqual(from_scene(Engine(old).scene), points)

    def test_v3_svsa_migrates_to_equivalent_targets_for_all_drive_layouts(self):
        for drive in (0., .35, 1.):
            old = default_project()
            old["schema_version"] = 3
            old["front"].pop("anti")
            old["rear"].pop("anti")
            old["front"]["anti_dive"] = .2
            old["rear"]["anti_lift"] = .1
            old["vehicle"]["drive_force_front"] = drive
            old["front"]["svsa"], old["rear"]["svsa"] = 2.5, 3.
            original = copy.deepcopy(old)
            new = validate(old)
            self.assertEqual(old, original)
            self.assertEqual(new["schema_version"], 7)
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

    def test_parameter_mode_force_shares_change_only_weighted_results(self):
        p = default_project()
        p["front"]["anti"] = .2
        p["rear"]["anti"] = .3
        p["vehicle"].update(brake_bias_front=.5, drive_force_front=.5)
        original = Engine(p)
        for brake, drive in ((.7, .35), (0., 0.), (1., 1.)):
            p["vehicle"].update(brake_bias_front=brake, drive_force_front=drive)
            current = Engine(p)
            for axle in ("front", "rear"):
                np.testing.assert_array_equal(current.sv[axle].template.initial_ic, original.sv[axle].template.initial_ic)
            self.assertEqual(from_scene(current.scene), from_scene(original.scene))
            for travel in ((0, 0), (15, -10)):
                before, after = original.solve_sv(*travel), current.solve_sv(*travel)
                self.assertEqual(after["points"], before["points"])
                for key, share in zip(CHANNELS, (brake, 1-brake, drive, 1-drive)):
                    self.assertAlmostEqual(after["metrics"][key], before["metrics"][key]*2*share)

    def test_v4_weighted_targets_migrate_with_same_geometry_for_all_drive_layouts(self):
        for drive in (0., .25, 1.):
            old = default_project()
            old["schema_version"] = 4
            old["vehicle"].update(brake_bias_front=.6, drive_force_front=drive)
            old["front"].pop("anti")
            old["rear"].pop("anti")
            old["front"].update(anti_dive=.18, anti_lift=.08, svsa=4.)
            old["rear"].update(anti_lift=.12, anti_squat=.06, svsa=3.)
            before = copy.deepcopy(old)
            engine = Engine(old)
            self.assertEqual(old, before)
            self.assertEqual(engine.project["schema_version"], 7)
            for index, (axle, brake_target, drive_target, brake_share, drive_share) in enumerate((
                ("front", .18, .08, .6, drive), ("rear", .12, .06, .4, 1-drive),
            )):
                slope_scale = engine.car.h_cog / engine.car.l
                brake_slope = brake_target / brake_share * slope_scale
                distance = old[axle]["svsa"]
                if drive_share:
                    distance = engine.car.load_Radius[index] / (brake_slope - drive_target/drive_share*slope_scale)
                t = engine.sv[axle].template
                np.testing.assert_allclose(t.initial_ic - t.contact_point,
                                           [t.direction_sign*distance, brake_slope*distance], atol=1e-10)
                self.assertAlmostEqual(engine.project[axle]["svsa"], distance)

    def test_v6_front_input_flips_sign_on_load_and_is_saved_once(self):
        p = default_project()
        p["front"].update(anti=.2, svsa=3.)
        expected = Engine(p)
        old = copy.deepcopy(p)
        old["schema_version"] = 6
        old["front"]["anti"] = -.2
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"v6.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            before = path.read_bytes()
            loaded = load_project(path)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(loaded, p)
            actual = Engine(loaded)
            self.assertEqual(from_scene(actual.scene), from_scene(expected.scene))
            self.assertEqual(actual.static_anti_metrics, expected.static_anti_metrics)
            save_project(path, loaded)
            self.assertEqual(load_project(path), p)


if __name__ == "__main__":
    unittest.main()
