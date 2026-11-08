"""Independent bump/droop limits, travel-driven roll and v1 migration."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from suspension_geometry_lab.analysis import calculate_mapping, make_plot
from suspension_geometry_lab.engine import Engine, Unreachable
from suspension_geometry_lab.hardpoints import from_scene
from suspension_geometry_lab.project import default_project, validate, load_project, save_project, travel_limits


def legacy_project():
    project = default_project()
    project["schema_version"] = 1
    project["vehicle"].pop("drive_force_front")
    a = project["analysis"]
    travel = a.pop("bump_mm")
    a.pop("droop_mm")
    a.update(travel_mm=travel, roll_deg=2., pitch_deg=1.)
    for axle in ("front", "rear"):
        project[axle].pop("anti")
        project[axle].update(anti_dive=.2, anti_lift=0.)
        project[axle]["svsa"] = 0.
    return project


class RangeSettingTests(unittest.TestCase):
    def test_asymmetric_travel_grids_keep_both_bounds_zero_and_exact_live_marker(self):
        project = default_project()
        project["analysis"].update(bump_mm=40, droop_mm=10, samples=5)
        engine = Engine(project)
        self.assertEqual(travel_limits(project), (-10, 40))
        for page in ("front", "rear", "sv"):
            mapping = calculate_mapping(engine, page)
            xs = [entry["axis_x"] for entry in mapping["grid"][0]]
            ys = [row[0]["axis_y"] for row in mapping["grid"]]
            low, high, zero = (10, 60, 50) if page == "sv" else (-10, 40, 0)
            for values in (xs, ys):
                self.assertEqual(min(values), low)
                self.assertEqual(max(values), high)
                self.assertIn(zero, values)
            for row in mapping["rows"]:
                if row["valid"]:
                    names = ("travel_front_mm", "travel_rear_mm") if page == "sv" else ("travel_left_mm", "travel_right_mm")
                    self.assertTrue(all(-10 <= row[key] <= 40 for key in names))
            state = engine.solve_sv(-7, 13) if page == "sv" else engine.fv[page].solve(-7, 13)
            key = "caster_front_deg" if page == "sv" else "camber_left_deg"
            position = make_plot(page, key, state, mapping, engine)["markers"][0]["position"]
            self.assertEqual(position[:2], [57., 37.] if page == "sv" else [-7., 13.])

    def test_zero_one_direction_is_allowed_and_invalid_extents_are_rejected(self):
        for bump, droop in ((0, 25), (25, 0)):
            project = default_project()
            project["analysis"].update(bump_mm=bump, droop_mm=droop, samples=5)
            validate(project)
            grid = calculate_mapping(Engine(project), "front")["grid"]
            self.assertEqual(grid[0][0]["axis_x"], -droop)
            self.assertEqual(grid[0][-1]["axis_x"], bump)
        for bump, droop in ((0, 0), (-1, 25), (25, -1), (201, 25), (25, 201), (True, 25), (25, float("nan"))):
            project = default_project()
            project["analysis"].update(bump_mm=bump, droop_mm=droop)
            with self.assertRaises(ValueError):
                validate(project)

    def test_pitch_reference_and_heave_angle_combinations_obey_stroke_limits(self):
        project = default_project()
        project["analysis"].update(bump_mm=15, droop_mm=5, samples=5)
        engine = Engine(project)
        baseline = Engine(default_project())
        self.assertNotIn("roll_deg", project["analysis"])
        self.assertNotIn("pitch_deg", project["analysis"])
        limit = engine.pitch_angle_limit()
        self.assertGreater(limit, 0)
        self.assertLess(limit, baseline.pitch_angle_limit())
        with self.assertRaisesRegex(Unreachable, "超出"):
            engine.sv_pose(15, limit)
        mapping = calculate_mapping(engine, "sv", "pose")
        self.assertEqual(mapping["grid"][0][0]["axis_y"], -limit)
        self.assertEqual(mapping["grid"][-1][0]["axis_y"], limit)
        self.assertTrue(any(not row["valid"] for row in mapping["rows"]))
        self.assertTrue(any(row["valid"] for row in mapping["rows"]))
        for row in mapping["rows"]:
            if row["valid"]:
                self.assertTrue(all(-5-1e-5 <= row[key] <= 15+1e-5 for key in ("travel_front_mm", "travel_rear_mm")))

    def test_roll_reaches_wheel_stroke_at_each_heave_without_a_separate_angle_cap(self):
        project = default_project()
        project["analysis"].update(bump_mm=40, droop_mm=10)
        engine = Engine(project)
        for axle in ("front", "rear"):
            solver = engine.fv[axle]
            self.assertEqual(solver.roll_bounds(-10), (0., 0.))
            self.assertEqual(solver.roll_bounds(40), (0., 0.))
            self.assertGreater(solver.roll_bounds(15)[1], solver.roll_bounds(0)[1])
            for heave in (-10, 0, 15, 35, 40):
                for angle in solver.roll_bounds(heave):
                    m = solver.pose(heave, angle)["metrics"]
                    travels = (m["travel_left_mm"], m["travel_right_mm"])
                    self.assertTrue(all(-10-1e-5 <= value <= 40+1e-5 for value in travels))
                    self.assertLess(min(abs(value-bound) for value in travels for bound in (-10, 40)), 1e-5)
                with self.assertRaisesRegex(Unreachable, "超出"):
                    solver.pose(heave, solver.roll_bounds(heave)[1]+.01)

    def test_roll_surface_is_the_full_travel_domain_projected_into_heave_roll(self):
        project = default_project()
        project["analysis"].update(bump_mm=40, droop_mm=10, samples=5)
        engine = Engine(project)
        for axle in ("front", "rear"):
            travel = calculate_mapping(engine, axle)
            pose = calculate_mapping(engine, axle, "pose")
            self.assertEqual(pose["total"], travel["total"])
            self.assertEqual(pose["valid"], pose["total"])
            for original, mapped in zip(travel["rows"], pose["rows"]):
                for key in ("travel_left_mm", "travel_right_mm", "camber_left_deg", "rc_y_prime_mm"):
                    self.assertEqual(mapped[key], original[key])
                self.assertEqual(mapped["axis_x"], original["heave_mm"])
                self.assertEqual(mapped["axis_y"], original["roll_deg"])
            for state in (engine.fv[axle].solve(-10, 40), engine.fv[axle].solve(40, -10)):
                point = make_plot(axle, "camber_left_deg", state, pose, engine)["markers"][0]["position"]
                self.assertEqual(point[:2], [15., state["metrics"]["roll_deg"]])
            # This pose fits the Heave/Roll bounding rectangle but exceeds one
            # wheel's stroke; a marker must not imply it belongs to the surface.
            outside = engine.fv[axle].solve(41, -9)
            plot = make_plot(axle, "camber_left_deg", outside, pose, engine)
            self.assertFalse(plot["markers"])
            self.assertIn("行程範圍", plot["current_status"])

    def test_v1_symmetric_ranges_upgrade_without_mutating_source(self):
        old = legacy_project()
        old["analysis"]["travel_mm"] = 30
        original = copy.deepcopy(old)
        upgraded = validate(old)
        self.assertEqual(old, original)
        self.assertEqual(upgraded["schema_version"], 7)
        self.assertEqual(travel_limits(upgraded), (-30, 30))
        self.assertFalse({"travel_mm", "roll_deg", "pitch_deg"} & set(upgraded["analysis"]))
        self.assertGreater(upgraded["front"]["svsa"], 0)
        self.assertEqual(Engine(old).project, upgraded)

    def test_old_svsa_override_preserves_geometry_and_saves_as_v7(self):
        old = legacy_project()
        old["front"]["svsa"], old["rear"]["svsa"] = 2.5, 3.
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"legacy.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            before = path.read_bytes()
            upgraded = load_project(path)
            self.assertEqual(path.read_bytes(), before)
            e = Engine(upgraded)
            for axle, distance in (("front", 2.5), ("rear", 3.)):
                t = e.sv[axle].template
                slope = old[axle]["anti_dive"]*e.car.h_cog/(old["vehicle"]["brake_bias_front"]*e.car.l)
                self.assertAlmostEqual(abs(t.initial_ic[0]-t.contact_point[0]), distance, places=11)
                self.assertAlmostEqual(t.initial_ic[1], distance*slope, places=11)
                self.assertAlmostEqual(e.derived_parameters[axle]["svsa"], distance, places=11)
                self.assertEqual(upgraded[axle]["svsa"], distance)
            save_project(path, old)
            self.assertEqual(load_project(path), upgraded)
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["schema_version"], 7)
        # The design-mode migration must not change manually supplied hardpoints.
        old["hardpoints"] = {"enabled": True, "corners": from_scene(Engine(default_project()).scene)}
        self.assertEqual(Engine(old).scene.geometry["hardpoints"], old["hardpoints"]["corners"])

    def test_removed_inputs_and_bad_legacy_data_are_rejected(self):
        for group, key in (("analysis", "travel_mm"), ("analysis", "roll_deg"), ("analysis", "pitch_deg"), ("front", "anti_squat"), ("rear", "anti_dive")):
            project = default_project()
            project[group][key] = 1
            with self.assertRaises(ValueError):
                validate(project)
        for value in (-1, True, float("nan")):
            old = legacy_project()
            old["front"]["svsa"] = value
            with self.assertRaises(ValueError):
                validate(old)


if __name__ == "__main__":
    unittest.main()
