"""Chassis X requests stay on their arm planes and inside clearance extrema."""
import copy
from pathlib import Path
import tempfile
import unittest

import numpy as np

from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.hardpoints import from_scene
from suspension_geometry_lab.project import default_project, mount_x_settings, save_project, load_project, validate
from suspension_geometry_lab.static.fv.mount_position_geometry import position_arm_mounts

GROUPS = (("front", "upper", "upper_arm_mount_geometry", "FR"),
          ("front", "lower", "lower_arm_mount_geometry", "FR"),
          ("rear", "upper", "rear_upper_arm_mount_geometry", "RR"),
          ("rear", "lower", "rear_lower_arm_mount_geometry", "RR"))


def requested_project():
    p = default_project()
    p["arm_mount_x"] = mount_x_settings(p)
    p["arm_mount_x"]["front"].update(upper_front_x=.2, upper_rear_x=-.2, lower_front_x=.05, lower_rear_x=-.05)
    p["arm_mount_x"]["rear"].update(upper_front_x=-1.2, upper_rear_x=-1.8, lower_front_x=-1.3, lower_rear_x=-1.7)
    return p


class MountPositionTests(unittest.TestCase):
    def test_eight_coordinates_preserve_y_solve_z_on_plane_and_mirror(self):
        p = requested_project()
        e = Engine(p)
        limits = Engine(default_project()).scene.geometry
        for axle, arm, group, corner in GROUPS:
            names = ("UFI", "URI") if arm == "upper" else ("LFI", "LRI")
            plane = e.scene.geometry["arm_planes"][corner+"_"+arm]
            data = e.scene.geometry[group][corner]
            left = e.scene.geometry[group][corner[0]+"L"]
            for name, key in zip(names, (arm+"_front_x", arm+"_rear_x")):
                point = data["mounts"][name]
                self.assertEqual(point[0], p["arm_mount_x"][axle][key])
                self.assertEqual(point[1], limits[group][corner]["target_y"])
                self.assertAlmostEqual(np.dot(plane["normal"], point-plane["vertices"][0]), 0, places=12)
                np.testing.assert_allclose(left["mounts"][name], point*[1, -1, 1], atol=1e-12)
                check = e.scene.geometry["mount_position_checks"][axle][key]
                self.assertFalse(check["fallback"])
                self.assertLessEqual(check["limit_min_x"], point[0])
                self.assertGreaterEqual(check["limit_max_x"], point[0])
        self.assertEqual(sum(map(len, from_scene(e.scene).values())), 24)

    def test_missing_settings_and_none_preserve_original_scene_exactly(self):
        missing = default_project()
        missing.pop("arm_mount_x")
        original = Engine(missing)
        p = default_project()
        p["arm_mount_x"] = mount_x_settings(p)
        automatic = Engine(p)
        self.assertEqual(len(original.scene.lines), len(automatic.scene.lines))
        for a, b in zip(original.scene.lines, automatic.scene.lines):
            np.testing.assert_array_equal(a.points, b.points)
        for axle, arm, group, corner in GROUPS:
            for point, xyz in original.scene.geometry[group][corner]["mounts"].items():
                np.testing.assert_array_equal(xyz, automatic.scene.geometry[group][corner]["mounts"][point])

    def test_exact_boundaries_and_one_automatic_endpoint_are_allowed(self):
        p = default_project()
        limits = Engine(p).scene.geometry["mount_position_checks"]
        p["arm_mount_x"] = mount_x_settings(p)
        for axle, arm, _, _ in GROUPS:
            for suffix, bound in (("front_x", "limit_max_x"), ("rear_x", "limit_min_x")):
                key = arm+"_"+suffix
                p["arm_mount_x"][axle][key] = limits[axle][key][bound]
        engine = Engine(p)
        self.assertTrue(all(not c["fallback"] for axle in engine.scene.geometry["mount_position_checks"].values() for c in axle.values()))
        p["arm_mount_x"]["front"].update(upper_front_x=.1, upper_rear_x=None)
        engine = Engine(p)
        self.assertEqual(engine.scene.geometry["mount_position_checks"]["front"]["upper_front_x"]["actual_x"], .1)
        self.assertEqual(engine.scene.geometry["mount_position_checks"]["front"]["upper_rear_x"]["actual_x"],
                         limits["front"]["upper_rear_x"]["limit_min_x"])

    def test_outside_either_bound_falls_back_entire_arm_with_exact_excess(self):
        baseline = Engine(default_project())
        for axle, arm, group, corner in GROUPS:
            for suffix, bound, direction in (("front_x", "limit_max_x", 1), ("rear_x", "limit_min_x", -1)):
                with self.subTest(axle=axle, arm=arm, suffix=suffix):
                    p = requested_project()
                    key = arm+"_"+suffix
                    limit = baseline.scene.geometry["mount_position_checks"][axle][key][bound]
                    p["arm_mount_x"][axle][key] = limit+direction*.012345
                    engine = Engine(p)
                    check = engine.scene.geometry["mount_position_checks"][axle][key]
                    self.assertTrue(check["fallback"])
                    self.assertAlmostEqual(check["excess_mm"], 12.345, places=9)
                    self.assertIn("12.345 mm", check["message"])
                    for name, xyz in baseline.scene.geometry[group][corner]["mounts"].items():
                        np.testing.assert_array_equal(engine.scene.geometry[group][corner]["mounts"][name], xyz)

    def test_reversed_and_coincident_pairs_use_extrema_without_relabeling(self):
        original = Engine(default_project())
        for front, rear in ((-.1, .1), (0, 0)):
            p = requested_project()
            p["arm_mount_x"]["front"].update(upper_front_x=front, upper_rear_x=rear)
            engine = Engine(p)
            for key in ("upper_front_x", "upper_rear_x"):
                self.assertTrue(engine.scene.geometry["mount_position_checks"]["front"][key]["fallback"])
                self.assertIn("前後順序錯誤", engine.scene.geometry["mount_position_checks"]["front"][key]["message"])
            for name, xyz in original.scene.geometry["upper_arm_mount_geometry"]["FR"]["mounts"].items():
                np.testing.assert_array_equal(engine.scene.geometry["upper_arm_mount_geometry"]["FR"]["mounts"][name], xyz)
            self.assertAlmostEqual(engine.scene.geometry["lower_arm_mount_geometry"]["FR"]["mounts"]["LFI"][0], .05)

    def test_limits_are_recomputed_when_lateral_mount_setting_changes(self):
        p = default_project()
        old = Engine(p).scene.geometry["mount_position_checks"]["front"]["upper_front_x"]
        p["front"]["upper_mount_x"] = .3
        new = Engine(p).scene.geometry["mount_position_checks"]["front"]["upper_front_x"]
        self.assertLess(new["limit_max_x"], old["limit_max_x"])
        p["arm_mount_x"] = mount_x_settings(p)
        p["arm_mount_x"]["front"]["upper_front_x"] = (new["limit_max_x"]+old["limit_max_x"])/2
        check = Engine(p).scene.geometry["mount_position_checks"]["front"]["upper_front_x"]
        self.assertTrue(check["fallback"])
        self.assertEqual(check["limit_max_x"], new["limit_max_x"])
        self.assertEqual(check["actual_x"], new["limit_max_x"])

    def test_plane_without_unique_z_falls_back_only_when_x_is_requested(self):
        geometry = Engine(default_project()).scene.geometry
        planes = copy.deepcopy(geometry["arm_planes"])
        planes["FR_upper"]["normal"] = np.array([1., 0., 0.])
        result = position_arm_mounts(geometry["upper_arm_mount_geometry"], planes,
                                     {"upper_front_x": .1}, arm="upper")
        self.assertIn("唯一求 Z", result["FR"]["position_checks"]["upper_front_x"]["message"])
        np.testing.assert_array_equal(result["FR"]["mounts"]["UFI"], geometry["upper_arm_mount_geometry"]["FR"]["mounts"]["UFI"])

    def test_saved_requests_remain_distinct_from_applied_fallback_and_validate(self):
        p = requested_project()
        p["arm_mount_x"]["front"]["upper_front_x"] = .4
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"mounts.json"
            save_project(path, p)
            loaded = load_project(path)
            self.assertEqual(loaded, p)
            check = Engine(loaded).scene.geometry["mount_position_checks"]["front"]["upper_front_x"]
            self.assertEqual(check["requested_x"], .4)
            self.assertLess(check["actual_x"], .4)
        for invalid in (True, float("nan"), float("inf"), "0.1"):
            candidate = copy.deepcopy(p)
            candidate["arm_mount_x"]["front"]["upper_front_x"] = invalid
            with self.assertRaises(ValueError):
                validate(candidate)


if __name__ == "__main__":
    unittest.main()
