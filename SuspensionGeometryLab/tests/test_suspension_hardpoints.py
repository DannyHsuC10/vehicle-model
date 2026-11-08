"""Hardpoint source preservation, geometry inversion and projected motion contracts."""
import copy
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.analysis import calculate_mapping, make_plot, static_point_rows
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.project import default_project, load_project, save_project, validate
from suspension_geometry_lab.wheel_end_geometry import rotation_matrix

TEST_TMP = Path(__file__).resolve().parents[1] / ".test-tmp"


def hardpoint_project():
    p = default_project()
    p["analysis"]["samples"] = 5
    p["hardpoints"] = {"enabled": True, "corners": hardpoints.from_scene(Engine(p).scene)}
    return p


class HardpointTests(unittest.TestCase):
    def test_generated_points_and_direct_scene_preserve_all_24_xyz(self):
        p = hardpoint_project()
        data = p["hardpoints"]["corners"]
        self.assertEqual(sum(map(len, data.values())), 24)
        engine = Engine(p)
        self.assertEqual(hardpoints.from_scene(engine.scene), data)
        self.assertEqual(engine.scene.geometry["hardpoints"], data)
        self.assertEqual(len(make_plot("static", "iso", engine=engine)["markers"]), 24)
        rows = {row["name"]: row for row in static_point_rows(engine)}
        self.assertEqual(len(rows), 24)
        self.assertIn("FR_UCA_F", rows)
        self.assertEqual(rows["FR_UBJ"]["x_m"], data["FR"]["UBJ"][0])
        for left, right in (("FL", "FR"), ("RL", "RR")):
            for key in hardpoints.POINTS:
                np.testing.assert_array_equal(np.array(data[left][key])*[1, -1, 1], data[right][key])

    def test_editing_either_side_mirrors_only_matching_point(self):
        data = hardpoint_project()["hardpoints"]["corners"]
        before = copy.deepcopy(data)
        for corner, key, xyz in (("FL", "UCA_F", [.2, .3, .25]), ("RR", "LBJ", [-1.5, -.57, .1])):
            hardpoints.set_point(data, corner, key, xyz)
            self.assertEqual(data[corner][key], xyz)
            self.assertEqual(data[hardpoints.MIRROR[corner]][key], [xyz[0], -xyz[1], xyz[2]])
        for corner in hardpoints.CORNERS:
            for key in hardpoints.POINTS:
                if (corner[0], key) not in (("F", "UCA_F"), ("R", "LBJ")):
                    self.assertEqual(data[corner][key], before[corner][key])
        saved = copy.deepcopy(data)
        for invalid in ([0, math.nan, .1], [0, True, .1], [0, .2], [0, -.2, .1]):
            with self.assertRaises(ValueError):
                hardpoints.set_point(data, "FL", "UBJ", invalid)
            self.assertEqual(data, saved)

    def test_axis_and_steering_use_edited_ball_joints_exactly(self):
        p = hardpoint_project()
        points = p["hardpoints"]["corners"]
        joint = np.array(points["FR"]["UBJ"]) + [-.01, .004, .006]
        hardpoints.set_point(points, "FR", "UBJ", joint.tolist())
        engine = Engine(p)
        kingpin = engine.scene.geometry["kingpins"]["FR"]
        np.testing.assert_array_equal(kingpin["UBJ"], joint)
        lower = np.array(points["FR"]["LBJ"])
        axis = joint-lower
        ground = lower-lower[2]/axis[2]*axis
        np.testing.assert_allclose(kingpin["ground"], ground, atol=1e-12)
        # A point on the wheel must rotate rigidly around the actual input axis.
        center = engine.scene.geometry["wheel_projection_geometry"]["FR"]["center"]
        expected = ground+rotation_matrix(axis, math.radians(23))@(center-ground)
        np.testing.assert_allclose(engine.solve_we(23)["points"]["center"], expected, atol=1e-12)
        self.assertAlmostEqual(engine.derived_parameters["front"]["kpi_deg"], math.degrees(math.atan2(axis[1], axis[2])))

    def test_known_geometry_and_parallel_side_view_have_defined_limits(self):
        p = hardpoint_project()
        # Horizontal X hinge axes give vertical SV motion and IC at infinity.
        coords = {"UCA_F": [.1, -.25, .3], "UCA_R": [-.2, -.25, .3], "UBJ": [-.03, -.55, .4],
                  "LCA_F": [.1, -.2, .15], "LCA_R": [-.2, -.2, .15], "LBJ": [-.01, -.6, .15]}
        for key, xyz in coords.items():
            hardpoints.set_point(p["hardpoints"]["corners"], "FR", key, xyz)
        engine = Engine(p)
        d = engine.derived_parameters["front"]
        self.assertAlmostEqual(d["fvsa"], .825)
        self.assertAlmostEqual(d["rc_height"], .625*.15/.825)
        self.assertAlmostEqual(d["kpi_deg"], math.degrees(math.atan(.05/.25)))
        self.assertAlmostEqual(d["caster_deg"], math.degrees(math.atan(.02/.25)))
        self.assertAlmostEqual(d["scrub_radius"], -.005)
        self.assertAlmostEqual(d["mechanical_trail"], .002)
        self.assertIsNone(d["svsa"])
        sv = engine.solve_sv(10, 0)
        self.assertIsNone(sv["metrics"]["anti_brake_front_pct"])
        self.assertNotIn("PC0", sv["points"])
        self.assertAlmostEqual(sv["points"]["front_contact"][1], .01, places=8)

    def test_parallel_fv_arms_return_na_without_stale_rc_markers(self):
        p = hardpoint_project()
        coords = {"UCA_F": [.1, -.25, .3], "UCA_R": [-.2, -.25, .3], "UBJ": [-.03, -.55, .4],
                  "LCA_F": [.1, -.2, .15], "LCA_R": [-.2, -.2, .15], "LBJ": [-.01, -.6, .15+.4/3]}
        for key, xyz in coords.items():
            hardpoints.set_point(p["hardpoints"]["corners"], "FR", key, xyz)
        engine = Engine(p)
        state = engine.fv["front"].solve(0, 0)
        self.assertIsNone(engine.derived_parameters["front"]["rc_height"])
        self.assertIsNone(state["metrics"]["rc_y_prime_mm"])
        self.assertNotIn("RC0", state["points"])
        self.assertNotIn("RC", state["points"])
        plot = make_plot("front", "motion", state, engine=engine)
        self.assertFalse({"RC", "RC0"} & {p["name"] for p in plot["markers"]})

    def test_project_roundtrip_mode_switch_and_old_schema_compatibility(self):
        old = default_project()
        self.assertAlmostEqual(Engine(old).fv["front"].solve(0, 0)["metrics"]["rc_y_prime_mm"], 35)
        p = hardpoint_project()
        hardpoints.set_point(p["hardpoints"]["corners"], "RR", "UCA_F", [-.2, -.25, .24])
        TEST_TMP.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=TEST_TMP) as tmp:
            path = Path(tmp)/"硬點專案.json"
            save_project(path, p)
            self.assertEqual(load_project(path), p)
            self.assertEqual(Engine(load_project(path)).scene.geometry["hardpoints"], p["hardpoints"]["corners"])
        p["hardpoints"]["enabled"] = False
        self.assertAlmostEqual(Engine(p).fv["front"].solve(0, 0)["metrics"]["rc_y_prime_mm"], 35)
        self.assertEqual(p["front"], old["front"])

    def test_invalid_shapes_asymmetry_and_degenerate_arms_are_rejected(self):
        for mutation in (lambda p: p["hardpoints"].update(enabled=1),
                         lambda p: p["hardpoints"]["corners"]["FR"].pop("UBJ"),
                         lambda p: p["hardpoints"]["corners"]["FL"]["UBJ"].__setitem__(0, .7)):
            p = hardpoint_project()
            mutation(p)
            with self.assertRaises(ValueError):
                validate(p)
        p = hardpoint_project()
        points = p["hardpoints"]["corners"]
        hardpoints.set_point(points, "FR", "UCA_F", points["FR"]["UCA_R"])
        with self.assertRaisesRegex(ValueError, "重合"):
            Engine(p)

    def test_longitudinal_pickup_edit_changes_sv_and_all_sweeps_work(self):
        p = hardpoint_project()
        baseline = Engine(p)
        xyz = np.array(p["hardpoints"]["corners"]["FR"]["UCA_F"]) + [.04, 0, .003]
        hardpoints.set_point(p["hardpoints"]["corners"], "FR", "UCA_F", xyz.tolist())
        engine = Engine(p)
        self.assertNotAlmostEqual(engine.derived_parameters["front"]["anti"], baseline.derived_parameters["front"]["anti"])
        for page in ("front", "rear", "sv", "we"):
            mapping = calculate_mapping(engine, page)
            self.assertGreater(mapping["valid"], 0)
        sweep = calculate_mapping(engine, "we")["sweep"]
        self.assertAlmostEqual(sweep[0]["axis_y"], engine.derived_parameters["front"]["caster_deg"])

    def test_sv_tangents_agree_with_spatial_hinge_rotation_at_zero(self):
        p = hardpoint_project()
        engine = Engine(p)
        points = p["hardpoints"]["corners"]["FR"]
        for kind, key, direction in zip(("UCA", "LCA"), ("UBJ", "LBJ"), engine.sv["front"].directions):
            first, last, joint = [np.array(points[k]) for k in (kind+"_F", kind+"_R", key)]
            delta = (first+rotation_matrix(last-first, 1e-6)@(joint-first)-joint)/1e-6
            projected = np.array([-delta[0], delta[2]])
            projected /= np.linalg.norm(projected)
            self.assertAlmostEqual(abs(np.dot(projected, direction)), 1, places=9)


if __name__ == "__main__":
    unittest.main()
