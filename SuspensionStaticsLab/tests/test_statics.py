import tempfile
import unittest
from math import pi, sqrt
from pathlib import Path

import numpy as np

from SuspensionStaticsLab.load_cases import ACCEL, AERO, BUMP, CASES, GRAVITY, STATIC, example_loads
from SuspensionStaticsLab.model import EXAMPLE, build_model, read_hardpoints
from SuspensionStaticsLab.solver import Load, Section, member_results, solve


def rod(name, a, b):
    return {"name": name, "a": a, "b": b}


class StaticsTests(unittest.TestCase):
    def test_pin_forces_sign_and_original_length_postprocessing(self):
        points = {"O": (0, 0, 0), "X": (.5, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 2)}
        model = build_model(points, {"fixed": list("XYZ"), "rods": [rod(axis, "O", axis) for axis in "XYZ"]})
        result = solve(model, [Load("O", (1000, -400, 0))])
        self.assertEqual(result.forces, {"X": -1000., "Y": 400., "Z": 0.})
        self.assertEqual(result.support_force, (-1000., 400., 0.))
        sections = dict.fromkeys(result.forces, Section(20, 10, 200))
        member = member_results(result, sections)["X"]
        self.assertAlmostEqual(member.area_mm2, 75 * pi)
        self.assertAlmostEqual(member.stress_mpa, -1000 / (75 * pi))
        self.assertAlmostEqual(member.strain, -1000 / (75 * pi * 200_000))
        self.assertAlmostEqual(member.change_mm, -1000 * 500 / (75 * pi * 200_000))
        self.assertEqual(model.points, points)

    def test_couple_is_balanced_by_axial_forces_with_lever_arms(self):
        points = {
            "O": (0, 0, 0), "X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1),
            "AX": (1, 0, 0), "AY": (0, 1, 0), "AZ": (0, 0, 1),
            "MX": (0, 1, 1), "MY": (1, 0, 1), "MZ": (1, 1, 0),
        }
        members = [rod(f"F{axis}", "O", f"A{axis}") for axis in "XYZ"]
        members += [rod(f"M{axis}", point, f"M{axis}") for axis, point in zip("XYZ", "YZX")]
        topology = {"fixed": ["AX", "AY", "AZ", "MX", "MY", "MZ"],
                    "bodies": {"upright": list("OXYZ")}, "rods": members}
        result = solve(build_model(points, topology), [Load("FX", (30, -40, 50), "A")])
        np.testing.assert_allclose(list(result.forces.values()), [-40, 50, 30, -30, 40, -50])
        np.testing.assert_allclose(result.support_moment, [-30, 40, -50])

    def test_insufficient_and_redundant_support(self):
        points = {"O": (0, 0, 0), "X": (1, 0, 0)}
        members = [rod("first", "O", "X")]
        model = build_model(points, {"fixed": ["X"], "rods": members})
        with self.assertRaisesRegex(ValueError, "無法平衡"):
            solve(model, [Load("O", (0, 50, 0))])
        with self.assertRaisesRegex(ValueError, "只傳力"):
            solve(model, [Load("first", (0, 0, 10), "A")])
        model = build_model(points, {"fixed": ["X"], "rods": members + [rod("second", "O", "X")]})
        with self.assertRaisesRegex(ValueError, "無唯一解"):
            solve(model, [Load("O", (50, 0, 0))])

    def test_example_cases_balance_all_corners(self):
        model = build_model(read_hardpoints(EXAMPLE))
        baseloads = example_loads(model, STATIC)
        baseline = np.array([load.xyz[2] for load in baseloads[1:5]])
        for case in CASES:
            with self.subTest(case=case):
                loads = example_loads(model, case)
                result = solve(model, loads)
                self.assertEqual(len(result.forces), 28)
                self.assertEqual(result.rank, result.equations)
                if case == BUMP:
                    self.assertEqual(result.support_force, (0., 0., -1800.))
                    np.testing.assert_allclose([v for n, v in result.forces.items() if not n.startswith("FL_")], 0., atol=1e-9)
                else:
                    np.testing.assert_allclose(result.support_force, 0., atol=1e-9)
                    np.testing.assert_allclose(result.support_moment, 0., atol=1e-9)
                    tire_z = np.array([load.xyz[2] for load in loads[1:5]])
                    if case == ACCEL:
                        self.assertLess(tire_z[:2].sum(), baseline[:2].sum())
                    if case == AERO:
                        self.assertAlmostEqual((tire_z - baseline)[:2].sum(), 360.)

    def test_cg_acceleration_combines_transfer_and_apparent_weight(self):
        model = build_model(read_hardpoints(EXAMPLE))
        mass = 280.
        ax, ay, az = 3., 4., 1.5

        def contact_loads(components):
            loads = example_loads(model, ACCEL, mass_kg=mass, acceleration_mps2=components)
            return np.array([load.xyz for load in loads[1:]])

        baseline = contact_loads((0., 0., 0.))
        longitudinal = contact_loads((ax, 0., 0.))
        lateral = contact_loads((0., ay, 0.))
        vertical = contact_loads((0., 0., az))
        combined = contact_loads((ax, ay, az))
        self.assertLess(longitudinal[:2, 2].sum(), baseline[:2, 2].sum())
        self.assertGreater(lateral[1::2, 2].sum(), baseline[1::2, 2].sum())
        np.testing.assert_allclose(combined, longitudinal + lateral + vertical - 2. * baseline)
        np.testing.assert_allclose(combined.sum(axis=0), (mass * ax, mass * ay, mass * (GRAVITY + az)))
        loads = example_loads(model, ACCEL, mass_kg=mass, acceleration_mps2=(ax, ay, az))
        np.testing.assert_allclose(loads[0].xyz, (-mass * ax, -mass * ay, -mass * (GRAVITY + az)))
        solution = solve(model, loads)
        np.testing.assert_allclose(solution.support_force, 0., atol=1e-9)
        np.testing.assert_allclose(solution.support_moment, 0., atol=1e-9)
        with self.assertRaisesRegex(ValueError, "離地"):
            contact_loads((0., 0., -2. * GRAVITY))
        with self.assertRaisesRegex(ValueError, "質量"):
            example_loads(model, ACCEL, mass_kg=0.)

    def test_circular_section_properties_about_centroid(self):
        for inner in (0., 10.):
            with self.subTest(inner=inner):
                outer = 20.
                properties = Section(outer, inner).properties()
                area = pi / 4. * (outer ** 2 - inner ** 2)
                second_moment = pi / 64. * (outer ** 4 - inner ** 4)
                self.assertAlmostEqual(properties.area_mm2, area)
                self.assertAlmostEqual(properties.second_moment_mm4, second_moment)
                self.assertAlmostEqual(properties.modulus_mm3, second_moment / (outer / 2.))
                self.assertAlmostEqual(properties.gyration_mm, sqrt(second_moment / area))
                self.assertAlmostEqual(properties.polar_moment_mm4, 2. * second_moment)
                self.assertAlmostEqual(properties.polar_modulus_mm3, 2. * second_moment / (outer / 2.))
                self.assertAlmostEqual(properties.polar_gyration_mm, sqrt(2. * second_moment / area))
        with self.assertRaisesRegex(ValueError, "外徑"):
            Section(19., 19.).properties()

    def test_cg_gravity_toggle_includes_weight_once(self):
        model = build_model(read_hardpoints(EXAMPLE))
        static = example_loads(model, STATIC)
        with_gravity = example_loads(model, ACCEL, acceleration_mps2=(0., 0., 0.), include_gravity=True)
        self.assertEqual(with_gravity, static)
        self.assertGreater(max(map(abs, solve(model, with_gravity).forces.values())), 1000.)
        zero = example_loads(model, ACCEL, acceleration_mps2=(0., 0., 0.), include_gravity=False)
        self.assertTrue(all(load.xyz == (0., 0., 0.) for load in zero))
        self.assertEqual(max(map(abs, solve(model, zero).forces.values())), 0.)
        vertical = example_loads(model, ACCEL, mass_kg=200., acceleration_mps2=(0., 0., 2.), include_gravity=False)
        self.assertAlmostEqual(vertical[0].xyz[2], -400.)
        self.assertAlmostEqual(sum(load.xyz[2] for load in vertical[1:]), 400.)

    def test_csv_mm_and_midspan_pushrod(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "points.csv"
            path.write_text("corner,point,x_mm,y_mm,z_mm\nFL,UFI,12,34,56\n", encoding="utf-8-sig")
            self.assertEqual(read_hardpoints(path), {"FL_UFI": (.012, .034, .056)})
        points = read_hardpoints(EXAMPLE)
        points["FL_PUSH_OUT"] = points.pop("FL_PUSH_OUT_UPRIGHT")
        model = build_model(points)
        result = solve(model, example_loads(model, STATIC))
        self.assertEqual(result.rank, result.equations)
        self.assertIn("FL_PUSH_ROD", result.forces)
        self.assertIn("FL_DAMPER", result.forces)
        self.assertNotIn("FL_LCA_F-LBJ", result.forces)

    def test_single_corner_without_prefix_survives_geometry_edit(self):
        points = {name[3:]: xyz for name, xyz in read_hardpoints(EXAMPLE).items() if name.startswith("FL_")}
        original = build_model(points)
        self.assertIn("FL_CONTACT", original.points)
        edited = build_model({**original.points, "WC": (.005, .625, .196)})
        self.assertEqual(edited.wheels, original.wheels)
        self.assertEqual(edited.rods, original.rods)
        self.assertEqual(solve(edited, example_loads(edited, BUMP)).rank, 12)


if __name__ == "__main__":
    unittest.main()
