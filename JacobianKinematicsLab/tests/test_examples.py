import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from JacobianKinematicsLab.model import CORNERS, EXAMPLES, JOINTS, MOUNTS, load_model, make_motion, read_hardpoints
from JacobianKinematicsLab.results import motion_ratio
from JacobianKinematicsLab.solver import solve


def write_points(path, points):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("name", "x_mm", "y_mm", "z_mm"))
        for name, position in points.items():
            writer.writerow((name, *(coordinate * 1000 for coordinate in position)))


class ExampleIntegrationTests(unittest.TestCase):
    def test_bundled_staticwireframe_defaults_and_mount_overrides(self):
        model = load_model(EXAMPLES / "hardpoints.csv")
        self.assertEqual(model.name, "StaticWireframe 基準車輛")
        self.assertEqual(model.samples, 21)
        editable = {drive.name: drive.amplitude_mm for drive in model.drives if not drive.locked}
        self.assertEqual(editable, {**{f"{corner}_WC_Z": 20 for corner in CORNERS}, "FRONT_RACK": 0})
        for name, x in (("FL_UCA_F", .235), ("FL_UCA_R", -.235), ("RL_UCA_F", -1.17),
                        ("RL_UCA_R", -1.565), ("RL_LCA_R", -1.565)):
            self.assertAlmostEqual(model.reference[model.point_names.index(name), 0], x)

    def test_four_wheel_heave_steering_and_pushrods_solve_together(self):
        model = load_model(EXAMPLES / "hardpoints.csv")
        travels = {"FL": 25, "FR": -12, "RL": 15, "RR": -7}
        amplitudes = {f"{corner}_WC_Z": travel for corner, travel in travels.items()}
        motion = make_motion(model, samples=41, amplitudes_mm=dict(amplitudes, FRONT_RACK=8))
        result = solve(model, motion)
        points = {name: i for i, name in enumerate(model.point_names)}
        links = {link.name: i for i, link in enumerate(model.links)}
        np.testing.assert_allclose(result.position[0], model.reference, atol=1e-12)
        for corner, travel in travels.items():
            wheel = points[f"{corner}_WC"]
            self.assertAlmostEqual(result.position[20, wheel, 2] - model.reference[wheel, 2], travel / 1000)
            rocker = points[f"{corner}_PUSH_IN"]
            self.assertGreater(np.linalg.norm(result.position[20, rocker] - model.reference[rocker]), .001)
            self.assertIn(f"{corner}_PUSH_ROD", links)
            self.assertIn(f"{corner}_RACK-TOE", links)
        for corner in ("FL", "FR"):
            rack = points[f"{corner}_RACK"]
            np.testing.assert_allclose(result.position[20, rack] - model.reference[rack], [0., .008, 0.], atol=1e-10)
            self.assertGreater(abs(result.azimuth_deg[20, links[f"{corner}_WHEEL_HEADING"]]), .5)
        for corner in ("RL", "RR"):
            rack = points[f"{corner}_RACK"]
            np.testing.assert_array_equal(result.velocity[:, rack], 0.)
            np.testing.assert_array_equal(result.acceleration[:, rack], 0.)

        a = [points[link.a] for link in model.links]
        b = [points[link.b] for link in model.links]
        vectors = result.position[:, b] - result.position[:, a]
        lengths = np.linalg.norm(vectors, axis=-1)
        self.assertLess(np.abs(lengths - lengths[0]).max(), 1e-10)
        velocity = result.velocity[:, b] - result.velocity[:, a]
        acceleration = result.acceleration[:, b] - result.acceleration[:, a]
        np.testing.assert_allclose(np.sum(vectors * velocity, axis=-1), 0., atol=1e-10)
        np.testing.assert_allclose(np.sum(vectors * acceleration + velocity**2, axis=-1), 0., atol=1e-10)
        ratio = motion_ratio(result, "FL_WC", "FL_PUSH_IN", drive="FL_WC_Z")
        self.assertTrue(np.isfinite(ratio.motion_ratio).all())
        np.testing.assert_allclose(abs(ratio.motion_ratio) * ratio.mechanical_advantage, 1.)

    def test_tilted_rack_uses_measured_axis_and_independent_profile(self):
        points = read_hardpoints(EXAMPLES / "hardpoints.csv")
        points["FL_RACK"] = tuple(np.array(points["FL_RACK"]) + [.018, 0., .010])
        with TemporaryDirectory() as directory:
            path = Path(directory) / "tilted_mm.csv"
            write_points(path, points)
            model = load_model(path)
        motion = make_motion(model, samples=41, amplitudes_mm={"FL_WC_Z": 10, "FRONT_RACK": 8},
                             profiles={"FL_WC_Z": "ramp", "FRONT_RACK": "sine"})
        result = solve(model, motion)
        left, right = (model.point_names.index(f"{corner}_RACK") for corner in ("FL", "FR"))
        axis = model.reference[left] - model.reference[right]
        axis /= np.linalg.norm(axis)
        index = result.drive_names.index("FRONT_RACK")
        for node in (left, right):
            np.testing.assert_allclose(result.position[:, node] - model.reference[node],
                                       motion.displacement[:, index, None] * axis, atol=1e-10)
            np.testing.assert_allclose(result.velocity[:, node], motion.velocity[:, index, None] * axis, atol=1e-10)
            np.testing.assert_allclose(result.acceleration[:, node], motion.acceleration[:, index, None] * axis, atol=1e-10)
        self.assertAlmostEqual(motion.displacement[10, index], .008)
        self.assertAlmostEqual(motion.displacement[30, index], -.008)
        wheel = result.drive_names.index("FL_WC_Z")
        self.assertAlmostEqual(motion.displacement[10, wheel], .0025)

    def test_older_suspension_export_remains_solvable(self):
        sample = read_hardpoints(EXAMPLES / "hardpoints.csv")
        points = {f"{corner}_{name}": sample[f"{corner}_{name}"]
                  for corner in CORNERS for name in (*MOUNTS, *JOINTS, "WC")}
        with TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.csv"
            write_points(path, points)
            model = load_model(path)
        result = solve(model, make_motion(model, samples=21))
        self.assertNotIn("FRONT_RACK", result.drive_names)
        for corner in CORNERS:
            center, forward = (model.point_names.index(f"{corner}_{node}") for node in ("WC", "FORWARD"))
            np.testing.assert_allclose(result.position[:, center, 1], result.position[:, forward, 1], atol=1e-10)


if __name__ == "__main__":
    unittest.main()
