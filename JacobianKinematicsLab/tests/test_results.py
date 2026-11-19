import csv
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

from JacobianKinematicsLab.model import Drive, Model, Motion
from JacobianKinematicsLab.results import export_results, motion_ratio
from JacobianKinematicsLab.solver import solve


def kinematic_history():
    time = np.array([0.0, 1.0])
    position = np.zeros((2, 2, 3))
    position[1, :, 2] = (0.1, -0.2)
    velocity = np.zeros_like(position)
    velocity[1, :, 2] = (0.3, -0.6)
    sensitivity = np.zeros((2, 2, 3, 2))
    sensitivity[:, :, 2, 0] = (1.0, -2.0)
    sensitivity[:, :, 2, 1] = (1.0, 3.0)
    return SimpleNamespace(
        time=time, point_names=("in", "out"), link_names=("rocker",), drive_names=("bump", "steer"),
        position=position, velocity=velocity, acceleration=np.zeros_like(position),
        sensitivity=sensitivity, azimuth_deg=np.array([[np.nan], [0.0]]),
        elevation_deg=np.array([[90.0], [70.0]]), angle_deg=np.array([[0.0], [20.0]]),
        angular_velocity=np.zeros((2, 1, 3)), angular_acceleration=np.zeros((2, 1, 3)),
    )


class ResultTests(unittest.TestCase):
    def test_node_displacement_and_export_use_the_static_reference(self):
        reference = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        drives = tuple(Drive(axis, "moving", tuple(direction)) for axis, direction in zip("xyz", np.eye(3)))
        model = Model("fixture", ("moving", "fixed"), reference, ("fixed",), (), drives)
        travel = np.array([[0.03, -0.02, 0.04], [-0.05, 0.01, 0.02]])
        zeros = np.zeros_like(travel)
        result = solve(model, Motion(np.array([0.0, 0.5]), travel, zeros, zeros))

        np.testing.assert_allclose(result.position[:, 0], reference[0] + travel)
        np.testing.assert_allclose(result.displacement[:, 0], travel)
        np.testing.assert_array_equal(result.displacement[:, 1], 0.)
        with TemporaryDirectory() as temp_dir:
            points, _ = export_results(temp_dir, result)
            with points.open(encoding="utf-8-sig", newline="") as handle:
                rows = list(csv.DictReader(handle))
        np.testing.assert_allclose([[float(row[f"d{axis}_m"]) for axis in "xyz"] for row in rows[::2]], travel)
        self.assertEqual(float(rows[0]["x_m"]), 1.03)

    def test_selected_drive_preserves_ratio_at_rest(self):
        result = kinematic_history()
        path = motion_ratio(result, "in", "out")
        self.assertTrue(np.isnan(path.motion_ratio[0]))
        self.assertEqual(path.status[0], "stationary")
        np.testing.assert_allclose(path.motion_ratio[1:], [-2.0])
        np.testing.assert_allclose(path.output_displacement, [0.0, -0.2])

        bump = motion_ratio(result, "in", "out", (0, 0, 4), "-z", drive="bump")
        np.testing.assert_allclose(bump.motion_ratio, [2.0, 2.0])
        np.testing.assert_allclose(bump.mechanical_advantage, [0.5, 0.5])
        steer = motion_ratio(result, "in", "out", drive="steer")
        np.testing.assert_allclose(steer.motion_ratio, [3.0, 3.0])

    def test_zero_projection_and_unavailable_drive(self):
        result = kinematic_history()
        ratio = motion_ratio(result, "in", "out", "z", "x", drive="bump")
        np.testing.assert_allclose(ratio.motion_ratio, [0.0, 0.0])
        self.assertTrue(np.isnan(ratio.mechanical_advantage).all())
        self.assertTrue((ratio.status == "output_projection_zero").all())
        result.sensitivity[0, :, :, 0] = np.nan
        ratio = motion_ratio(result, "in", "out", drive="bump")
        self.assertTrue(np.isnan(ratio.motion_ratio[0]))
        self.assertEqual(ratio.status[0], "drive_unavailable")

    def test_exports_all_samples_and_empty_undefined_cells(self):
        result = kinematic_history()
        ratio = motion_ratio(result, "in", "out")
        with TemporaryDirectory() as temp_dir:
            paths = export_results(Path(temp_dir), result, ratio)
            self.assertEqual([path.name for path in paths],
                             ["point_kinematics.csv", "rod_kinematics.csv", "motion_ratio.csv"])
            tables = []
            for path in paths:
                with path.open(encoding="utf-8-sig", newline="") as handle:
                    tables.append(list(csv.DictReader(handle)))

        points, rods, ratios = tables
        self.assertEqual([row["point"] for row in points], ["in", "out", "in", "out"])
        self.assertEqual(float(points[-1]["vz_m_s"]), -0.6)
        self.assertEqual(rods[0]["azimuth_deg"], "")
        self.assertEqual(float(rods[-1]["angle_from_reference_deg"]), 20.0)
        self.assertEqual(ratios[0]["motion_ratio"], "")
        self.assertEqual(float(ratios[1]["ideal_mechanical_advantage"]), 0.5)


if __name__ == "__main__":
    unittest.main()
