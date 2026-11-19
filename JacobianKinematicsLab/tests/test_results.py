import csv
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

from JacobianKinematicsLab.results import export_results, motion_ratio


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
