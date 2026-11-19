"""Verify directed rod angles and rotation rates against endpoint motion."""

import unittest

import numpy as np

from MultibodyGeometryLab.geometry import ARM_LINKS, LINKS, POINTS, build_vehicle
from MultibodyGeometryLab.project import default_project
from MultibodyGeometryLab.solver import CornerSolver


INDEX = {point: index for index, point in enumerate(POINTS)}
TIE_ROD = ("TOE", "RACK")
JOUNCE = (.018, .090, -.180)
RACK = (-.006, .035, .090)


def relative(values, link):
    a, b = link
    return values[INDEX[b]] - values[INDEX[a]]


def driver_position(driver, time):
    position, velocity, acceleration = driver
    return position + velocity * time + .5 * acceleration * time**2


def reconstructed_direction(azimuth, elevation):
    azimuth, elevation = np.radians([azimuth, elevation])
    return np.array([np.cos(elevation) * np.cos(azimuth),
                     np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])


class RodKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_vehicle(default_project())

    def test_static_angles_describe_every_directed_link(self):
        for corner in self.model.corners:
            pose = CornerSolver(corner).solve((0, 0, 0), (0, 0, 0))
            self.assertEqual(set(pose.rod_motion), set(LINKS))
            self.assertEqual(set(pose.arm_motion), {"UCA", "LCA"})
            for link, rod in pose.rod_motion.items():
                with self.subTest(corner=corner.name, link=link):
                    vector = relative(pose.position, link)
                    expected = vector / np.linalg.norm(vector)
                    if rod.azimuth_deg is None:
                        self.assertAlmostEqual(abs(rod.elevation_deg), 90)
                        np.testing.assert_allclose(expected[:2], 0, atol=1e-10)
                    else:
                        direction = reconstructed_direction(rod.azimuth_deg, rod.elevation_deg)
                        np.testing.assert_allclose(direction, expected, atol=1e-12)
                    self.assertAlmostEqual(rod.from_static_deg, 0)
                    np.testing.assert_allclose(rod.angular_velocity, 0, atol=1e-10)
                    np.testing.assert_allclose(rod.angular_acceleration, 0, atol=1e-10)
            for arm, hinge in pose.arm_motion.items():
                axis = relative(pose.position, (arm + "_F", arm + "_R"))
                np.testing.assert_allclose(hinge.axis, axis / np.linalg.norm(axis), atol=1e-12)
                self.assertAlmostEqual(hinge.angle_deg, 0)
                self.assertAlmostEqual(hinge.velocity, 0)
                self.assertAlmostEqual(hinge.acceleration, 0)

    def test_reported_rotations_reconstruct_all_moving_rod_endpoints(self):
        expected_models = dict.fromkeys(LINKS, "upright")
        expected_models.update(dict.fromkeys(ARM_LINKS[:2], "upper_arm"))
        expected_models.update(dict.fromkeys(ARM_LINKS[2:], "lower_arm"))
        expected_models[TIE_ROD] = "axis_only"
        for corner in self.model.corners:
            pose = CornerSolver(corner).solve(JOUNCE, RACK)
            for link, rod in pose.rod_motion.items():
                with self.subTest(corner=corner.name, link=link):
                    self.assertEqual(rod.rotation_model, expected_models[link])
                    vector = relative(pose.position, link)
                    baseline = relative(np.asarray(corner.reference), link)
                    expected_angle = np.degrees(np.arccos(np.clip(
                        np.dot(vector, baseline) / (np.linalg.norm(vector) * np.linalg.norm(baseline)), -1., 1.)))
                    self.assertAlmostEqual(rod.from_static_deg, expected_angle, delta=2e-6)
                    omega, alpha = rod.angular_velocity, rod.angular_acceleration
                    if rod.rotation_model == "axis_only":
                        self.assertAlmostEqual(np.dot(omega, vector), 0, places=10)
                    elif rod.rotation_model in ("upper_arm", "lower_arm"):
                        hinge = pose.arm_motion["UCA" if rod.rotation_model == "upper_arm" else "LCA"]
                        np.testing.assert_allclose(omega, hinge.velocity * hinge.axis, atol=1e-10, rtol=0)
                        np.testing.assert_allclose(alpha, hinge.acceleration * hinge.axis, atol=1e-10, rtol=0)
                    np.testing.assert_allclose(np.cross(omega, vector), relative(pose.velocity, link),
                                               atol=1e-8, rtol=0)
                    np.testing.assert_allclose(np.cross(alpha, vector) + np.cross(omega, np.cross(omega, vector)),
                                               relative(pose.acceleration, link), atol=1e-7, rtol=0)

    def test_tie_rod_and_hinge_rates_match_finite_differences(self):
        step = .002
        for corner in self.model.corners:
            solver = CornerSolver(corner)
            pose = solver.solve(JOUNCE, RACK)
            samples = [
                solver.solve((driver_position(JOUNCE, offset * step), 0., 0.),
                             (driver_position(RACK, offset * step), 0., 0.)).position
                for offset in (-2, -1, 0, 1, 2)
            ]

            def derivatives(data):
                m2, m1, zero, p1, p2 = data
                return ((m2 - 8 * m1 + 8 * p1 - p2) / (12 * step),
                        (-m2 + 16 * m1 - 30 * zero + 16 * p1 - p2) / (12 * step**2))

            axes = np.asarray([relative(points, TIE_ROD) for points in samples])
            axes /= np.linalg.norm(axes, axis=1, keepdims=True)
            axis_rate, axis_acceleration = derivatives(axes)
            rod = pose.rod_motion[TIE_ROD]
            with self.subTest(corner=corner.name, link=TIE_ROD):
                np.testing.assert_allclose(rod.angular_velocity, np.cross(axes[2], axis_rate),
                                           atol=2e-6, rtol=0)
                np.testing.assert_allclose(rod.angular_acceleration, np.cross(axes[2], axis_acceleration),
                                           atol=2e-4, rtol=0)

            for arm, joint in (("UCA", "UBJ"), ("LCA", "LBJ")):
                hinge = pose.arm_motion[arm]
                front = arm + "_F"
                axis = relative(samples[2], (front, arm + "_R"))
                axis /= np.linalg.norm(axis)
                start = relative(np.asarray(corner.reference), (front, joint))
                x = start - np.dot(start, axis) * axis
                y = np.cross(axis, x)
                rays = np.asarray([relative(points, (front, joint)) for points in samples])
                angles = np.arctan2(rays @ y, rays @ x)
                rate, acceleration = derivatives(angles)
                with self.subTest(corner=corner.name, arm=arm):
                    self.assertAlmostEqual(hinge.angle_deg, np.degrees(angles[2]), places=7)
                    self.assertAlmostEqual(hinge.velocity, rate, delta=2e-6)
                    self.assertAlmostEqual(hinge.acceleration, acceleration, delta=2e-4)


if __name__ == "__main__":
    unittest.main()
