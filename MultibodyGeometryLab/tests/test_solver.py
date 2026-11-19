"""Numerical checks for the four-corner constrained mechanism."""

import unittest

import numpy as np

from MultibodyGeometryLab.geometry import LINKS, POINTS, build_vehicle
from MultibodyGeometryLab.project import default_project
from MultibodyGeometryLab.solver import CornerSolver, Unreachable, solve_vehicle


WHEEL = POINTS.index("WC")
RACK = POINTS.index("RACK")


class SolverTests(unittest.TestCase):
    def setUp(self):
        self.project = default_project()
        self.model = build_vehicle(self.project)

    def test_zero_inputs_reproduce_the_static_geometry(self):
        result = solve_vehicle(self.model, self.project["motion"])
        self.assertEqual(set(result.corners), {"FL", "FR", "RL", "RR"})
        for corner in self.model.corners:
            with self.subTest(corner=corner.name):
                pose = result.corners[corner.name]
                np.testing.assert_allclose(pose.position, corner.reference, rtol=0, atol=1e-10)
                np.testing.assert_allclose(pose.velocity, 0, rtol=0, atol=1e-12)
                np.testing.assert_allclose(pose.acceleration, 0, rtol=0, atol=1e-12)

    def test_combined_rack_and_travel_preserve_every_link_length(self):
        motion = self.project["motion"]
        for name, travel in (("FL", 20), ("FR", -12), ("RL", -8), ("RR", 14),
                             ("front", 5), ("rear", -3)):
            motion[name]["position_mm"] = travel
        result = solve_vehicle(self.model, motion)
        for corner in self.model.corners:
            with self.subTest(corner=corner.name):
                pose = result.corners[corner.name]
                reference = np.asarray(corner.reference)
                delta = pose.position - reference
                self.assertAlmostEqual(delta[WHEEL, 2], motion[corner.name]["position_mm"] / 1000, places=8)
                np.testing.assert_allclose(delta[RACK], [0, motion[corner.axle]["position_mm"] / 1000, 0],
                                           atol=1e-12, rtol=0)
                self.assertGreater(abs(pose.toe_deg), 0.1)
                for start, end in LINKS:
                    a, b = POINTS.index(start), POINTS.index(end)
                    actual = np.linalg.norm(pose.position[a] - pose.position[b])
                    baseline = np.linalg.norm(reference[a] - reference[b])
                    self.assertAlmostEqual(actual, baseline, places=8, msg=f"{start}–{end}")

    def test_jacobian_rates_match_position_finite_differences(self):
        jounce = (0.018, 0.100, -0.200)
        rack = (-0.006, 0.040, 0.120)
        step = 0.002

        def position_at(driver, time):
            position, velocity, acceleration = driver
            return position + velocity * time + 0.5 * acceleration * time**2

        for corner in self.model.corners:
            with self.subTest(corner=corner.name):
                solver = CornerSolver(corner)
                pose = solver.solve(jounce, rack)

                def solve_position(time):
                    # Perturb only the position inputs to independently estimate derivatives.
                    return solver.solve((position_at(jounce, time), 0, 0),
                                        (position_at(rack, time), 0, 0)).position

                before, after = solve_position(-step), solve_position(step)
                numerical_velocity = (after - before) / (2 * step)
                numerical_acceleration = (after - 2 * pose.position + before) / step**2
                np.testing.assert_allclose(pose.velocity, numerical_velocity, atol=1e-6, rtol=0)
                np.testing.assert_allclose(pose.acceleration, numerical_acceleration, atol=1e-4, rtol=0)

    def test_invalid_and_unreachable_inputs_leave_solver_reusable(self):
        corner = self.model.corners[0]
        solver = CornerSolver(corner)
        zero = (0, 0, 0)
        with self.assertRaises(ValueError):
            solver.solve((0, 0), zero)
        with self.assertRaises(ValueError):
            solver.solve((0, float("nan"), 0), zero)
        with self.assertRaises(Unreachable):
            solver.solve((0.45, 0, 0), zero)
        np.testing.assert_allclose(solver.solve(zero, zero).position, corner.reference, atol=1e-10, rtol=0)


if __name__ == "__main__":
    unittest.main()
