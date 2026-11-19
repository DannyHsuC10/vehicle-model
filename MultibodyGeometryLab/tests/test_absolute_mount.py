"""A slanted push rod uses the current line between its two mounting points."""

import unittest

import numpy as np

from MultibodyGeometryLab.deformable import build_dynamics_model, sample_dampers
from MultibodyGeometryLab.geometry import POINTS, build_vehicle
from MultibodyGeometryLab.project import default_project, validate
from MultibodyGeometryLab.rigid_body import simulate_rigid_road
from MultibodyGeometryLab.solver import CornerSolver, VehicleResult


ZERO = (0., 0., 0.)
UBJ = POINTS.index("UBJ")
MOUNT_MM = (70., 430., 520.)


class AbsoluteMountTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        project = default_project()
        project["dynamics"]["corners"]["FL"].update(
            attachment="UBJ", mount_reference="body_coordinates", mount_mm=list(MOUNT_MM),
        )
        project["dynamics"]["simulation"].update(
            target="FL", bump_mm=20., bump_seconds=.24, duration_seconds=.50,
        )
        cls.project = validate(project)
        cls.geometry = build_vehicle(cls.project)
        cls.vehicle = cls.project["geometry"]["vehicle"]
        cls.settings = cls.project["dynamics"]
        cls.model = build_dynamics_model(cls.geometry, cls.vehicle, cls.settings)

    def test_static_chassis_xyz_and_wheel_motion_set_the_force_direction(self):
        mount = np.asarray(MOUNT_MM) / 1000
        np.testing.assert_array_equal(self.model.corners["FL"].fixed_point, mount)
        self.assertEqual(self.settings["corners"]["FR"]["mount_reference"], "attachment")
        solvers = {corner.name: CornerSolver(corner) for corner in self.geometry.corners}
        static = {name: solver.solve(ZERO, ZERO) for name, solver in solvers.items()}
        moved = {**static, "FL": solvers["FL"].solve((.025, .04, 0.), ZERO)}
        dampers = [sample_dampers(self.model, VehicleResult(poses))["FL"]
                   for poses in (static, moved)]
        for damper, poses in zip(dampers, (static, moved)):
            pose = poses["FL"]
            np.testing.assert_array_equal(damper.fixed_point, mount)
            np.testing.assert_array_equal(damper.moving_point, pose.position[UBJ])
            axis = pose.position[UBJ] - mount
            expected_angle = np.degrees(np.arctan2(np.linalg.norm(axis[:2]), abs(axis[2])))
            self.assertAlmostEqual(damper.inclination_deg, expected_angle, places=10)
            np.testing.assert_allclose(damper.force_on_chassis, -damper.force * axis / np.linalg.norm(axis),
                                       atol=1e-9, rtol=0)
            # The component delivered along wheel-end velocity matches axial power.
            self.assertAlmostEqual(float(-damper.force_on_chassis @ pose.velocity[UBJ]),
                                   damper.force * damper.velocity, places=9)

        rest, bump = dampers
        self.assertGreater(rest.inclination_deg, 20.)
        self.assertGreater(abs(rest.inclination_deg - bump.inclination_deg), 1.)
        self.assertGreater(np.linalg.norm(rest.force_on_chassis[:2]), 100.)
        rest_direction = rest.force_on_chassis / np.linalg.norm(rest.force_on_chassis)
        bump_direction = bump.force_on_chassis / np.linalg.norm(bump.force_on_chassis)
        self.assertGreater(np.linalg.norm(rest_direction - bump_direction), .01)

    def test_rigid_road_rotates_the_current_damper_force_to_ground_axes(self):
        frames = simulate_rigid_road(self.model, self.settings, self.vehicle["mass"])
        mount = np.asarray(MOUNT_MM) / 1000
        direction = []
        for frame in frames:
            damper = frame.dampers["FL"]
            np.testing.assert_array_equal(damper.fixed_point, mount)
            chassis_force = frame.mount_loads["FL"].by_mount["DAMPER"]
            expected = frame.rotation @ damper.force_on_chassis
            np.testing.assert_allclose(chassis_force, expected, atol=1e-9, rtol=0)
            direction.append(chassis_force / np.linalg.norm(chassis_force))

        self.assertGreater(max(np.linalg.norm(vector - direction[0]) for vector in direction[1:]), .01)
        self.assertGreater(max(abs(frame.body.angles_deg[0]) for frame in frames), .05)


if __name__ == "__main__":
    unittest.main()
