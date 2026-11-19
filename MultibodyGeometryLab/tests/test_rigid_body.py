"""Mass, force, and moment balance regressions for the rigid chassis."""

import unittest

import numpy as np

from MultibodyGeometryLab.deformable import build_dynamics_model
from MultibodyGeometryLab.geometry import FIXED, POINTS, build_vehicle
from MultibodyGeometryLab.project import default_project
from MultibodyGeometryLab.rigid_body import simulate_rigid_road


INERTIA = {"ixx": 55., "iyy": 95., "izz": 125.}
HUB = POINTS.index("WC")


def road_settings(settings, bump_mm, inertia):
    return {
        **settings,
        "chassis_inertia": inertia.copy(),
        "simulation": {"target": "FL", "bump_mm": bump_mm,
                       "bump_seconds": .24, "duration_seconds": .50},
    }


class RigidBodyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        project = default_project()
        cls.vehicle = project["geometry"]["vehicle"]
        cls.settings = road_settings(project["dynamics"], 20., INERTIA)
        cls.geometry = build_vehicle(project)
        cls.model = build_dynamics_model(cls.geometry, cls.vehicle, cls.settings)
        cls.frames = simulate_rigid_road(cls.model, cls.settings, cls.vehicle["mass"])

    def test_no_input_preserves_equilibrium_with_asymmetric_wheel_masses(self):
        project = default_project()
        settings = road_settings(project["dynamics"], 0., INERTIA)
        settings["corners"]["FL"]["mass_kg"] = 15.
        settings["corners"]["RR"]["mass_kg"] = 8.
        model = build_dynamics_model(self.geometry, self.vehicle, settings)
        mass = self.vehicle["mass"]
        frames = simulate_rigid_road(model, settings, mass)
        sprung_mass = mass - sum(spec.mass for spec in model.corners.values())
        wheel_moments = sum(model.corners[corner.name].mass * np.asarray(corner.reference[HUB])
                            for corner in self.geometry.corners)
        self.assertAlmostEqual(frames[-1].time, .50)
        for frame in frames:
            # The geometry specifies total-vehicle CG, so the sprung pivot
            # and wheel masses together must reproduce that CG at rest.
            np.testing.assert_allclose(sprung_mass * frame.pivot + wheel_moments,
                                       mass * np.asarray(self.geometry.cg), atol=1e-7, rtol=0)
            np.testing.assert_allclose(frame.translation, 0., atol=1e-8, rtol=0)
            np.testing.assert_allclose(frame.rotation, np.eye(3), atol=1e-8, rtol=0)
            np.testing.assert_allclose(frame.body.acceleration, 0., atol=1e-7, rtol=0)
            np.testing.assert_allclose(frame.body.angular_acceleration, 0., atol=1e-7, rtol=0)

    def test_single_wheel_bump_rotates_chassis_and_changes_with_roll_inertia(self):
        baseline_angles = np.array([frame.body.angles_deg for frame in self.frames])
        self.assertGreater(np.ptp(baseline_angles[:, 0]), .08)
        self.assertGreater(np.ptp(baseline_angles[:, 1]), .08)

        heavier_roll = road_settings(self.settings, 20., {**INERTIA, "ixx": 105.})
        frames = simulate_rigid_road(self.model, heavier_roll, self.vehicle["mass"])
        early = next(i for i, frame in enumerate(self.frames) if frame.time >= .04)
        self.assertAlmostEqual(frames[early].time, self.frames[early].time)
        initial_roll_accel = abs(self.frames[early].body.angular_acceleration[0])
        increased_roll_mass_accel = abs(frames[early].body.angular_acceleration[0])
        self.assertGreater(initial_roll_accel, .1)
        self.assertLess(increased_roll_mass_accel, .85 * initial_roll_accel)

    def test_sprung_chassis_force_and_moment_balance(self):
        mass = self.vehicle["mass"] - sum(spec.mass for spec in self.model.corners.values())
        inertia = np.diag([INERTIA[axis] for axis in ("ixx", "iyy", "izz")])
        samples = [frame for frame in self.frames if .08 <= frame.time <= .4]
        self.assertGreater(len(samples), 4)
        for frame in samples:
            with self.subTest(time=frame.time):
                np.testing.assert_allclose(frame.rotation.T @ frame.rotation, np.eye(3), atol=1e-10, rtol=0)
                np.testing.assert_allclose(frame.translation, frame.body.displacement, atol=1e-10, rtol=0)
                net_force = np.array([0., 0., -mass * self.model.gravity])
                net_torque = np.zeros(3)
                for corner in self.model.corners:
                    chassis_loads = frame.mount_loads[corner]
                    self.assertEqual(set(chassis_loads.by_mount), {*FIXED, "DAMPER"})
                    for name, force in chassis_loads.by_mount.items():
                        if name == "DAMPER":
                            point = frame.dampers[corner].fixed_point
                        else:
                            point = frame.result.corners[corner].position[POINTS.index(name)]
                        world_lever = frame.rotation @ (point - frame.pivot)
                        net_force += force
                        net_torque += np.cross(world_lever, force)

                np.testing.assert_allclose(net_force, mass * frame.body.acceleration, atol=1e-5, rtol=0)
                world_inertia = frame.rotation @ inertia @ frame.rotation.T
                angular_momentum = world_inertia @ frame.body.angular_velocity
                momentum_rate = world_inertia @ frame.body.angular_acceleration
                momentum_rate += np.cross(frame.body.angular_velocity, angular_momentum)
                np.testing.assert_allclose(net_torque, momentum_rate, atol=1e-5, rtol=0)


if __name__ == "__main__":
    unittest.main()
