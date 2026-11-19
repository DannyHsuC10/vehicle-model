"""Road-input, rigid-body attachment, and load-transfer regressions."""

import unittest

import numpy as np

from MultibodyGeometryLab.deformable import build_dynamics_model
from MultibodyGeometryLab.geometry import ARM_LINKS, POINTS, build_vehicle
from MultibodyGeometryLab.heave import simulate_road_heave
from MultibodyGeometryLab.project import default_project


ARM_MOUNTS = tuple(mount for _, mount in ARM_LINKS)
ARM_INDICES = [POINTS.index(mount) for mount in ARM_MOUNTS]
WHEEL = POINTS.index("WC")


def world_points(frame, points):
    return (points - frame.pivot) @ frame.rotation.T + frame.pivot + frame.translation


class CoupledRoadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        project = default_project()
        vehicle = project["geometry"]["vehicle"]
        cls.settings = project["dynamics"]
        cls.mass = vehicle["mass"]
        cls.model = build_dynamics_model(build_vehicle(project), vehicle, cls.settings)
        cls.sprung_mass = cls.mass - sum(spec.mass for spec in cls.model.corners.values())

    def run_road(self, target, bump_mm):
        settings = {**self.settings,
                    "simulation": {"target": target, "bump_mm": bump_mm,
                                   "bump_seconds": .25, "duration_seconds": .75}}
        return simulate_road_heave(self.model, settings, self.mass)

    def assert_force_balance(self, frame):
        body_accel = frame.heave.acceleration
        body_load_z = frame.heave.force_z - self.sprung_mass * self.model.gravity
        for corner, spec in self.model.corners.items():
            pose = frame.result.corners[corner]
            damper = frame.dampers[corner]
            mounts = frame.mount_loads[corner]
            self.assertEqual(set(mounts.by_mount), {*ARM_MOUNTS, "RACK", "DAMPER"})
            np.testing.assert_allclose(mounts.total, np.sum(list(mounts.by_mount.values()), axis=0),
                                       atol=1e-7, rtol=0)
            # External ground force and wheel weight minus wheel inertia give
            # the force this corner transmits to the chassis, independent of
            # how the individual linkage constraint forces are recovered.
            hub_world_accel = pose.acceleration[WHEEL] + [0., 0., body_accel]
            expected_load = np.array([0., 0., damper.tire_force - spec.mass * self.model.gravity])
            expected_load -= spec.mass * hub_world_accel
            np.testing.assert_allclose(mounts.total, expected_load, atol=1e-6, rtol=0)
            for joint, mount in ARM_LINKS:
                axis = pose.position[POINTS.index(joint)] - pose.position[POINTS.index(mount)]
                np.testing.assert_allclose(np.cross(mounts.by_mount[mount], axis), 0., atol=1e-6, rtol=0)
            body_load_z += mounts.total[2]
        self.assertAlmostEqual(body_load_z, self.sprung_mass * body_accel, delta=1e-6)

    def test_single_wheel_bump_transfers_load_to_shared_chassis(self):
        frames = self.run_road("FL", 20.)
        first = frames[0]
        self.assertAlmostEqual(max(frame.dampers["FL"].road_height for frame in frames), .020)
        self.assertGreater(max(frame.heave.displacement for frame in frames), .001)
        self.assertGreater(max(frame.heave.acceleration for frame in frames), .10)
        self.assertLess(min(frame.heave.acceleration for frame in frames), -.10)
        for corner in ("FR", "RL", "RR"):
            self.assertTrue(all(frame.dampers[corner].road_height == 0. for frame in frames))
            hub_heights = [world_points(frame, frame.result.corners[corner].position[WHEEL])[2]
                           for frame in frames]
            self.assertGreater(np.ptp(hub_heights), .0001)
        peak = min(frames, key=lambda frame: frame.dampers["FL"].deflection)
        self.assertLess(peak.dampers["FL"].deflection, -.005)
        self.assertGreater(max(
            np.linalg.norm(peak.mount_loads["FL"].by_mount[mount] -
                           first.mount_loads["FL"].by_mount[mount])
            for mount in ARM_MOUNTS
        ), 10.)
        for frame in (first, peak, max(frames, key=lambda frame: frame.heave.displacement), frames[-1]):
            self.assert_force_balance(frame)
            np.testing.assert_allclose(frame.translation, [0., 0., frame.heave.displacement], atol=1e-10, rtol=0)
            for corner in self.model.corners:
                initial_mounts = first.result.corners[corner].position[ARM_INDICES]
                current_mounts = frame.result.corners[corner].position[ARM_INDICES]
                delta = world_points(frame, current_mounts) - world_points(first, initial_mounts)
                np.testing.assert_allclose(delta, np.tile(frame.translation, (len(ARM_INDICES), 1)),
                                           atol=1e-10, rtol=0)
                initial_damper = world_points(first, first.dampers[corner].fixed_point)
                current_damper = world_points(frame, frame.dampers[corner].fixed_point)
                np.testing.assert_allclose(current_damper - initial_damper, frame.translation, atol=1e-10, rtol=0)

    def test_four_wheel_bump_lifts_body_from_loaded_equilibrium(self):
        frames = self.run_road("all", 20.)
        first = frames[0]
        self.assertAlmostEqual(first.heave.displacement, 0.)
        self.assertAlmostEqual(first.heave.acceleration, 0., delta=1e-9)
        self.assertAlmostEqual(sum(element.tire_force for element in first.dampers.values()),
                               self.mass * self.model.gravity, delta=1e-7)
        self.assert_force_balance(first)
        crest = max(frames, key=lambda frame: frame.dampers["FL"].road_height)
        self.assertGreater(crest.heave.displacement, .001)
        for corner in self.model.corners:
            self.assertAlmostEqual(crest.dampers[corner].road_height, .020)
            self.assertLess(crest.dampers[corner].deflection, -.001)
        self.assert_force_balance(crest)
        self.assertLess(min(frame.heave.acceleration for frame in frames if frame.time > crest.time), -.1)


if __name__ == "__main__":
    unittest.main()
