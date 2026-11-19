"""Regression checks for the prescribed motion presets."""

import unittest

import numpy as np

from MultibodyGeometryLab.animation import PRESETS, build_animation
from MultibodyGeometryLab.geometry import LINKS, POINTS, build_vehicle
from MultibodyGeometryLab.project import default_project


WHEEL = POINTS.index("WC")
RACK = POINTS.index("RACK")
MOUNT = POINTS.index("UCA_F")
START, END = np.asarray([(POINTS.index(a), POINTS.index(b)) for a, b in LINKS]).T


def to_world(frame, positions):
    return (positions - frame.pivot) @ frame.rotation.T + frame.pivot + frame.translation


class AnimationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_vehicle(default_project())
        cls.reference = {corner.name: np.asarray(corner.reference) for corner in cls.model.corners}
        cls.duration = 4.0
        cls.frames = {
            "single_wheel": build_animation(cls.model, "single_wheel", 18, cls.duration, corner="RL"),
            "heave": build_animation(cls.model, "heave", 20, cls.duration),
            "roll": build_animation(cls.model, "roll", 3, cls.duration),
            "front_steer": build_animation(cls.model, "front_steer", 5, cls.duration),
        }

    def test_presets_return_to_neutral_and_preserve_link_lengths(self):
        self.assertTrue(set(self.frames) <= set(PRESETS))
        for preset, frames in self.frames.items():
            with self.subTest(preset=preset):
                self.assertGreater(len(frames), 10)
                self.assertAlmostEqual(frames[0].time, 0)
                self.assertAlmostEqual(frames[-1].time, self.duration)
                self.assertTrue(all(a.time < b.time for a, b in zip(frames, frames[1:])))
                for name, reference in self.reference.items():
                    for frame in (frames[0], frames[-1]):
                        pose = frame.result.corners[name]
                        np.testing.assert_allclose(to_world(frame, pose.position), reference,
                                                   atol=1e-10, rtol=0)
                        np.testing.assert_allclose(pose.velocity, 0, atol=1e-10, rtol=0)
                        np.testing.assert_allclose(pose.acceleration, 0, atol=1e-10, rtol=0)
                    positions = np.asarray([to_world(frame, frame.result.corners[name].position)
                                            for frame in frames])
                    lengths = np.linalg.norm(positions[:, START] - positions[:, END], axis=-1)
                    neutral_lengths = np.linalg.norm(reference[START] - reference[END], axis=-1)
                    np.testing.assert_allclose(lengths - neutral_lengths, 0, atol=1e-8, rtol=0)

    def test_single_wheel_lifts_only_the_selected_corner(self):
        frames = self.frames["single_wheel"]
        for name, reference in self.reference.items():
            positions = np.asarray([to_world(frame, frame.result.corners[name].position) for frame in frames])
            if name == "RL":
                height = positions[:, WHEEL, 2] - reference[WHEEL, 2]
                self.assertAlmostEqual(max(height), 0.018, delta=2e-5)
                self.assertGreaterEqual(min(height), -1e-10)
                np.testing.assert_allclose(positions[:, MOUNT] - reference[MOUNT], 0, atol=1e-10, rtol=0)
            else:
                np.testing.assert_allclose(positions - reference, 0, atol=1e-10, rtol=0)

    def reference_wheel_height_errors(self, frames):
        """Return the signed Z error of each world-space wheel center."""
        return np.asarray([
            [to_world(frame, frame.result.corners[name].position[WHEEL])[2] - reference[WHEEL, 2]
             for name, reference in self.reference.items()] for frame in frames
        ])

    def test_heave_moves_the_body_while_holding_each_world_wheel_height(self):
        frames = self.frames["heave"]
        peak = max(frames, key=lambda frame: frame.translation[2])
        np.testing.assert_allclose(peak.translation, [0, 0, .020], atol=2e-5, rtol=0)
        self.assertAlmostEqual(min(frame.translation[2] for frame in frames), -.020, delta=2e-5)
        np.testing.assert_allclose(peak.rotation, np.eye(3), atol=1e-12, rtol=0)
        for name, reference in self.reference.items():
            mount = peak.result.corners[name].position[MOUNT]
            np.testing.assert_allclose(to_world(peak, mount) - reference[MOUNT], [0, 0, .020],
                                       atol=2e-5, rtol=0)
        np.testing.assert_allclose(self.reference_wheel_height_errors(frames), 0, atol=1e-8, rtol=0)

    def test_roll_lifts_left_chassis_and_holds_each_world_wheel_height(self):
        frames = self.frames["roll"]

        def angle(frame):
            return np.degrees(np.arctan2(frame.rotation[2, 1], frame.rotation[1, 1]))

        peak = max(frames, key=angle)
        self.assertAlmostEqual(angle(peak), 3.0, delta=.003)
        self.assertAlmostEqual(min(map(angle, frames)), -3.0, delta=.003)
        np.testing.assert_allclose(peak.rotation @ [1, 0, 0], [1, 0, 0], atol=1e-12, rtol=0)
        np.testing.assert_allclose(to_world(peak, np.asarray(self.model.cg)), self.model.cg, atol=1e-12, rtol=0)
        for name, reference in self.reference.items():
            mount_z = to_world(peak, peak.result.corners[name].position[MOUNT])[2]
            self.assertGreater((mount_z - reference[MOUNT, 2]) * (1 if name.endswith("L") else -1), 0)
        np.testing.assert_allclose(self.reference_wheel_height_errors(frames), 0, atol=1e-8, rtol=0)

    def test_roll_jacobian_rates_match_body_frame_position_derivatives(self):
        frames = self.frames["roll"]
        i = len(frames) // 3
        frame = frames[i]
        h = frames[i + 1].time - frame.time
        for name in self.reference:
            with self.subTest(corner=name):
                q_m2, q_m1, q_0, q_p1, q_p2 = (f.result.corners[name].position for f in frames[i - 2:i + 3])
                # Five-point stencils keep the temporal discretization error
                # below the threshold for missing moving-plane terms.
                velocity = (q_m2 - 8 * q_m1 + 8 * q_p1 - q_p2) / (12 * h)
                acceleration = (-q_m2 + 16 * q_m1 - 30 * q_0 + 16 * q_p1 - q_p2) / (12 * h**2)
                pose = frame.result.corners[name]
                np.testing.assert_allclose(pose.velocity, velocity, atol=2e-5, rtol=0)
                np.testing.assert_allclose(pose.acceleration, acceleration, atol=2e-4, rtol=0)

    def test_front_steer_moves_only_the_front_rack_and_wheel_ends(self):
        frames = self.frames["front_steer"]
        peak = max(frames, key=lambda frame: frame.result.corners["FL"].position[RACK, 1])
        for name, reference in self.reference.items():
            pose = peak.result.corners[name]
            if name.startswith("F"):
                np.testing.assert_allclose(pose.position[RACK] - reference[RACK], [0, .005, 0],
                                           atol=5e-6, rtol=0)
                self.assertGreater(abs(pose.toe_deg), 1.0)
            else:
                np.testing.assert_allclose(pose.position, reference, atol=1e-10, rtol=0)


if __name__ == "__main__":
    unittest.main()
