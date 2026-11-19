import unittest
from types import SimpleNamespace

import numpy as np

from JacobianKinematicsLab.solver import Unreachable, solve


def rocker(*, duplicate_drive=False):
    def drive(name, direction):
        return SimpleNamespace(name=name, point="P", b="pivot", direction=direction)

    drives = (drive("X", (1.0, 0.0, 0.0)), drive("Y", (0.0, 1.0, 0.0)))
    if duplicate_drive:
        drives += (drive("X_copy", (1.0, 0.0, 0.0)),)
    return SimpleNamespace(
        point_names=("pivot", "P", "chassis"),
        reference=((0.0, 0.0, 0.0), (0.6, 0.0, 0.8), (1.0, 0.0, 0.0)),
        fixed=("pivot", "chassis"),
        links=(SimpleNamespace(name="rod", a="pivot", b="P"),
               SimpleNamespace(name="body", a="pivot", b="chassis")),
        drives=drives,
    )


def motion(displacement, velocity, acceleration):
    return SimpleNamespace(time=np.arange(len(displacement)), displacement=displacement,
                           velocity=velocity, acceleration=acceleration)


class SolverTest(unittest.TestCase):
    def test_rocker_matches_closed_form_kinematics(self):
        q = np.array([0.0, 0.17, -0.13])
        v = np.array([0.06, 0.0, -0.08])
        a = np.array([0.02, -0.03, 0.01])
        zero = np.zeros(len(q))
        result = solve(rocker(), motion(*(np.column_stack((values, zero)) for values in (q, v, a))))

        x = 0.6 + q
        z = np.sqrt(1 - x**2)
        expected_position = np.column_stack((x, zero, z))
        expected_velocity = np.column_stack((v, zero, -x * v / z))
        expected_acceleration = np.column_stack((a, zero, -x * a / z - v**2 / z**3))
        np.testing.assert_allclose(result.position[:, 1], expected_position, atol=1e-10)
        np.testing.assert_allclose(result.velocity[:, 1], expected_velocity, atol=1e-10)
        np.testing.assert_allclose(result.acceleration[:, 1], expected_acceleration, atol=1e-10)
        np.testing.assert_allclose(result.sensitivity[:, 1, :, 0], np.column_stack((1 + zero, zero, -x / z)), atol=1e-10)
        np.testing.assert_allclose(result.angular_velocity[:, 0], np.column_stack((zero, v / z, zero)), atol=1e-10)
        np.testing.assert_allclose(result.angular_acceleration[:, 0],
                                   np.column_stack((zero, a / z + x * v**2 / z**3, zero)), atol=1e-10)
        np.testing.assert_allclose(result.elevation_deg[:, 0], np.degrees(np.arctan2(z, x)))
        np.testing.assert_allclose(result.angle_deg[:, 0], abs(np.degrees(np.arcsin(x) - np.arcsin(0.6))), atol=1e-8)
        np.testing.assert_array_equal(result.angular_velocity[:, 1], 0.0)

    def test_relative_drive_moves_both_endpoints(self):
        model = SimpleNamespace(
            point_names=("A", "B"), reference=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            fixed=(), links=(), drives=tuple(
                SimpleNamespace(name=f"{point}_{axis}", point=point, b=b, direction=direction)
                for point, b in (("A", None), ("B", "A"))
                for axis, direction in zip("XYZ", np.eye(3))
            ),
        )
        drive_values = [[0.03, 0.0, 0.0, 0.02, 0.0, 0.0]]
        state = solve(model, motion(drive_values, drive_values, drive_values))
        np.testing.assert_allclose(state.position[0, :, 0], [0.03, 1.05])
        np.testing.assert_allclose(state.velocity[0, :, 0], [0.03, 0.05])
        np.testing.assert_allclose(state.acceleration[0, :, 0], [0.03, 0.05])

    def test_rigid_attached_points_follow_position_velocity_and_acceleration(self):
        a = np.array([1.2, -.7, 3.4])
        u, v = np.array([.3, .1, .2]), np.array([-.1, .4, .24])
        plane = .2 * u + .4 * v
        offset = plane + 1.3 * np.cross(u, v)
        relative = np.stack((u, v, plane, offset))
        model = SimpleNamespace(
            point_names=("A", "B", "C", "PLANE", "OFFSET", "BODY"),
            reference=np.vstack((a, a + relative, a + [-.36, -.15, .3])),
            fixed=("A", "BODY"),
            links=(SimpleNamespace(name="indicator", a="BODY", b="OFFSET", constrained=False),),
            attachments=tuple(SimpleNamespace(point=p, a="A", b="B", c="C") for p in ("PLANE", "OFFSET")),
            drives=tuple(
                SimpleNamespace(name=f"{point}_{axis}", point=point, b=None, direction=direction)
                for point in ("B", "C") for axis, direction in zip("XYZ", np.eye(3))
            ),
        )
        axis = np.array([2., -1., 3.]) / np.sqrt(14.)
        theta = np.array([0., .23, -.17])[:, None, None]
        oriented = (relative * np.cos(theta) + np.cross(axis, relative) * np.sin(theta)
                    + (relative @ axis)[:, None] * axis * (1. - np.cos(theta)))
        omega = np.array([.34, -.44, .28])[:, None, None] * axis
        alpha = np.array([-.15, .08, .32])[:, None, None] * axis
        displacement = oriented - relative
        velocity = np.cross(omega, oriented)
        acceleration = np.cross(alpha, oriented) + np.cross(omega, velocity)
        result = solve(model, motion(*(values[:, :2].reshape(-1, 6) for values in
                                       (displacement, velocity, acceleration))))
        np.testing.assert_allclose(result.position[:, 1:5], a + oriented, atol=1e-10)
        np.testing.assert_allclose(result.velocity[:, 1:5], velocity, atol=1e-10)
        np.testing.assert_allclose(result.acceleration[:, 1:5], acceleration, atol=1e-10)
        self.assertEqual(result.link_names, ("indicator",))
        self.assertTrue(np.isfinite(result.angular_velocity).all())
        self.assertTrue(np.isfinite(result.sensitivity).all())

    def test_collinear_attachment_anchors_are_rejected(self):
        model = rocker()
        model.reference = np.vstack((model.reference, (2., 0., 0.)))
        model.point_names += ("other",)
        model.attachments = (SimpleNamespace(point="P", a="pivot", b="chassis", c="other"),)
        with self.assertRaisesRegex(Unreachable, "附著點 P.*共線"):
            solve(model, motion([[0., 0.]], [[0., 0.]], [[0., 0.]]))

    def test_free_and_singular_geometry_are_rejected(self):
        free = rocker()
        free.drives = free.drives[:1]
        with self.assertRaisesRegex(Unreachable, "秩不足"):
            solve(free, motion([[0.0]], [[0.0]], [[0.0]]))
        with self.assertRaisesRegex(Unreachable, "t = 0 s"):
            solve(rocker(), motion([[0.4, 0.0]], [[0.0, 0.0]], [[0.0, 0.0]]))

    def test_continuation_span_is_bounded(self):
        with self.assertRaisesRegex(Unreachable, "續接求解範圍"):
            solve(rocker(), motion([[1e300, 0.0]], [[0.0, 0.0]], [[0.0, 0.0]]))
        with self.assertRaisesRegex(Unreachable, "加速度超出可計算範圍"):
            solve(rocker(), motion([[0.0, 0.0]], [[1e200, 0.0]], [[0.0, 0.0]]))

    def test_azimuth_unwraps_turns_and_preserves_vertical_gaps(self):
        model = rocker()
        model.reference = ((0.0, 0.0, 0.0), (-0.8, 0.6, 0.0), (1.0, 0.0, 0.0))
        model.drives[0].direction = (0.0, 0.0, 1.0)
        displacements = [[0.0, -0.5], [0.0, -0.7]]
        zeros = np.zeros((2, 2))
        rotating = solve(model, motion(displacements, zeros, zeros))
        expected = 180 + np.degrees(np.arcsin([-0.1, 0.1]))
        np.testing.assert_allclose(rotating.azimuth_deg[:, 0], expected)

        model = rocker()
        model.reference = ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        displacements = [[-0.1, 0.0], [0.0, 0.0], [0.1, 0.0]]
        zeros = np.zeros((3, 2))
        crossing = solve(model, motion(displacements, zeros, zeros))
        np.testing.assert_allclose(crossing.azimuth_deg[:, 0], [180.0, np.nan, 0.0], atol=1e-10)

    def test_redundant_drives_require_compatible_derivatives(self):
        model = rocker(duplicate_drive=True)
        state = solve(model, motion([[0.04, 0.0, 0.04]], [[0.1, 0.0, 0.1]], [[0.0, 0.0, 0.0]]))
        np.testing.assert_allclose(state.velocity[0, 1, 0], 0.1)
        self.assertTrue(np.isnan(state.sensitivity[..., 0]).all())
        self.assertTrue(np.isfinite(state.sensitivity[..., 1]).all())
        with self.assertRaisesRegex(Unreachable, "速度"):
            solve(model, motion([[0.0, 0.0, 0.0]], [[0.1, 0.0, 0.0]], [[0.0, 0.0, 0.0]]))


if __name__ == "__main__":
    unittest.main()
