"""Regression checks for deformable coilovers and constrained wheel dynamics."""

import unittest

import numpy as np

from MultibodyGeometryLab.deformable import build_dynamics_model, sample_dampers
from MultibodyGeometryLab.dynamics import simulate_dynamics
from MultibodyGeometryLab.geometry import LINKS, POINTS, build_vehicle
from MultibodyGeometryLab.project import default_project, validate
from MultibodyGeometryLab.solver import CornerSolver, VehicleResult


ZERO = (0., 0., 0.)
WHEEL = POINTS.index("WC")
UPPER_JOINT = POINTS.index("UBJ")
LINK_START, LINK_END = np.asarray([(POINTS.index(a), POINTS.index(b)) for a, b in LINKS]).T


def make_model(project, geometry=None):
    if geometry is None:
        geometry = build_vehicle(project)
    return build_dynamics_model(geometry, project["geometry"]["vehicle"], project["dynamics"])


def point_at(solver, travel):
    return solver.solve((travel, 0., 0.), ZERO).position


class DamperGeometryTests(unittest.TestCase):
    def setUp(self):
        self.project = default_project()
        self.geometry = build_vehicle(self.project)

    def test_default_push_rods_share_ubj_and_are_parallel_at_static_ride_height(self):
        revised = default_project()
        revised["geometry"]["vehicle"].update(track_front=1.30, track_rear=1.28)
        revised["geometry"]["front"]["caster_deg"] = 6.0
        for design, project in (("default", self.project), ("revised", revised)):
            geometry = build_vehicle(project)
            model = make_model(project, geometry)
            solvers = {corner.name: CornerSolver(corner) for corner in geometry.corners}
            motion = VehicleResult({name: solver.solve((.01, .02, 0.), ZERO) for name, solver in solvers.items()})
            dampers = sample_dampers(model, motion)
            for corner in geometry.corners:
                name = corner.name
                with self.subTest(design=design, corner=name):
                    spec = model.corners[name]
                    damper = dampers[name]
                    self.assertEqual(spec.attachment_index, UPPER_JOINT)
                    np.testing.assert_allclose(
                        np.asarray(corner.reference[UPPER_JOINT]) - spec.fixed_point,
                        [0., 0., -.25], atol=1e-10, rtol=0,
                    )
                    np.testing.assert_array_equal(damper.fixed_point, spec.fixed_point)
                    np.testing.assert_allclose(
                        damper.moving_point, motion.corners[name].position[UPPER_JOINT],
                        atol=1e-10, rtol=0,
                    )
                    self.assertLess(damper.length, spec.static_length)

    def test_legacy_mount_offset_without_reference_is_relative_to_wheel_center(self):
        settings = self.project["dynamics"]["corners"]["FL"]
        settings.update(attachment="UBJ", mount_mm=[12., -8., 270.])
        settings.pop("mount_reference", None)
        project = validate(self.project)
        self.assertEqual(project["dynamics"]["corners"]["FL"]["mount_reference"], "wheel_center")
        model = make_model(project, self.geometry)
        corner = next(corner for corner in self.geometry.corners if corner.name == "FL")
        expected = np.asarray(corner.reference[WHEEL]) + np.asarray(settings["mount_mm"]) / 1000
        np.testing.assert_allclose(model.corners["FL"].fixed_point, expected, atol=1e-10, rtol=0)
        self.assertEqual(model.corners["FL"].attachment_index, UPPER_JOINT)

    def test_static_preload_balances_gravity_and_tire_force(self):
        model = make_model(self.project, self.geometry)
        solvers = {corner.name: CornerSolver(corner) for corner in self.geometry.corners}
        static = VehicleResult({name: solver.solve(ZERO, ZERO) for name, solver in solvers.items()})
        dampers = sample_dampers(model, static, {name: (0., 0.) for name in solvers})
        step = .0001
        for name, solver in solvers.items():
            with self.subTest(corner=name):
                spec = model.corners[name]
                damper = dampers[name]
                def length(travel):
                    return np.linalg.norm(point_at(solver, travel)[spec.attachment_index] - spec.fixed_point)

                slope = (length(step) - length(-step)) / (2 * step)
                self.assertAlmostEqual(damper.length, length(0.))
                self.assertAlmostEqual(damper.deflection, 0.)
                self.assertAlmostEqual(damper.velocity, 0.)
                self.assertAlmostEqual(damper.acceleration, 0.)
                self.assertAlmostEqual(damper.tire_force, spec.static_load, delta=1e-8)
                self.assertAlmostEqual(damper.damping_force, 0.)
                self.assertAlmostEqual(damper.spring_force, spec.stiffness * (spec.free_length - damper.length))
                self.assertAlmostEqual(damper.tire_force - spec.mass * model.gravity + damper.force * slope,
                                       0., delta=1e-4)

    def test_damper_deforms_alongside_rigid_links_and_matches_finite_differences(self):
        settings = self.project["dynamics"]["corners"]["FL"]
        settings.update(attachment="UBJ", mount_mm=[-30., -20., 300.])
        model = make_model(self.project, self.geometry)
        solver = CornerSolver(next(corner for corner in self.geometry.corners if corner.name == "FL"))
        spec = model.corners["FL"]
        jounce = (.015, .08, -.16)
        rack = (.004, -.02, .10)
        pose = solver.solve(jounce, rack)
        states = {corner.name: CornerSolver(corner).solve(ZERO, ZERO) for corner in self.geometry.corners}
        states["FL"] = pose
        damper = sample_dampers(model, VehicleResult(states))["FL"]
        step = .002

        def axis_at(time):
            travel = jounce[0] + jounce[1] * time + .5 * jounce[2] * time**2
            steer = rack[0] + rack[1] * time + .5 * rack[2] * time**2
            points = solver.solve((travel, 0., 0.), (steer, 0., 0.)).position
            return points[spec.attachment_index] - spec.fixed_point

        axes = np.array([axis_at(step * i) for i in (-2, -1, 0, 1, 2)])
        lengths = np.linalg.norm(axes, axis=1)
        m2, m1, center, p1, p2 = lengths
        speed = (m2 - 8 * m1 + 8 * p1 - p2) / (12 * step)
        acceleration = (-m2 + 16 * m1 - 30 * center + 16 * p1 - p2) / (12 * step**2)
        self.assertAlmostEqual(damper.length, center)
        self.assertGreater(abs(damper.deflection), .004)
        self.assertAlmostEqual(damper.deflection, center - spec.static_length)
        self.assertAlmostEqual(damper.velocity, speed, delta=1e-6)
        self.assertAlmostEqual(damper.acceleration, acceleration, delta=1e-4)
        u_m2, u_m1, u, u_p1, u_p2 = axes / lengths[:, None]
        u_rate = (u_m2 - 8 * u_m1 + 8 * u_p1 - u_p2) / (12 * step)
        u_acc = (-u_m2 + 16 * u_m1 - 30 * u + 16 * u_p1 - u_p2) / (12 * step**2)
        np.testing.assert_allclose(damper.orientation.angular_velocity, np.cross(u, u_rate), atol=3e-6, rtol=0)
        np.testing.assert_allclose(damper.orientation.angular_acceleration, np.cross(u, u_acc), atol=3e-4, rtol=0)
        self.assertAlmostEqual(damper.damping_force, -spec.damping * damper.velocity)
        self.assertAlmostEqual(damper.force, damper.spring_force + damper.damping_force)
        self.assertIsNone(damper.tire_force)
        lengths = np.linalg.norm(pose.position[LINK_START] - pose.position[LINK_END], axis=1)
        np.testing.assert_allclose(lengths, solver.geometry.lengths, atol=1e-8, rtol=0)

    def test_tire_contact_cannot_pull_wheel_toward_road(self):
        model = make_model(self.project, self.geometry)
        self.assertEqual(model.tire_force(model.free_radius + .02, 0., 0., 0.), 0.)
        self.assertGreater(model.tire_force(model.free_radius - .01, 0., 0., 0.), 0.)
        if model.tire_damping > 0:
            release_speed = .02 * model.tire_stiffness / model.tire_damping
            self.assertEqual(model.tire_force(model.free_radius - .01, release_speed, 0., 0.), 0.)

    def test_coincident_damper_mounts_fail_validation(self):
        self.project["dynamics"]["corners"]["RL"].update(attachment="WC", mount_mm=[0., 0., 0.])
        with self.assertRaises(ValueError):
            make_model(self.project, self.geometry)


class WheelDynamicsTests(unittest.TestCase):
    def test_wheels_start_at_equilibrium_without_a_road_input(self):
        project = default_project()
        settings = project["dynamics"]
        settings["corners"]["FR"].update(mass_kg=14., spring_n_m=22000., damper_ns_m=700.)
        settings["simulation"].update(target="all", bump_mm=0., duration_seconds=.5)
        geometry = build_vehicle(project)
        model = make_model(project, geometry)
        reference = {corner.name: np.asarray(corner.reference) for corner in geometry.corners}
        frames = simulate_dynamics(model, settings)
        self.assertAlmostEqual(frames[0].time, 0.)
        self.assertAlmostEqual(frames[-1].time, .5)
        self.assertGreater(len(frames), 2)
        for name in reference:
            with self.subTest(corner=name):
                for frame in frames:
                    position = frame.result.corners[name].position
                    np.testing.assert_allclose(position, reference[name], atol=1e-8, rtol=0)
                    np.testing.assert_allclose(frame.result.corners[name].velocity, 0, atol=1e-7, rtol=0)
                    np.testing.assert_allclose(frame.result.corners[name].acceleration, 0, atol=1e-6, rtol=0)
                    self.assertAlmostEqual(frame.dampers[name].tire_force, model.corners[name].static_load,
                                           delta=1e-4)

    def test_bump_excites_only_target_wheel_with_consistent_force_and_geometry(self):
        project = default_project()
        settings = project["dynamics"]
        settings["corners"]["FL"].update(
            mass_kg=13., spring_n_m=26000., damper_ns_m=800., attachment="UBJ",
            mount_mm=[-25., -10., 310.])
        settings["simulation"].update(target="FL", bump_mm=20., bump_seconds=.25, duration_seconds=1.0)
        geometry = build_vehicle(project)
        model = make_model(project, geometry)
        self.assertEqual(model.corners["FL"].mass, 13.)
        self.assertEqual(model.corners["FL"].stiffness, 26000.)
        self.assertEqual(model.corners["FL"].damping, 800.)
        self.assertEqual(model.corners["FL"].attachment_index, POINTS.index("UBJ"))
        self.assertNotEqual(model.corners["FL"].mass, model.corners["FR"].mass)
        frames = simulate_dynamics(model, settings)
        expected_times = [frame.time for frame in frames]
        self.assertAlmostEqual(expected_times[0], 0.)
        self.assertAlmostEqual(expected_times[-1], 1.)
        self.assertTrue(all(a < b for a, b in zip(expected_times, expected_times[1:])))
        self.assertGreater(max(frame.dampers["FL"].road_height for frame in frames), .017)
        self.assertEqual(frames[-1].dampers["FL"].road_height, 0.)

        for corner in geometry.corners:
            name = corner.name
            with self.subTest(corner=name):
                positions = np.asarray([frame.result.corners[name].position for frame in frames])
                lengths = np.linalg.norm(positions[:, LINK_START] - positions[:, LINK_END], axis=2)
                np.testing.assert_allclose(lengths, np.broadcast_to(corner.lengths, lengths.shape),
                                           atol=1e-8, rtol=0)
                tire_forces = np.asarray([frame.dampers[name].tire_force for frame in frames])
                self.assertTrue(np.isfinite(tire_forces).all())
                self.assertGreaterEqual(tire_forces.min(), 0.)
                if name == "FL":
                    travel = positions[:, WHEEL, 2] - corner.reference[WHEEL][2]
                    self.assertGreater(travel.max(), .003)
                    self.assertLess(abs(travel[-1]), travel.max() / 2)
                    self.assertGreater(np.ptp([frame.dampers[name].length for frame in frames]), .003)
                    self.check_force_balance(corner, model, frames)
                else:
                    np.testing.assert_allclose(positions, np.broadcast_to(corner.reference, positions.shape),
                                               atol=1e-8, rtol=0)

    def check_force_balance(self, corner, model, frames):
        spec = model.corners[corner.name]
        solver = CornerSolver(corner)
        step = .0001
        samples = [frame for frame in frames if .06 < frame.time < .4]
        self.assertTrue(samples)
        for frame in samples:
            pose = frame.result.corners[corner.name]
            damper = frame.dampers[corner.name]
            travel = pose.position[WHEEL, 2] - corner.reference[WHEEL][2]
            path_slope = (point_at(solver, travel + step) - point_at(solver, travel - step)) / (2 * step)
            axis = (damper.moving_point - damper.fixed_point) / damper.length
            damper_ratio = axis @ path_slope[spec.attachment_index]
            spring = spec.stiffness * (spec.free_length - damper.length)
            damping = -spec.damping * damper.velocity
            applied = damper.tire_force - spec.mass * model.gravity + (spring + damping) * damper_ratio
            inertial = spec.mass * (pose.acceleration[WHEEL] @ path_slope[WHEEL])
            self.assertAlmostEqual(damper.force, spring + damping)
            self.assertAlmostEqual(inertial, applied, delta=.003)


if __name__ == "__main__":
    unittest.main()
