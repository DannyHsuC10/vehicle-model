"""Physics regressions for CG-loaded vehicle heave."""

import unittest

import numpy as np

from MultibodyGeometryLab.deformable import build_dynamics_model
from MultibodyGeometryLab.geometry import POINTS, build_vehicle
from MultibodyGeometryLab.heave import simulate_heave
from MultibodyGeometryLab.project import default_project
from MultibodyGeometryLab.solver import CornerSolver


WHEEL = POINTS.index("WC")
ZERO = (0., 0., 0.)


class HeaveTests(unittest.TestCase):
    def setUp(self):
        project = default_project()
        self.vehicle = project["geometry"]["vehicle"]
        self.geometry = build_vehicle(project)
        dynamics = project["dynamics"]
        dynamics["corners"]["FL"].update(mass_kg=13., spring_n_m=26000.)
        dynamics["corners"]["RR"].update(mass_kg=11., damper_ns_m=1100.)
        self.model = build_dynamics_model(self.geometry, self.vehicle, dynamics)
        self.total_mass = self.vehicle["mass"]

    def run_heave(self, force_down_n, duration_seconds):
        settings = {"force_down_n": force_down_n, "ramp_seconds": .2,
                    "duration_seconds": duration_seconds}
        return simulate_heave(self.model, settings, self.total_mass)

    def test_gravity_and_preload_hold_static_equilibrium_without_cg_force(self):
        frames = self.run_heave(0., .5)
        self.assertGreater(len(frames), 2)
        self.assertAlmostEqual(frames[-1].time, .5)
        for frame in frames:
            sample = frame.heave
            np.testing.assert_allclose(
                [sample.displacement, sample.velocity, sample.acceleration, sample.force_z],
                0., atol=1e-7, rtol=0,
            )
            for name, spec in self.model.corners.items():
                self.assertAlmostEqual(frame.dampers[name].deflection, 0., delta=1e-9)
                self.assertAlmostEqual(frame.dampers[name].tire_force, spec.static_load, delta=1e-5)

    def test_downward_cg_force_compresses_all_springs_and_transfers_load_to_ground(self):
        force_down_n = 600.
        frames = self.run_heave(force_down_n, 2.5)
        first, final = frames[0], frames[-1]
        self.assertAlmostEqual(first.heave.force_z, 0.)
        self.assertAlmostEqual(final.heave.force_z, -force_down_n)
        self.assertLess(final.heave.displacement, -.003)
        self.assertLess(abs(final.heave.velocity), .002)
        for name in self.model.corners:
            with self.subTest(corner=name):
                self.assertLess(final.dampers[name].deflection, -.001)
                self.assertGreater(final.dampers[name].spring_force, first.dampers[name].spring_force)
        tire_load = sum(damper.tire_force for damper in final.dampers.values())
        self.assertAlmostEqual(tire_load, self.total_mass * self.model.gravity + force_down_n, delta=4.)

    def test_wheel_and_body_accelerations_follow_world_force_balance(self):
        frames = self.run_heave(600., .55)
        corners = {corner.name: CornerSolver(corner) for corner in self.geometry.corners}
        samples = [frame for frame in frames if .06 < frame.time < .35]
        self.assertGreater(len(samples), 2)
        step = .0001
        for frame in samples:
            sample = frame.heave
            np.testing.assert_allclose(frame.translation, [0., 0., sample.displacement], atol=1e-12, rtol=0)
            resultant_inertia = self.total_mass * sample.acceleration
            net_force = sample.force_z - self.total_mass * self.model.gravity
            for name, spec in self.model.corners.items():
                pose = frame.result.corners[name]
                world_wheel_height = pose.position[WHEEL, 2] + sample.displacement
                world_wheel_velocity = pose.velocity[WHEEL, 2] + sample.velocity
                compression = self.model.free_radius - world_wheel_height
                tire_force = (
                    max(0., self.model.tire_stiffness * compression - self.model.tire_damping * world_wheel_velocity)
                    if compression > 0. else 0.
                )
                damper = frame.dampers[name]
                self.assertAlmostEqual(damper.tire_force, tire_force, delta=1e-6)
                net_force += tire_force
                resultant_inertia += spec.mass * pose.acceleration[WHEEL, 2]

                solver = corners[name]
                travel = pose.position[WHEEL, 2] - solver.reference[WHEEL, 2]
                before = solver.solve((travel - step, 0., 0.), ZERO).position
                after = solver.solve((travel + step, 0., 0.), ZERO).position
                path_tangent = (after - before) / (2 * step)
                damper_axis = (damper.moving_point - damper.fixed_point) / damper.length
                damper_ratio = damper_axis @ path_tangent[spec.attachment_index]
                applied = tire_force - spec.mass * self.model.gravity + damper.force * damper_ratio
                world_acceleration = pose.acceleration[WHEEL] + [0., 0., sample.acceleration]
                wheel_inertia = spec.mass * (world_acceleration @ path_tangent[WHEEL])
                self.assertAlmostEqual(wheel_inertia, applied, delta=.01)
            self.assertAlmostEqual(resultant_inertia, net_force, delta=1e-6)

    def test_requires_positive_sprung_mass(self):
        settings = {"force_down_n": 600., "ramp_seconds": .2, "duration_seconds": .5}
        unsprung_mass = sum(spec.mass for spec in self.model.corners.values())
        with self.assertRaises(ValueError):
            simulate_heave(self.model, settings, unsprung_mass)

    def test_zero_suspension_and_tire_damping_sustains_heave_oscillation(self):
        project = default_project()
        dynamics = project["dynamics"]
        dynamics["tire_damping_ns_m"] = 0.
        for settings in dynamics["corners"].values():
            settings["damper_ns_m"] = 0.
        model = build_dynamics_model(self.geometry, self.vehicle, dynamics)
        frames = simulate_heave(model, {"force_down_n": 600., "ramp_seconds": .05,
                                        "duration_seconds": 3.}, self.total_mass)
        times = np.array([frame.time for frame in frames])
        heights = np.array([frame.heave.displacement for frame in frames])
        speeds = np.array([frame.heave.velocity for frame in frames])
        late = times > 1.4
        late_speeds = speeds[late]
        self.assertGreaterEqual(np.count_nonzero(late_speeds[:-1] * late_speeds[1:] < 0.), 4)
        early_span = np.ptp(heights[(times >= .3) & (times < 1.3)])
        late_span = np.ptp(heights[times >= 2.])
        self.assertGreater(early_span, .003)
        self.assertGreater(late_span, .9 * early_span)
        self.assertLess(heights[times >= 2.].min(), -.003)
        for name in model.corners:
            with self.subTest(corner=name):
                dampers = [frame.dampers[name] for frame in frames]
                self.assertGreater(min(damper.tire_force for damper in dampers), 0.)
                np.testing.assert_array_equal([damper.damping_force for damper in dampers], 0.)
                self.assertLess(min(damper.deflection for damper in dampers), -.002)


if __name__ == "__main__":
    unittest.main()
