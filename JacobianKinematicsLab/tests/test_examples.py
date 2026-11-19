import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from JacobianKinematicsLab.model import CORNERS, EXAMPLES, JOINTS, MOUNTS, load_model, make_motion, read_hardpoints
from JacobianKinematicsLab.results import export_results, motion_ratio
from JacobianKinematicsLab.solver import solve
from JacobianKinematicsLab.suspension import _ackermann_percent, sweep_front_axle, sweep_suspension
from JacobianKinematicsLab.wheel_frame import TireSetup


def write_points(path, points):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("name", "x_mm", "y_mm", "z_mm"))
        for name, position in points.items():
            writer.writerow((name, *(coordinate * 1000 for coordinate in position)))


class ExampleIntegrationTests(unittest.TestCase):
    def test_edited_points_rebuild_static_geometry_without_exporting_generated_markers(self):
        path = EXAMPLES / "hardpoints.csv"
        edited = read_hardpoints(path)
        edited["FL_WC"] = tuple(np.array(edited["FL_WC"]) + [0., 0., .002])
        model = load_model(path, points=edited)
        result = solve(model, make_motion(model, samples=5))
        index = model.point_names.index("FL_WC")
        np.testing.assert_array_equal(result.position[0, index], edited["FL_WC"])
        self.assertIn("FL_FORWARD", result.internal_points)
        self.assertIn("FL_WHEEL_HEADING", result.link_names)
        with TemporaryDirectory() as directory:
            point_path, _ = export_results(directory, result)
            with point_path.open(encoding="utf-8-sig", newline="") as handle:
                self.assertEqual({row["point"] for row in csv.DictReader(handle)}, set(edited))

    def test_bundled_staticwireframe_defaults_and_mount_overrides(self):
        model = load_model(EXAMPLES / "hardpoints.csv")
        self.assertEqual(model.name, "StaticWireframe 基準車輛")
        self.assertEqual(model.samples, 21)
        editable = {drive.name: drive.amplitude_mm for drive in model.drives if not drive.locked}
        self.assertEqual(editable, {**{f"{corner}_WC_Z": 20 for corner in CORNERS}, "FRONT_RACK": 0})
        for name, x in (("FL_UCA_F", .235), ("FL_UCA_R", -.235), ("RL_UCA_F", -1.17),
                        ("RL_UCA_R", -1.565), ("RL_LCA_R", -1.565)):
            self.assertAlmostEqual(model.reference[model.point_names.index(name), 0], x)

    def test_four_wheel_heave_steering_and_pushrods_solve_together(self):
        model = load_model(EXAMPLES / "hardpoints.csv")
        travels = {"FL": 25, "FR": -12, "RL": 15, "RR": -7}
        amplitudes = {f"{corner}_WC_Z": travel for corner, travel in travels.items()}
        motion = make_motion(model, samples=41, amplitudes_mm=dict(amplitudes, FRONT_RACK=8))
        result = solve(model, motion)
        points = {name: i for i, name in enumerate(model.point_names)}
        links = {link.name: i for i, link in enumerate(model.links)}
        np.testing.assert_allclose(result.position[0], model.reference, atol=1e-12)
        for corner, travel in travels.items():
            wheel = points[f"{corner}_WC"]
            self.assertAlmostEqual(result.position[20, wheel, 2] - model.reference[wheel, 2], travel / 1000)
            rocker = points[f"{corner}_PUSH_IN"]
            self.assertGreater(np.linalg.norm(result.position[20, rocker] - model.reference[rocker]), .001)
            self.assertIn(f"{corner}_PUSH_ROD", links)
            self.assertIn(f"{corner}_RACK-TOE", links)
        for corner in ("FL", "FR"):
            rack = points[f"{corner}_RACK"]
            np.testing.assert_allclose(result.position[20, rack] - model.reference[rack], [0., .008, 0.], atol=1e-10)
            self.assertGreater(abs(result.azimuth_deg[20, links[f"{corner}_WHEEL_HEADING"]]), .5)
        for corner in ("RL", "RR"):
            rack = points[f"{corner}_RACK"]
            np.testing.assert_array_equal(result.velocity[:, rack], 0.)
            np.testing.assert_array_equal(result.acceleration[:, rack], 0.)

        a = [points[link.a] for link in model.links]
        b = [points[link.b] for link in model.links]
        vectors = result.position[:, b] - result.position[:, a]
        lengths = np.linalg.norm(vectors, axis=-1)
        self.assertLess(np.abs(lengths - lengths[0]).max(), 1e-10)
        velocity = result.velocity[:, b] - result.velocity[:, a]
        acceleration = result.acceleration[:, b] - result.acceleration[:, a]
        np.testing.assert_allclose(np.sum(vectors * velocity, axis=-1), 0., atol=1e-10)
        np.testing.assert_allclose(np.sum(vectors * acceleration + velocity**2, axis=-1), 0., atol=1e-10)
        ratio = motion_ratio(result, "FL_WC", "FL_PUSH_IN", drive="FL_WC_Z")
        self.assertTrue(np.isfinite(ratio.motion_ratio).all())
        np.testing.assert_allclose(abs(ratio.motion_ratio) * ratio.mechanical_advantage, 1.)

    def test_tilted_rack_uses_measured_axis_and_independent_profile(self):
        points = read_hardpoints(EXAMPLES / "hardpoints.csv")
        points["FL_RACK"] = tuple(np.array(points["FL_RACK"]) + [.018, 0., .010])
        with TemporaryDirectory() as directory:
            path = Path(directory) / "tilted_mm.csv"
            write_points(path, points)
            model = load_model(path)
        motion = make_motion(model, samples=41, amplitudes_mm={"FL_WC_Z": 10, "FRONT_RACK": 8},
                             profiles={"FL_WC_Z": "ramp", "FRONT_RACK": "sine"})
        result = solve(model, motion)
        left, right = (model.point_names.index(f"{corner}_RACK") for corner in ("FL", "FR"))
        axis = model.reference[left] - model.reference[right]
        axis /= np.linalg.norm(axis)
        index = result.drive_names.index("FRONT_RACK")
        for node in (left, right):
            np.testing.assert_allclose(result.position[:, node] - model.reference[node],
                                       motion.displacement[:, index, None] * axis, atol=1e-10)
            np.testing.assert_allclose(result.velocity[:, node], motion.velocity[:, index, None] * axis, atol=1e-10)
            np.testing.assert_allclose(result.acceleration[:, node], motion.acceleration[:, index, None] * axis, atol=1e-10)
        self.assertAlmostEqual(motion.displacement[10, index], .008)
        self.assertAlmostEqual(motion.displacement[30, index], -.008)
        wheel = result.drive_names.index("FL_WC_Z")
        self.assertAlmostEqual(motion.displacement[10, wheel], .0025)

    def test_older_suspension_export_remains_solvable(self):
        sample = read_hardpoints(EXAMPLES / "hardpoints.csv")
        points = {f"{corner}_{name}": sample[f"{corner}_{name}"]
                  for corner in CORNERS for name in (*MOUNTS, *JOINTS, "WC")}
        with TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.csv"
            write_points(path, points)
            model = load_model(path)
        result = solve(model, make_motion(model, samples=21))
        self.assertNotIn("FRONT_RACK", result.drive_names)
        for corner in CORNERS:
            center, forward = (model.point_names.index(f"{corner}_{node}") for node in ("WC", "FORWARD"))
            np.testing.assert_allclose(result.position[:, center, 1], result.position[:, forward, 1], atol=1e-10)

    def test_sweep_derivatives_hold_rack_fixed_at_each_steering_setting(self):
        model = load_model(EXAMPLES / "hardpoints.csv")
        sweep = sweep_suspension(model, "FL", (-.01, .01), (-5., 5.))
        self.assertEqual(sweep.rack_mm, (-5., 0., 5.))
        zero = int(np.flatnonzero(sweep.travel_mm == 0.)[0])
        np.testing.assert_allclose(sweep.bump_steer_deg[:, zero], 0.)
        self.assertGreater(np.ptp(sweep.toe_deg[:, zero]), 1.)
        self.assertGreater(np.ptp(sweep.camber_gain[:, zero]), 1e-4)
        for values, rates in ((sweep.camber_deg, sweep.camber_gain),
                              (sweep.bump_steer_deg, sweep.bump_steer_rate)):
            span = sweep.travel_mm[zero + 1] - sweep.travel_mm[zero - 1]
            finite_difference = (values[:, zero + 1] - values[:, zero - 1]) / span
            np.testing.assert_allclose(rates[:, zero], finite_difference, atol=1e-6)

    def test_rear_sweep_uses_one_input_and_reports_wheel_center_spacing(self):
        model = load_model(EXAMPLES / "hardpoints.csv")
        sweep = sweep_suspension(model, "RL", (0., 20.), (-5., 5.))
        self.assertEqual(sweep.rack_mm, (0.,))
        np.testing.assert_allclose(sweep.track_mm[0, 0], 1240.)
        np.testing.assert_allclose(sweep.wheelbase_mm[0, 0], 1530.)
        motion = make_motion(model, samples=3, amplitudes_mm={"FL_WC_Z": 0, "FR_WC_Z": 0, "RR_WC_Z": 0})
        result = solve(model, motion)
        rear_left, rear_right, front_left = (result.position[1, model.point_names.index(name)]
                                            for name in ("RL_WC", "RR_WC", "FL_WC"))
        np.testing.assert_allclose(sweep.track_mm[0, -1], 1000. * abs(rear_left[1] - rear_right[1]))
        np.testing.assert_allclose(sweep.wheelbase_mm[0, -1], 1000. * abs(front_left[0] - rear_left[0]))

    def test_front_axle_sweep_uses_the_same_alignment_and_zero_rack_pose(self):
        model = load_model(EXAMPLES / "hardpoints.csv")
        tires = {"FL": TireSetup(toe_deg=.8, camber_deg=-2.), "FR": TireSetup(toe_deg=1.2, camber_deg=-1.)}
        static = sweep_front_axle(model, 0., 0., (-4., 4.), tires=tires)
        zero = int(np.flatnonzero(static.rack_mm == 0.)[0])
        np.testing.assert_allclose(static.yaw_deg[:, zero], [-.8, 1.2])
        np.testing.assert_allclose(static.steer_deg[:, zero], 0.)
        self.assertTrue(np.isnan(static.ackermann_pct[zero]))
        single = sweep_suspension(model, "FL", (-5., 5.), tires=tires)
        static_wheel = int(np.flatnonzero(single.travel_mm == 0.)[0])
        self.assertAlmostEqual(single.toe_deg[0, static_wheel], .8)
        self.assertAlmostEqual(single.camber_deg[0, static_wheel], -2.)

        tilted = sweep_front_axle(model, 5., 1., (-4., 4.), tires=tires)
        self.assertAlmostEqual(np.mean(tilted.travel_mm), 5.)
        self.assertAlmostEqual(np.diff(tilted.travel_mm)[0], -1250. * np.tan(np.deg2rad(1.)))
        self.assertFalse(np.allclose(tilted.yaw_deg[:, zero], static.yaw_deg[:, zero]))
        np.testing.assert_allclose(tilted.steer_deg, tilted.yaw_deg - tilted.yaw_deg[:, zero, None])
        np.testing.assert_allclose(tilted.difference_deg, tilted.steer_deg[0] - tilted.steer_deg[1])

    def test_ackermann_percent_handles_both_turn_directions_and_parallel_steering(self):
        track, wheelbase, radius = 1200., 1500., 3300.
        angles = np.rad2deg(np.arctan2(wheelbase, radius + np.array([-.5, .5]) * track))
        steering = np.column_stack((angles, -angles[::-1], [10., 10.], [0., 0.], [-5., 5.]))
        percentages = _ackermann_percent(steering, track, wheelbase)
        np.testing.assert_allclose(percentages[:3], [100., 100., 0.], atol=1e-10)
        self.assertTrue(np.isnan(percentages[3:]).all())

    def test_bundled_custom_mechanism_loads_and_drives_the_rocker(self):
        model = load_model(EXAMPLES / "pushrod_rocker.csv", EXAMPLES / "pushrod_rocker.json")
        result = solve(model, make_motion(model))
        slider, tip = (model.point_names.index(name) for name in ("PUSHROD_SLIDER", "ROCKER_TIP"))
        middle = len(result.time) // 2
        np.testing.assert_allclose(result.displacement[middle, slider], [0., 0., .020], atol=1e-10)
        np.testing.assert_allclose(result.displacement[-1], 0., atol=1e-10)
        self.assertGreater(result.displacement[middle, tip, 2], .01)
        np.testing.assert_allclose(np.linalg.norm(result.position[:, slider] - result.position[:, tip], axis=1), .2)


if __name__ == "__main__":
    unittest.main()
