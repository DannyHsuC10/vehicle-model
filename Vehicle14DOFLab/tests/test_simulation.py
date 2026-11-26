import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import numpy as np

from Vehicle14DOFLab._vendor.lapsim.motor import load_motor_curve
from Vehicle14DOFLab._vendor.pacejka.model import api
from Vehicle14DOFLab.aero_setup import VehicleAero, aero_settings
from Vehicle14DOFLab.geometry_tables import GeometryTable
from Vehicle14DOFLab.lap import Driver, LapIncomplete, Track, simulate_laps
from Vehicle14DOFLab.model import Controls, DOF, SPIN, STATE_SIZE, VERTICAL, Vehicle14DOF, flat_road
from Vehicle14DOFLab.straight import simulate_straight
from Vehicle14DOFLab.suspension_curves import FORMATS, read_curve_csv, save_curve_template


SKIDPAD = Path(__file__).resolve().parents[1] / "examples/skidpad.csv"


class MotorCsvTests(unittest.TestCase):
    def test_single_curve_and_voltage_envelope_use_their_own_interpolation_axes(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "motor.csv"
            for header in ("rpm,T_shaft_Nm", "speed,torque"):
                with self.subTest(header=header):
                    path.write_text(f"{header}\n2000,10\n0,20\n", encoding="utf-8")
                    curve = load_motor_curve(path, 530.)
                    np.testing.assert_allclose(curve.torque_at([0., 1000., 2001.]), [20., 15., 0.])
            path.write_text("vdc_V,rpm,T_shaft_Nm\n400,0,10\n400,2000,6\n600,0,20\n600,2000,10\n",
                            encoding="utf-8")
            envelope = load_motor_curve(path, 500.)
            np.testing.assert_allclose(envelope.torque_at([0., 1000., 2001.]), [15., 11.5, 0.])
            with self.assertRaises(ValueError):
                load_motor_curve(path, 700.)


class VehicleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = Vehicle14DOF()

    def test_rest_equilibrium_and_weight_balance(self):
        state = self.vehicle.initial_state()
        self.assertEqual(state.shape, (STATE_SIZE,))
        rates = self.vehicle.evaluate(state, Controls(), flat_road(0.))
        np.testing.assert_allclose(rates, 0., atol=1e-7)
        row = self.vehicle.evaluate(state, Controls(), flat_road(0.), telemetry=True)
        self.assertAlmostEqual(sum(row[f"{corner}_fz_n"] for corner in ("FL", "FR", "RL", "RR")),
                               self.vehicle.suspension.total_mass * 9.81, places=5)

    def test_tire_slip_generates_force_and_reacts_on_wheel_spin(self):
        state = self.vehicle.initial_state(speed=8.)
        coordinates = self.vehicle.evaluate(state, Controls(), flat_road(0.))
        state[DOF + SPIN.start] += 3.
        slipping = self.vehicle.evaluate(state, Controls(), flat_road(0.))
        self.assertGreater(slipping[DOF], coordinates[DOF])
        self.assertLess(slipping[DOF + SPIN.start], coordinates[DOF + SPIN.start])

    def test_unloaded_tire_has_no_contact_force(self):
        state = self.vehicle.initial_state(speed=8.)
        state[VERTICAL.start + 3] += .03
        state[DOF + SPIN.start] += 10.
        row = self.vehicle.evaluate(state, Controls(steer_rad=.1), flat_road(0.), telemetry=True)
        self.assertEqual([row[f"FL_{key}"] for key in ("fz_n", "fx_n", "fy_n", "mz_nm", "rolling_resistance_nm")],
                         [0.] * 5)

    def test_rolling_resistance_uses_sae_with_actual_per_wheel_inputs(self):
        vehicle = Vehicle14DOF()
        speed = 17.
        state = vehicle.initial_state(speed=speed)
        vehicle.tires.pressure = np.array([72000., 80000., 88000., 96000.])
        row = vehicle.evaluate(state, Controls(), flat_road(0.), telemetry=True)
        corners = vehicle.suspension.corners
        fz = np.array([row[f"{wheel}_fz_n"] for wheel in corners])
        condition = api.create_calculation_input_from_config(
            FZ=fz, SA=0., V=speed, pressure=vehicle.tires.pressure, tir_params=vehicle.tires.params,
        )
        expected = api.MF_Universal_solver("RRT_SAE", condition)
        np.testing.assert_allclose([row[f"{wheel}_rolling_resistance_nm"] for wheel in corners], expected)
        np.testing.assert_allclose(vehicle.tires.rolling_resistance(fz, -speed, -100.), -expected)

    def test_front_left_bump_increases_vertical_tire_force(self):
        state = self.vehicle.initial_state()
        flat = self.vehicle.evaluate(state, Controls(), flat_road(0.), telemetry=True)
        bump = (np.array([.01, 0., 0., 0.]), np.zeros(4))
        response = self.vehicle.evaluate(state, Controls(), bump, telemetry=True)
        self.assertAlmostEqual(response["FL_fz_n"] - flat["FL_fz_n"], 800.)
        self.assertEqual(response["FR_fz_n"], flat["FR_fz_n"])
        self.assertAlmostEqual(flat["FL_rolling_radius_m"] - response["FL_rolling_radius_m"], .01)
        self.assertEqual(response["FR_rolling_radius_m"], flat["FR_rolling_radius_m"])

    def test_flying_start_uses_each_loaded_radius_for_wheel_speed(self):
        vehicle = Vehicle14DOF()
        state = vehicle.initial_state(speed=17.)
        row = vehicle.evaluate(state, Controls(), flat_road(0.), telemetry=True)
        for corner in vehicle.suspension.corners:
            self.assertLess(row[f"{corner}_rolling_radius_m"], vehicle.radius)
            self.assertAlmostEqual(row[f"{corner}_rolling_radius_m"] * row[f"{corner}_wheel_speed_rad_s"], 17.)

    def test_hydraulic_brake_uses_each_axles_master_and_disc(self):
        vehicle = Vehicle14DOF()
        config = {"front_brake_bias": .60, "max_pedal_force_n": 500.,
                  "hydraulic_brake": {"front_master_diameter_m": .012, "rear_master_diameter_m": .016,
                                      "front_disc_diameter_m": .18, "rear_disc_diameter_m": .16}}
        expected = [500 * 2.8 * .60 * (.0254 / .012)**2 * 4 * .55 * .09,
                    500 * 2.8 * .40 * (.0254 / .016)**2 * 2 * .55 * .08]
        np.testing.assert_allclose(vehicle.maximum_brake_torque(config), np.repeat(expected, 2))
        legacy = config | {"brake_model": "leverage", "brake_leverage": [1., 2.]}
        np.testing.assert_allclose(vehicle.maximum_brake_torque(legacy), np.repeat(expected, 2))
        state = vehicle.initial_state(speed=15.)
        row = vehicle.evaluate(state, Controls(brake=.2), flat_road(0.), telemetry=True)
        np.testing.assert_allclose([row[f"{c}_brake_nm"] for c in vehicle.suspension.corners],
                                   -.2 * vehicle.brake_capacity)

    def test_bottoming_preserves_force_and_uses_total_rate(self):
        vehicle = Vehicle14DOF()
        vehicle.initial_state()
        _, component = next((key, component) for key, component in vehicle.suspension.active if component.jounce_rate)
        stop = component.jounce
        self.assertAlmostEqual(stop - component.static_stroke, component.jounce_travel)
        base_force = component.spring_force(stop)
        self.assertAlmostEqual(component.stop_force(stop), 0.)
        self.assertAlmostEqual(component.spring_force(stop + 1.) + component.stop_force(stop + 1.),
                               base_force + component.jounce_rate)

    def test_quadratic_aero_affects_static_trim_and_telemetry(self):
        vehicle = Vehicle14DOF()
        settings = aero_settings({"aero": {"enabled": True}}, SKIDPAD.parent)
        vehicle.aero = VehicleAero(settings, vehicle.wheel_x[0], -vehicle.wheel_x[2],
                                   vehicle.track_widths.mean(), vehicle.height)
        rows = []
        for speed in (10., 20.):
            state = vehicle.initial_state(speed=speed)
            row = vehicle.evaluate(state, Controls(), flat_road(0.), telemetry=True)
            normal_load = sum(row[f"{corner}_fz_n"] for corner in vehicle.suspension.corners)
            self.assertAlmostEqual(normal_load, vehicle.suspension.total_mass * 9.81 + row["downforce_n"])
            rows.append(row)
        self.assertAlmostEqual(rows[1]["downforce_n"], 4. * rows[0]["downforce_n"])
        self.assertAlmostEqual(rows[1]["drag_n"], 4. * rows[0]["drag_n"])

    def test_attitude_aero_matches_lapsim_coefficients_and_applies_cop_moments(self):
        vehicle = Vehicle14DOF()
        speed = 18.
        state = vehicle.initial_state(speed=speed)
        trim = vehicle.evaluate(state, Controls(), flat_road(0.), telemetry=True)
        self.assertAlmostEqual(sum(trim[f"{corner}_fz_n"] for corner in vehicle.suspension.corners),
                               vehicle.suspension.total_mass * 9.81 + trim["downforce_n"])
        heave, roll, pitch = -.02, .04, -.03
        state[VERTICAL.start:VERTICAL.start + 3] = vehicle.suspension.static_reference[:3] + [heave, roll, pitch]
        row = vehicle.evaluate(state, Controls(), flat_road(0.), telemetry=True)
        expected = vehicle.aero.modal.calculate_from_modal(speed, heave, pitch, roll)
        for key, source in (("downforce_n", "F_down"), ("drag_n", "F_drag"),
                            ("CL", "CL"), ("CD", "CD"), ("cop_y_m", "COP_y")):
            self.assertAlmostEqual(row[key], expected[source])
            self.assertNotAlmostEqual(row[key], trim[key])
        self.assertAlmostEqual(row["cop_x_m"], expected["COP_x"] + vehicle.aero.axle_midpoint)
        self.assertNotAlmostEqual(row["cop_x_m"], trim["cop_x_m"])
        x, y, z = (row[f"cop_{axis}_m"] for axis in "xyz")
        z -= vehicle.height + heave
        drag, downforce = row["drag_n"], row["downforce_n"]
        self.assertAlmostEqual(row["aero_roll_nm"], -y * downforce)
        self.assertAlmostEqual(row["aero_pitch_nm"], -x * downforce + z * drag)
        self.assertAlmostEqual(row["aero_yaw_nm"], y * drag)
        aero_on = vehicle.evaluate(state, Controls(), flat_road(0.))
        vehicle.aero.enabled = False
        aero_off = vehicle.evaluate(state, Controls(), flat_road(0.))
        np.testing.assert_allclose(
            (aero_on - aero_off)[DOF + VERTICAL.start:DOF + VERTICAL.start + 3] * vehicle.suspension.mass[:3],
            [-downforce, row["aero_roll_nm"], row["aero_pitch_nm"]],
        )
        self.assertAlmostEqual((aero_on - aero_off)[DOF + 2] * vehicle.yaw_inertia, row["aero_yaw_nm"])

    def test_straight_times_the_75_m_crossing_at_full_throttle(self):
        rows = simulate_straight(self.vehicle, 75., time_limit=15.)
        self.assertEqual(rows[0]["speed_mps"], 0.)
        self.assertEqual(rows[-1]["x_m"], 75.)
        self.assertEqual(rows[-1]["throttle"], 1.)
        self.assertGreater(rows[-1]["speed_mps"], 0.)
        self.assertLess(rows[-2]["distance_m"], 75.)

    def test_enabled_geometry_routes_tire_forces_without_adding_vertical_load(self):
        vehicle = Vehicle14DOF()
        state = vehicle.initial_state(speed=14.)
        state[DOF + SPIN.start:] *= .95
        command = Controls(steer_rad=.12)
        road = flat_road(0.)
        baseline = vehicle.evaluate(state, command, road)
        vehicle.geometry_load_transfer = True
        active = vehicle.evaluate(state, command, road)
        row = vehicle.evaluate(state, command, road, telemetry=True)
        self.assertGreater(row["FL_roll_center_force_n"], 0.)
        self.assertAlmostEqual(row["FL_roll_center_force_n"], -row["FR_roll_center_force_n"])
        self.assertLess(row["FL_anti_force_n"], 0.)
        self.assertGreater(row["RL_anti_force_n"], 0.)
        height_indices = DOF + np.array([VERTICAL.start, *range(VERTICAL.start + 3, VERTICAL.stop)])
        self.assertAlmostEqual((active - baseline)[height_indices] @ vehicle.suspension.mass[[0, 3, 4, 5, 6]], 0.)

    def test_camber_gain_changes_magic_formula_force_at_jounce(self):
        vehicle = Vehicle14DOF()
        state = vehicle.initial_state(speed=10.)
        road = flat_road(0.)
        vehicle.camber_gain[0] = np.radians(-1.) / 25.
        vehicle.use_camber_gain = True
        before = vehicle.evaluate(state, Controls(), road, telemetry=True)
        compressed = state.copy()
        compressed[VERTICAL.start + 3] += .005
        with_gain = vehicle.evaluate(compressed, Controls(), road, telemetry=True)
        vehicle.use_camber_gain = False
        without_gain = vehicle.evaluate(compressed, Controls(), road, telemetry=True)
        self.assertAlmostEqual(with_gain["FL_camber_deg"], before["FL_camber_deg"] - .2)
        self.assertNotAlmostEqual(with_gain["FL_fy_n"], without_gain["FL_fy_n"])

    def test_roll_center_surface_uses_axle_travel_and_clamps_beyond_grid(self):
        vehicle = Vehicle14DOF()
        spec = {
            "axes": ["front_travel_mm", "rear_travel_mm"],
            "rows": [[front, rear, 20. + 2 * front + rear, 30. + front + 3 * rear]
                     for front in (0., 20.) for rear in (0., 20.)],
        }
        table = GeometryTable("roll_center", spec)
        self.assertAlmostEqual(table.sample(0, 25., -5.), 60.)
        with self.assertRaisesRegex(ValueError, "矩形網格"):
            GeometryTable("roll_center", {**spec, "rows": spec["rows"][:-1]})
        vehicle.geometry_tables["roll_center"] = table
        roll, _ = vehicle.geometry_reactions(np.zeros(4), np.array([300., 300., 400., 400.]),
                                             np.array([10., 0., 0., 20.]))
        self.assertAlmostEqual(roll[0], .040 * 600. / vehicle.track_widths[0])
        self.assertAlmostEqual(roll[2], .065 * 800. / vehicle.track_widths[1])

    def test_camber_surface_replaces_gain_and_uses_the_road_wheel_steer(self):
        vehicle = Vehicle14DOF()
        state = vehicle.initial_state(speed=12.)
        vehicle.camber_gain.fill(.01)
        vehicle.use_camber_gain = True
        vehicle.geometry_tables["camber"] = GeometryTable("camber", {
            "axes": ["travel_mm", "steer_deg"],
            "rows": [[travel, steer, *[-.02 * travel + .03 * steer] * 4]
                     for travel in (-30., 30.) for steer in (-30., 30.)],
        })
        report = vehicle.evaluate(state, Controls(steer_rad=.10), flat_road(0.), telemetry=True)
        travel = 1000. * (vehicle.suspension.travel @ state[VERTICAL]) - vehicle.suspension.tender_reference_wheel_mm
        self.assertAlmostEqual(report["FL_camber_deg"], -.02 * travel[0] + .03 * report["FL_steer_deg"])

    def test_bump_steer_surface_updates_actual_tire_steer_and_side_force(self):
        vehicle = Vehicle14DOF()
        state = vehicle.initial_state(speed=12.)
        state[VERTICAL.start + 3] += .005
        command = Controls(steer_rad=.10)
        baseline = vehicle.evaluate(state, command, flat_road(0.), telemetry=True)
        vehicle.geometry_tables["bump_steer"] = GeometryTable("bump_steer", {
            "axes": ["travel_mm", "steer_deg"],
            "rows": [[travel, steer, .10 * travel + .02 * steer, 0., 0., 0.]
                     for travel in (-30., 30.) for steer in (-30., 30.)],
        })
        report = vehicle.evaluate(state, command, flat_road(0.), telemetry=True)
        travel = 1000. * (vehicle.suspension.travel @ state[VERTICAL]) - vehicle.suspension.tender_reference_wheel_mm
        expected = .10 * travel[0] + .02 * report["FL_nominal_steer_deg"]
        self.assertAlmostEqual(report["FL_bump_steer_deg"], expected)
        self.assertAlmostEqual(report["FL_steer_deg"], baseline["FL_steer_deg"] + expected)
        self.assertNotAlmostEqual(report["FL_fy_n"], baseline["FL_fy_n"])

    def test_roll_center_and_anti_steer_surfaces_add_deltas_to_base(self):
        vehicle = Vehicle14DOF()
        fx, fy = np.ones(4) * 400., np.ones(4) * 300.
        travel = np.zeros(4)
        base_roll, base_anti = vehicle.geometry_reactions(fx, fy, travel)
        for kind, value, count in (("roll_center", .5, 2), ("anti", .2, 4)):
            vehicle.geometry_tables[kind] = GeometryTable(kind, {
                "relative": True, "axes": ["travel_mm", "steer_deg"],
                "rows": [[t, s, *[s * value] * count] for t in (-20., 20.) for s in (-20., 20.)],
            })
        active_roll, active_anti = vehicle.geometry_reactions(fx, fy, travel, steer_deg=10.)
        self.assertAlmostEqual(active_roll[0] - base_roll[0], .005 * fy[:2].sum() / vehicle.track_widths[0])
        self.assertAlmostEqual(active_anti[0] - base_anti[0], .02 * vehicle.height * fx.sum() / (2 * vehicle.wheelbase))


class LapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = Track.from_csv(SKIDPAD)

    def test_skidpad_wraps_and_times_a_dynamic_lap(self):
        vehicle = Vehicle14DOF()
        driver = Driver(self.track, vehicle.wheelbase)
        self.assertAlmostEqual(max(driver.speed_profile)**2 / 9.125, 1.5 * 9.81)
        result = simulate_laps(vehicle, self.track, driver, time_limit=20.)
        self.assertEqual(len(result.lap_times), 1)
        self.assertGreater(result.lap_times[0], 5.)
        self.assertLess(result.lap_times[0], 7.)
        self.assertLess(result.max_centerline_error_m, .5)
        finish = result.telemetry[-1]
        self.assertEqual(finish["distance_m"], self.track.length)
        self.assertAlmostEqual(finish["time_s"], result.lap_times[0])
        self.assertEqual(result.timed_boundaries_s, (0., finish["time_s"]))
        self.assertGreater(abs(finish["yaw_rate_deg_s"]), 20.)

    def test_default_speed_profile_caps_target_at_100_kmh(self):
        # A zero-curvature speed-planning input isolates the configured speed cap.
        track = SimpleNamespace(curvature=np.zeros(3), ds=np.ones(3))
        driver = Driver(track, 1.53)
        self.assertAlmostEqual(3.6 * max(driver.speed_profile), 100.)

    def test_timeout_never_returns_a_lap_time(self):
        vehicle = Vehicle14DOF()
        driver = Driver(self.track, vehicle.wheelbase)
        with self.assertRaisesRegex(LapIncomplete, "No finish"):
            simulate_laps(vehicle, self.track, driver, time_limit=.05)


class CurveCsvTests(unittest.TestCase):
    def test_each_template_selects_the_correct_solver_table(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "curve.csv"
            for family, mode in FORMATS:
                save_curve_template(path, family, mode)
                actual, rows = read_curve_csv(path, family)
                self.assertEqual(actual, mode)
                self.assertTrue(rows)


if __name__ == "__main__":
    unittest.main()
