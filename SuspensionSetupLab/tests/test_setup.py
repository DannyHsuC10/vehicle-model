from copy import deepcopy
from math import sqrt
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from setup_lab.components import KCSystem
from setup_lab.engine import Cancelled, G, Vehicle, prepare_vehicle, simulate
from setup_lab.load_transfer import TransferDistribution, contact_patch
from setup_lab.project import (CORNERS, compile_project, default_project, import_linear, linear, open_project,
                               read_recording, save_project, transfer_options, with_static_tender_travel, with_total_limit_rates)
from setup_lab.road import Road


class LoadTransferTests(unittest.TestCase):
    def test_geometry_lookup_changes_wheel_forces_using_each_axle_travel(self):
        project = default_project()
        project["model"] = "rigid"
        project["vehicle"].update(h_rc_f=.02, h_rc_r=.04, dive_front=.1, lift_rear=.2)
        fixed = Vehicle(project, compile_project(project))
        project["geometry_tables"] = {
            "roll_center": [[0., 20., 40.], [10., 60., 80.]],
            "anti": [[0., 10., 20., 30., 40.], [10., 50., 60., 70., 80.]],
        }
        lookup = Vehicle(project, compile_project(project))
        zero = np.zeros(4)
        trim = lookup.equilibrium((zero, zero, 0., 0.))
        fixed.equilibrium((zero, zero, 0., 0.))
        q = trim.copy()
        q[lookup.nb:] += [.005, .005, .015, .015]
        v = np.zeros(lookup.n)
        inputs = zero, zero, -4., 3.
        change = (lookup.forces(q, v, inputs) - fixed.forces(q, v, inputs)) * lookup.mass
        p = project["vehicle"]
        rc = p["m"] * 3. * np.array([.5 * .02 / p["tf"], -.5 * .02 / p["tf"],
                                      .5 * .04 / p["tr"], -.5 * .04 / p["tr"]])
        anti = -p["m"] * 4. * p["h_cg"] / (2 * lookup.wheelbase) * np.array([.2, .2, -.4, -.4])
        np.testing.assert_allclose(change, lookup.travel.T @ (rc + anti), atol=1e-9)
        np.testing.assert_allclose(TransferDistribution(project, transfer_options({})).rc, [.02, .04])

        project["vehicle"].update(h_rc_f=.01, h_rc_r=.015)
        project["geometry_modes"] = {"roll_center": "fixed"}
        mixed = Vehicle(project, compile_project(project))
        self.assertEqual(mixed.geometry.active_kinds, {"anti"})
        self.assertAlmostEqual(mixed.geometry.value("h_rc_f", (10., 10.)), .01)
        self.assertAlmostEqual(mixed.geometry.value("dive_front", (10., 10.)), .5)
        np.testing.assert_allclose(TransferDistribution(project, transfer_options({})).rc, [.01, .015])

    def test_contact_patch_has_zero_area_for_a_lifted_tire(self):
        for load in (-50., 0.):
            self.assertEqual(contact_patch(load, 80., 180.), (0., 0., 0.))
        area, length, width = contact_patch(800., 80., 180.)
        self.assertAlmostEqual(area, 100.)
        self.assertAlmostEqual(np.pi * length * width / 400, area)

    def test_distribution_balances_three_paths_and_uses_configured_roll_channels(self):
        project = default_project()
        project["channels"]["roll_f"] = linear(100000, 0, 1.)
        project["channels"]["roll_r"] = linear(60000, 0, .8)
        model = TransferDistribution(project, transfer_options({}))
        tracks = np.array([project["vehicle"]["tf"], project["vehicle"]["tr"]])
        np.testing.assert_allclose(model.roll_stiffness, [100000 * (tracks[0] / 2)**2, 60000 * (.8 * tracks[1] / 2)**2])
        left, right = model.at(1.), model.at(-1.)
        self.assertAlmostEqual(left["paths"].sum(), project["vehicle"]["m"] * G * project["vehicle"]["h_cg"])
        self.assertAlmostEqual(left["moment_error"], 0.)
        self.assertAlmostEqual(sum(left["loads"]), project["vehicle"]["m"] * G)
        np.testing.assert_allclose(left["loads"], right["loads"][[1, 0, 3, 2]])
        self.assertAlmostEqual(left["front_share"], left["lateral"][0] / sum(left["lateral"]))

    def test_pacejka_balances_axle_demand_and_uses_each_axle_pressure(self):
        project = default_project()
        model = TransferDistribution(project, transfer_options({"pressure_f": 95., "pressure_r": 65.}))
        left, right = model.at(1.), model.at(-1.)
        np.testing.assert_allclose(np.sum(np.reshape(left["forces"], (2, 2)), axis=1),
                                   model.mass * G * model.fractions)
        np.testing.assert_allclose(left["forces"], -np.asarray(right["forces"])[[1, 0, 3, 2]])
        np.testing.assert_allclose(left["slips"], -np.asarray(right["slips"]))
        self.assertGreater(min(left["slips"]), 0.)
        self.assertAlmostEqual(left["steer"], -right["steer"])
        self.assertEqual(model.at(0.)["grip"], [1., 1.])

        from Pacejka_tire_model.model import api

        params = api.load_tir("Hoosier_43075_R20_W8_candidate.tir")
        sides = np.array([1., -1., 1., -1.])
        conditions = api.CalculationInput(FZ=left["loads"], SA=-sides * np.repeat(left["slips"], 2),
                                          pressure=[95000., 95000., 65000., 65000.], tir_params=params)
        expected = sides * api.MF_Universal_solver("FY0", conditions)
        np.testing.assert_allclose(left["forces"], expected, atol=.5, rtol=0.)

        lower_pressure = TransferDistribution(project, transfer_options({"pressure_f": 65., "pressure_r": 65.})).at(1.)
        np.testing.assert_allclose(left["loads"], lower_pressure["loads"])
        self.assertNotAlmostEqual(left["slips"][0], lower_pressure["slips"][0])
        self.assertAlmostEqual(left["slips"][1], lower_pressure["slips"][1])

    def test_pacejka_reports_no_equilibrium_above_peak_and_zero_force_at_lift(self):
        project = default_project()
        project["vehicle"]["h_cg"] = .12
        saturated = TransferDistribution(project, transfer_options({})).at(3.)
        self.assertTrue(np.all(saturated["loads"] > 0.))
        self.assertEqual(saturated["slips"], [None, None])
        self.assertEqual(saturated["forces"], [None] * 4)
        self.assertIsNone(saturated["steer"])

        lifted = TransferDistribution(default_project(), transfer_options({})).at(3.)
        self.assertLess(lifted["loads"][0], 0.)
        self.assertEqual(lifted["forces"][0], 0.)


class ComponentTests(unittest.TestCase):
    def test_ratio_damping_uses_axle_mass_tender_slope_and_motion_ratio(self):
        project = default_project()
        settings = project["channels"]["heave_f"]
        settings.update(damper_ratio=True, mr=[[-10, .7], [10, .9]],
                        compression=[[0, .4], [.1, .2]], rebound=[[0, .8], [.2, .5]])
        component = compile_project(project)[0]
        axle_mass = (321 - 4 * 15) / 2
        series_rate = (35 * 25 / (35 + 25)) * 1000
        critical = 2 * sqrt(series_rate * axle_mass / 4) / .8
        self.assertAlmostEqual(component.damper_force(.15), critical * (.4 * .1 + .2 * .05))
        self.assertAlmostEqual(component.damper_force(-.3), -critical * (.8 * .2 + .5 * .1))

        settings["installed_mm"] = 10.
        with_preload = compile_project(project)[0]
        self.assertAlmostEqual(with_preload.damper_force(.05), critical * .4 * .05)
        self.assertEqual(settings["compression"], [[0, .4], [.1, .2]])

    def test_multi_stage_spring_and_asymmetric_damper_are_continuous(self):
        settings = linear(10000, 1000)
        settings.update(spring_kind="stages", spring=[[-10, 5], [0, 10], [10, 20], [20, 40]],
                        compression=[[0, 1000], [.05, 500], [.15, 900]],
                        rebound=[[0, 2000], [.05, 1000]], jounce_mm=20, jounce_rate=50,
                        rebound_mm=15, rebound_rate=30, limit_rate_mode="additive")
        component = KCSystem(settings)
        for travel, expected in ((-12, -60), (10, 100), (15, 200), (20, 300), (25, 500)):
            self.assertAlmostEqual(component.spring_force(travel), expected)
        for speed, expected in ((0, 0), (.05, 50), (.1, 75), (.2, 145), (-.1, -150)):
            self.assertAlmostEqual(component.damper_force(speed), expected)
        self.assertEqual(component.stop_force(22), 100)
        self.assertEqual(component.stop_force(-17), -60)

    def test_total_limit_rate_overrides_later_spring_breakpoints(self):
        settings = linear(35000, 0)
        settings.update(spring_kind="stages", spring=[[0, 35], [12, 120]], jounce_mm=10, jounce_rate=80)
        component = KCSystem(settings)
        for stroke, expected in ((9, 315), (10, 350), (11, 430), (13, 590), (20, 1150)):
            self.assertAlmostEqual(component.force(stroke / 1000, 0.), expected)

    def test_legacy_limit_conversion_uses_the_slope_outside_each_limit(self):
        project = default_project()
        settings = project["channels"]["heave_f"]
        settings.pop("limit_rate_mode")
        settings.update(spring_kind="stages", spring=[[-20, 15], [-10, 25], [0, 30], [10, 45]],
                        jounce_mm=10, jounce_rate=80, rebound_mm=10, rebound_rate=40)
        converted = with_total_limit_rates(project)["channels"]["heave_f"]
        self.assertEqual((converted["jounce_rate"], converted["rebound_rate"]), (125., 55.))
        before, after = KCSystem(settings), KCSystem(converted)
        for travel in (-.04, -.01, 0., .01, .04):
            self.assertAlmostEqual(before.force(travel, 0.), after.force(travel, 0.))

    def test_tender_binding_uses_additional_travel_after_static_compression(self):
        settings = linear(60000, 1000)
        settings.update(spring_kind="tender", main_rate=60, tender_rate=30, tender_after_static_mm=6, installed_mm=2)
        component = KCSystem(settings)
        self.assertAlmostEqual(component.spring_force(-3), -20)
        self.assertAlmostEqual(component.spring_force(-2), 0)
        self.assertAlmostEqual(component.spring_force(-1), 20)
        self.assertAlmostEqual(component.spring_force(0), 40)
        self.assertAlmostEqual(component.spring_force(4), 120)
        component.set_static_reference(4.)
        self.assertAlmostEqual(component.spring_force(4), 120)
        self.assertAlmostEqual(component.spring_force(10), 240)
        self.assertAlmostEqual(component.spring_force(11), 300)
        stiffer_main = KCSystem(dict(settings, main_rate=120))
        stiffer_main.set_static_reference(4.)
        self.assertEqual(stiffer_main.spring_breakpoints()[-1], 10.)
        self.assertEqual(stiffer_main.spring_slope(10., from_right=True), 120.)

        legacy = dict(settings, tender_travel=4)
        legacy.pop("tender_after_static_mm")
        converted = with_static_tender_travel({"channels": {"heave_f": legacy}})["channels"]["heave_f"]
        self.assertEqual(converted, settings)
        legacy_component = KCSystem(legacy)
        legacy_component.set_static_reference(4.)
        self.assertEqual(legacy_component.spring_breakpoints(), component.spring_breakpoints())

    def test_variable_mr_preserves_virtual_work_with_preload(self):
        settings = linear(35000, 1000)
        settings.update(installed_mm=2, mr=[[-10, .8], [10, 1.2]])
        component = KCSystem(settings)
        at, shaft, speed, *_ = component.force(.005, 2., True)
        self.assertAlmostEqual(shaft, 5.25)
        self.assertAlmostEqual(speed, 2.2)
        self.assertAlmostEqual(at, 1.1 * (35 * 7.25 + 2200))
        delta = .000001
        tangent = (component.force(delta, 0.) - component.force(-delta, 0.)) / (2 * delta)
        self.assertAlmostEqual(tangent, 35000 + 70 * 20, places=4)

    def test_dyno_and_spring_force_tables(self):
        settings = linear(35000, 1000)
        settings.update(spring_kind="force", spring=[[-10, -100], [0, 0], [10, 200]],
                        damper_kind="force", compression=[[0, 0], [.1, 200], [.2, 250]], rebound=[[0, 0], [.1, 400]])
        component = KCSystem(settings)
        self.assertAlmostEqual(component.spring_force(15), 300)
        self.assertAlmostEqual(component.damper_force(.15), 225)
        self.assertAlmostEqual(component.damper_force(-.3), -400)
        settings["rebound"] = [[0, 0], [.1, -10]]
        with self.assertRaises(ValueError):
            KCSystem(settings)


class ModelTests(unittest.TestCase):
    def test_full_car_projects_axle_heave_and_roll_at_half_ratio(self):
        project = default_project()
        for axle, heave_rate, roll_rate in (("f", 100000, 40000), ("r", 120000, 60000)):
            project["channels"][f"heave_{axle}"] = linear(heave_rate, 0, .8)
            project["channels"][f"roll_{axle}"] = linear(roll_rate, 0, 1.)
        for model in ("rigid", "flex"):
            project["model"] = model
            car = Vehicle(project, compile_project(project))
            v = np.zeros(car.n)
            inputs = (np.zeros(4), np.zeros(4), 0., 0.)
            for wheel, expected_stroke, expected_force in (
                (0, [.4, 0., .5, 0.], [26., 6., 0., 0.]),
                (3, [0., .4, 0., -.5], [0., 0., 4.2, 34.2]),
            ):
                with self.subTest(model=model, wheel=wheel):
                    q = np.zeros(car.n)
                    q[car.nb + wheel] = .001
                    _, _, readings, forces, _ = car.forces(q, v, inputs, True)
                    np.testing.assert_allclose(readings[:4, 1], expected_stroke)
                    np.testing.assert_allclose(forces, expected_force)

    def test_compression_stop_uses_the_no_aero_static_reference(self):
        project = default_project()
        project["model"] = "quarter"
        project["case"]["speed"] = 10.
        project["aero"].update(enabled=True, reference_speed_kmh=36., reference_force_n=1600.)
        settings = project["channels"]["heave_f"]
        for kind in ("single", "tender"):
            with self.subTest(kind=kind):
                settings.update(spring_kind=kind, jounce_mm=2., jounce_rate=120., tender_after_static_mm=2.)
                car = Vehicle(project, compile_project(project))
                zero = np.zeros(1)
                inputs = (zero, zero, 0., 0.)
                no_aero = car.equilibrium(inputs, include_aero=False)
                component = car.active[0][1]
                threshold = component.jounce
                self.assertGreater(component.static_stroke, 2.)
                self.assertAlmostEqual(threshold - component.static_stroke, 2.)
                if kind == "tender":
                    self.assertEqual(threshold, component.tender_bind_stroke)
                self.assertEqual(component.stop_force(threshold), 0.)
                elastic = component.spring_force(threshold + 1.) + component.stop_force(threshold + 1.)
                self.assertAlmostEqual(elastic - component.spring_force(threshold), 120.)
                trim = car.equilibrium(inputs)
                acceleration, _, readings, *_ = car.forces(trim, np.zeros(car.n), inputs, True)
                self.assertEqual(component.jounce, threshold)
                self.assertGreater(readings[0, 5], 0.)
                self.assertGreater(float((car.travel @ (trim - no_aero))[0]), 0.)
                np.testing.assert_allclose(acceleration, 0., atol=1e-7)

    def test_downforce_can_bind_tender_without_shifting_its_threshold(self):
        project = default_project()
        project["model"] = "quarter"
        project["case"].update(duration=.3, start=.05, height=0., speed=10.)
        project["aero"].update(reference_speed_kmh=36., reference_force_n=1200.)
        off = simulate(project)
        project["aero"]["enabled"] = True
        on = simulate(project)

        off_switch = off.tender_switches["heave_f"]
        on_switch = on.tender_switches["heave_f"]
        self.assertAlmostEqual(on_switch.wheel_threshold_mm, off_switch.wheel_threshold_mm)
        self.assertAlmostEqual(on_switch.wheel_reference_mm, off.metrics["FL"][3])
        self.assertFalse(off_switch.starts_bound)
        self.assertTrue(on_switch.starts_bound)
        self.assertGreater(on.metrics["FL"][3], on_switch.wheel_threshold_mm)
        self.assertAlmostEqual(on.metrics["FL"][0] - off.metrics["FL"][0], 300., delta=1e-6)
        np.testing.assert_allclose(on.signals["升沉加速度 [m/s²]"], 0., atol=1e-7)

    def test_downforce_scales_with_case_speed_and_loads_the_static_trim(self):
        project = default_project()
        project["case"].update(duration=.3, start=.05, dt=.005, speed=10.)
        project["aero"].update(reference_speed_kmh=36., reference_force_n=150.)
        off = simulate(project)
        project["aero"]["enabled"] = True
        for speed, expected in ((0., 0.), (10., 150.), (20., 600.)):
            with self.subTest(speed=speed):
                project["case"]["speed"] = speed
                result = simulate(project)
                np.testing.assert_allclose(result.signals["全車下壓力 [N]"], expected)
                self.assertAlmostEqual(sum(result.metrics[c][0] for c in CORNERS), 321 * G + expected, delta=1e-6)
                self.assertEqual(result.signature == off.signature, speed == 10.)
        project.pop("aero")
        project["case"]["speed"] = 10.
        legacy = simulate(project)
        np.testing.assert_array_equal(legacy.signals["FL [N]"], off.signals["FL [N]"])

    def test_reduced_models_use_the_selected_corners_share_of_downforce(self):
        project = default_project()
        project["vehicle"].update(lf=.6, lr=.9)
        project["case"]["speed"] = 10.
        project["aero"].update(enabled=True, reference_speed_kmh=36., reference_force_n=400.)
        project["model_sources"].update(half_lr="rear", half_fr="right", quarter="FL")
        for model, expected in (("half_lr", 160.), ("half_fr", 200.), ("quarter", 120.)):
            with self.subTest(model=model):
                project["model"] = model
                project["case"]["target"] = "all"
                vehicle = Vehicle(project, compile_project(project))
                zero_wheels = np.zeros(vehicle.nw)
                inputs = (zero_wheels, zero_wheels, 0., 0.)
                trim = vehicle.equilibrium(inputs)
                _, loads, *_ = vehicle.forces(trim, np.zeros(vehicle.n), inputs, True)
                self.assertAlmostEqual(sum(loads), vehicle.total_mass * G + expected, delta=1e-6)

    def test_trim_balances_weight_and_braking_in_both_models(self):
        project = default_project()
        for model, dof in (("rigid", 7), ("flex", 8)):
            with self.subTest(model=model):
                project["model"] = model
                vehicle = Vehicle(project, compile_project(project))
                self.assertEqual(vehicle.n, dof)
                inputs = (np.zeros(4), np.zeros(4), -4., 0.)
                trim = vehicle.equilibrium(inputs)
                acceleration, load, *_ = vehicle.forces(trim, np.zeros(dof), inputs, True)
                np.testing.assert_allclose(acceleration, 0., atol=1e-7)
                self.assertAlmostEqual(sum(load), G * project["vehicle"]["m"], delta=1e-6)
                expected = G * 321 / 2 + 321 * 4 * .3 / 1.53
                self.assertAlmostEqual(sum(load[:2]), expected, delta=1e-6)

    def test_lift_recontact_and_tolerance_convergence(self):
        project = default_project()
        project["case"]["height"] = 20.
        a = simulate(project)
        b = simulate(project, tolerance=1e-7)
        self.assertEqual(a.time[-1], project["case"]["duration"])
        for corner in CORNERS:
            load = a.signals[f"{corner} [N]"]
            suspension = a.signals[f"{corner} 輪端懸吊力 [N]"]
            np.testing.assert_allclose(load, b.signals[f"{corner} [N]"], atol=.2)
            self.assertAlmostEqual(load.min(), a.metrics[corner][1])
            self.assertGreater(np.max(np.abs(load - suspension)), 1.)
            self.assertAlmostEqual(a.signals[f"{corner} 輪端行程 [mm]"][0], a.metrics[corner][3])
            self.assertGreaterEqual(a.metrics[corner][1], 0.)
        self.assertGreater(a.metrics["FL"][-1], 0.)
        self.assertGreater(a.signals["FL [N]"][-1], 0.)

    def test_component_change_reaches_model_under_identical_input(self):
        project = default_project()
        project["channels"]["heave_f"]["spring_kind"] = "single"
        project["case"].update(duration=.8, start=.1, dt=.004)
        a = simulate(project)
        heave = project["channels"]["heave_f"]
        heave.update(spring_kind="tender", main_rate=55, tender_rate=25, tender_after_static_mm=5., damper_kind="stages",
                     compression=[[0, 1600], [.03, 900], [.12, 500]], rebound=[[0, 3600], [.05, 1800]],
                     jounce_mm=6, jounce_rate=70.)
        b = simulate(project)
        self.assertEqual(a.signature, b.signature)
        self.assertGreater(np.max(np.abs(a.signals["FL [N]"] - b.signals["FL [N]"])), 40)
        self.assertGreater(np.max(b.signals["前軸升沉 限位塊力 [N]"]), 0)
        self.assertEqual(a.project["channels"]["heave_f"]["spring_kind"], "single")

    def test_unstable_configuration_rejected_and_cancellation(self):
        project = default_project()
        for component in project["channels"].values():
            component["enabled"] = False
        with self.assertRaisesRegex(ValueError, "靜態平衡"):
            simulate(project)
        with self.assertRaises(Cancelled):
            simulate(default_project(), cancel=lambda: True)


class ReducedModelTests(unittest.TestCase):
    def test_half_lr_frame_filters_geometry_and_keeps_equilibrium(self):
        p = default_project()
        p["model"] = "half_lr"
        p["case"].update(scenario="cornering", target="all", ay=5., start=.05, ramp=.02, duration=1., dt=.002)
        self.assertIsNotNone(prepare_vehicle(p)[0].roll_frame)
        p["roll_center_frame"] = {"enabled": False}
        instant = simulate(p)
        p["roll_center_frame"] = {"enabled": True, "stiffness": 50000., "damping": 1500., "mass": 65.}
        filtered = simulate(p)
        setting = p["roll_center_frame"]
        for key in ("stiffness", "damping", "mass"):
            variant = deepcopy(p)
            variant["roll_center_frame"][key] *= 1.5
            changed = simulate(variant)
            self.assertGreater(np.max(np.abs(changed.signals["Roll Center 傳遞力 [N]"]
                                             - filtered.signals["Roll Center 傳遞力 [N]"])), .1)

        command = filtered.signals["Roll Center 指令 [N]"]
        reaction = filtered.signals["Roll Center 傳遞力 [N]"]
        np.testing.assert_allclose(command, instant.signals["幾何載重轉移量 [N]"])
        np.testing.assert_allclose(reaction, filtered.signals["幾何載重轉移量 [N]"])
        self.assertGreater(np.max(np.abs(command - reaction)), 1.)
        self.assertAlmostEqual(command[-1], reaction[-1], places=3)
        self.assertEqual(filtered.motion.shape, instant.motion.shape)
        car, road = prepare_vehicle(p)
        height, velocity, _, ay = road.at(1.)
        inputs = height, velocity, 0., ay
        q = car.equilibrium(inputs)
        np.testing.assert_allclose(car.forces(q, np.zeros(car.n), inputs), 0., atol=1e-7)
        self.assertAlmostEqual(q[-1] * setting["stiffness"], command[-1])

    def test_left_right_load_transfer_uses_signed_half_differences_and_actual_sample_times(self):
        project = default_project()
        project["model"] = "half_lr"
        project["geometry_tables"] = {"roll_center": [[-20., 10., 30.], [20., 50., 70.]]}
        project["tires"]["RL"]["mu"] += 1.
        project["case"].update(scenario="cornering", target="all", ay=-5., duration=.701, dt=.004, start=.08, ramp=.12)
        project["model_sources"]["half_lr"] = "rear"
        result = simulate(project)
        car, _ = prepare_vehicle(project)
        ay = result.signals["側向加速度 [m/s²]"]
        height = result.signals["後 Roll Center [mm]"] / 1000
        np.testing.assert_allclose(result.signals["Roll Center 指令 [N]"],
                                   car.total_mass * ay * height / project["vehicle"]["tr"])
        geometric = result.signals["Roll Center 傳遞力 [N]"]
        suspension = (result.signals["RR 輪端懸吊力 [N]"] - result.signals["RL 輪端懸吊力 [N]"]) / 2
        tire = (result.signals["RR [N]"] - result.signals["RL [N]"]) / 2
        self.assertNotEqual(tire[0], 0.)
        self.assertNotAlmostEqual(result.time[-1] - result.time[-2], project["case"]["dt"])
        for label, transfer in (("幾何", geometric), ("懸吊", suspension), ("路徑合計", geometric + suspension), ("輪胎端", tire)):
            with self.subTest(label=label):
                np.testing.assert_allclose(result.signals[f"{label}載重轉移量 [N]"], transfer - transfer[0], atol=1e-9)
                rate = result.signals[f"{label}載重轉移速率 [N/s]"]
                np.testing.assert_allclose(rate, np.gradient(transfer - transfer[0], result.time), atol=1e-8)
                i = np.argmax(np.abs(rate))
                self.assertEqual(result.metrics["load_transfer_rate"][label], (rate[i], result.time[i]))
        tire_rate = result.signals["輪胎端載重轉移速率 [N/s]"]
        self.assertLess(tire_rate.min(), -10.)
        self.assertGreater(tire_rate.max(), 10.)
        self.assertGreater(np.max(np.abs(result.signals["路徑合計載重轉移速率 [N/s]"] - tire_rate)), 10.)

    def test_quarter_projects_heave_and_roll_travel_and_force_by_the_same_ratio(self):
        p = default_project()
        p["model"] = "quarter"
        p["case"]["target"] = "all"
        for axle, heave_mr, roll_mr in (("f", .7, 1.4), ("r", .85, 1.2)):
            p["channels"][f"heave_{axle}"] = dict(linear(60000, 1000, heave_mr), spring_kind="tender", tender_rate=30.)
            p["channels"][f"roll_{axle}"] = linear(40000, 2000, roll_mr)
        # Old cases may still carry the removed field.
        p["case"]["quarter_mr"] = .5
        for corner, heave_mr, roll_mr in (("FL", .7, 1.4), ("FR", .7, 1.4), ("RL", .85, 1.2), ("RR", .85, 1.2)):
            with self.subTest(corner=corner):
                p["model_sources"]["quarter"] = corner
                car = Vehicle(p, compile_project(p))
                _, _, readings, suspension, _ = car.forces(
                    np.array([0., .025]), np.array([0., .2]), (np.zeros(1), np.zeros(1), 0., 0.), True)
                indices = [i for i, _ in car.active]
                self.assertEqual(len(indices), 2)
                np.testing.assert_allclose(readings[indices, 1], [12.5 * heave_mr, 12.5 * roll_mr])
                expected_heave = (heave_mr / 2)**2 * (25. * 20. + .2 * 1000.)
                expected_roll = (roll_mr / 2)**2 * (25. * 40. + .2 * 2000.)
                self.assertAlmostEqual(suspension[0], expected_heave + expected_roll)

    def test_quarter_tender_crossings_follow_preload_and_heave_mr(self):
        p = default_project()
        p["model"] = "quarter"
        p["case"].update(duration=.6, start=.1)
        baseline = simulate(p).tender_switches["heave_f"]
        self.assertFalse(baseline.starts_bound)
        self.assertEqual(baseline.binding_times.size + baseline.release_times.size, 0)

        p["channels"]["heave_f"].update(tender_after_static_mm=2., installed_mm=1.2, mr=[[0, .8], [20, 1.2]])
        result = simulate(p)
        switches = result.tender_switches["heave_f"]
        self.assertFalse(switches.starts_bound)
        self.assertAlmostEqual(switches.wheel_threshold_mm, result.metrics["FL"][3] + 2. / (.5 * 1.2))
        self.assertGreater(switches.binding_times.size, 0)
        self.assertGreater(switches.release_times.size, 0)
        release, binding = switches.release_times[0], switches.binding_times[0]
        self.assertLess(binding, release)
        shaft = result.signals["前軸升沉 軸心行程 [mm]"]
        np.testing.assert_allclose(np.interp([release, binding], result.time, shaft), shaft[0] + 2.)
        self.assertAlmostEqual(result.signals["FL 輪端懸吊力 [N]"][0], (321 - 4 * 15) / 4 * G, places=5)

    def test_quarter_limit_lines_use_the_channel_mr_and_active_stop_positions(self):
        project = default_project()
        project["model"] = "quarter"
        project["case"].update(duration=.3, start=.05)
        project["channels"]["heave_f"].update(
            mr=[[-20, .8], [20, 1.2]], tender_after_static_mm=2., jounce_mm=2., jounce_rate=100.,
            rebound_mm=9., rebound_rate=80.)
        project["channels"]["roll_f"].update(mr=[[0, .75]], jounce_mm=3., jounce_rate=95.)

        result = simulate(project)
        heave, roll = (result.wheel_travel_limits[key] for key in ("heave_f", "roll_f"))
        self.assertAlmostEqual(heave["compression"], result.tender_switches["heave_f"].wheel_threshold_mm)
        # A -20 mm Heave channel displacement gives -18 mm at the shaft. The
        # negative threshold is within the table, where MR varies with travel.
        car = Vehicle(project, compile_project(project))
        self.assertAlmostEqual(car.active[0][1].motion_ratio.integrated(heave["rebound"] / 2), -9.)
        self.assertAlmostEqual(roll["compression"], result.metrics["FL"][3] + 3. / (.75 / 2))
        self.assertNotIn("rebound", roll)

    def test_trim_and_load_transfer_use_selected_mass_and_axis(self):
        for model, source, corners, ax, ay in (("quarter", "RR", ("RR",), 0, 0),
                                               ("half_lr", "rear", ("RL", "RR"), 0, 4),
                                               ("half_fr", "right", ("FR", "RR"), -4, 0)):
            with self.subTest(model=model):
                p = default_project()
                p["model"] = model
                p["model_sources"][model] = source
                p["case"]["target"] = "all"
                p["vehicle"].update(lf=.6, lr=.9)
                p["tires"]["RR"]["mu"] = 20.
                car = Vehicle(p, compile_project(p))
                self.assertEqual(car.corners, corners)
                self.assertEqual(car.n, {"quarter": 2, "half_lr": 5, "half_fr": 4}[model])
                sprung = (321 - 65) * {"quarter": .2, "half_lr": .4, "half_fr": .5}[model]
                total = sprung + sum(p["tires"][c]["mu"] for c in corners)
                zero = np.zeros(len(corners))
                inputs = zero, zero, ax, ay
                q = car.equilibrium(inputs)
                acceleration, loads, *_ = car.forces(q, np.zeros(car.n), inputs, True)
                np.testing.assert_allclose(acceleration, 0., atol=1e-8)
                self.assertAlmostEqual(sum(loads), total * G, delta=1e-6)
                if model == "half_lr":
                    self.assertAlmostEqual(loads[0], (sprung / 2 + 15) * G - total * ay * .3 / 1.24)
                if model == "half_fr":
                    self.assertAlmostEqual(loads[0], (sprung * .6 + 15) * G - total * ax * .3 / 1.5)

    def test_left_right_half_car_projects_heave_and_roll_with_half_travel(self):
        p = default_project()
        p["model"] = "half_lr"
        p["case"]["target"] = "all"
        p["channels"]["heave_f"] = linear(100000, 0, .8)
        p["channels"]["roll_f"] = linear(40000, 0, 1.)
        car = Vehicle(p, compile_project(p))
        q, v = np.zeros(car.n), np.zeros(car.n)
        zero = np.zeros(car.nw)
        inputs = (zero, zero, 0., 0.)
        q[car.wheels] = [.001, 0.]
        _, _, channels, wheel_force, _ = car.forces(q, v, inputs, True)
        np.testing.assert_allclose(channels[[0, 2], 1], [.4, .5])
        np.testing.assert_allclose(wheel_force, [26., 6.])
        q[car.wheels] = .001
        _, _, channels, wheel_force, _ = car.forces(q, v, inputs, True)
        np.testing.assert_allclose(channels[[0, 2], 1], [.8, 0.])
        np.testing.assert_allclose(wheel_force, [32., 32.])
        q[car.wheels] = [.001, -.001]
        _, _, channels, wheel_force, _ = car.forces(q, v, inputs, True)
        np.testing.assert_allclose(channels[[0, 2], 1], [0., 1.])
        np.testing.assert_allclose(wheel_force, [20., -20.])

    def test_front_rear_half_car_uses_axle_heave_travel_and_half_its_reaction(self):
        p = default_project()
        p["model"] = "half_fr"
        p["case"]["target"] = "all"
        p["channels"]["heave_f"] = linear(40000, 0, 1.)
        p["channels"]["heave_r"] = linear(60000, 0, .8)
        car = Vehicle(p, compile_project(p))
        zero = np.zeros(car.nw)
        for axle, mr, wheel_rate in ((0, 1., 20.), (1, .8, 19.2)):
            with self.subTest(axle=axle):
                q = np.zeros(car.n)
                q[car.nb + axle] = .001
                _, _, readings, force, _ = car.forces(q, np.zeros(car.n), (zero, zero, 0., 0.), True)
                self.assertAlmostEqual(readings[axle, 1], mr)
                self.assertAlmostEqual(force[axle], wheel_rate)
                self.assertEqual(force[1 - axle], 0.)

    def test_front_rear_half_car_matches_symmetric_full_car_pitch_under_ax(self):
        p = default_project()
        p["case"].update(scenario="maneuver", target="all", ax=-7., ay=0., speed=10., start=.05, ramp=.25, duration=1.)
        p["aero"].update(enabled=True, reference_speed_kmh=36., reference_force_n=400.)
        p["channels"]["heave_f"].update(damper_ratio=True, compression=[[0, .7], [.25, .4]],
                                        rebound=[[0, .6], [.25, .3]], tender_after_static_mm=3.)
        full = simulate(p)
        p["model"] = "half_fr"
        p["model_sources"]["half_fr"] = "right"
        half = simulate(p)
        pitch = "俯仰 [deg]"
        self.assertGreater(np.max(np.abs(full.signals[pitch])), .1)
        for name in (pitch, "前軸升沉 軸心行程 [mm]", "後軸升沉 軸心行程 [mm]", "FR [N]", "RR [N]"):
            np.testing.assert_allclose(half.signals[name], full.signals[name], atol=1e-7)

    def test_each_local_model_responds_to_staged_spring_and_asymmetric_damper(self):
        for model, channel, response in (("quarter", "heave_f", "升沉 [mm]"),
                                          ("half_lr", "roll_f", "側傾 [deg]"),
                                          ("half_fr", "heave_f", "俯仰 [deg]")):
            with self.subTest(model=model):
                p = default_project()
                p["model"] = model
                p["case"].update(duration=.55, start=.1, dt=.005)
                baseline = simulate(p)
                p["channels"][channel].update(spring_kind="stages", spring=[[-3, 18], [0, 25], [3, 80]],
                                              compression=[[0, 900], [.04, 400]], rebound=[[0, 2100], [.06, 1100]])
                adjusted = simulate(p)
                self.assertEqual(baseline.signature, adjusted.signature)
                self.assertGreater(np.max(np.abs(adjusted.signals[response] - baseline.signals[response])), .01)
                self.assertGreater(np.max(np.abs(adjusted.signals["FL [N]"] - baseline.signals["FL [N]"])), 10)

    def test_replay_filters_wheel_and_axis_and_marks_comparison_source(self):
        p = default_project()
        p["model"] = "half_fr"
        p["model_sources"]["half_fr"] = "right"
        p["case"].update(scenario="csv", dt=.01, recording=[[0, 0, 0, 0, 0, 0, 0],
                                                         [.1, 1, 2, 3, 4, -2, 4], [.2, 0, 0, 0, 0, 0, 0]])
        result = simulate(p)
        self.assertEqual(result.corners, ("FR", "RR"))
        self.assertEqual(result.signals["路面 FR [mm]"][10], 2.)
        self.assertEqual(result.signals["路面 RR [mm]"][10], 4.)
        self.assertEqual(result.signals["縱向加速度 [m/s²]"][10], -2.)
        self.assertNotIn("側向加速度 [m/s²]", result.signals)
        self.assertNotIn("FL [N]", result.signals)
        p["model_sources"]["half_fr"] = "left"
        self.assertNotEqual(result.signature, simulate(p).signature)


class IsoRoadTests(unittest.TestCase):
    def test_four_wheel_profiles_repeat_and_match_submodels(self):
        case = default_project()["case"]
        case.update(scenario="iso_b", target="all", seed=32)
        wheelbase = 1.53
        times = np.linspace(.6, 1.8, 41)
        road = Road(case, wheelbase)
        traces = np.array([road.at(t)[0] for t in times])
        repeat = Road(dict(case), wheelbase)
        np.testing.assert_array_equal(traces, [repeat.at(t)[0] for t in times])
        for i in range(len(CORNERS)):
            for j in range(i):
                self.assertGreater(np.max(np.abs(traces[:, i] - traces[:, j])), 1e-4)
        for corners, indices in ((("RL", "RR"), [2, 3]), (("FR", "RR"), [1, 3]), (("RR",), [3])):
            subset = Road(case, wheelbase, corners=corners)
            np.testing.assert_array_equal([subset.at(t)[0] for t in times], traces[:, indices])

    def test_delayed_rear_wheels_follow_distinct_left_and_right_tracks(self):
        case = default_project()["case"]
        case.update(scenario="iso_c", target="all", seed=41, speed=10., start=.1, rear_delay=True)
        wheelbase = 1.5
        delay = wheelbase / case["speed"]
        road = Road(case, wheelbase)
        t = .18
        z, velocity, *_ = road.at(t)
        self.assertNotEqual(z[0], z[1])
        np.testing.assert_array_equal(z[2:], 0.)
        rear_z, rear_velocity, *_ = road.at(t + delay)
        np.testing.assert_allclose(rear_z[2:], z[:2], atol=1e-12)
        np.testing.assert_allclose(rear_velocity[2:], velocity[:2], atol=1e-12)
        rear_only = Road(dict(case, target="rear"), wheelbase)
        same_onset = Road(dict(case, target="rear", rear_delay=False, start=case["start"] + delay), wheelbase)
        self.assertEqual(rear_only.onsets, same_onset.onsets)
        self.assertNotEqual(rear_only.signature(), same_onset.signature())

    def test_class_seed_speed_and_rear_delay_drive_the_same_spatial_profile(self):
        case = default_project()["case"]
        case.update(scenario="iso_a", speed=10., seed=32, start=.1, target="left", rear_delay=True)
        wheelbase = 1.5
        baseline = Road(case, wheelbase)
        t = .18
        z, velocity, *_ = baseline.at(t)
        self.assertNotEqual(z[0], 0.)
        np.testing.assert_array_equal(z[1:], 0.)
        np.testing.assert_allclose(baseline.at(t + wheelbase / case["speed"])[0][2], z[0], atol=1e-12)
        repeat = Road(dict(case), wheelbase)
        np.testing.assert_array_equal(repeat.at(t)[0], z)
        fast = Road(dict(case, speed=20.), wheelbase)
        np.testing.assert_allclose(fast.at(.14)[0], z, atol=1e-12)
        np.testing.assert_allclose(fast.at(.14)[1], 2 * velocity, atol=1e-12)
        for scenario, factor in (("iso_b", 2), ("iso_c", 4)):
            road = Road(dict(case, scenario=scenario), wheelbase)
            np.testing.assert_allclose(road.at(t)[0], factor * z, atol=1e-12)
        other_seed = Road(dict(case, seed=33), wheelbase)
        self.assertNotEqual(other_seed.at(t)[0][0], z[0])
        self.assertNotEqual(other_seed.signature(), baseline.signature())
        self.assertNotEqual(fast.signature(), baseline.signature())

    def test_lateral_command_can_run_alone_or_with_iso(self):
        p = default_project()
        p["model"] = "half_lr"
        p["case"].update(scenario="cornering", target="all", start=.05, ramp=.10, duration=.5, ay=3.)
        maneuver = simulate(p)
        np.testing.assert_array_equal(maneuver.signals["路面 FL [mm]"], 0.)
        self.assertGreater(maneuver.signals["側傾 [deg]"][-1], 0.)
        self.assertLess(maneuver.signals["FL [N]"][-1], maneuver.signals["FR [N]"][-1])
        p["case"].update(scenario="iso_b", iso_lateral=True)
        combined = simulate(p)
        np.testing.assert_array_equal(combined.signals["側向加速度 [m/s²]"], maneuver.signals["側向加速度 [m/s²]"])
        self.assertGreater(np.max(np.abs(combined.signals["路面 FL [mm]"])), 0.)
        p["case"]["iso_lateral"] = False
        iso = simulate(p)
        np.testing.assert_array_equal(combined.signals["路面 FL [mm]"], iso.signals["路面 FL [mm]"])
        self.assertNotEqual(combined.signature, iso.signature)
        self.assertGreater(np.max(np.abs(combined.signals["側傾 [deg]"] - iso.signals["側傾 [deg]"])), .01)

    def test_rejects_aliasing_and_unsupported_lateral_axis(self):
        p = default_project()
        p["model"] = "quarter"
        p["case"].update(scenario="iso_c", speed=30., dt=.01)
        with self.assertRaisesRegex(ValueError, "輸出間隔"):
            compile_project(p)
        p["case"].update(dt=.002, seed=2.5)
        with self.assertRaisesRegex(ValueError, "種子須為整數"):
            compile_project(p)
        p["case"].update(scenario="cornering", seed=0)
        with self.assertRaisesRegex(ValueError, "全車及 1/2 左右"):
            compile_project(p)


class ProjectTests(unittest.TestCase):
    def test_replay_interpolates_data_and_roundtrips(self):
        path = Path(__file__).resolve().parents[1] / "examples" / "road-replay.csv"
        project = default_project()
        project["case"].update(scenario="csv", recording=read_recording(path), recording_name=path.name)
        road = Road(project["case"], 1.53)
        height, speed, ax, ay = road.at(.45)
        self.assertAlmostEqual(height[0], .01)
        self.assertAlmostEqual(speed[0], 0.)
        self.assertEqual((ax, ay), (-3., 2.))
        self.assertGreater(road.at(.425)[1][0], 0.)
        base = path.parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            destination = Path(folder) / "setup.json"
            save_project(destination, project)
            loaded, native = open_project(destination)
            self.assertEqual(loaded, project)
            self.assertEqual(native, destination)
        result = simulate(project)
        self.assertAlmostEqual(result.signals["路面 FL [mm]"][225], 10)

    def test_legacy_import_converts_channel_and_tire_units(self):
        full = dict(m=321, lf=.765, lr=.765, heave_f_k=35000, heave_r_k=31500, roll_f_k=20000,
                    roll_r_k=18000, tire_f_k=80000, tire_f_mr=.8, tire_r_k=80000, tender_enabled=True,
                    corner_f_k=70000, corner_f_c=3500, corner_f_mr=.9)
        project = import_linear({"parameters": {"full": full}})
        self.assertEqual(project["channels"]["FL"], project["channels"]["FR"])
        self.assertEqual(project["channels"]["FL"]["spring"], [[0, 70]])
        self.assertAlmostEqual(project["tires"]["FL"]["k"], 51.2)
        self.assertTrue(any("Tender" in note for note in project["notes"]))
        compile_project(project)

    def test_matched_signature_follows_effective_rear_delay(self):
        project = default_project()
        case = project["case"]
        case.update(target="left", rear_delay=True)
        a = Road(case, 1.5)
        b = Road(deepcopy(case), 1.7)
        self.assertNotEqual(a.signature(), b.signature())
        self.assertEqual(a.at(case["start"] + .09)[0][2], 0.)
