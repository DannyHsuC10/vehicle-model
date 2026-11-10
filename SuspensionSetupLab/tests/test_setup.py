from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from setup_lab.components import KCSystem
from setup_lab.engine import Cancelled, G, Vehicle, simulate
from setup_lab.project import (CORNERS, compile_project, default_project, import_linear, linear, open_project,
                               read_recording, save_project)
from setup_lab.road import Road


class ComponentTests(unittest.TestCase):
    def test_multi_stage_spring_and_asymmetric_damper_are_continuous(self):
        settings = linear(10000, 1000)
        settings.update(spring=[[-10, 5], [0, 10], [10, 20], [20, 40]],
                        compression=[[0, 1000], [.05, 500], [.15, 900]],
                        rebound=[[0, 2000], [.05, 1000]], jounce_mm=20, jounce_rate=50,
                        rebound_mm=15, rebound_rate=30)
        component = KCSystem(settings)
        for travel, expected in ((-12, -60), (10, 100), (15, 200), (20, 300), (25, 500)):
            self.assertAlmostEqual(component.spring_force(travel), expected)
        for speed, expected in ((0, 0), (.05, 50), (.1, 75), (.2, 145), (-.1, -150)):
            self.assertAlmostEqual(component.damper_force(speed), expected)
        self.assertEqual(component.stop_force(22), 100)
        self.assertEqual(component.stop_force(-17), -60)

    def test_tender_binding_uses_tender_travel_and_installed_compression(self):
        settings = linear(60000, 1000)
        settings.update(spring_kind="tender", main_rate=60, tender_rate=30, tender_travel=4, installed_mm=2)
        component = KCSystem(settings)
        self.assertAlmostEqual(component.spring_force(-3), 0)
        self.assertAlmostEqual(component.spring_force(0), 40)
        self.assertAlmostEqual(component.spring_force(4), 120)
        self.assertAlmostEqual(component.spring_force(5), 180)

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
                self.assertAlmostEqual(sum(load), G * project["vehicle"]["m"])
                expected = G * 321 / 2 + 321 * 4 * .3 / 1.53
                self.assertAlmostEqual(sum(load[:2]), expected)

    def test_lift_recontact_and_tolerance_convergence(self):
        project = default_project()
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
        project["case"].update(duration=.8, start=.1, dt=.004)
        a = simulate(project)
        heave = project["channels"]["heave_f"]
        heave.update(spring_kind="tender", main_rate=55, tender_rate=25, tender_travel=5,
                     compression=[[0, 1600], [.03, 900], [.12, 500]], rebound=[[0, 3600], [.05, 1800]],
                     jounce_mm=15, jounce_rate=70.)
        b = simulate(project)
        self.assertEqual(a.signature, b.signature)
        self.assertGreater(np.max(np.abs(a.signals["FL [N]"] - b.signals["FL [N]"])), 40)
        self.assertGreater(np.max(b.signals["前軸升沉 限位塊力 [N]"]), 0)
        self.assertEqual(a.project["channels"]["heave_f"]["spring_kind"], "stages")

    def test_unstable_configuration_rejected_and_cancellation(self):
        project = default_project()
        for component in project["channels"].values():
            component["enabled"] = False
        with self.assertRaisesRegex(ValueError, "靜態平衡"):
            simulate(project)
        with self.assertRaises(Cancelled):
            simulate(default_project(), cancel=lambda: True)


class ReducedModelTests(unittest.TestCase):
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
                self.assertEqual(car.n, 2 if model == "quarter" else 4)
                sprung = (321 - 65) * {"quarter": .2, "half_lr": .4, "half_fr": .5}[model]
                total = sprung + sum(p["tires"][c]["mu"] for c in corners)
                zero = np.zeros(len(corners))
                inputs = zero, zero, ax, ay
                q = car.equilibrium(inputs)
                acceleration, loads, *_ = car.forces(q, np.zeros(car.n), inputs, True)
                np.testing.assert_allclose(acceleration, 0., atol=1e-8)
                self.assertAlmostEqual(sum(loads), total * G)
                if model == "half_lr":
                    self.assertAlmostEqual(loads[0], (sprung / 2 + 15) * G - total * ay * .3 / 1.24)
                if model == "half_fr":
                    self.assertAlmostEqual(loads[0], (sprung * .6 + 15) * G - total * ax * .3 / 1.5)

    def test_symmetric_heave_uses_same_shaft_curve_as_full_car(self):
        p = default_project()
        p["case"]["target"] = "all"
        p["channels"]["heave_f"].update(spring=[[0, 20], [15, 80]], compression=[[0, 1000], [.1, 300]],
                                        mr=[[0, .9], [30, 1.05]])
        readings = {}
        for model in ("rigid", "quarter", "half_lr", "half_fr"):
            p["model"] = model
            car = Vehicle(p, compile_project(p))
            q, v = np.zeros(car.n), np.zeros(car.n)
            q[car.nb:] = .012
            v[car.nb:] = .15
            zero = np.zeros(car.nw)
            _, _, channels, wheel_force, _ = car.forces(q, v, (zero, zero, 0, 0), True)
            readings[model] = channels[0], wheel_force[0]
        for model in ("quarter", "half_lr", "half_fr"):
            np.testing.assert_allclose(readings[model][0], readings["rigid"][0])
            self.assertAlmostEqual(readings[model][1], readings["rigid"][1])

    def test_each_local_model_responds_to_staged_spring_and_asymmetric_damper(self):
        for model, channel, response in (("quarter", "heave_f", "升沉 [mm]"),
                                          ("half_lr", "roll_f", "側傾 [deg]"),
                                          ("half_fr", "heave_f", "俯仰 [deg]")):
            with self.subTest(model=model):
                p = default_project()
                p["model"] = model
                p["case"].update(duration=.55, start=.1, dt=.005)
                baseline = simulate(p)
                p["channels"][channel].update(spring=[[-3, 18], [0, 25], [3, 80]],
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
