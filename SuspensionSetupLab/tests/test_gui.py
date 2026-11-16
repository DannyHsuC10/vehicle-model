import csv
from importlib.util import find_spec
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import tkinter as tk
import unittest
from unittest.mock import patch

import numpy as np

from setup_lab.analysis_page import write_csv
from setup_lab.engine import prepare_vehicle, simulate
from setup_lab.geometry_lookup import MODES
from setup_lab.gui import DAMPER_MODES, MR_MODES, SPRINGS, SetupApp
from setup_lab.lateral_tire import DEFAULT_TIR
from setup_lab.project import CORNERS, MODELS, SCENARIOS, compile_project, default_project, open_project, save_project
from setup_lab.vehicle_3d import FullVehicleView


class DesktopTests(unittest.TestCase):
    def make_app(self, project):
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        self.addCleanup(root.destroy)
        root.withdraw()
        return root, SetupApp(root, project)

    def finish_job(self, root, app, page):
        self.assertIsNotNone(app.job)
        deadline = time.monotonic() + 10
        while app.job is not None and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        self.assertIsNone(app.job)
        self.assertIsNotNone(page.result)

    def test_lap_recording_prompts_before_filling_missing_road_columns(self):
        root, app = self.make_app(default_project())
        page = app.analysis_pages["full"]
        original = page.case_input()
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "lap_result.csv"
            path.write_text("distance_m,time_s,phase,ay_m_s2,ax_m_s2,z_fr_mm\n"
                            "0,0,accelerate,0,0,0\n"
                            "2,0.2,brake,-3,-2,5\n", encoding="utf-8")
            with patch("setup_lab.analysis_page.filedialog.askopenfilename", return_value=str(path)), \
                    patch("setup_lab.analysis_page.messagebox.askyesno", side_effect=[False, True]) as confirm, \
                    patch.object(app, "error") as error:
                page.load_recording()
                self.assertEqual(page.case_input(), original)
                message = confirm.call_args.args[1]
                for column in ("z_fl_mm", "z_rl_mm", "z_rr_mm"):
                    self.assertIn(column, message)
                for column in ("z_fr_mm", "ax_mps2", "ay_mps2"):
                    self.assertNotIn(column, message)
                page.load_recording()
                case = page.case_input()["case"]
                self.assertEqual(case["scenario"], "csv")
                self.assertEqual(case["recording_name"], path.name)
                self.assertEqual(case["recording"], [[0., 0., 0., 0., 0., 0., 0.], [.2, 0., 5., 0., 0., -2., -3.]])
                error.assert_not_called()

                path.write_text("distance_m,ax_m_s2,ay_m_s2\n0,0,0\n2,-2,-3\n", encoding="utf-8")
                page.load_recording()
                self.assertIn("time_s", str(error.call_args.args[0]))
                self.assertEqual(confirm.call_count, 2)
                self.assertEqual(page.case_input()["case"], case)

    def test_pure_calculation_exports_wheel_kinematics_without_setting_playback_result(self):
        project = default_project()
        project["case"].update(scenario="combined", target="FL", duration=.22, start=.04, dt=.005)
        root, app = self.make_app(project)
        page = app.analysis
        expected = simulate(app.collect())
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "simulation.csv"
            with patch("setup_lab.gui.filedialog.asksaveasfilename", return_value=str(path)), \
                    patch.object(page, "set_result") as set_result:
                page.compute_csv_button.invoke()
                self.assertTrue(page.compute_csv_button.instate(("disabled",)))
                deadline = time.monotonic() + 10
                while app.job is not None and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertIsNone(app.job)
                set_result.assert_not_called()
            with path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), len(expected.time))
        np.testing.assert_array_equal([float(row["time_s"]) for row in rows], expected.time)
        for name in ("升沉 [mm]", "前側傾 [deg]", "俯仰 [deg]", "FL 輪端行程 [mm]"):
            np.testing.assert_allclose([float(row[name]) for row in rows], expected.signals[name], atol=1e-12)
        for corner in ("FL", "FR", "RL", "RR"):
            np.testing.assert_allclose([float(row[f"{corner} 輪端位移 [mm]"]) for row in rows],
                                       expected.signals[f"{corner} [mm]"])
            np.testing.assert_allclose([float(row[f"{corner} 輪荷 [N]"]) for row in rows], expected.signals[f"{corner} [N]"])
            for suffix in ("輪端垂直速度 [m/s]", "輪端垂直加速度 [m/s²]", "簧上位移 [mm]"):
                self.assertTrue(np.isfinite([float(row[f"{corner} {suffix}"]) for row in rows]).all())

    def test_aero_page_previews_case_speed_and_saves_reference_point(self):
        root, app = self.make_app(default_project())
        app.analysis.case_vars["speed"].set("20")
        aero = app.aero_page
        app.pages.select(aero)
        root.update()
        aero.fields["reference_speed_kmh"].set("36")
        aero.fields["reference_force_n"].set("150")
        aero.enabled.set(True)
        aero.draw()
        curve, reference, operating = aero.chart.spec["series"]
        self.assertEqual((curve["x"][0], curve["y"][0]), (0., 0.))
        self.assertEqual((reference["x"], reference["y"]), ([36.], [150.]))
        self.assertEqual((operating["x"], operating["y"]), ([72.], [600.]))
        project = app.collect()
        self.assertEqual(project["aero"], {"enabled": True, "reference_speed_kmh": 36., "reference_force_n": 150.})
        app.set_project(project, None)
        self.assertEqual(app.aero_page.settings(), project["aero"])

    def test_load_transfer_imports_tir_and_restores_it_without_the_source_file(self):
        _, app = self.make_app(default_project())
        page = app.transfer_page
        page.draw()
        original = page.contact_model.at(1.)["slips"]
        self.assertEqual(page.tir_name.get(), DEFAULT_TIR)
        repo = Path(__file__).resolve().parents[2]
        fixture = repo / "new" / "Pacejka_tire_model" / "tir" / "FSAE_43075R20_W8.tir"
        base = repo / "SuspensionSetupLab" / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            source = Path(folder) / "測試輪胎.tir"
            source.write_bytes(fixture.read_bytes())
            with patch("setup_lab.load_transfer_page.filedialog.askopenfilename", return_value=str(source)):
                page.choose_tir()
            self.assertEqual(page.tir_name.get(), source.name)
            updated = page.contact_model.at(1.)["slips"]
            self.assertNotAlmostEqual(updated[0], original[0])
            self.assertNotAlmostEqual(updated[1], original[1])

            source.write_text("invalid TIR", encoding="utf-8")
            with patch("setup_lab.load_transfer_page.filedialog.askopenfilename", return_value=str(source)), \
                    patch.object(app, "error") as error:
                page.choose_tir()
                error.assert_called_once()
            self.assertEqual(page.contact_model.at(1.)["slips"], updated)

            destination = Path(folder) / "project.json"
            save_project(destination, app.collect(include_pages=True))
            source.unlink()
            app.set_project(*open_project(destination))
            page.draw()
            self.assertEqual(page.tir_name.get(), source.name)
            self.assertEqual(page.contact_model.at(1.)["slips"], updated)

        page.tir_reset.invoke()
        self.assertEqual(page.tir_name.get(), DEFAULT_TIR)
        self.assertEqual(page.contact_model.at(1.)["slips"], original)
        self.assertNotIn("tire_tir", page.options())

    def test_load_transfer_contact_slider_and_legacy_project_settings(self):
        p = default_project()
        p["vehicle"]["h_cg"] = .45
        p["aero"].update(enabled=True, reference_force_n=100.)
        p["load_transfer"] = {
            "rigid": {"ax": -1., "ay": .25, "pressure_f": 95., "pressure_r": 60., "width_f": 175., "width_r": 190.},
            "distribution": {"radius": 30., "ay": 1.25, "pressure_r": 85., "mu": .01, "b": 1., "e": 0.},
        }
        root, app = self.make_app(p)
        tabs = [app.pages.tab(tab, "text").strip() for tab in app.pages.tabs()]
        self.assertIn("載重轉移分配", tabs)
        self.assertNotIn("剛體載重轉移", tabs)
        page = app.transfer_page
        app.pages.select(page)
        page.views.select(page.contact_view)
        root.geometry("1024x690")
        root.deiconify()
        root.update()
        app.run()
        chart = page.charts["接地面"]
        self.assertTrue(chart.winfo_ismapped())
        self.assertTrue(page.contact_slider.winfo_ismapped())
        self.assertEqual(len(page.wheels.get_children()), 4)
        self.assertEqual(len(page.paths.get_children()), 3)
        self.assertEqual(len(page.charts["三路徑力矩"].spec["series"]), 6)
        self.assertEqual([series["name"] for series in chart.spec["series"]], list(CORNERS))
        self.assertEqual(page.contact_ay.get(), 1.25)
        self.assertEqual(page.options()["radius"], 30.)
        self.assertEqual(page.options()["pressure_f"], 95.)
        self.assertEqual(page.options()["pressure_r"], 85.)
        self.assertEqual(page.options()["width_r"], 190.)
        self.assertFalse({"mu", "b", "e"} & page.fields.keys())
        self.assertGreaterEqual(chart.canvas.winfo_width(), 300)
        self.assertGreaterEqual(chart.canvas.winfo_height(), 220)

        def contact_rows():
            return {values[0]: [float(value) for value in values[1:]]
                    for item in page.contact_wheels.get_children()
                    for values in (page.contact_wheels.item(item, "values"),)}

        original_view = chart.view.copy()
        page.contact_slider.set(0.)
        straight = contact_rows()
        straight_width = np.ptp(chart.spec["series"][1]["y"])
        self.assertEqual(straight["FL"], straight["FR"])
        page.contact_slider.set(1.)
        root.update()
        left = contact_rows()
        distribution = page.contact_model.at(1.)
        for corner, load in zip(CORNERS, distribution["loads"]):
            self.assertAlmostEqual(left[corner][0], load, delta=.051)
        self.assertAlmostEqual(left["FR"][1], distribution["loads"][1] / 95. * 10, delta=.051)
        self.assertGreater(left["FR"][1], straight["FR"][1])
        self.assertGreater(np.ptp(chart.spec["series"][1]["y"]), straight_width)
        page.contact_slider.set(-1.)
        root.update()
        self.assertEqual(contact_rows()["FL"], left["FR"])
        self.assertEqual(chart.view, original_view)
        page.contact_slider.set(3.)
        self.assertIn("抬輪", page.contact_readout.cget("text"))
        self.assertEqual(contact_rows()["FL"][1:], [0., 0., 0.])
        self.assertEqual(chart.spec["series"][0]["marker"], "x")

        page.fields["pressure_f"].set("110")
        page.fields["ay"].set("1.5")
        app.run()
        self.assertEqual(page.contact_ay.get(), 1.5)
        self.assertAlmostEqual(contact_rows()["FR"][1], page.contact_model.at(1.5)["loads"][1] / 110. * 10, delta=.051)
        snapshot = app.collect(include_pages=True)
        self.assertEqual(set(snapshot["load_transfer"]), {"distribution"})
        self.assertEqual(snapshot["load_transfer"]["distribution"]["ay"], 1.5)
        self.assertFalse({"mu", "b", "e"} & snapshot["load_transfer"]["distribution"].keys())
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "transfer.json"
            save_project(path, p)
            legacy, _ = open_project(path)
            app.set_project(legacy, None)
            self.assertEqual(page.options()["pressure_f"], 95.)
            save_project(path, snapshot)
            loaded, _ = open_project(path)
            app.set_project(loaded, None)
            self.assertEqual(page.options()["pressure_f"], 110.)

    def test_geometry_csv_import_is_saved_and_used_by_simulation(self):
        p = default_project()
        p["case"].update(scenario="maneuver", target="all", duration=.4, start=.05, ramp=.1)
        root, app = self.make_app(p)
        page = app.geometry_page
        app.pages.select(page)
        root.geometry("1024x690")
        root.deiconify()
        root.update()
        self.assertNotIn("h_rc_f", app.vehicle_vars)
        for kind, chart in page.charts.items():
            self.assertTrue(chart.winfo_ismapped())
            self.assertGreaterEqual(chart.canvas.winfo_width(), 300)
            self.assertTrue(page.fixed_forms[kind].winfo_ismapped())
            self.assertFalse(page.csv_actions[kind].winfo_ismapped())
        page.fixed_vars["dive_front"].set(".42")
        self.assertEqual(page.charts["anti"].spec["series"][0]["y"], [42., 42.])

        def select(kind, mode):
            selector = page.selectors[kind]
            selector.set(MODES[mode])
            selector.event_generate("<<ComboboxSelected>>")
            root.update()

        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "curve.csv"
            with patch("setup_lab.geometry_page.filedialog.askopenfilename", return_value=str(path)), patch.object(app, "error") as error:
                select("roll_center", "lookup")
                self.assertTrue(page.csv_actions["roll_center"].winfo_ismapped())
                self.assertFalse(page.fixed_forms["roll_center"].winfo_ismapped())
                self.assertTrue(page.fixed_forms["anti"].winfo_ismapped())
                self.assertIsNone(page.charts["roll_center"].spec)
                with self.assertRaisesRegex(ValueError, "匯入 Roll Center CSV"):
                    compile_project(app.collect())
                path.write_text("travel_mm,front_rc_mm,rear_rc_mm\n-50,5,10\n0,25,30\n50,65,75\n", encoding="utf-8")
                page.import_table("roll_center")
                select("anti", "lookup")
                path.write_text("travel_mm,front_anti_dive_pct,rear_anti_lift_pct,front_anti_lift_pct,rear_anti_squat_pct\n"
                                "-50,0,0,0,0\n0,15,20,25,30\n50,40,45,50,55\n", encoding="utf-8")
                page.import_table("anti")
                self.assertEqual(page.charts["roll_center"].spec["series"][0]["y"], [5., 25., 65.])
                self.assertEqual(page.charts["anti"].spec["series"][0]["y"], [0., 15., 40.])
                error.assert_not_called()
                before = app.collect()
                path.write_text("travel_mm,front_rc_mm,rear_rc_mm\n0,25,30\n0,65,75\n", encoding="utf-8")
                page.import_table("roll_center")
                error.assert_called_once()
                self.assertEqual(app.collect(), before)
            save_project(path, before)
            loaded, _ = open_project(path)
            app.set_project(loaded, None)
            self.assertEqual(page.tables, before["geometry_tables"])
            result = simulate(app.collect())
            for group in ("Roll Center", "Anti"):
                for name in result.groups[group]:
                    self.assertGreater(np.ptp(result.signals[name]), .01)
            app.pages.select(page)
            root.update()
            select("anti", "fixed")
            self.assertEqual(page.charts["anti"].spec["series"][0]["y"], [42., 42.])
            save_project(path, app.collect())
            fixed, _ = open_project(path)
            self.assertEqual(fixed["geometry_tables"], before["geometry_tables"])
            self.assertEqual(fixed["geometry_modes"], {"anti": "fixed"})
            vehicle, _ = prepare_vehicle(fixed)
            self.assertAlmostEqual(vehicle.geometry.value("dive_front", (50., 50.)), .42)
            self.assertAlmostEqual(vehicle.geometry.value("h_rc_f", (50., 50.)), .065)
            fixed_result = simulate(fixed)
            self.assertIn("Roll Center", fixed_result.groups)
            self.assertNotIn("Anti", fixed_result.groups)
            app.set_project(fixed, None)
            self.assertEqual(page.selectors["anti"].get(), MODES["fixed"])
            app.pages.select(page)
            root.update()
            select("anti", "lookup")
            self.assertEqual(page.charts["anti"].spec["series"][0]["y"], [0., 15., 40.])
            self.assertEqual(app.collect(), before)

    def test_csv_force_lookup_import_updates_only_validated_channel_tables(self):
        root, app = self.make_app(default_project())
        app.pages.select(app.pages.tabs()[-1])
        app.channel_list.selection_set("heave_r")
        app.select_channel()
        original = app.collect()["channels"]
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "curve.csv"
            with patch("setup_lab.gui.filedialog.askopenfilename", return_value=str(path)), patch.object(app, "error") as error:
                self.assertFalse(app.curve_import_buttons["spring"].grid_info())
                self.assertFalse(app.curve_import_buttons["dampers"].grid_info())
                self.assertFalse(app.curve_import_buttons["mr"].grid_info())
                app.spring_kind.set(SPRINGS["force"])
                app.component_mode()
                self.assertTrue(app.curve_import_buttons["spring"].grid_info())
                self.assertFalse(app.curve_import_buttons["dampers"].grid_info())
                path.write_text("travel_mm,force_n\n-5,-150\n0,0\n10,400\n", encoding="utf-8-sig")
                app.curve_import_buttons["spring"].invoke()
                app.component_vars["installed_mm"].set("2")
                app.damper_kind.set(DAMPER_MODES["force"])
                app.change_damper_mode()
                self.assertTrue(app.curve_import_buttons["dampers"].grid_info())
                path.write_text("speed_mps,compression_n,rebound_n\n0,0,0\n0.05,70,90\n0.30,240,340\n",
                                encoding="utf-8")
                app.curve_import_buttons["dampers"].invoke()
                app.mr_kind.set(MR_MODES["lookup"])
                app.change_mr_mode()
                self.assertTrue(app.curve_import_buttons["mr"].grid_info())
                self.assertFalse(app.mr_fixed_form.grid_info())
                for key in ("main_rate", "tender_rate", "tender_after_static_mm"):
                    self.assertTrue(all(not widget.grid_info() for widget in app.component_inputs[key]))
                self.assertTrue(all(widget.grid_info() for widget in app.component_inputs["jounce_mm"]))
                self.assertFalse(any(form.grid_info() for form in app.damper_forms.values()))
                path.write_text("travel_mm,mr\n0,0.9\n15,1.1\n", encoding="utf-8")
                app.curve_import_buttons["mr"].invoke()
                error.assert_not_called()
                root.update_idletasks()
                spring = app.spring_chart.spec["series"][0]
                vehicle, road = prepare_vehicle(app.collect())
                vehicle.equilibrium(road.at(0.), include_aero=False)
                rear_heave = next(component for i, component in vehicle.active if i == 1)
                self.assertEqual(app.spring_chart.spec["ylabel"], "力 (N)")
                self.assertEqual(app.spring_chart.spec["xlabel"], "相對無空力靜態行程 (mm)")
                np.testing.assert_allclose(np.interp(np.array([-7., -2., 0., 8.]) - rear_heave.static_stroke,
                                                     spring["x"], spring["y"]), [-150., 0., 80., 400.], atol=1e-10)
                self.assertEqual(app.damper_chart.spec["series"][0]["y"], [-340., -340., -90., 0., 70., 240., 240.])
                mr = app.mr_chart.spec["series"][0]
                np.testing.assert_allclose(np.interp([-10., 7.5, 20.], mr["x"], mr["y"]), [.9, 1., 1.1])
                project = app.collect()
                channel = project["channels"]["heave_r"]
                self.assertEqual(channel["spring_kind"], "force")
                self.assertEqual(channel["damper_kind"], "force")
                self.assertEqual(channel["spring"], [[-5, -150], [0, 0], [10, 400]])
                self.assertEqual(channel["compression"], [[0, 0], [.05, 70], [.30, 240]])
                self.assertEqual(channel["rebound"], [[0, 0], [.05, 90], [.30, 340]])
                self.assertEqual(channel["mr"], [[0, .9], [15, 1.1]])
                self.assertEqual(project["channels"]["heave_f"], original["heave_f"])

                path.write_text("0,0,0\n0.1,999,-50\n", encoding="utf-8")
                app.import_force_csv("dampers")
                error.assert_called_once()
                self.assertEqual(app.collect(), project)

            saved = Path(folder) / "project.json"
            save_project(saved, project)
            self.assertEqual(open_project(saved)[0], project)

    def test_mr_dropdown_restores_fixed_value_and_single_point_lookup(self):
        root, app = self.make_app(default_project())
        self.assertIs(app.mr_fixed_form.master, app.component_inputs["installed_mm"][1].master)
        app.mr_var.set("0.8")
        root.update_idletasks()
        self.assertTrue(all(value == .8 for value in app.mr_chart.spec["series"][0]["y"]))
        app.mr_kind.set(MR_MODES["lookup"])
        app.change_mr_mode()
        with self.assertRaisesRegex(ValueError, "CSV"):
            app.collect()

        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "mr.csv"
            path.write_text("travel_mm,mr\n0,0.95\n", encoding="utf-8")
            with patch("setup_lab.gui.filedialog.askopenfilename", return_value=str(path)):
                app.curve_import_buttons["mr"].invoke()
            app.mr_kind.set(MR_MODES["fixed"])
            app.change_mr_mode()
            self.assertEqual(app.collect()["channels"]["heave_f"]["mr"], [[0, .8]])
            app.mr_kind.set(MR_MODES["lookup"])
            app.change_mr_mode()
            project = app.collect()
            self.assertEqual(project["channels"]["heave_f"]["mr"], [[0, .95]])
            saved = Path(folder) / "setup.json"
            save_project(saved, project)
            app.set_project(open_project(saved)[0], saved)
            self.assertEqual(app.mr_kind.get(), MR_MODES["lookup"])
            self.assertTrue(app.curve_import_buttons["mr"].grid_info())

    def test_damper_modes_and_independent_switch_speeds(self):
        root, app = self.make_app(default_project())
        self.assertEqual(app.damper_kind.get(), DAMPER_MODES["bilinear"])
        app.damper_kind.set(DAMPER_MODES["single"])
        app.change_damper_mode()
        app.damper_vars["rate"].set("1000")
        self.assertEqual(app.collect()["channels"]["heave_f"]["rebound"], [[0, 1000]])

        app.damper_kind.set(DAMPER_MODES["split"])
        app.change_damper_mode()
        app.damper_vars["rebound"].set("1800")
        self.assertEqual(app.collect()["channels"]["heave_f"]["rebound"], [[0, 1800]])

        app.damper_kind.set(DAMPER_MODES["bilinear"])
        app.change_damper_mode()
        for key, value in (("compression", 1000), ("rebound", 1800),
                           ("compression_switch", .1), ("rebound_switch", .2),
                           ("compression_high", 200), ("rebound_high", 300)):
            app.damper_vars[key].set(str(value))
        root.update_idletasks()
        series = app.damper_chart.spec["series"][0]
        np.testing.assert_allclose(np.interp([-.3, -.2, .1, .3], series["x"], series["y"]),
                                   [-390., -360., 100., 140.])
        project = app.collect()
        compile_project(project)
        app.set_project(project, None)
        self.assertEqual(app.damper_kind.get(), DAMPER_MODES["bilinear"])
        self.assertEqual(float(app.damper_vars["rebound_switch"].get()), .2)

    def test_damper_ratio_checkbox_converts_and_persists_each_channel(self):
        root, app = self.make_app(default_project())
        root.update_idletasks()
        before = app.damper_chart.spec["series"][0]
        app.damper_ratio_check.invoke()
        root.update_idletasks()
        self.assertIn("阻尼比", app.damper_labels["bilinear", "compression"].cget("text"))
        self.assertEqual(float(app.damper_vars["compression_switch"].get()), .1)
        np.testing.assert_allclose(app.damper_chart.spec["series"][0]["y"], before["y"])

        app.damper_vars["compression"].set(".4")
        app.damper_vars["compression_high"].set(".2")
        app.channel_list.selection_set("roll_f")
        app.select_channel()
        self.assertFalse(app.damper_ratio.get())
        project = app.collect()
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as directory:
            path = Path(directory) / "ratio.json"
            save_project(path, project)
            saved, _ = open_project(path)
        app.set_project(saved, None)
        self.assertTrue(app.damper_ratio.get())
        self.assertEqual(float(app.damper_vars["compression"].get()), .4)
        self.assertEqual(float(app.damper_vars["compression_high"].get()), .2)

        app.damper_kind.set(DAMPER_MODES["force"])
        app.change_damper_mode()
        self.assertFalse(app.damper_ratio.get())
        self.assertFalse(app.damper_ratio_check.winfo_manager())
        app.damper_kind.set(DAMPER_MODES["bilinear"])
        app.change_damper_mode()
        self.assertTrue(app.damper_ratio.get())
        app.damper_ratio_check.invoke()
        converted = app.collect()
        self.assertNotIn("damper_ratio", converted["channels"]["heave_f"])
        ratio_damper, coefficient_damper = compile_project(saved)[0], compile_project(converted)[0]
        for velocity in (-.4, -.05, .05, .4):
            self.assertAlmostEqual(ratio_damper.damper_force(velocity), coefficient_damper.damper_force(velocity))

    def test_elastic_chart_includes_both_travel_stops(self):
        p = default_project()
        p["channels"]["heave_f"]["spring_kind"] = "single"
        root, app = self.make_app(p)
        for name, value in (("jounce_mm", 10), ("jounce_rate", 80), ("rebound_mm", 15), ("rebound_rate", 50)):
            app.component_vars[name].set(str(value))
        root.update_idletasks()
        series = app.spring_chart.spec["series"][0]
        vehicle, road = prepare_vehicle(app.collect())
        trim = vehicle.equilibrium(road.at(0.), include_aero=False)
        static = float(vehicle.channel_travel[0] @ trim) * 1000
        rebound = -15. - static
        jounce_force = 35. * (static + 10.)
        points = [rebound * 1.1, rebound, 0., 10., 11.]
        np.testing.assert_allclose(app.spring_chart.view[:2], [points[0], points[-1]])
        for point in points:
            self.assertTrue(np.isclose(series["x"], point).any())
        np.testing.assert_allclose(np.interp(points, series["x"], series["y"]),
                                   [-525. + .1 * rebound * 50., -525., 35. * static, jounce_force, jounce_force + 80.])

    def test_draft_background_run_and_saved_baseline(self):
        p = default_project()
        p["case"].update(duration=.5, start=.1, dt=.004)
        root, app = self.make_app(p)
        full, left_right, front_rear, quarter = (app.analysis_pages[key] for key in ("full", "half_lr", "half_fr", "quarter"))
        self.assertEqual(app.collect(), p)
        app.pages.select(app.pages.tabs()[-1])
        app.damper_kind.set(DAMPER_MODES["split"])
        app.change_damper_mode()
        app.damper_vars["compression"].set("1000")
        app.channel_list.selection_set("heave_r")
        app.select_channel()
        app.channel_list.selection_set("heave_f")
        app.select_channel()
        self.assertEqual(app.collect()["channels"]["heave_f"]["compression"], [[0, 1000]])
        app.run()
        self.finish_job(root, app, full)
        full.pin()
        baseline = full.result
        app.set_project(p, None)
        self.assertIs(full.baseline, baseline)
        app.run()
        self.finish_job(root, app, full)
        self.assertIs(full.comparison(), baseline)
        full.group.set("輪荷")
        full.select_group()
        self.assertEqual(len(full.chart.spec["series"]), 8)

        app.pages.select(quarter)
        root.update()
        quarter.source_widget.current(3)
        quarter.case_mode()
        self.assertEqual(app.current_channel, "heave_f")
        self.assertNotIn("quarter_mr", quarter.case_vars)
        quarter.case["quarter_mr"] = .8
        self.assertNotIn(SCENARIOS["maneuver"], quarter.scenario_widget["values"])
        app.channel_list.selection_set("heave_r")
        app.select_channel()
        app.component_vars["jounce_mm"].set("7")
        app.component_vars["jounce_rate"].set("100")
        app.component_vars["rebound_rate"].set("80")
        compile_project(app.collect())
        app.run()
        app.pages.select(left_right)
        self.finish_job(root, app, quarter)
        self.assertEqual(quarter.result.corners, ("RR",))
        self.assertNotIn("quarter_mr", quarter.result.project["case"])
        self.assertIn("後軸側傾", quarter.result.groups)
        quarter.group.set("輪端行程")
        quarter.select_group()
        for label in ("後軸升沉壓縮限位", "後軸升沉伸長限位"):
            limit = next(line for line in quarter.chart.spec["series"] if line.get("name", "").startswith(label))
            self.assertTrue(limit["dash"])
            self.assertEqual(limit["y"][0], limit["y"][1])
        self.assertEqual(len(quarter.metrics.get_children()), 1)
        self.assertIsNone(left_right.result)
        self.assertIsNone(quarter.comparison())
        self.assertIs(full.comparison(), baseline)

        left_right.scenario.set(SCENARIOS["maneuver"])
        left_right.case_mode()
        self.assertTrue(left_right.case_widgets["ax"].instate(("disabled",)))
        self.assertTrue(left_right.case_widgets["ay"].instate(("!disabled",)))
        left_right.scenario.set(SCENARIOS["iso_c"])
        left_right.case_mode()
        self.assertTrue(left_right.case_widgets["seed"].instate(("!disabled",)))
        self.assertTrue(left_right.case_widgets["ay"].instate(("disabled",)))
        left_right.iso_lateral_widget.invoke()
        self.assertTrue(left_right.case_widgets["ay"].instate(("!disabled",)))
        self.assertTrue(left_right.case_input()["case"]["iso_lateral"])
        self.assertIn(SCENARIOS["iso_a"], quarter.scenario_widget["values"])
        self.assertNotIn(SCENARIOS["cornering"], quarter.scenario_widget["values"])

        p["model"] = "half_fr"
        p["model_sources"]["half_fr"] = "right"
        p["case"]["target"] = "front"
        app.set_project(p, None)
        self.assertIs(app.analysis, front_rear)
        self.assertEqual(app.collect()["case"]["target"], "FR")
        self.assertEqual(app.collect()["model_sources"]["half_fr"], "right")

    def test_component_enable_checkboxes_are_model_specific_and_persist(self):
        project = default_project()
        root, app = self.make_app(project)
        app.pages.select(app.component_page)
        root.geometry("1024x690")
        root.deiconify()
        app.channel_list.selection_set("roll_f")
        app.select_channel()
        root.update()
        checks = app.channel_model_checks
        self.assertEqual(app.channel_list["columns"], ("channel",))
        self.assertTrue(all(check.winfo_ismapped() for check in checks.values()))
        self.assertTrue(checks["half_fr"].instate(("disabled",)))
        self.assertTrue(checks["quarter"].instate(("selected",)))
        checks["quarter"].invoke()
        self.assertFalse(checks["quarter"].instate(("selected",)))
        self.assertTrue(checks["full"].instate(("selected",)))
        self.assertTrue(checks["half_lr"].instate(("selected",)))

        quarter = app.analysis_pages["quarter"]
        app.pages.select(quarter)
        root.update()
        quarter.source_widget.current(3)
        quarter.case_mode()
        app.pages.select(app.component_page)
        root.update()
        self.assertEqual(app.current_channel, "roll_f")
        self.assertTrue(checks["quarter"].instate(("!disabled", "!selected")))
        self.assertTrue(checks["full"].instate(("selected",)))

        quarter.source_widget.current(0)
        quarter.case_mode()
        snapshot = app.collect(include_pages=True)
        self.assertEqual(snapshot["channels"]["roll_f"]["enabled_models"], {"quarter": False})
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "model-channels.json"
            save_project(path, snapshot)
            loaded, _ = open_project(path)
        app.set_project(loaded, None)
        app.channel_list.selection_set("roll_f")
        app.select_channel()
        self.assertTrue(checks["quarter"].instate(("!selected",)))
        self.assertTrue(checks["full"].instate(("selected",)))
        result = simulate(app.collect())
        self.assertNotIn("前軸側傾", result.groups)
        self.assertIn("前軸升沉", result.groups)
        self.assertTrue(any("未計入已停用的通道" in message and "前軸側傾" in message for message in result.warnings))
        for family in ("full", "half_lr"):
            car, _ = prepare_vehicle(app.collect(app.analysis_pages[family]))
            self.assertIn(2, [i for i, _ in car.active])

        app.channel_list.selection_set("FL")
        app.select_channel()
        self.assertTrue(checks["half_lr"].instate(("!selected", "!disabled")))
        checks["half_lr"].invoke()
        car, _ = prepare_vehicle(app.collect(app.analysis_pages["half_lr"]))
        self.assertIn(6, [i for i, _ in car.active])
        car, _ = prepare_vehicle(app.collect(app.analysis_pages["full"]))
        self.assertNotIn(6, [i for i, _ in car.active])

    def test_front_rear_chart_shows_each_heave_limit_and_tender_crossing(self):
        project = default_project()
        project["model"] = "half_fr"
        project["model_sources"]["half_fr"] = "right"
        project["case"].update(target="all", duration=.62, start=.08, height=30.)
        for key, tender, mr in (("heave_f", 2., [[0., .8]]), ("heave_r", 3., [[-20., .6], [20., 1.1]])):
            project["channels"][key].update(spring_kind="tender", tender_after_static_mm=tender, mr=mr,
                                            jounce_mm=4., jounce_rate=100., rebound_mm=9., rebound_rate=80.)
        root, app = self.make_app(project)
        page = app.analysis
        root.geometry("1024x690")
        root.deiconify()
        app.run()
        self.finish_job(root, app, page)
        page.group.set("輪端行程")
        page.select_group()
        root.update()
        result = page.result
        self.assertEqual(result.corners, ("FR", "RR"))
        self.assertTrue(page.tender_note.winfo_ismapped())
        self.assertGreaterEqual(page.chart.canvas.winfo_height(), 220)

        car, road = prepare_vehicle(project)
        car.equilibrium(road.at(0.))
        series = page.chart.spec["series"]
        for (channel, component), corner, wheel_line in zip(car.active, result.corners, series):
            key, label = (("heave_f", "前軸升沉"), ("heave_r", "後軸升沉"))[channel]
            reference = result.metrics[corner][3]
            ratio = component.motion_ratio
            static_stroke = ratio.integrated(reference)
            info = result.tender_switches[key]
            self.assertAlmostEqual(info.wheel_reference_mm, reference)
            for title, wheel, shaft in (
                ("壓縮限位", result.wheel_travel_limits[key]["compression"], static_stroke + 4.),
                ("伸長限位", result.wheel_travel_limits[key]["rebound"], -9.),
                ("併圈門檻", info.wheel_threshold_mm, component.tender_bind_stroke),
            ):
                self.assertAlmostEqual(ratio.integrated(wheel), shaft)
                line = next(s for s in series if s.get("name", "").startswith(label + title))
                self.assertEqual(line["y"], [wheel, wheel])
                self.assertEqual(line["color"], wheel_line["color"])
            wheel_travel = result.signals[f"{corner} 輪端行程 [mm]"]
            for times, shape in ((info.binding_times, "+"), (info.release_times, "x")):
                self.assertEqual(len(times), 1)
                np.testing.assert_allclose(np.interp(times, result.time, wheel_travel), info.wheel_threshold_mm,
                                           atol=1e-3)
                self.assertTrue(any(s.get("marker") == shape and s["x"] == times.tolist() and
                                    s["color"] == wheel_line["color"] for s in series))
        self.assertIn("前軸升沉", page.tender_note.cget("text"))
        self.assertIn("後軸升沉", page.tender_note.cget("text"))
        page.signal.set("RR 輪端行程 [mm]")
        page.draw_result()
        names = [s.get("name", "") for s in page.chart.spec["series"]]
        self.assertTrue(any(name.startswith("後軸升沉併圈") for name in names))
        self.assertFalse(any(name.startswith("前軸升沉") for name in names))
        page.group.set("輪荷")
        page.select_group()
        self.assertFalse(page.tender_note.winfo_manager())

    def test_quarter_chart_marks_extrema_for_selected_curves(self):
        project = default_project()
        project["model"] = "quarter"
        project["case"].update(target="FL", duration=.35, start=.05)
        root, app = self.make_app(project)
        page = app.analysis
        root.geometry("1024x690")
        root.deiconify()
        page.set_result(simulate(project))
        page.pin()
        project["tires"]["FL"]["k"] *= 1.2
        page.set_result(simulate(project))
        page.group.set("輪荷")
        page.select_group()
        root.update()

        self.assertIsNotNone(page.comparison())
        curves = page.chart.spec["series"]
        self.assertEqual([curve["name"] for curve in curves[:2]], ["B FL", "A FL"])
        for run, markers in ((page.result, curves[2:4]), (page.baseline, curves[4:6])):
            loads = run.signals["FL [N]"]
            for marker, index, shape in zip(markers, (np.argmax(loads), np.argmin(loads)), ("triangle_up", "triangle_down")):
                self.assertEqual((marker["x"], marker["y"]), ([run.time[index]], [loads[index]]))
                self.assertEqual(marker["marker"], shape)
        note = page.extrema_note.cget("text")
        self.assertIn("B FL", note)
        self.assertIn("A FL", note)
        self.assertIn("▲ 最大", note)
        self.assertIn("▼ 最小", note)
        self.assertIn(f"{page.result.time[np.argmax(page.result.signals['FL [N]'])]:.5f} s", note)
        self.assertTrue(page.extrema_note.winfo_ismapped())
        triangles = [coords for kind, coords, *_ in page.chart.primitives if kind == "line" and len(coords) == 8]
        self.assertEqual(len(triangles), 4)

        page.clear_baseline()
        page.group.set("空力")
        page.select_group()
        page.signal.set("全車下壓力 [N]")
        page.draw_result()
        curves = page.chart.spec["series"]
        self.assertEqual(len(curves), 2)
        self.assertEqual(curves[1]["marker"], "diamond")
        self.assertIn("最大＝最小", page.extrema_note.cget("text"))
        page.set_result(None)
        self.assertFalse(page.extrema_note.winfo_manager())

    def test_full_car_attitude_overlays_pitch_and_roll(self):
        p = default_project()
        p["case"].update(duration=.4, start=.05, target="FL")
        root, app = self.make_app(p)
        page = app.analysis_pages["full"]
        for model, angles in (("rigid", ("俯仰", "側傾")), ("flex", ("俯仰", "前側傾", "後側傾"))):
            with self.subTest(model=model):
                p["model"] = model
                result = simulate(p)
                page.set_result(result)
                self.assertEqual(page.group.get(), "車身姿態")
                self.assertEqual(page.signal.get(), "全部")
                self.assertEqual(page.chart.spec["ylabel"], "deg")
                self.assertNotIn("載重轉移速率", result.groups)
                series = page.chart.spec["series"]
                self.assertEqual(tuple(line["name"] for line in series), angles)
                for line in series:
                    np.testing.assert_allclose(line["x"], result.time)
                    np.testing.assert_allclose(line["y"], result.signals[f"{line['name']} [deg]"])
                    self.assertGreater(np.max(np.abs(line["y"])), 0.)

    def test_left_right_load_transfer_charts_peaks_and_csv(self):
        project = default_project()
        project["model"] = "half_lr"
        project["case"].update(scenario="cornering", target="all", ay=-5., duration=.4, start=.05, ramp=.12)
        root, app = self.make_app(project)
        page = app.analysis
        app.run()
        self.finish_job(root, app, page)
        result = page.result
        page.group.set("載重轉移量")
        page.select_group()
        self.assertEqual(page.chart.spec["ylabel"], "N")
        curves = page.chart.spec["series"]
        self.assertEqual(len(curves), 8)
        for name, marker in zip(result.groups["載重轉移量"], curves[4:]):
            values = result.signals[name]
            peak = np.argmax(np.abs(values))
            self.assertLess(values[peak], 0.)
            self.assertEqual((marker["x"], marker["y"]), ([result.time[peak]], [values[peak]]))
            self.assertIn(f"{values[peak]:+,.4g} N（{result.time[peak]:.4f} s）", page.extrema_note.cget("text"))
        page.group.set("載重轉移速率")
        page.select_group()
        self.assertEqual(page.signal.get(), "全部")
        self.assertEqual(page.chart.spec["ylabel"], "N/s")
        curves = page.chart.spec["series"]
        self.assertEqual(len(curves), 8)
        for curve, marker in zip(curves[:4], curves[4:]):
            rate, time = result.metrics["load_transfer_rate"][curve["name"].removesuffix("載重轉移速率")]
            self.assertEqual((marker["x"], marker["y"]), ([time], [rate]))
            self.assertEqual(marker["marker"], "x")
            self.assertIn(f"{rate:+,.4g} N/s（{time:.4f} s）", page.extrema_note.cget("text"))
        self.assertIn("載重轉移速率峰值", page.details.get("1.0", "end"))
        self.assertIn("輪胎端", page.details.get("1.0", "end"))

        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "transfer.csv"
            write_csv(path, result)
            with path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), len(result.time))
            for group in ("載重轉移量", "載重轉移速率"):
                for signal in result.groups[group]:
                    np.testing.assert_allclose([float(row[signal]) for row in rows], result.signals[signal])

        page.pin()
        project["vehicle"]["h_rc_f"] *= 2
        page.set_result(simulate(project))
        page.signal.set("輪胎端載重轉移速率 [N/s]")
        page.draw_result()
        self.assertIn("× B／＋ A", page.extrema_note.cget("text"))
        self.assertIn("輪胎端峰值", page.extrema_note.cget("text"))
        self.assertNotIn("幾何峰值", page.extrema_note.cget("text"))
        self.assertEqual([line["marker"] for line in page.chart.spec["series"][2:]], ["x", "+"])
        page.group.set("輪荷")
        page.select_group()
        self.assertFalse(page.extrema_note.winfo_manager())

    def test_roll_center_scan_compares_21_heights_and_opens_best_run(self):
        project = default_project()
        project["model"] = "half_lr"
        project["model_sources"]["half_lr"] = "rear"
        project["vehicle"]["h_rc_r"] = .030
        project["case"].update(scenario="cornering", target="all", ay=-5., duration=.24, start=.03, ramp=.1, dt=.004)
        root, app = self.make_app(project)
        page = app.analysis
        page.scan_button.invoke()
        self.assertIsNotNone(app.job)
        self.assertTrue(page.scan_button.instate(("disabled",)))
        deadline = time.monotonic() + 15
        while app.job is not None and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        self.assertIsNone(app.job)
        scan = page.scan_result
        self.assertIsNotNone(scan)
        self.assertEqual(scan.heights, list(range(20, 41)))
        self.assertEqual(scan.axle, "rear")
        self.assertEqual(scan.peaks[scan.best_index], min(scan.peaks))
        self.assertEqual(scan.peaks[10], abs(simulate(project).metrics["load_transfer_rate"]["輪胎端"][0]))
        self.assertEqual(scan.best_result.project["vehicle"]["h_rc_f"], project["vehicle"]["h_rc_f"])
        self.assertEqual(app.collect()["vehicle"]["h_rc_r"], .030)
        self.assertIn("最低峰值", page.scan_summary.get())
        self.assertEqual(page.scan_chart.spec["series"][0]["x"], scan.heights)
        page.scan_best_button.invoke()
        self.assertIs(page.result, scan.best_result)
        self.assertEqual(page.signal.get(), "輪胎端載重轉移速率 [N/s]")
        self.assertEqual(page.outputs.select(), page.plots)

    def test_left_right_frame_form_runs_and_round_trips(self):
        project = default_project()
        project["model"] = "half_lr"
        project["case"].update(scenario="cornering", target="all", ay=5., duration=.35, start=.05, ramp=.12)
        root, app = self.make_app(project)
        page = app.analysis
        app.pages.select(app.vehicle_page)
        root.geometry("1024x690")
        root.deiconify()
        root.update()
        self.assertTrue(app.roll_frame_enabled.get())
        for widget in app.roll_frame_widgets.values():
            self.assertTrue(widget.winfo_ismapped())
            self.assertTrue(widget.instate(("!disabled",)))
        app.roll_frame_vars["mass"].set("70")
        app.pages.select(page)
        root.update()
        app.run()
        self.finish_job(root, app, page)
        page.group.set("車架傳力")
        page.select_group()
        self.assertEqual([line["name"] for line in page.chart.spec["series"]],
                         ["Roll Center 指令", "Roll Center 傳遞力"])
        snapshot = app.collect(app.analysis_pages["full"], include_pages=True)
        self.assertEqual(snapshot["roll_center_frame"]["mass"], 70.)
        self.assertNotIn("roll_center_frame", snapshot["page_cases"]["half_lr"]["case"])
        app.set_project(snapshot, None)
        self.assertTrue(app.roll_frame_enabled.get())
        self.assertEqual(float(app.roll_frame_vars["mass"].get()), 70.)

        project["case"]["roll_center_frame"] = {"enabled": False, "mass": 80.}
        app.set_project(project, None)
        self.assertFalse(app.roll_frame_enabled.get())
        self.assertTrue(app.roll_frame_widgets["mass"].instate(("disabled",)))
        migrated = app.collect()
        self.assertNotIn("roll_center_frame", migrated["case"])
        self.assertEqual(migrated["roll_center_frame"]["mass"], 80.)
        self.assertIsNone(prepare_vehicle(migrated)[0].roll_frame)

        legacy = default_project()
        legacy["page_cases"] = {"half_lr": {"model": "half_lr", "case": project["case"]}}
        app.set_project(legacy, None)
        self.assertFalse(app.roll_frame_enabled.get())
        self.assertEqual(float(app.roll_frame_vars["mass"].get()), 80.)

    def test_playback_and_project_round_trip_across_analysis_pages(self):
        p = default_project()
        p["case"].update(duration=.4, start=.05, dt=.005)
        p["page_cases"] = {"half": {"model": "half_fr", "case": {**p["case"], "duration": .6, "ax": -3.}}}
        compile_project(p)
        root, app = self.make_app(p)
        self.assertEqual([app.pages.tab(tab, "text").strip() for tab in app.pages.tabs()[:4]],
                         ["全車", "1/2 左右", "1/2 前後", "1/4 車"])
        left_right, front_rear = (app.analysis_pages[key] for key in ("half_lr", "half_fr"))
        self.assertEqual(front_rear.case_input()["case"]["ax"], -3.)
        self.assertEqual(left_right.case_input()["case"]["ax"], -5.)
        for family, model, projection in (("full", "flex", None), ("half_lr", "half_lr", "前視"),
                                          ("half_fr", "half_fr", "側視"), ("quarter", "quarter", "側視")):
            page = app.analysis_pages[family]
            app.pages.select(page)
            root.update()
            page.model.set(MODELS[model])
            page.case_mode()
            if family in ("half_lr", "half_fr"):
                page.target.set("FL")
            page.case_vars["duration"].set(.4)
            page.case_vars["start"].set(.05)
            app.run()
            self.finish_job(root, app, page)
            if family in ("half_lr", "half_fr"):
                angle = "側傾" if family == "half_lr" else "俯仰"
                self.assertEqual(page.group.get(), "車身姿態")
                page.signal.set(f"{angle} [deg]")
                page.draw_result()
                self.assertEqual(page.chart.spec["ylabel"], "deg")
                series, = page.chart.spec["series"]
                self.assertEqual(series["name"], angle)
                np.testing.assert_allclose(series["y"], page.result.signals[f"{angle} [deg]"])
                self.assertGreater(np.max(np.abs(series["y"])), 0.)
            if projection is None:
                self.assertIsInstance(page.motion, FullVehicleView)
            else:
                self.assertEqual(page.motion.projection, projection)
            page.playback.seek(.17)
            index = page.playback.index
            self.assertGreater(index, 0)
            self.assertEqual(page.motion.index, index)
            self.assertEqual(page.dashboard.index, index)
            self.assertEqual(page.chart.playback_time, page.result.time[index])
        left_right.pin()
        self.assertEqual(left_right.result.corners, ("FL", "FR"))
        self.assertEqual(front_rear.result.corners, ("FL", "RL"))
        self.assertIsNone(front_rear.baseline)
        quarter = app.analysis_pages["quarter"]
        quarter.playback.toggle()
        app.pages.select(app.analysis_pages["full"])
        root.update()
        self.assertIsNone(quarter.playback.timer)
        self.assertIsNotNone(quarter.result)
        state = app.editor_state()
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "project.json"
            save_project(path, app.collect(include_pages=True))
            loaded, _ = open_project(path)
            app.set_project(loaded, path)
        self.assertEqual(app.editor_state(), state)

    @unittest.skipUnless(find_spec("pyvista"), "PyVista required for rendering STL")
    def test_full_car_renders_stl_and_uses_each_wheels_displacement(self):
        root, app = self.make_app(default_project())
        root.deiconify()
        root.update()
        page = app.analysis_pages["full"]
        for model in ("rigid", "flex"):
            with self.subTest(model=model):
                project = default_project()
                project["model"] = model
                project["case"].update(duration=.4, start=.05, height=35.)
                result = simulate(project)
                page.set_result(result)
                root.update()
                view = page.motion
                self.assertIsNone(view.error)
                self.assertIsNotNone(view.photo)
                np.testing.assert_array_equal(view.scene.chassis.points, view.scene.vertices)
                offsets = 1000 * (result.motion[:, :, 1] - result.motion[0, :, 1])
                index = int(np.argmax(offsets[:, 0]))
                self.assertGreater(offsets[index, 0], 1.)
                page.playback.show(index)
                root.update()
                self.assertEqual(view.drawn_index, page.dashboard.index)
                self.assertEqual(page.chart.playback_time, result.time[index])
                np.testing.assert_allclose([actor.position[2] for actor in view.scene.wheel_actors], offsets[index])
                self.assertFalse(np.array_equal(view.scene.chassis.points, view.scene.vertices))
