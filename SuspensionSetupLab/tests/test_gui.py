from importlib.util import find_spec
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import tkinter as tk
import unittest
from unittest.mock import patch

import numpy as np

from setup_lab.engine import simulate
from setup_lab.gui import DAMPER_MODES, MR_MODES, SPRINGS, SetupApp
from setup_lab.project import MODELS, SCENARIOS, compile_project, default_project, open_project, save_project
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
                for key in ("main_rate", "tender_rate", "tender_travel"):
                    self.assertTrue(all(not widget.grid_info() for widget in app.component_inputs[key]))
                self.assertTrue(all(widget.grid_info() for widget in app.component_inputs["jounce_mm"]))
                self.assertFalse(any(form.grid_info() for form in app.damper_forms.values()))
                path.write_text("travel_mm,mr\n0,0.9\n15,1.1\n", encoding="utf-8")
                app.curve_import_buttons["mr"].invoke()
                error.assert_not_called()
                root.update_idletasks()
                spring = app.spring_chart.spec["series"][0]
                spring_forces = dict(zip(spring["x"], spring["y"]))
                self.assertEqual(app.spring_chart.spec["ylabel"], "力 (N)")
                self.assertEqual([spring_forces[x] for x in (-7., -2., 0., 8.)], [-150., 0., 80., 400.])
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
        app.damper_vars["rate"].set("1000")
        self.assertEqual(app.collect()["channels"]["heave_f"]["rebound"], [[0, 1000]])

        app.damper_kind.set(DAMPER_MODES["split"])
        app.change_damper_mode()
        app.damper_vars["rebound"].set("1800")
        self.assertEqual(app.collect()["channels"]["heave_f"]["rebound"], [[0, 1800]])

        app.damper_kind.set(DAMPER_MODES["bilinear"])
        app.change_damper_mode()
        for key, value in (("compression_switch", .1), ("rebound_switch", .2),
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

    def test_elastic_chart_includes_both_travel_stops(self):
        p = default_project()
        p["channels"]["heave_f"]["spring_kind"] = "single"
        root, app = self.make_app(p)
        for name, value in (("jounce_mm", 10), ("jounce_rate", 80), ("rebound_mm", 15), ("rebound_rate", 50)):
            app.component_vars[name].set(str(value))
        root.update_idletasks()
        series = app.spring_chart.spec["series"][0]
        self.assertIn(-15., series["x"])
        self.assertIn(10., series["x"])
        np.testing.assert_allclose(app.spring_chart.view[:2], [-16.5, 11.])
        np.testing.assert_allclose([series["x"][0], series["x"][-1]], [-16.5, 11.])
        np.testing.assert_allclose(np.interp([-16.5, -15., 10., 11.], series["x"], series["y"]),
                                   [-600., -525., 350., 430.])

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
        self.assertEqual(len(full.chart.spec["series"]), 8)

        app.pages.select(quarter)
        root.update()
        quarter.source_widget.current(3)
        quarter.case_mode()
        self.assertEqual(app.channel_list.set("heave_r", "state"), "啟用")
        self.assertEqual(app.channel_list.set("roll_r", "state"), "不參與")
        self.assertNotIn(SCENARIOS["maneuver"], quarter.scenario_widget["values"])
        compile_project(app.collect())
        app.run()
        app.pages.select(left_right)
        self.finish_job(root, app, quarter)
        self.assertEqual(quarter.result.corners, ("RR",))
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
            page.case_vars["duration"].set(.4)
            page.case_vars["start"].set(.05)
            app.run()
            self.finish_job(root, app, page)
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
