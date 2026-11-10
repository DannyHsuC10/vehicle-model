from importlib.util import find_spec
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import tkinter as tk
import unittest
from unittest.mock import patch

import numpy as np

from setup_lab.engine import simulate
from setup_lab.gui import SetupApp
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

    def test_csv_force_lookup_import_updates_only_validated_channel_tables(self):
        _, app = self.make_app(default_project())
        app.pages.select(app.pages.tabs()[-1])
        app.channel_list.selection_set("heave_r")
        app.select_channel()
        original = app.collect()["channels"]
        base = Path(__file__).resolve().parents[1] / ".test-tmp"
        base.mkdir(exist_ok=True)
        with TemporaryDirectory(dir=base) as folder:
            path = Path(folder) / "curve.csv"
            with patch("setup_lab.gui.filedialog.askopenfilename", return_value=str(path)), patch.object(app, "error") as error:
                path.write_text("travel_mm,force_n\n-5,-150\n0,0\n10,400\n", encoding="utf-8-sig")
                app.import_force_csv("spring")
                path.write_text("speed_mps,compression_n,rebound_n\n0,0,0\n0.1,120,175\n", encoding="utf-8")
                app.import_force_csv("dampers")
                path.write_text("0,0\n0.15,210\n", encoding="utf-8")
                app.import_force_csv("rebound")
                error.assert_not_called()
                project = app.collect()
                channel = project["channels"]["heave_r"]
                self.assertEqual(channel["spring_kind"], "force")
                self.assertEqual(channel["damper_kind"], "force")
                self.assertEqual(channel["spring"], [[-5, -150], [0, 0], [10, 400]])
                self.assertEqual(channel["compression"], [[0, 0], [.1, 120]])
                self.assertEqual(channel["rebound"], [[0, 0], [.15, 210]])
                self.assertEqual(project["channels"]["heave_f"], original["heave_f"])

                path.write_text("0,0,0\n0.1,999,-50\n", encoding="utf-8")
                app.import_force_csv("dampers")
                error.assert_called_once()
                self.assertEqual(app.collect(), project)

            saved = Path(folder) / "project.json"
            save_project(saved, project)
            self.assertEqual(open_project(saved)[0], project)

    def test_draft_preview_background_run_and_saved_baseline(self):
        p = default_project()
        p["case"].update(duration=.5, start=.1, dt=.004)
        root, app = self.make_app(p)
        full, left_right, front_rear, quarter = (app.analysis_pages[key] for key in ("full", "half_lr", "half_fr", "quarter"))
        self.assertEqual(app.collect(), p)
        app.pages.select(app.pages.tabs()[-1])
        app.tables["compression"].insert("end", "\n0.05\t1000")
        app.channel_list.selection_set("heave_r")
        app.select_channel()
        app.channel_list.selection_set("heave_f")
        app.select_channel()
        self.assertEqual(app.collect()["channels"]["heave_f"]["compression"][-1], [.05, 1000])
        app.preview()
        self.assertTrue(any(isinstance(child, tk.Toplevel) for child in root.winfo_children()))

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
