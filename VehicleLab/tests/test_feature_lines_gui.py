"""Opt-in construction-line widget and export integration tests."""
import copy
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from vehicle_lab.linalg import cross, norm, sub
from vehicle_lab.workflows import defaults, run_tool


@unittest.skipUnless(os.environ.get("VEHICLE_LAB_GUI_TEST")=="1","Opt-in native window test")
class FeatureLineGuiTests(unittest.TestCase):
    def setUp(self):
        from vehicle_lab.gui import App
        self.app=App();self.app.busy=True;self.app.update()
        self.addCleanup(self.app.destroy)

    def receive(self,key):
        app=self.app;panel=app.panels[key]
        app.notebook.select(panel)
        options=defaults(key)
        result=run_tool(key,app.vehicle,app.geo,options)
        panel.receive(app.revision,result,None,copy.deepcopy(app.vehicle),
                      {**options,"geometry_snapshot":copy.deepcopy(app.geo)})
        app.busy=True;app.update()
        return panel,result

    def assert_kingpins_follow_uprights(self,panel):
        uprights=[(a,b) for a,b,kind in panel.chart.spec["segments"] if kind=="upright"]
        kingpins=[(a,b) for a,b,kind in panel.chart.spec["overlays"] if kind=="kingpin"]
        self.assertTrue(kingpins)
        for a,b in kingpins:
            self.assertTrue(any(
                norm(cross(sub(a,u),sub(v,u)))<1e-7 and
                norm(cross(sub(b,u),sub(v,u)))<1e-7
                for u,v in uprights),"Kingpin overlay must follow a displayed upright")

    def test_independent_toggles_preserve_view_and_physical_geometry(self):
        for key in ("geometry","steering","roll"):
            with self.subTest(tool=key):
                panel=self.app.panels[key]
                self.assertIn("disabled",panel.projection_lines_check.state())
                self.assertIn("disabled",panel.kingpin_lines_check.state())
                panel,result=self.receive(key)
                original=copy.deepcopy(result["segments"])
                original_revision=self.app.revision
                self.assertTrue(panel.vars["projection_lines"].get())
                self.assertTrue(panel.vars["kingpin_lines"].get())
                for view_index,view in enumerate(("Iso","Front","Rear","Side","Top")):
                    panel.plot_select.current(view_index);panel.show_plot()
                    self.assertEqual(panel.chart.spec["projection"],view)
                    self.assertTrue(panel.chart.spec["overlays"])
                    kinds={s[2] for s in panel.chart.spec["overlays"]}
                    self.assertIn("kingpin",kinds)
                    self.assert_kingpins_follow_uprights(panel)
                    if view=="Top":self.assertEqual(kinds,{"kingpin"})
                    else:self.assertIn("projection_upper",kinds)
                panel.plot_select.current(1);panel.show_plot()
                viewport=list(panel.chart.view)
                both=copy.deepcopy(panel.chart.spec["overlays"])
                with patch.object(panel,"calculate") as calculate:
                    panel.projection_lines_check.invoke()
                    self.assertEqual({s[2] for s in panel.chart.spec["overlays"]},{"kingpin"})
                    panel.kingpin_lines_check.invoke()
                    self.assertEqual(panel.chart.spec["overlays"],[])
                    panel.projection_lines_check.invoke()
                    self.assertTrue(panel.chart.spec["overlays"])
                    self.assertNotIn("kingpin",{s[2] for s in panel.chart.spec["overlays"]})
                    panel.kingpin_lines_check.invoke()
                    calculate.assert_not_called()
                self.assertEqual(panel.chart.spec["overlays"],both)
                self.assertEqual(panel.chart.view,viewport)
                self.assertEqual(panel.chart.spec["segments"],original)
                self.assertEqual(result["segments"],original)
                self.assertEqual(self.app.revision,original_revision)
                panel.plot_select.current(5);panel.show_plot()
                self.assertNotIn("segments",panel.chart.spec)
                self.assertIn("disabled",panel.projection_lines_check.state())
                self.assertIn("disabled",panel.kingpin_lines_check.state())

    def test_snapshot_animation_and_body_motion_keep_overlays_current(self):
        panel,result=self.receive("geometry")
        original=copy.deepcopy(result["segments"])
        original_overlays=copy.deepcopy(panel.chart.spec["overlays"])
        self.app.geo["corners"]["FL"]["UBJ"][2]+=.02
        self.app.vehicle.track_front+=.1
        panel.kingpin_lines_check.invoke();panel.kingpin_lines_check.invoke()
        self.assertEqual(panel.chart.spec["overlays"],original_overlays)
        panel.draw_geometry_frame(.7)
        self.assertNotEqual(panel.chart.spec["overlays"],original_overlays)
        self.assertNotEqual(panel.chart.spec["segments"],original)
        self.assert_kingpins_follow_uprights(panel)
        self.assertEqual(result["segments"],original)
        for body_motion in (False,True):
            panel.body_motion.set(body_motion);panel.motion_mode_changed()
            panel.toggle_animation()
            self.assertTrue(panel.playing)
            timer=panel.animation_after
            self.assertIsNotNone(timer)
            panel.projection_lines_check.invoke()
            self.assertTrue(panel.playing)
            self.assertEqual(panel.animation_after,timer)
            self.assertEqual({s[2] for s in panel.chart.spec["overlays"]},{"kingpin"})
            self.assert_kingpins_follow_uprights(panel)
            panel.projection_lines_check.invoke()
            self.assertTrue(panel.playing)
            self.assertIn("projection_upper",{s[2] for s in panel.chart.spec["overlays"]})
            if body_motion:self.assertIn("chassis",{s[2] for s in panel.chart.spec["segments"]})
            panel.toggle_animation()
            self.assertFalse(panel.playing)
            self.assertIsNone(panel.animation_after)

    def test_export_styles_and_project_display_settings_roundtrip(self):
        panel,_=self.receive("geometry")
        panel.plot_select.current(1);panel.show_plot()
        self.assertIn("stroke-dasharray=",panel.chart.svg())
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory)
            if importlib.util.find_spec("PIL"):
                path=directory/"construction.png"
                panel.chart.png(path)
                self.assertGreater(path.stat().st_size,1000)
            panel.projection_lines_check.invoke();panel.kingpin_lines_check.invoke()
            self.assertNotIn("stroke-dasharray=",panel.chart.svg())
            path=directory/"project.json"
            with patch("vehicle_lab.gui.filedialog.asksaveasfilename",return_value=str(path)):
                self.app.save()
            self.assertTrue(path.exists())
            panel.vars["projection_lines"].set(True);panel.vars["kingpin_lines"].set(True)
            with patch("vehicle_lab.gui.filedialog.askopenfilename",return_value=str(path)), \
                    patch.object(panel,"calculate"):
                self.app.load()
            self.assertFalse(panel.vars["projection_lines"].get())
            self.assertFalse(panel.vars["kingpin_lines"].get())
