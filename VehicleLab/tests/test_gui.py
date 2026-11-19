"""Opt-in Tk widget/render tests: set VEHICLE_LAB_GUI_TEST=1 before discovery."""
import os
import copy
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from dataclasses import replace
from vehicle_lab.models import read_project,write_project
from vehicle_lab.workflows import defaults,run_tool


@unittest.skipUnless(os.environ.get("VEHICLE_LAB_GUI_TEST")=="1","Opt-in native window test")
class GuiTests(unittest.TestCase):
    def test_body_motion_preview_toggle_views_and_animation_lifecycle(self):
        from vehicle_lab.gui import App
        app=App();app.busy=True;app.update()
        try:
            for key in ("geometry","roll"):
                with self.subTest(tool=key):
                    panel=app.panels[key];app.notebook.select(panel)
                    options=defaults(key)
                    if key=="geometry":options.update(heave_mm=5.,roll_deg=1.,pitch_deg=.5)
                    result=run_tool(key,app.vehicle,app.geo,options)
                    original=copy.deepcopy(result["segments"])
                    panel.receive(app.revision,result,None,copy.deepcopy(app.vehicle),
                                  {**options,"geometry_snapshot":copy.deepcopy(app.geo)})
                    app.busy=True;app.update()
                    self.assertFalse(panel.body_motion.get())
                    panel.body_motion_check.invoke();app.update()
                    self.assertTrue(panel.body_motion.get())
                    self.assertIn("chassis",[s[2] for s in panel.chart.spec["segments"]])
                    ground=[s for s in panel.chart.spec["segments"] if s[2]=="ground"]
                    self.assertTrue(ground)
                    self.assertTrue(all(a[2]==b[2]==0 for a,b,_ in ground))
                    static=copy.deepcopy(panel.chart.spec["segments"])
                    # Changes to the editable vehicle cannot affect a result preview.
                    old_vehicle=app.vehicle;app.vehicle=replace(app.vehicle,wheelbase=2.)
                    panel.draw_geometry_frame()
                    self.assertEqual(panel.chart.spec["segments"],static)
                    app.vehicle=old_vehicle
                    panel.draw_geometry_frame(.7)
                    self.assertNotEqual(panel.chart.spec["segments"],static)
                    self.assertEqual([s for s in panel.chart.spec["segments"] if s[2]=="ground"],ground)
                    self.assertEqual(result["segments"],original)
                    panel.chart.svg()  # New segment styles must render and export.
                    for _ in range(2):
                        panel.toggle_animation()
                        timer=panel.animation_after
                        self.assertTrue(panel.playing)
                        self.assertIsNotNone(timer)
                        panel.toggle_animation()
                        self.assertFalse(panel.playing)
                        self.assertIsNone(panel.animation_after)
                        self.assertNotIn(timer,app.tk.splitlist(app.tk.call("after","info")))
                        self.assertEqual(panel.chart.spec["segments"],static)
                    panel.plot_select.current(1);panel.show_plot()
                    self.assertEqual(panel.chart.spec["projection"],"Front")
                    self.assertIn("chassis",[s[2] for s in panel.chart.spec["segments"]])
                    panel.plot_select.current(5);panel.show_plot()
                    self.assertNotIn("segments",panel.chart.spec)
                    self.assertIn("disabled",panel.body_motion_check.state())
                    panel.plot_select.current(0);panel.show_plot()
                    panel.body_motion_check.invoke()
                    self.assertEqual(panel.chart.spec["segments"],original)
                    panel.body_motion_check.invoke()
                    with patch("vehicle_lab.gui.geom.solve_body_pose",side_effect=ValueError("test unreachable pose")):
                        panel.toggle_animation()
                    self.assertFalse(panel.playing)
                    self.assertIsNone(panel.animation_after)
                    self.assertIn("姿態無法求解",panel.animation_note.get())
                    panel.body_motion_check.invoke()
                    self.assertNotIn("無法求解",panel.animation_note.get())
        finally:app.destroy()

    def test_fsae_editors_and_motor_csv_preview(self):
        from vehicle_lab.gui import App,VehicleEditor,GeometryEditor
        app=App();app.busy=True;app.update()
        try:
            editor=VehicleEditor(app);app.update()
            keys=editor.tree.get_children()
            self.assertIn("motor_curve_csv",keys)
            self.assertIn("reduction_front",keys)
            self.assertIn("reduction_rear",keys)
            self.assertIn("unsprung_front",keys)
            self.assertIn("unsprung_rear",keys)
            self.assertFalse(any(key in keys for key in ("unsprung_fl","unsprung_fr","unsprung_rl","unsprung_rr")))
            self.assertNotIn("gears",keys)
            self.assertNotIn("differential",keys)
            preview=editor.preview_motor_curve();app.update()
            self.assertTrue(preview.chart.primitives)
            preview.destroy();editor.destroy()
            hardpoints=GeometryEditor(app);app.update()
            self.assertFalse(hasattr(hardpoints,"types"))
            self.assertEqual(len(hardpoints.coordinate_rows()),32)
            hardpoints.destroy()
        finally:app.destroy()

    def test_setting_pages_have_one_instance_and_can_reopen(self):
        from vehicle_lab.gui import App
        app=App();app.busy=True;app.update()
        try:
            for key,open_page in (("vehicle",app.open_vehicle_editor),("geometry",app.open_geometry_editor),
                    ("rate",app.open_rate_editor),("track",app.open_track_editor),("help",app.help)):
                with self.subTest(page=key):
                    first=open_page();app.update()
                    first.withdraw()
                    second=open_page();app.update()
                    self.assertIs(first,second)
                    self.assertEqual(first.state(),"normal")
                    self.assertIs(app._dialogs[key],first)
                    first.destroy();app.update()
                    self.assertNotIn(key,app._dialogs)
                    reopened=open_page();app.update()
                    self.assertIsNot(reopened,first)
                    reopened.destroy()
            # The same hardpoint page is shared by the top bar and tool panels.
            hardpoints=app.open_geometry_editor()
            from tkinter import ttk
            button=next(w for w in app.panels["geometry"].inputs.winfo_children()
                if isinstance(w,ttk.Button) and w.cget("text")=="編輯懸吊 / 轉向硬點")
            button.invoke()
            self.assertIs(app._dialogs["geometry"],hardpoints)
        finally:app.destroy()

    def test_parameter_and_preview_windows_preserve_pending_edits(self):
        from vehicle_lab.gui import App
        app=App();app.busy=True;app.update()
        try:
            editor=app.open_vehicle_editor();app.update()
            def edit(key):
                editor.tree.see(key);app.update()
                return editor.edit(SimpleNamespace(y=editor.tree.bbox(key)[1]+3))
            parameter=edit("name")
            parameter.text.delete("1.0","end");parameter.text.insert("1.0","Pending vehicle name")
            self.assertIs(edit("curb_mass"),parameter)
            self.assertEqual(parameter.parameter_key,"name")
            self.assertEqual(parameter.text.get("1.0","end").strip(),"Pending vehicle name")
            self.assertIs(app.open_vehicle_editor(),editor)
            preview=editor.preview_motor_curve()
            self.assertIs(editor.preview_motor_curve(),preview)
            preview.destroy()
            self.assertIsNot(editor.preview_motor_curve(),preview)
            rate=app.open_rate_editor(editor);rate.value.set("3.75")
            self.assertIs(app.open_rate_editor(),rate)
            self.assertEqual(rate.value.get(),"3.75")
            # Child dialogs must not outlive their owner or keep stale handles.
            editor.destroy();app.update()
            self.assertFalse(parameter.winfo_exists())
            self.assertFalse(rate.winfo_exists())
            self.assertNotIn("vehicle",app._dialogs)
            self.assertNotIn("rate",app._dialogs)
            fresh=app.open_vehicle_editor();app.update()
            self.assertIsNot(fresh,editor)
            self.assertEqual(fresh.data["name"],app.vehicle.name)
            self.assertIsNot(app.open_rate_editor(fresh),rate)
        finally:app.destroy()

    def test_project_file_dialog_reentry_and_cancellation(self):
        from vehicle_lab.gui import App
        app=App();app.busy=True;app.update()
        try:
            def reenter(**options):
                app.load();app.save()
                return ""
            with patch("vehicle_lab.gui.filedialog.askopenfilename",side_effect=reenter) as opening, \
                    patch("vehicle_lab.gui.filedialog.asksaveasfilename",return_value="") as saving:
                app.load();app.load()
                self.assertEqual(opening.call_count,2)
                saving.assert_not_called()
                self.assertFalse(app._project_dialog_active)
                app.save();saving.assert_called_once()
            with patch("vehicle_lab.gui.filedialog.asksaveasfilename",side_effect=RuntimeError("dialog error")):
                with self.assertRaises(RuntimeError):app.save()
            self.assertFalse(app._project_dialog_active)
            with patch("vehicle_lab.gui.filedialog.asksaveasfilename",return_value="") as saving:
                app.save();saving.assert_called_once()
        finally:app.destroy()

    def test_input_snapshot_commits_only_with_successful_result(self):
        from vehicle_lab.gui import App
        app=App();app.busy=True;app.update()
        try:
            panel=app.panels["aero"]
            options=defaults("aero");old_vehicle=app.vehicle
            old_result=run_tool("aero",old_vehicle,app.geo,options)
            panel.receive(app.revision,old_result,None,old_vehicle,options)
            app.vehicle=replace(old_vehicle,cd=.8)
            with patch("vehicle_lab.gui.threading.Thread"):
                panel.calculate()
            self.assertIs(panel.result,old_result)
            self.assertIs(panel.result_vehicle,old_vehicle)
            with patch("vehicle_lab.gui.messagebox.showerror"):
                panel.receive(app.revision,None,ValueError("simulated failure"))
            self.assertIs(panel.result_vehicle,old_vehicle)
            self.assertIs(panel.result,old_result)
            new_result=run_tool("aero",app.vehicle,app.geo,options)
            panel.receive(app.revision,new_result,None,app.vehicle,options)
            self.assertIs(panel.result,new_result)
            self.assertIs(panel.result_vehicle,app.vehicle)
        finally:app.destroy()

    def test_legacy_project_preserves_cli_settings(self):
        from vehicle_lab.gui import App
        app=App();app.busy=True;app.update()
        try:
            with tempfile.TemporaryDirectory() as directory:
                source=Path(directory)/"legacy.json";saved=Path(directory)/"saved.json"
                legacy={"step":{"step_mm":30},"damper":{"end":.4},
                        "distribution":{"radius":28},"rigid":{"ax_g":.8}}
                write_project(source,app.vehicle,app.geo,{**legacy,"aero":{"speed_kmh":90}})
                with patch.object(app,"project_file_dialog",side_effect=[str(source),str(saved)]), \
                     patch("vehicle_lab.gui.messagebox.showerror") as show_error:
                    app.load();app.save()
                    show_error.assert_not_called()
                _,_,settings=read_project(saved)
                self.assertEqual(settings["aero"]["speed_kmh"],90)
                self.assertEqual({key:settings[key] for key in legacy},legacy)
        finally:app.destroy()

    def test_five_panels_render_and_export(self):
        from vehicle_lab.gui import App,VehicleEditor,GeometryEditor,RateEditor,TrackEditor
        app=App()
        errors=[]
        app.report_callback_exception=lambda kind,value,tb:errors.append(str(value))
        app.busy=True  # Prevent automatic workers; these tests deliver results explicitly.
        app.update()
        try:
            tabs=[app.notebook.tab(tab,"text") for tab in app.notebook.tabs()]
            self.assertEqual(tabs,["懸吊幾何","轉向","側傾","空氣力學","圈速"])
            self.assertIs(app.active(),app.panels["geometry"])
            with tempfile.TemporaryDirectory() as directory:
                for key,panel in app.panels.items():
                    app.notebook.select(panel)
                    options=defaults(key);result=run_tool(key,app.vehicle,app.geo,options,app.track)
                    panel.result_vehicle=app.vehicle;panel.result_options=options
                    panel.receive(app.revision,result,None);app.busy=True
                    app.update()
                    for index in range(len(panel.plot_specs)):
                        panel.plot_select.current(index);panel.show_plot();app.update()
                        svg=panel.chart.svg()
                        self.assertIn("<svg",svg);self.assertGreater(len(svg),1000)
                    if key in ("geometry","lap"):
                        panel.plot_select.current(0);panel.show_plot();app.update()
                        panel.chart.png(Path(directory)/(key+".png"))
                        self.assertGreater((Path(directory)/(key+".png")).stat().st_size,1000)
                for editor in (VehicleEditor,GeometryEditor,RateEditor,TrackEditor):
                    dialog=editor(app);app.update();dialog.destroy()
                self.assertEqual(errors,[])
        finally:app.destroy()
