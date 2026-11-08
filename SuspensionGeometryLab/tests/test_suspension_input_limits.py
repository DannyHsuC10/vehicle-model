"""Input guardrails apply equally to drafts, files and committed projects."""
import copy
import gc
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.input_limits import FIELD_LIMITS
from suspension_geometry_lab.project import FIELDS, default_project, load_project, save_project, validate, MOUNT_X_FIELDS


class InputLimitTests(unittest.TestCase):
    def test_edited_values_snap_to_steps_and_legacy_values_are_preserved(self):
        from types import SimpleNamespace
        from suspension_geometry_lab.gui import read_entries

        project = default_project()
        project["front"]["caster_deg"] = 5.123
        entries = {field: SimpleNamespace(get=lambda text=text: text)
                   for field, text in {("vehicle", "track_front"): "1.253",
                                       ("front", "anti"): "0.225",
                                       ("front", "scrub_radius"): "-0.0015",
                                       ("front", "caster_deg"): "5.123"}.items()}
        result = read_entries(project, entries)
        self.assertEqual(result["vehicle"]["track_front"], 1.255)
        self.assertEqual(result["front"]["anti"], .25)
        self.assertEqual(result["front"]["scrub_radius"], -.002)
        self.assertEqual(result["front"]["caster_deg"], 5.123)

    def test_defaults_are_equal_drive_and_keep_the_original_static_geometry(self):
        project = default_project()
        self.assertEqual(project["vehicle"]["drive_force_front"], .5)
        engine = Engine(project)
        previous = copy.deepcopy(project)
        previous["vehicle"]["drive_force_front"] = 0.
        baseline = Engine(previous)
        self.assertEqual(len(engine.scene.lines), len(baseline.scene.lines))
        for actual, expected in zip(engine.scene.lines, baseline.scene.lines):
            np.testing.assert_allclose(actual.points, expected.points, rtol=0, atol=1e-12)
        for axle in ("front", "rear"):
            self.assertAlmostEqual(engine.derived_parameters[axle]["svsa"], project[axle]["svsa"])

    def test_every_editable_field_rejects_both_lower_and_upper_overflow(self):
        for group, fields in FIELDS.items():
            self.assertTrue(set(fields) <= set(FIELD_LIMITS[group]))
            for key in fields:
                limit = FIELD_LIMITS[group][key]
                for value in (limit.low-1, limit.high+1):
                    with self.subTest(group=group, key=key, value=value):
                        project = default_project()
                        project[group][key] = value
                        with self.assertRaisesRegex(ValueError, "允許範圍|有限數值"):
                            validate(project)

    def test_signed_targets_and_exact_closed_endpoints_remain_available(self):
        project = default_project()
        project["front"].update(anti=-.05, scrub_radius=-.02, mechanical_trail=-.01)
        project["rear"]["anti"] = -.05
        project["analysis"].update(bump_mm=0, droop_mm=200, samples=51,
                                   caster_min_deg=-45, caster_max_deg=45, steer_deg=80)
        validate(project)
        Engine(project)
        for drive in (0., 1.):
            project["vehicle"].update(drive_force_front=drive, brake_bias_front=drive)
            validate(project)
        for group, key, value in (("analysis", "steer_deg", 0), ("analysis", "samples", 5.5)):
            bad = default_project()
            bad[group][key] = value
            with self.assertRaises(ValueError):
                validate(bad)

    def test_related_parameters_reject_individually_in_range_but_impossible_combinations(self):
        changes = ({"cg_to_front": 1.53}, {"ground_clearance": .4},
                   {"rim_inner_diameter": .5}, {"rim_inner_diameter": .15, "rim_gap": .1},
                   {"mass": 2000, "tire_stiffness": 10000})
        for values in changes:
            with self.subTest(values=values):
                project = default_project()
                project["vehicle"].update(values)
                with self.assertRaises(ValueError):
                    validate(project)
        for group, values in (("front", {"upper_mount_x": .625}),
                              ("analysis", {"bump_mm": 0, "droop_mm": 0}),
                              ("analysis", {"caster_min_deg": 15, "caster_max_deg": 15})):
            project = default_project()
            project[group].update(values)
            with self.assertRaises(ValueError):
                validate(project)

    def test_loading_and_saving_invalid_values_never_overwrites_a_good_file(self):
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"design.json"
            good = default_project()
            save_project(path, good)
            before = path.read_bytes()
            bad = copy.deepcopy(good)
            bad["vehicle"]["mass"] = 10000
            with self.assertRaises(ValueError):
                save_project(path, bad)
            self.assertEqual(path.read_bytes(), before)
            path.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_project(path)

    def test_hardpoints_have_no_numeric_bounds_but_still_reject_nonfinite_and_wrong_side(self):
        project = default_project()
        points = hardpoints.from_scene(Engine(project).scene)
        project["hardpoints"] = {"enabled": True, "corners": points}
        validate(project)
        coordinates = [20., 5., -4.]
        hardpoints.set_point(points, "FL", "UBJ", coordinates)
        self.assertEqual(points["FL"]["UBJ"], coordinates)
        self.assertEqual(points["FR"]["UBJ"], [20., -5., -4.])
        validate(project)
        before = copy.deepcopy(points)
        for bad in ([float("nan"), 5., -4.], [20., float("inf"), -4.], [20., -5., -4.]):
            with self.assertRaises(ValueError):
                hardpoints.set_point(points, "FL", "UBJ", bad)
            self.assertEqual(points, before)
        project["arm_mount_x"]["front"]["upper_front_x"] = 11.
        validate(project)
        root = Path(__file__).resolve().parents[1]/".test-tmp"
        root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root) as tmp:
            path = Path(tmp)/"unbounded-hardpoints.json"
            save_project(path, project)
            self.assertEqual(load_project(path), project)

    def test_default_mounts_are_explicit_null_and_use_geometry_extrema(self):
        project = default_project()
        engine = Engine(project)
        for axle in ("front", "rear"):
            self.assertEqual(project["arm_mount_x"][axle], {key: None for key in MOUNT_X_FIELDS})
            for key, check in engine.scene.geometry["mount_position_checks"][axle].items():
                self.assertIsNone(check["requested_x"])
                bound = "limit_max_x" if "front_x" in key else "limit_min_x"
                self.assertEqual(check["actual_x"], check[bound])
                self.assertFalse(check["fallback"])

    def test_name_length_is_checked(self):
        for value in ("", "  ", "車"*129):
            project = default_project()
            project["name"] = value
            with self.assertRaises(ValueError):
                validate(project)


@unittest.skipUnless(os.environ.get("SUSPENSION_GEOMETRY_LAB_GUI_TEST") == "1", "Opt-in native window tests")
class InputLimitGuiTests(unittest.TestCase):
    def setUp(self):
        from suspension_geometry_lab.gui import App
        project = default_project()
        project["analysis"]["samples"] = 5
        self.app = App(project)
        self.app.update()

    def tearDown(self):
        self.app.dirty = False
        self.app.close()
        self.app.worker_thread.join(timeout=5)
        self.assertFalse(self.app.worker_thread.is_alive())
        self.app = None
        gc.collect()

    def test_all_numeric_boxes_show_ranges_and_invalid_drafts_cannot_commit(self):
        from suspension_geometry_lab.gui import ParameterEditor
        editor = ParameterEditor(self.app)
        self.assertTrue(all(var.get() == "" for var in editor.mount_x_entries.values()))
        for field, entry in editor.entry_widgets.items():
            self.assertIn("允許範圍", entry.range_label.cget("text"), field)
        for panel in self.app.panels.values():
            for field, entry in panel.entry_widgets.items():
                self.assertIn("允許範圍", entry.range_label.cget("text"), field)
        original, revision = copy.deepcopy(self.app.project), self.app.revision
        var = editor.entries[("vehicle", "mass")]
        var.set("10000")
        entry = editor.entry_widgets[("vehicle", "mass")]
        self.assertEqual(entry.cget("style"), "Invalid.TEntry")
        self.assertIn("請修正輸入", entry.range_label.cget("text"))
        with patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
            editor.apply()
        error.assert_called_once()
        self.assertEqual((self.app.project, self.app.revision), (original, revision))
        var.set("330")
        self.assertEqual(entry.cget("style"), "TEntry")
        editor.apply()
        self.assertEqual(self.app.project["vehicle"]["mass"], 330)
        panel = self.app.panels["front"]
        panel.entries[("vehicle", "track_front")].set("-1")
        with patch("suspension_geometry_lab.gui.messagebox.showerror") as error:
            panel.apply()
        error.assert_called_once()
        self.assertEqual(self.app.project["vehicle"]["track_front"], original["vehicle"]["track_front"])

    def test_bad_mount_import_is_transactional(self):
        from suspension_geometry_lab.project import mount_x_settings
        original, revision = copy.deepcopy(self.app.project), self.app.revision
        bad = copy.deepcopy(original)
        bad["arm_mount_x"] = mount_x_settings(bad)
        bad["arm_mount_x"]["front"]["upper_front_x"] = .4
        with self.assertRaisesRegex(ValueError, "超過極限"):
            self.app.apply_project(bad)
        self.assertEqual((self.app.project, self.app.revision), (original, revision))


if __name__ == "__main__":
    unittest.main()
