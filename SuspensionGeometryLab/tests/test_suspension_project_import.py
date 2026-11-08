import copy
import json
from pathlib import Path
import tempfile
import unittest

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.project import default_project, load_project
from suspension_geometry_lab.project_import import fill_import_project, read_import_project


class ProjectImportTests(unittest.TestCase):
    def setUp(self):
        tmp_root = Path(__file__).resolve().parents[1] / ".test-tmp"
        tmp_root.mkdir(exist_ok=True)
        directory = tempfile.TemporaryDirectory(dir=tmp_root)
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "import.json"

    def read(self, project):
        self.path.write_text(json.dumps(project), encoding="utf-8-sig")
        return read_import_project(self.path)

    def test_reports_absent_null_and_empty_fields_and_preserves_existing_values(self):
        source = default_project()
        source["name"] = "  "
        source["vehicle"].pop("mass")
        source["vehicle"].update(tire_width=None, drive_force_front=0)
        source["rear"]["caster_deg"] = ""
        source.pop("analysis")
        source["arm_mount_x"]["front"].pop("upper_front_x")
        draft, fields = self.read(source)
        before = self.path.read_bytes()
        supplied = {("name",): "補齊範例", ("vehicle", "mass"): "320.5",
                    ("arm_mount_x", "front", "upper_front_x"): "200"}
        expected = set(supplied) | {("vehicle", "tire_width"), ("rear", "caster_deg")}
        expected |= {("analysis", key) for key in default_project()["analysis"]}
        self.assertEqual({field.path for field in fields}, expected)

        completed = fill_import_project(draft, fields, [supplied.get(field.path, "") for field in fields])
        repaired = Engine(completed).project
        self.assertEqual(repaired["name"], "補齊範例")
        self.assertEqual(repaired["vehicle"]["mass"], 320.5)
        self.assertEqual(repaired["vehicle"]["drive_force_front"], 0)
        self.assertEqual(repaired["vehicle"]["tire_width"], default_project()["vehicle"]["tire_width"])
        self.assertEqual(repaired["analysis"], default_project()["analysis"])
        self.assertEqual(repaired["arm_mount_x"]["front"]["upper_front_x"], .2)
        self.assertIsNone(repaired["arm_mount_x"]["front"]["upper_rear_x"])
        self.assertEqual(self.path.read_bytes(), before)
        with self.assertRaises(ValueError):
            load_project(self.path)

    def test_hardpoint_defaults_mirror_existing_coordinates_or_use_baseline(self):
        source = default_project()
        baseline = hardpoints.from_scene(Engine(source).scene)
        points = copy.deepcopy(baseline)
        x, y, z = points["FL"]["UBJ"]
        hardpoints.set_point(points, "FL", "UBJ", [x, y + .0004, z])
        expected = copy.deepcopy(points)
        points["FR"]["UBJ"] = [x]
        points["FL"]["LBJ"][2] = None
        points["FR"]["LBJ"][2] = ""
        points.pop("RR")
        source["hardpoints"] = {"enabled": True, "corners": points}
        draft, fields = self.read(source)
        by_path = {field.path: field for field in fields}
        self.assertEqual(len(fields), 22)
        self.assertEqual(by_path["hardpoints", "corners", "FR", "UBJ", 1].default, -(y + .0004))
        self.assertEqual(by_path["hardpoints", "corners", "FL", "LBJ", 2].default, baseline["FL"]["LBJ"][2])
        completed = fill_import_project(draft, fields, [""] * len(fields))
        self.assertEqual(Engine(completed).project["hardpoints"]["corners"], expected)

    def test_repairs_defaults_using_each_supported_schema_and_keeps_optional_groups_optional(self):
        baseline = default_project()
        for version in range(1, 8):
            with self.subTest(version=version):
                draft, fields = self.read({"schema_version": version})
                completed = Engine(fill_import_project(draft, fields, [""] * len(fields))).project
                self.assertEqual(completed["schema_version"], 7)
                self.assertNotIn("hardpoints", completed)
                self.assertNotIn("arm_mount_x", completed)
                for axle in ("front", "rear"):
                    self.assertAlmostEqual(completed[axle]["anti"], baseline[axle]["anti"])
                    self.assertAlmostEqual(completed[axle]["svsa"], baseline[axle]["svsa"])

    def test_rejects_foreign_format_and_does_not_replace_invalid_nonempty_input(self):
        with self.assertRaises(ValueError):
            self.read({"schema_version": 1, "parameters": {}})
        source = default_project()
        source["vehicle"]["mass"] = "heavy"
        draft, missing = self.read(source)
        self.assertEqual(missing, [])
        with self.assertRaisesRegex(ValueError, r"vehicle\.mass"):
            Engine(draft)
