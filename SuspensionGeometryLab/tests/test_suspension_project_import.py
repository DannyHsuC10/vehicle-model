import copy
import json
from pathlib import Path
import tempfile
import unittest

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.project import default_project, load_project, save_project
from suspension_geometry_lab.project_import import fill_import_project, read_import_project


SIZING_APPLICATION = "VehicleInitialSizingLab-DynamicsExport"


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

    def test_imports_sizing_sections_and_saves_a_separate_geometry_project(self):
        source = {
            "application": SIZING_APPLICATION, "schema_version": 2, "name": "Sizing",
            "vehicle": {"m": 320, "lf": .75, "lr": .78, "tf": 1.25, "tr": 1.24, "h_cg": .3,
                        "gravity": 9.81, "free_radius": .2, "brake_front": .75, "drive_front": 0},
            "dynamics": {"anti_front": .1, "anti_rear": .12, "tire_f_k": 80000, "tire_r_k": 80000,
                         "body_twist_c": None},
            "geometry": {"h_rc_f": .027, "h_rc_r": .033, "fvsa_f": .97, "fvsa_r": .98,
                         "kpi_f_deg": 8, "kpi_r_deg": 9, "caster_f_deg": 6, "caster_r_deg": 0,
                         "clearance_steer_deg": 30, "bump_mm": 26, "droop_mm": 25, "steer_deg": 29},
        }
        draft, fields = self.read(source)
        before = self.path.read_bytes()
        missing = {field.location for field in fields}
        self.assertEqual(len(missing), 21)
        self.assertTrue({"vehicle.tire_width", "front.svsa", "rear.lower_mount_x", "analysis.samples"} <= missing)
        completed = fill_import_project(draft, fields, [".195" if field.location == "vehicle.tire_width" else ""
                                                        for field in fields])
        engine = Engine(completed)
        expected_vehicle = {"mass": 320, "wheelbase": 1.53, "cg_to_front": .75, "cg_height": .3,
                            "track_front": 1.25, "track_rear": 1.24, "gravity": 9.81, "free_radius": .2,
                            "clearance_steer_deg": 30, "brake_bias_front": .75, "drive_force_front": 0,
                            "tire_stiffness": 80000, "tire_width": .195}
        self.assertEqual({key: engine.project["vehicle"][key] for key in expected_vehicle}, expected_vehicle)
        for axle, expected in (("front", (.027, .97, 8, 6, .1)), ("rear", (.033, .98, 9, 0, .12))):
            keys = ("rc_height", "fvsa", "kpi_deg", "caster_deg", "anti")
            self.assertEqual(tuple(engine.project[axle][key] for key in keys), expected)
        self.assertEqual(tuple(engine.project["analysis"][key] for key in ("bump_mm", "droop_mm", "steer_deg")),
                         (26, 25, 29))
        saved = self.path.with_name("geometry.json")
        save_project(saved, engine.project)
        self.assertEqual(load_project(saved), engine.project)
        self.assertEqual(self.path.read_bytes(), before)

    def test_imports_sizing_v1_and_preserves_missing_geometry(self):
        source = {
            "application": SIZING_APPLICATION, "schema_version": 1, "name": "Legacy Sizing",
            "parameters": {"full": {"m": 320, "lf": .75, "lr": .78, "h_rc_f": .027, "h_rc_r": .033,
                                    "dive_front": .1, "lift_rear": .12, "tire_FL_k": 80000, "tire_RR_k": 80000}},
            "geometry": {"vehicle": {"free_radius": .2, "gravity": 9.81, "clearance_steer_deg": 30},
                         "front": {"fvsa": .97, "kpi_deg": 8, "caster_deg": None},
                         "rear": {"fvsa": .98, "kpi_deg": 9, "caster_deg": 0},
                         "analysis": {"bump_mm": 26, "droop_mm": 25, "steer_deg": 29}},
        }
        draft, fields = self.read(source)
        missing = {field.location for field in fields}
        self.assertIn("front.caster_deg", missing)
        self.assertNotIn("rear.caster_deg", missing)
        completed = Engine(fill_import_project(draft, fields, [""] * len(fields))).project
        self.assertEqual(completed["vehicle"]["tire_stiffness"], 80000)
        self.assertEqual(completed["vehicle"]["free_radius"], .2)
        self.assertEqual(completed["front"]["rc_height"], .027)
        self.assertEqual(completed["rear"]["fvsa"], .98)
        self.assertEqual(completed["front"]["anti"], .1)
        self.assertEqual(completed["rear"]["anti"], .12)
        self.assertEqual(completed["analysis"]["steer_deg"], 29)

    def test_sizing_requests_shared_stiffness_if_axles_differ_and_rejects_malformed_values(self):
        source = {"application": SIZING_APPLICATION, "schema_version": 2,
                  "dynamics": {"tire_f_k": 80000, "tire_r_k": 81000}}
        _, fields = self.read(source)
        self.assertIn("vehicle.tire_stiffness", {field.location for field in fields})
        for change, error in (({"schema_version": 4}, "schema_version"),
                              ({"geometry": []}, "geometry"),
                              ({"vehicle": {"lf": .75, "lr": "bad"}}, "lr"),
                              ({"dynamics": {"tire_f_k": "bad"}}, "輪胎剛性")):
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, error):
                self.read(source | change)

    def test_rejects_foreign_format_and_does_not_replace_invalid_nonempty_input(self):
        with self.assertRaises(ValueError):
            self.read({"schema_version": 1, "parameters": {}})
        source = default_project()
        source["vehicle"]["mass"] = "heavy"
        draft, missing = self.read(source)
        self.assertEqual(missing, [])
        with self.assertRaisesRegex(ValueError, r"vehicle\.mass"):
            Engine(draft)
