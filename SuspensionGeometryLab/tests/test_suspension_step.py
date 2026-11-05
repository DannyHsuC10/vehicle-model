"""Verify the STEP wireframe's connectivity, units and source geometry."""
from pathlib import Path
import re
import tempfile
import unittest

import numpy as np

from suspension_geometry_lab import hardpoints
from suspension_geometry_lab.engine import Engine
from suspension_geometry_lab.export import write_step
from suspension_geometry_lab.project import default_project


class StepTests(unittest.TestCase):
    def export_curves(self, engine, page, state=None, **options):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "懸吊.step"
            write_step(path, engine, page, state, **options)
            data = path.read_text(encoding="ascii")
        self.assertTrue(data.startswith("ISO-10303-21;\n"))
        self.assertTrue(data.endswith("END-ISO-10303-21;\n"))
        self.assertIn("SI_UNIT(.MILLI.,.METRE.)", data)
        self.assertIn("GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION", data)
        entities = dict(re.findall(r"^(#\d+)=(.+);$", data, re.MULTILINE))
        points = {ref: [float(x) for x in match[1].split(",")]
                  for ref, value in entities.items()
                  if (match := re.fullmatch(r"CARTESIAN_POINT\('',\(([^)]+)\)\)", value))}
        curve_set, = [match[1].split(",") for value in entities.values()
                      if (match := re.fullmatch(r"GEOMETRIC_CURVE_SET\('',\(([^)]+)\)\)", value))]
        curves = []
        for ref in curve_set:
            match = re.fullmatch(r"POLYLINE\('([^']+)',\(([^)]+)\)\)", entities[ref])
            vertices = np.array([points[point] for point in match[2].split(",")])
            self.assertEqual(vertices.shape[1], 3)
            self.assertTrue(np.all(np.any(np.diff(vertices, axis=0) != 0, axis=1)))
            curves.append((match[1], vertices))
        return curves

    def test_current_fv_and_sv_poses_export_in_vehicle_coordinates(self):
        engine = Engine(default_project())
        for page in ("front", "rear", "sv"):
            with self.subTest(page=page):
                state = engine.sv_pose(7, .5) if page == "sv" else engine.fv[page].pose(7, .5)
                curves = self.export_curves(engine, page, state, show_construction=False)
                expected = [line for line in state["lines"] if line["fit"] and not line["construction"]]
                self.assertEqual([name for name, _ in curves], [line["name"] for line in expected])
                for (_, xyz), line in zip(curves, expected):
                    u, z = np.asarray(line["points"]).T
                    if page == "sv":
                        metres = np.column_stack((-engine.car.l-u, np.zeros_like(u), z))
                    else:
                        x = 0 if page == "front" else -engine.car.l
                        metres = np.column_stack((np.full_like(u, x), -u, z))
                    np.testing.assert_allclose(xyz, metres*1000, atol=1e-8, rtol=0)
                with_references = self.export_curves(engine, page, state)
                self.assertEqual([name for name, _ in with_references].count("force"),
                                 sum(line["construction"] for line in state["lines"]))

    def test_full_vehicle_links_use_parameter_mounts_and_edited_hardpoints(self):
        project = default_project()
        project["arm_mount_x"]["front"].update(upper_front_x=.2, upper_rear_x=-.2)
        engine = Engine(project)
        points = hardpoints.from_scene(engine.scene)
        self.assertAlmostEqual(points["FR"]["UCA_F"][0], .2)
        for direct in (False, True):
            with self.subTest(hardpoint_mode=direct):
                if direct:
                    x, y, z = points["FR"]["UCA_F"]
                    hardpoints.set_point(points, "FR", "UCA_F", (x+.01, y, z))
                    project["hardpoints"] = {"enabled": True, "corners": points}
                    engine = Engine(project)
                curves = dict(self.export_curves(engine, "static"))
                self.assertEqual(len(curves), 20)
                for corner, p in points.items():
                    for name, a, b in (("upper arm UFI", "UBJ", "UCA_F"),
                                       ("upper arm URI", "UBJ", "UCA_R"),
                                       ("lower arm LFI", "LBJ", "LCA_F"),
                                       ("lower arm LRI", "LBJ", "LCA_R"),
                                       ("upright", "LBJ", "UBJ")):
                        np.testing.assert_allclose(curves[f"{corner} {name}"],
                                                   np.array([p[a], p[b]])*1000, atol=1e-8, rtol=0)

    def test_unsolved_pose_cannot_overwrite_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "existing.step"
            path.write_bytes(b"previous export")
            with self.assertRaises(ValueError):
                write_step(path, Engine(default_project()), "front")
            self.assertEqual(path.read_bytes(), b"previous export")
