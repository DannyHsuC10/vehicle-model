import json
import math
from pathlib import Path
import tempfile
import unittest
from dataclasses import replace
from unittest.mock import patch
from vehicle_lab.models import Vehicle,write_project,read_project
from vehicle_lab.geometry import default_geometry,segments
from vehicle_lab.lap import PRESETS,simulate_lap,read_track,track_from_centerline,discretize,acceleration,axle_capacities,lateral_limit,sensitivities
from vehicle_lab.motor import read_motor_curve,motor_forces,max_vehicle_speed
from vehicle_lab.models import G
from vehicle_lab.workflows import NAMES,defaults,run_tool
from vehicle_lab.export import write_csv,write_report,write_stl,write_step


class LapAndIOTests(unittest.TestCase):
    def setUp(self):self.v=Vehicle();self.geo=default_geometry(self.v)

    def test_all_nine_workflows(self):
        for key in NAMES:
            with self.subTest(key=key):
                result=run_tool(key,self.v,self.geo,defaults(key))
                self.assertTrue(result["rows"]);self.assertTrue(result["plots"])

    def test_closed_speed_boundary(self):
        rows,m=simulate_lap(self.v,PRESETS["示範 GP（非真實賽道）"],step=10)
        self.assertAlmostEqual(rows[0]["entry_kmh"],rows[-1]["exit_kmh"],places=8)
        self.assertGreater(m["lap_s"],0)
        self.assertAlmostEqual(sum(r["dt_s"] for r in rows),m["lap_s"])

    def test_open_start_and_motor_ceiling_without_gears(self):
        rows,m=simulate_lap(self.v,PRESETS["直線 75 m（開放起點）"],closed=False,step=2)
        self.assertAlmostEqual(rows[0]["entry_kmh"],0)
        self.assertGreater(m["end_kmh"],0)
        self.assertAlmostEqual(m["distance_m"],75)
        curve=read_motor_curve(self.v.motor_curve_csv)
        self.assertTrue(all(r["front_rpm"]<=curve.max_rpm+1e-5 and r["rear_rpm"]<=curve.max_rpm+1e-5 for r in rows))
        self.assertFalse(any(key in rows[0] for key in ("gear","rpm","upshift")))
        self.assertNotIn("shifts",m)

    def test_more_motor_torque_faster_straight(self):
        options=dict(track=PRESETS["直線 75 m（開放起點）"],closed=False,step=2)
        _,a=simulate_lap(self.v,**options,torque_scale=.8)
        _,b=simulate_lap(self.v,**options,torque_scale=1.2)
        self.assertLess(b["lap_s"],a["lap_s"])

    def test_more_grip_faster_circle(self):
        course=PRESETS["定圓 9.125 m"]
        _,a=simulate_lap(self.v,course,step=2)
        _,b=simulate_lap(replace(self.v,mu_lat=self.v.mu_lat*1.2,mu_long=self.v.mu_long*1.2),course,step=2)
        self.assertLess(b["lap_s"],a["lap_s"])

    def test_lap_reads_one_immutable_motor_curve(self):
        curve=read_motor_curve(self.v.motor_curve_csv)
        with patch("vehicle_lab.lap.read_motor_curve",return_value=curve) as reader:
            rows,summary=simulate_lap(self.v,PRESETS["定圓 9.125 m"],step=2)
        self.assertEqual(reader.call_count,1)
        self.assertEqual(summary["motor_csv_sha256"],curve.sha256)
        self.assertEqual(summary["motor_csv_rows"],len(curve.rpm))

    def test_front_rear_reduction_difference_respects_both_motor_ceilings(self):
        v=replace(self.v,reduction_front=10,reduction_rear=16,tire_diameter_front=.44,tire_diameter_rear=.46)
        curve=read_motor_curve(v.motor_curve_csv)
        rows,summary=simulate_lap(v,PRESETS["直線 75 m（開放起點）"],closed=False,step=2)
        self.assertLessEqual(summary["top_kmh"],max_vehicle_speed(v,curve)*3.6+1e-5)
        for row in rows:
            self.assertLessEqual(max(row["front_rpm"],row["rear_rpm"]),curve.max_rpm+1e-5)
            expected=motor_forces(v,row["speed_kmh"]/3.6,curve=curve)
            self.assertAlmostEqual(row["front_wheel_torque_Nm"],expected["front_wheel_torque_Nm"])
            self.assertAlmostEqual(row["rear_wheel_torque_Nm"],expected["rear_wheel_torque_Nm"])

    def test_sensitivity_uses_motor_torque_and_one_map(self):
        curve=read_motor_curve(self.v.motor_curve_csv)
        with patch("vehicle_lab.lap.read_motor_curve",return_value=curve) as reader:
            rows=sensitivities(self.v,PRESETS["示範 Autocross"],.05,.08,step=10)
        self.assertEqual(reader.call_count,1)
        self.assertEqual(rows[1]["change"],"Motor torque +10%")
        self.assertEqual(len(rows),9)

    def test_acceleration_keeps_both_axles_laterally_feasible(self):
        speed=lateral_limit(self.v,1/40,.05,.08,True,80)*.99
        ax=acceleration(self.v,speed,1/40,0,.05,.08,True,1.)
        caps=axle_capacities(self.v,speed,speed*speed/40,ax,.05,.08,True)
        for i in (0,1):
            self.assertGreaterEqual(caps[i][1]+1e-4,self.v.mass*speed*speed/40*(self.v.front_fraction if i==0 else 1-self.v.front_fraction))

    def test_sensitivity_accepts_ui_torque_endpoints_without_clamping(self):
        v=replace(self.v,rolling_coefficient=0)
        course=[{"length_m":20,"radius_m":0,"grade_percent":0}]
        for scale in (.01,5.):
            with self.subTest(torque_scale=scale):
                rows=sensitivities(v,course,.05,.08,step=10,closed=False,aero_on=False,torque_scale=scale)
                self.assertEqual(len(rows),9)
                self.assertLessEqual(rows[1]["lap_s"],rows[0]["lap_s"])
                self.assertGreaterEqual(rows[2]["lap_s"],rows[0]["lap_s"])
                if scale==.01:
                    self.assertLess(rows[1]["lap_s"],rows[0]["lap_s"])
                    self.assertGreater(rows[2]["lap_s"],rows[0]["lap_s"])
        for scale in (0,-1,float("nan"),float("inf")):
            with self.subTest(invalid_scale=scale):
                with self.assertRaisesRegex(ValueError,"Motor torque scale"):
                    simulate_lap(v,course,torque_scale=scale)

    def test_project_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"project.json"
            write_project(path,self.v,self.geo,{"step":defaults("step")})
            v,g,s=read_project(path)
            self.assertEqual(v,self.v);self.assertEqual(g,self.geo)
            self.assertEqual(s["step"],defaults("step"))

    def test_exports_and_track_import(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            write_csv(root/"track.csv",PRESETS["定圓 9.125 m"])
            self.assertEqual(read_track(root/"track.csv"),PRESETS["定圓 9.125 m"])
            result=run_tool("aero",self.v,self.geo,defaults("aero"))
            write_report(root/"report.html","空力",self.v,defaults("aero"),result,"<svg></svg>")
            self.assertIn("列印",(root/"report.html").read_text(encoding="utf-8"))
            lines=segments(self.v,self.geo)
            write_stl(root/"links.stl",lines);write_step(root/"links.step",lines)
            self.assertIn("facet normal",(root/"links.stl").read_text())
            self.assertIn("GEOMETRIC_CURVE_SET",(root/"links.step").read_text())

    def test_centerline_circle(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"circle.csv"
            write_csv(path,[{"x_m":9.125*math.cos(i*2*math.pi/100),"y_m":9.125*math.sin(i*2*math.pi/100)} for i in range(100)])
            course=track_from_centerline(path)
            self.assertAlmostEqual(course[0]["radius_m"],9.125,places=6)

    def test_invalid_course(self):
        with self.assertRaises(ValueError):simulate_lap(self.v,[{"length_m":-10,"radius_m":0}])
        with self.assertRaises(ValueError):simulate_lap(self.v,PRESETS["定圓 9.125 m"],step=0)

    def test_fsae_test_course_dimensions_and_speed_boundaries(self):
        circle=PRESETS["定圓 9.125 m"]
        straight=PRESETS["直線 75 m（開放起點）"]
        self.assertEqual(circle,[{"length_m":2*math.pi*9.125,"radius_m":9.125,"grade_percent":0}])
        self.assertEqual(straight,[{"length_m":75,"radius_m":0,"grade_percent":0}])
        self.assertNotIn("定圓 40 m",PRESETS)
        self.assertNotIn("直線 1000 m（開放起點）",PRESETS)
        for course,length,curvature,closed in ((circle,2*math.pi*9.125,1/9.125,True),(straight,75.,0.,False)):
            with self.subTest(closed=closed):
                cells=discretize(course,2)
                self.assertAlmostEqual(sum(cell["ds"] for cell in cells),length)
                self.assertTrue(all(cell["curvature"]==curvature for cell in cells))
                rows,summary=simulate_lap(self.v,course,step=2,closed=closed)
                self.assertAlmostEqual(summary["distance_m"],length)
                self.assertGreater(summary["lap_s"],0)
                if closed:
                    self.assertAlmostEqual(rows[0]["entry_kmh"],rows[-1]["exit_kmh"])
                else:
                    self.assertEqual(rows[0]["entry_kmh"],0)
                    self.assertGreater(rows[-1]["exit_kmh"],0)


if __name__=="__main__":unittest.main()
