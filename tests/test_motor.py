from dataclasses import FrozenInstanceError,replace
import math
import os
from pathlib import Path
import tempfile
import unittest

from vehicle_lab.models import Vehicle
from vehicle_lab.motor import APP_ROOT,MotorCurve,read_motor_curve,motor_forces,max_vehicle_speed,drive_force_limit


class MotorTests(unittest.TestCase):
    def setUp(self):self.v=Vehicle()

    def test_supplied_table_and_midpoint_interpolation(self):
        curve=read_motor_curve(self.v.motor_curve_csv)
        self.assertEqual(len(curve.rpm),500)
        self.assertEqual(curve.max_rpm,22000)
        self.assertAlmostEqual(curve.torque_at(0),22.30565254)
        mid=(curve.rpm[0]+curve.rpm[1])/2
        self.assertAlmostEqual(curve.torque_at(mid),(curve.torque[0]+curve.torque[1])/2)
        self.assertEqual(len(curve.sha256),64)
        with self.assertRaises(FrozenInstanceError):curve.source="changed"

    def test_table_path_is_independent_of_cwd(self):
        old=Path.cwd()
        with tempfile.TemporaryDirectory() as directory:
            try:
                os.chdir(directory)
                curve=read_motor_curve("input_data/motor_speed_vs_torque.csv")
                self.assertEqual(Path(curve.source),APP_ROOT/"input_data/motor_speed_vs_torque.csv")
            finally:os.chdir(old)

    def test_unsorted_rows_are_sorted_and_bom_is_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"map.csv"
            path.write_text("speed,torque\n2000,0\n0,20\n1000,10\n",encoding="utf-8-sig")
            curve=read_motor_curve(path)
            self.assertEqual(curve.rpm,(0.,1000.,2000.))
            self.assertEqual(curve.torque_at(500),15.)

    def test_invalid_maps_are_rejected(self):
        examples=("rpm,torque\n0,10\n1000,5\n", "speed,torque\n0,10\n0,5\n",
            "speed,torque\n0,10\n", "speed,torque\n0,10\n1000,nan\n",
            "speed,torque\n-1,10\n1000,5\n",
            "speed,torque\n0,10\n1000,inf\n")
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"map.csv"
            for text in examples:
                with self.subTest(text=text):
                    path.write_text(text,encoding="utf-8")
                    with self.assertRaises(ValueError):read_motor_curve(path)
            with self.assertRaisesRegex(ValueError,"Cannot read motor torque CSV"):
                read_motor_curve(Path(directory)/"does_not_exist.csv")

    def test_endpoint_hold_and_no_overspeed_extrapolation(self):
        curve=MotorCurve((100.,1000.),(20.,10.),"test","test")
        self.assertEqual(curve.torque_at(0),20)
        self.assertEqual(curve.torque_at(1000),10)
        self.assertEqual(curve.torque_at(1000.001),0)
        for rpm in (-1,float("nan"),float("inf")):
            with self.assertRaises(ValueError):curve.torque_at(rpm)

    def test_four_identical_motors_multiply_per_wheel_force_by_four(self):
        curve=read_motor_curve(self.v.motor_curve_csv)
        out=motor_forces(self.v,0,curve=curve)
        per_wheel=curve.torque_at(0)*self.v.reduction_front*self.v.reduction_efficiency_front/(self.v.tire_diameter_front/2)
        self.assertAlmostEqual(out["total_force_N"],4*per_wheel)
        self.assertAlmostEqual(out["front_force_N"],2*out["front_per_wheel_force_N"])
        self.assertAlmostEqual(out["rear_force_N"],2*out["rear_per_wheel_force_N"])

    def test_front_and_rear_have_separate_rpm_lookup_and_reduction(self):
        curve=MotorCurve((0.,1000.,2000.),(20.,10.,0.),"test","test")
        v=replace(self.v,reduction_front=10,reduction_rear=20)
        speed=(1000/10)*2*math.pi*(v.tire_diameter_front/2)/60
        out=motor_forces(v,speed,curve=curve)
        self.assertAlmostEqual(out["front_rpm"],1000)
        self.assertAlmostEqual(out["rear_rpm"],2000)
        self.assertAlmostEqual(out["front_motor_torque_Nm"],10)
        self.assertAlmostEqual(out["rear_motor_torque_Nm"],0)
        self.assertAlmostEqual(out["front_wheel_torque_Nm"],10*10*.97)

    def test_rpm_ceiling_uses_tighter_axle_and_actual_wheel_radius(self):
        curve=MotorCurve((0.,2000.),(20.,0.),"test","test")
        v=replace(self.v,reduction_front=10,reduction_rear=20,tire_diameter_front=.4,tire_diameter_rear=.5)
        expected=2000*2*math.pi*.25/(60*20)
        self.assertAlmostEqual(max_vehicle_speed(v,curve),expected)
        out=motor_forces(v,expected,curve=curve)
        self.assertLessEqual(out["front_rpm"],curve.max_rpm+1e-6)
        self.assertAlmostEqual(out["rear_rpm"],curve.max_rpm)

    def test_efficiency_and_torque_scale(self):
        curve=read_motor_curve(self.v.motor_curve_csv)
        lossless=motor_forces(replace(self.v,reduction_efficiency_front=1,reduction_efficiency_rear=1),5,curve=curve)
        out=motor_forces(self.v,5,.8,curve)
        self.assertAlmostEqual(out["total_force_N"]/lossless["total_force_N"],.8*.97)

    def test_independent_wheel_limits_do_not_pool_or_couple(self):
        motors=[100,100,100,100]
        tires=[0,1000,1000,1000]
        self.assertEqual(drive_force_limit(motors,tires),300)
        self.assertNotEqual(drive_force_limit(motors,tires),min(sum(motors),sum(tires)))
        self.assertEqual(drive_force_limit([100,100,0,0],[10,1000,1000,1000]),110)

    def test_signed_csv_is_interpolated_before_drive_torque_is_clipped(self):
        curve=MotorCurve((0.,1000.),(10.,-10.),"test","test")
        self.assertEqual(curve.torque_at(500),0)
        self.assertEqual(curve.torque_at(750),-5)
        v=replace(self.v,reduction_front=10,reduction_rear=10)
        speed=750/10*2*math.pi*(v.tire_diameter_front/2)/60
        out=motor_forces(v,speed,curve=curve)
        self.assertAlmostEqual(out["front_lookup_torque_Nm"],-5)
        self.assertEqual(out["front_motor_torque_Nm"],0)
        self.assertEqual(out["total_force_N"],0)


if __name__=="__main__":unittest.main()
