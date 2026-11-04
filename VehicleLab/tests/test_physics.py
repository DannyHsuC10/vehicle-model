import math
import unittest
from dataclasses import asdict,replace
from vehicle_lab.models import Vehicle, G
from vehicle_lab import physics as p


class PhysicsTests(unittest.TestCase):
    def setUp(self):self.v=Vehicle()

    def test_aero_reference(self):
        a=p.aero(self.v,40)
        self.assertAlmostEqual(a["body_drag_N"],926.1)
        self.assertAlmostEqual(a["body_power_kW"],37.044)

    def test_aero_scaling_and_axle_balance(self):
        a=p.aero(self.v,20);b=p.aero(self.v,40)
        self.assertAlmostEqual(b["wing_down_N"],4*a["wing_down_N"])
        self.assertAlmostEqual(b["body_power_kW"],8*a["body_power_kW"])
        self.assertAlmostEqual(b["front_down_N"]+b["rear_down_N"],b["wing_down_N"]+b["body_front_N"]+b["body_rear_N"])
        wing_only=p.aero(replace(self.v,cla_front=0,cla_rear=0),40)
        self.assertLess(wing_only["front_down_N"],0)

    def test_damper_continuity_and_sign(self):
        knee=.05
        self.assertAlmostEqual(p.damper_force(knee,1000,200,knee),50)
        self.assertAlmostEqual(p.damper_force(knee+1e-8,1000,200,knee),50+2e-6)
        self.assertEqual(p.damper_force(-.1,1000,200,knee),-60)

    def test_step_matches_analytic_linear_underdamped_solution(self):
        mass,rate,c=300,30000,1800
        rows,metrics=p.step_response(mass,rate,c,c,.05,3,count=1201)
        wn=math.sqrt(rate/mass);zeta=c/(2*math.sqrt(mass*rate));wd=wn*math.sqrt(1-zeta*zeta)
        for row in rows[::30]:
            t=row["time_s"]
            expected=.05*(1-math.exp(-zeta*wn*t)*(math.cos(wd*t)+zeta/math.sqrt(1-zeta*zeta)*math.sin(wd*t)))
            self.assertAlmostEqual(row["body_m"],expected,places=8)
        self.assertAlmostEqual(metrics["frequency_Hz"],wn/(2*math.pi))
        self.assertIsNotNone(metrics["settle_absolute_s"])

    def test_step_negative_and_delay(self):
        a,_=p.step_response(300,30000,1500,1500,.03,2)
        b,_=p.step_response(300,30000,1500,1500,-.03,2,delay=.2)
        for x,y in zip(a[::100],b[::100]):
            self.assertAlmostEqual(x["body_m"],-y["body_m"])
            self.assertAlmostEqual(y["time_s"]-x["time_s"],.2)

    def test_nonsettled_response_is_none(self):
        _,m=p.step_response(300,30000,0,0,.05,1)
        self.assertIsNone(m["settle_absolute_s"])

    def test_stiff_step_is_stable(self):
        r,_=p.step_response(5,5e6,10000,10000,.02,2,count=401)
        self.assertTrue(all(math.isfinite(x["body_m"]) for x in r))
        self.assertAlmostEqual(r[-1]["body_m"],.02,places=6)

    def test_distribution_conserves_force_and_moment(self):
        for v in (self.v,replace(self.v,track_rear=1.8,front_fraction=.53,unsprung_front=22.5,unsprung_rear=13)):
            for ay in (-1.1,0,.7):
                d=p.distribution(v,ay,.06,.15)
                self.assertAlmostEqual(d["moment_error"],0,places=9)
                self.assertAlmostEqual(sum(d["loads"]),v.mass*G)
                self.assertAlmostEqual(d["delta"][0]*v.track_front+d["delta"][1]*v.track_rear,v.mass*ay*G*v.cg_height)

    def test_bar_changes_split_not_total_moment(self):
        a=p.distribution(self.v,1,.05,.1)
        b=p.distribution(replace(self.v,arb_front=100000),1,.05,.1)
        self.assertGreater(b["delta"][0],a["delta"][0])
        self.assertAlmostEqual(sum(a["moments"]),sum(b["moments"]))

    def test_rigid_sign_and_conservation(self):
        d=p.rigid_transfer(self.v,1,.5)
        self.assertGreater(sum(d["loads"][:2]),sum(d["static"][:2]))
        self.assertGreater(d["loads"][1],d["loads"][0])
        self.assertAlmostEqual(sum(d["loads"]),self.v.mass*G)
        self.assertLess(sum(p.rigid_transfer(self.v,-1,0)["loads"][:2]),sum(d["static"][:2]))

    def test_contact_patch_area_and_lift(self):
        a=p.contact_patch(4000,200000,.235,.32)
        self.assertAlmostEqual(a["area_m2"],.02)
        self.assertAlmostEqual(math.pi*a["length_m"]*a["width_m"]/4,.02)
        self.assertTrue(p.contact_patch(-50,200000,.235,.32)["lifted"])

    def test_load_sensitivity(self):
        self.assertLess(p.tire_peak(self.v,8000),2*p.tire_peak(self.v,4000))
        self.assertEqual(p.tire_peak(self.v,-5),0)

    def test_magic_formula_inverse_and_saturation(self):
        alpha=p.slip_for_force(2000,5000)
        self.assertAlmostEqual(p.magic_force(alpha,5000),2000,places=6)
        self.assertIsNone(p.slip_for_force(6000,5000))

    def test_density(self):
        self.assertAlmostEqual(p.air_density(0,15),1.225,places=3)
        self.assertLess(p.air_density(2000,15),p.air_density(0,15))

    def test_bad_vehicle(self):
        for update in ({"curb_mass":float("nan")},{"front_fraction":1},{"unsprung_front":10000},
                       {"reduction_front":0},{"reduction_efficiency_rear":1.1},{"motor_curve_csv":""}):
            with self.assertRaises(ValueError):replace(self.v,**update).validate()

    def test_legacy_drivetrain_settings_are_rejected(self):
        with self.assertRaises(ValueError):Vehicle.from_dict({"gears":[1]})

    def test_symmetric_unsprung_mass_is_per_wheel(self):
        v=replace(self.v,unsprung_front=12,unsprung_rear=15)
        v.validate()
        self.assertAlmostEqual(v.sprung(True),v.mass*v.front_fraction-24)
        self.assertAlmostEqual(v.sprung(False),v.mass*(1-v.front_fraction)-30)
        d=p.distribution(v,0,.04,.05)
        self.assertEqual(d["loads"][0],d["loads"][1])
        self.assertEqual(d["loads"][2],d["loads"][3])
        self.assertAlmostEqual(sum(d["loads"]),v.mass*G)
        turning=p.distribution(v,1,.04,.05)
        self.assertAlmostEqual(turning["paths"]["unsprung"][0],24*G*v.unsprung_height_front)
        self.assertAlmostEqual(turning["paths"]["unsprung"][1],30*G*v.unsprung_height_rear)
        self.assertAlmostEqual(turning["moment_error"],0,places=9)

    def test_legacy_corner_unsprung_masses_preserve_axle_totals(self):
        old={**asdict(self.v),"unsprung_fl":8,"unsprung_fr":12,"unsprung_rl":11,"unsprung_rr":15}
        del old["unsprung_front"];del old["unsprung_rear"]
        before=dict(old)
        v=Vehicle.from_dict(old)
        self.assertEqual(v.unsprung_front,10)
        self.assertEqual(v.unsprung_rear,13)
        self.assertAlmostEqual(v.sprung(True),v.mass*v.front_fraction-20)
        self.assertAlmostEqual(v.sprung(False),v.mass*(1-v.front_fraction)-26)
        self.assertEqual(old,before)
        self.assertFalse(set(asdict(v)).intersection(("unsprung_fl","unsprung_fr","unsprung_rl","unsprung_rr")))

    def test_invalid_or_mixed_legacy_unsprung_data_are_rejected(self):
        corners={"unsprung_fl":8,"unsprung_fr":12,"unsprung_rl":11,"unsprung_rr":15}
        invalid=[{"unsprung_fl":8},{**corners,"unsprung_front":10},{**corners,"unsprung_rear":13}]
        for bad in (-1,float("nan"),float("inf"),True,"9"):
            invalid.append({**corners,"unsprung_fl":bad})
        for data in invalid:
            with self.subTest(data=data):
                with self.assertRaises(ValueError):Vehicle.from_dict(data)

    def test_fsae_defaults(self):
        self.assertEqual(self.v.mass,325)
        self.assertAlmostEqual(self.v.wheelbase,1.55)
        self.assertEqual(self.v.reduction_front,12)
        self.assertEqual(self.v.unsprung_front,9)
        self.assertEqual(self.v.unsprung_rear,9)
        self.assertFalse(hasattr(self.v,"gears"))
