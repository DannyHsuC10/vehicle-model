import math
import copy
import unittest
from vehicle_lab.models import Vehicle, CORNERS
from vehicle_lab import geometry as g
from vehicle_lab.linalg import norm,sub,add,rotate,dot,cross,line_intersection


class GeometryTests(unittest.TestCase):
    def setUp(self):self.v=Vehicle();self.geo=g.default_geometry(self.v)

    def test_static_identity_and_mirror(self):
        w=g.solve_vehicle(self.v,self.geo)
        for c in CORNERS:self.assertLess(w[c]["residual_m"],1e-7)
        self.assertAlmostEqual(w["FL"]["center"][1],-w["FR"]["center"][1])
        self.assertAlmostEqual(w["FL"]["toe_deg"],self.v.toe_front)

    def test_wishbone_constraints_during_bump_and_steer(self):
        for travel in (-.04,.04):
            for rack in (-.02,.02):
                w=g.solve_corner(self.v,self.geo,"FL",travel,rack)
                p=w["points"];base=self.geo["corners"]["FL"]
                for a,b in (("UFI","UBJ"),("URI","UBJ"),("LFI","LBJ"),("LRI","LBJ")):
                    self.assertAlmostEqual(norm(sub(p[a],p[b])),norm(sub(base[a],base[b])),places=6)
                self.assertAlmostEqual(norm(sub(p["ROD_IN"],p["ROD_OUT"])),norm(sub(base["ROD_IN"],base["ROD_OUT"])),places=6)
                self.assertAlmostEqual(w["contact"][2],travel,places=6)

    def test_body_pose_zero_matches_static_geometry(self):
        static=g.solve_vehicle(self.v,self.geo)
        grounded=g.solve_body_pose(self.v,self.geo)
        for corner in CORNERS:
            for key in ("center","forward","up","contact","axis","axis_origin"):
                self.assertLess(norm(sub(grounded[corner][key],static[corner][key])),5e-7)
            for key in g.HARDPOINTS:
                self.assertLess(norm(sub(grounded[corner]["points"][key],static[corner]["points"][key])),5e-7)

    def test_body_pose_keeps_actual_tire_circle_on_ground(self):
        # Test the wheel plane itself: transforming an old body-frame contact
        # point can hide a tire that actually sinks below a flat road.
        poses=({"heave":.02},{"heave":-.02},
               {"roll":math.radians(2)},{"roll":math.radians(-2)},
               {"pitch":math.radians(1.5)},{"pitch":math.radians(-1.5)},
               {"heave":.008,"roll":math.radians(1),"pitch":math.radians(.8)},
               {"heave":-.008,"roll":math.radians(-1),"pitch":math.radians(-.8)})
        for pose in poses:
            with self.subTest(pose=pose):
                wheels=g.solve_body_pose(self.v,self.geo,**pose)
                for corner,wheel in wheels.items():
                    with self.subTest(corner=corner):
                        f,u=wheel["forward"],wheel["up"]
                        self.assertAlmostEqual(norm(f),1,places=8)
                        self.assertAlmostEqual(norm(u),1,places=8)
                        self.assertAlmostEqual(dot(f,u),0,places=8)
                        lowest=wheel["center"][2]-wheel["radius"]*math.hypot(f[2],u[2])
                        self.assertLess(abs(lowest),3e-7)
                        self.assertLess(abs(wheel["contact"][2]),3e-7)
                        radial=sub(wheel["contact"],wheel["center"])
                        self.assertAlmostEqual(norm(radial),wheel["radius"],places=7)
                        self.assertAlmostEqual(dot(radial,cross(u,f)),0,places=7)

    def test_body_pose_moves_chassis_with_expected_signs(self):
        pivot=[-.6,0,.05]
        cases=((.015,0.,0.),(-.015,0.,0.),
               (0.,math.radians(2),0.),(0.,math.radians(-2),0.),
               (0.,0.,math.radians(1.2)),(0.,0.,math.radians(-1.2)))
        for heave,roll,pitch in cases:
            with self.subTest(heave=heave,roll=roll,pitch=pitch):
                wheels=g.solve_body_pose(self.v,self.geo,heave=heave,roll=roll,pitch=pitch,pivot=pivot)
                for corner in CORNERS:
                    for key in ("UFI","URI","LFI","LRI","ROD_IN"):
                        expected=add(add(pivot,[0,0,-heave]),
                                     rotate(sub(self.geo["corners"][corner][key],pivot),[roll,pitch,0]))
                        self.assertLess(norm(sub(wheels[corner]["points"][key],expected)),1e-9)
                if roll>0:
                    self.assertGreater(wheels["FL"]["points"]["LFI"][2],
                                       wheels["FR"]["points"]["LFI"][2])
                if pitch>0:
                    front_delta=wheels["FL"]["points"]["LFI"][2]-self.geo["corners"]["FL"]["LFI"][2]
                    rear_delta=wheels["RL"]["points"]["LFI"][2]-self.geo["corners"]["RL"]["LFI"][2]
                    self.assertLess(front_delta,rear_delta)

    def test_body_pose_preserves_links_with_alignment_and_different_tires(self):
        self.v.camber_front=-3.2;self.v.camber_rear=-1.7
        self.v.toe_front=.45;self.v.toe_rear=.2
        self.v.tire_diameter_front=.44;self.v.tire_diameter_rear=.49
        self.geo=g.default_geometry(self.v)
        for direction in (-1,1):
            with self.subTest(direction=direction):
                wheels=g.solve_body_pose(self.v,self.geo,heave=.005*direction,
                    roll=math.radians(1.1)*direction,pitch=math.radians(.7)*direction,
                    rack=.008*direction,rear_ratio=.4)
                for corner,wheel in wheels.items():
                    points=wheel["points"];base=self.geo["corners"][corner]
                    for a,b in (("UFI","UBJ"),("URI","UBJ"),("LFI","LBJ"),("LRI","LBJ"),
                                ("ROD_IN","ROD_OUT"),("LBJ","UBJ")):
                        self.assertAlmostEqual(norm(sub(points[a],points[b])),norm(sub(base[a],base[b])),places=6)
                    radius=(self.v.tire_diameter_front if corner.startswith("F") else self.v.tire_diameter_rear)/2
                    self.assertEqual(wheel["radius"],radius)
                    self.assertLess(abs(wheel["contact"][2]),3e-7)
                    lowest=wheel["center"][2]-radius*math.hypot(wheel["forward"][2],wheel["up"][2])
                    self.assertLess(abs(lowest),3e-7)

    def test_only_double_wishbone_geometry_is_supported(self):
        self.assertEqual(list(g.TYPES),["double_wishbone"])
        self.assertEqual(set(self.geo),{"types","corners"})
        for points in self.geo["corners"].values():self.assertEqual(set(points),set(g.HARDPOINTS))
        self.geo["types"]["front"]="unsupported"
        with self.assertRaises(ValueError):g.validate_geometry(self.geo)
        with self.assertRaises(ValueError):g.solve_vehicle(self.v,self.geo)

    def test_all_four_corners_and_axle_wrapper(self):
        for travel in (-.04,0,.04):
            wheels=g.solve_vehicle(self.v,self.geo,{c:travel for c in CORNERS})
            self.assertTrue(all(abs(wheels[c]["contact"][2]-travel)<3e-7 for c in CORNERS))
            self.assertIsNotNone(g.roll_center(self.v,self.geo,"front",wheels))
        axle=g.solve_axle(self.v,self.geo,"rear",.02,-.02)
        self.assertAlmostEqual(axle["RL"]["contact"][2],.02,places=6)
        self.assertAlmostEqual(axle["RR"]["contact"][2],-.02,places=6)

    def test_camber_and_toe_static_offsets(self):
        self.v.camber_front=-2;self.v.toe_front=.3
        w=g.solve_vehicle(self.v,self.geo)
        for c in ("FL","FR"):
            self.assertAlmostEqual(w[c]["camber_deg"],-2,places=3)
            self.assertAlmostEqual(w[c]["toe_deg"],.3,places=3)

    def test_roll_center_is_centered(self):
        for axle in ("front","rear"):
            rc=g.roll_center(self.v,self.geo,axle)
            self.assertAlmostEqual(rc[0],0,places=7)
            self.assertTrue(-.5<rc[1]<.5)

    def test_construction_view_filters_and_independent_toggles(self):
        wheels=g.solve_vehicle(self.v,self.geo);original=copy.deepcopy(wheels)
        for view,corner_count,projection_count in (("Front",2,6),("Rear",2,6),("Side",4,12),("Iso",4,24),("Top",4,0)):
            with self.subTest(view=view):
                overlays=g.construction_segments(self.v,wheels,view)
                self.assertEqual(sum(kind=="kingpin" for _,_,kind in overlays),corner_count)
                self.assertEqual(len(overlays),corner_count+projection_count)
                self.assertEqual(len(g.construction_segments(self.v,wheels,view,kingpin_lines=False)),projection_count)
                self.assertEqual(len(g.construction_segments(self.v,wheels,view,projection_lines=False)),corner_count)
                self.assertEqual(g.construction_segments(self.v,wheels,view,False,False),[])
        self.assertEqual(wheels,original)

    def test_front_construction_uses_arm_planes_and_actual_roll_center(self):
        wheels=g.solve_body_pose(self.v,self.geo,heave=.008,roll=math.radians(1),pitch=math.radians(.5))
        for view,corners,axle in (("Front",("FL","FR"),"front"),("Rear",("RL","RR"),"rear")):
            overlays=g.construction_segments(self.v,wheels,view,kingpin_lines=False)
            contacts=[]
            for index,corner in enumerate(corners):
                wheel=wheels[corner];points=wheel["points"]
                upper,lower,contact=overlays[index*3:index*3+3]
                for segment,keys in ((upper,("UFI","URI","UBJ")),(lower,("LFI","LRI","LBJ"))):
                    normal=cross(sub(points[keys[1]],points[keys[0]]),sub(points[keys[2]],points[keys[0]]))
                    for endpoint in segment[:2]:
                        self.assertAlmostEqual(endpoint[0],wheel["contact"][0])
                        self.assertAlmostEqual(dot(normal,sub(endpoint,points[keys[0]])),0,places=8)
                front=lambda p:[p[1],p[2]]
                projected=[(front(a),front(b)) for a,b,_ in (upper,lower,contact)]
                actual_ic=line_intersection(*projected[0],*projected[1])
                expected_ic=g.instant_center(points,"front",wheel["contact"][0])
                self.assertLess(norm(sub(actual_ic,expected_ic)),1e-6)
                # The contact-force line must pass through the contact patch
                # and the IC, including the opposite ray toward the RC.
                a,b=projected[2];direction=sub(b,a)
                for point in (front(wheel["contact"]),expected_ic,g.roll_center(self.v,self.geo,axle,wheels)):
                    offset=sub(point,a)
                    self.assertAlmostEqual(direction[0]*offset[1]-direction[1]*offset[0],0,places=6)
                    self.assertGreaterEqual(dot(offset,direction)/dot(direction,direction),-1e-8)
                    self.assertLessEqual(dot(offset,direction)/dot(direction,direction),1+1e-8)
                contacts.append(projected[2])
            self.assertLess(norm(sub(line_intersection(*contacts[0],*contacts[1]),g.roll_center(self.v,self.geo,axle,wheels))),1e-7)

    def test_side_construction_reaches_ic_and_anti_evaluation(self):
        wheels=g.solve_vehicle(self.v,self.geo,{c:.012 for c in CORNERS},rack=.006)
        overlays=g.construction_segments(self.v,wheels,"Side",kingpin_lines=False)
        side=lambda p:[p[0],p[2]]
        for index,corner in enumerate(CORNERS):
            wheel=wheels[corner];upper,lower,contact=overlays[index*3:index*3+3]
            projected=[(side(a),side(b)) for a,b,_ in (upper,lower,contact)]
            ic=line_intersection(*projected[0],*projected[1])
            expected=g.instant_center(wheel["points"],"side",wheel["contact"][1])
            self.assertLess(norm(sub(ic,expected)),1e-7)
            a,b=projected[2];direction=sub(b,a)
            at=-self.v.wheelbase*(1-self.v.front_fraction) if corner.startswith("F") else 0.
            height=a[1]+(at-a[0])*direction[1]/direction[0]
            contact_point=side(wheel["contact"])
            expected_height=contact_point[1]+(at-contact_point[0])*(expected[1]-contact_point[1])/(expected[0]-contact_point[0])
            self.assertAlmostEqual(height,expected_height,places=8)
            for point in (contact_point,expected,[at,height]):
                offset=sub(point,a)
                self.assertAlmostEqual(direction[0]*offset[1]-direction[1]*offset[0],0,places=7)
                self.assertTrue(-1e-8<=dot(offset,direction)/dot(direction,direction)<=1+1e-8)
            self.assertTrue(all(abs(p[1]-wheel["contact"][1])<1e-9 for s in (upper,lower,contact) for p in s[:2]))

    def test_kingpin_extensions_follow_posed_axis_and_contact_height(self):
        wheels=g.solve_body_pose(self.v,self.geo,heave=.01,roll=math.radians(1.2),rack=.007)
        overlays=g.construction_segments(self.v,wheels,"Iso",projection_lines=False)
        for corner,(a,b,kind) in zip(CORNERS,overlays):
            wheel=wheels[corner];lower=wheel["points"]["LBJ"];upper=wheel["points"]["UBJ"]
            self.assertEqual(kind,"kingpin")
            self.assertLess(norm(cross(sub(a,lower),sub(upper,lower))),1e-9)
            self.assertLess(norm(cross(sub(b,lower),sub(upper,lower))),1e-9)
            self.assertAlmostEqual(a[2],wheel["contact"][2],places=8)
            self.assertGreater(b[2],upper[2])

    def test_parallel_degenerate_and_distant_construction_is_finite(self):
        wheels=g.solve_vehicle(self.v,self.geo)
        distant=g.construction_segments(self.v,wheels,"Front")
        self.assertGreater(max(abs(x) for a,b,_ in distant for p in (a,b) for x in p),100)
        for corner,wheel in wheels.items():
            x=wheel["contact"][0];y=wheel["contact"][1]
            wheel["points"].update(LFI=[x-.2,y-.3,.1],LRI=[x+.2,y-.3,.1],LBJ=[x,y,.1],
                                   UFI=[x-.2,y-.3,.3],URI=[x+.2,y-.3,.3],UBJ=[x,y,.3])
        parallel=g.construction_segments(self.v,wheels,"Front")
        self.assertNotIn("contact_line",[kind for _,_,kind in parallel])
        self.assertTrue(all(math.isfinite(x) for a,b,_ in parallel for p in (a,b) for x in p))
        for wheel in wheels.values():
            lower=wheel["points"]["LBJ"]
            wheel["points"]["UBJ"]=add(lower,[.2,.05,0.])
        horizontal=g.construction_segments(self.v,wheels,"Iso",projection_lines=False)
        self.assertEqual(len(horizontal),4)
        self.assertTrue(all(norm(sub(a,b))<1 for a,b,_ in horizontal))
        for wheel in wheels.values():
            wheel["points"]={key:list(wheel["points"]["LBJ"]) for key in g.HARDPOINTS}
        self.assertEqual(g.construction_segments(self.v,wheels,"Iso"),[])

    def test_steering_pressure_has_no_effect_in_straight_braking(self):
        a,_,_=g.steering_metrics(self.v,self.geo,ax_g=.8,ay_g=0)
        self.v.pressure_front*=1.5
        b,_,_=g.steering_metrics(self.v,self.geo,ax_g=.8,ay_g=0)
        self.assertAlmostEqual(a[0]["kingpin_Nm"],b[0]["kingpin_Nm"])

    def test_steering_has_ackermann(self):
        rows,_,ack=g.steering_metrics(self.v,self.geo,.025)
        self.assertIsNotNone(ack)
        self.assertGreater(abs(rows[0]["steer_deg"]),2)
        self.assertNotAlmostEqual(rows[0]["steer_deg"],rows[1]["steer_deg"],places=2)

    def test_geometry_goals(self):
        out=g.tune_geometry(self.v,self.geo,"front",.08,20)
        self.assertAlmostEqual(g.roll_center(self.v,out,"front")[1],.08,places=5)
        self.assertAlmostEqual(g.anti_percent(self.v,out,"front"),20,places=3)
        self.assertNotEqual(out,self.geo)

    def test_kingpin_goals(self):
        out=g.set_kingpin_goals(self.v,self.geo,"front",.03,.035)
        rows,_,_=g.steering_metrics(self.v,out)
        self.assertAlmostEqual(rows[0]["scrub_mm"],30,places=5)
        self.assertAlmostEqual(rows[1]["trail_mm"],35,places=5)

    def test_reverse_ackermann_sign(self):
        self.assertLess(g.ackermann_percent(10,20,2.5,1.5),0)
        self.assertGreater(g.ackermann_percent(20,10,2.5,1.5),0)

    def test_fsae_nominal_roll_centers_and_sweep(self):
        self.assertAlmostEqual(g.roll_center(self.v,self.geo,"front")[1],.0400223065,places=6)
        self.assertAlmostEqual(g.roll_center(self.v,self.geo,"rear")[1],.0499934637,places=6)
        self.assertTrue(all(row["status"]=="ok" for row in g.geometry_sweep(self.v,self.geo,.04,9)))

    def test_acceleration_loads_follow_front_rear_reduction_gain(self):
        self.v.reduction_front=8
        self.v.reduction_rear=12
        rows,_,_=g.steering_metrics(self.v,self.geo,ax_g=-.5,ay_g=0)
        # With equal efficiencies/radii, front and rear torque fractions are 0.4/0.6.
        # At zero lateral force the moment is exactly Fx times scrub.
        for i,row in enumerate(rows):
            fraction=.4 if i<2 else .6
            fx=self.v.mass*9.80665*.5*fraction/2
            self.assertAlmostEqual(row["kingpin_Nm"],fx*row["scrub_mm"]/1000,places=6)
