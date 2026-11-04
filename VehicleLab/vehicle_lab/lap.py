"""Segment point-mass lap solver with periodic forward/backward speed passes."""
import csv
import math
from dataclasses import replace
from pathlib import Path
from .models import G
from .physics import distribution, tire_peak
from .motor import read_motor_curve, motor_forces, max_vehicle_speed, drive_force_limit

PRESETS={
    "示範 GP（非真實賽道）": [
        {"length_m":600,"radius_m":0,"grade_percent":0},
        {"length_m":110,"radius_m":55,"grade_percent":0},
        {"length_m":250,"radius_m":0,"grade_percent":1},
        {"length_m":140,"radius_m":-70,"grade_percent":-1},
        {"length_m":160,"radius_m":40,"grade_percent":0},
        {"length_m":400,"radius_m":0,"grade_percent":0},
        {"length_m":157.08,"radius_m":50,"grade_percent":0},
        {"length_m":350,"radius_m":0,"grade_percent":0},
        {"length_m":160,"radius_m":85,"grade_percent":0}],
    "定圓 9.125 m": [{"length_m":2*math.pi*9.125,"radius_m":9.125,"grade_percent":0}],
    "示範 Autocross": [
        {"length_m":85,"radius_m":0,"grade_percent":0},
        {"length_m":35,"radius_m":18,"grade_percent":0},
        {"length_m":50,"radius_m":-25,"grade_percent":0},
        {"length_m":50,"radius_m":25,"grade_percent":0},
        {"length_m":100,"radius_m":0,"grade_percent":0},
        {"length_m":50,"radius_m":16,"grade_percent":0}],
    "直線 75 m（開放起點）": [{"length_m":75,"radius_m":0,"grade_percent":0}]}


def validate_track(track):
    if not track or len(track)>10000: raise ValueError("Track must have 1..10000 segments")
    length=0
    for row in track:
        for key in ("length_m","radius_m","grade_percent"):
            if not math.isfinite(row.get(key,0)): raise ValueError("Track values must be finite")
        if row["length_m"]<=0: raise ValueError("Segment length must be > 0")
        if 0<abs(row["radius_m"])<2: raise ValueError("Radius is signed metres, 0=straight; minimum 2 m")
        if abs(row.get("grade_percent",0))>60: raise ValueError("Grade must be within +/-60 percent")
        length+=row["length_m"]
    if length>100000: raise ValueError("Track exceeds 100 km")


def read_track(path):
    with Path(path).open(encoding="utf-8-sig",newline="") as handle:
        source=list(csv.DictReader(handle))
    track=[{key:float(r.get(key) or 0) for key in ("length_m","radius_m","grade_percent")} for r in source]
    validate_track(track)
    return track


def track_from_centerline(path,closed=True):
    """Read x_m,y_m[,z_m]. Coordinates must be local Cartesian metres, not GPS."""
    with Path(path).open(encoding="utf-8-sig",newline="") as handle:
        points=[(float(r["x_m"]),float(r["y_m"]),float(r.get("z_m") or 0)) for r in csv.DictReader(handle)]
    if len(points)<4: raise ValueError("Centerline requires at least 4 points")
    if math.dist(points[0],points[-1])<.001: points.pop()
    track=[]
    count=len(points)
    for i in range(count if closed else count-1):
        prev=points[(i-1)%count] if closed or i else points[0]
        a,b=points[i],points[(i+1)%count]
        ab=math.hypot(b[0]-a[0],b[1]-a[1])
        if ab<.01: continue
        pa=math.hypot(a[0]-prev[0],a[1]-prev[1])
        pb=math.hypot(b[0]-prev[0],b[1]-prev[1])
        twice=(a[0]-prev[0])*(b[1]-prev[1])-(a[1]-prev[1])*(b[0]-prev[0])
        curvature=2*twice/max(pa*ab*pb,1e-12)
        track.append({"length_m":ab,"radius_m":0 if abs(curvature)<1e-6 else 1/curvature,
                      "grade_percent":100*(b[2]-a[2])/ab})
    validate_track(track)
    return track


def discretize(track,step):
    validate_track(track)
    if not 1<=step<=50: raise ValueError("Lap spatial step must be 1..50 m")
    cells=[]; distance=0.; heading=0.; x=0.; y=0.; z=0.
    for index,segment in enumerate(track):
        count=max(1,math.ceil(segment["length_m"]/step))
        ds=segment["length_m"]/count
        radius=segment["radius_m"]
        curvature=0 if radius==0 else 1/radius
        for _ in range(count):
            cells.append({"distance_m":distance,"ds":ds,"curvature":curvature,
                "grade":segment.get("grade_percent",0)/100,"segment":index+1,"x_m":x,"y_m":y,"elevation_m":z})
            if abs(curvature)>1e-12:
                x+=(math.sin(heading+curvature*ds)-math.sin(heading))/curvature
                y+=(-math.cos(heading+curvature*ds)+math.cos(heading))/curvature
            else:
                x+=ds*math.cos(heading); y+=ds*math.sin(heading)
            heading+=curvature*ds
            z+=ds*segment.get("grade_percent",0)/100
            distance+=ds
    if len(cells)>20000: raise ValueError("Track discretization too large; increase spatial step")
    return cells


def wheel_capacities(v,speed,ay,ax,rc_front,rc_rear,aero_on):
    """Individual longitudinal tire limits plus axle lateral capacities."""
    d=distribution(v,ay/G,rc_front,rc_rear,speed,aero_on)
    shift=v.mass*ax*v.cg_height/v.wheelbase
    loads=[load+(-shift/2 if i<2 else shift/2) for i,load in enumerate(d["loads"])]
    longitudinal=[];lateral_capacities=[]
    for i in (0,1):
        pair=loads[2*i:2*i+2]
        camber=v.camber_front if i==0 else v.camber_rear
        lateral=sum(tire_peak(v,f,True,camber) for f in pair)
        demand=v.mass*abs(ay)*(v.front_fraction if i==0 else 1-v.front_fraction)
        ellipse=math.sqrt(max(0,1-(demand/max(lateral,1e-9))**2))
        longitudinal.extend(tire_peak(v,f,False)*ellipse for f in pair)
        lateral_capacities.append(lateral)
    return {"longitudinal":longitudinal,"lateral":lateral_capacities,"loads":loads}


def axle_capacities(v,speed,ay,ax,rc_front,rc_rear,aero_on):
    caps=wheel_capacities(v,speed,ay,ax,rc_front,rc_rear,aero_on)
    return [(sum(caps["longitudinal"][2*i:2*i+2]),caps["lateral"][i]) for i in (0,1)]


def lateral_limit(v,curvature,rc_front,rc_rear,aero_on,max_speed):
    if abs(curvature)<1e-12: return max_speed
    lo,hi=0.,max_speed
    for _ in range(27):
        speed=(lo+hi)/2; ay=speed*speed*abs(curvature)
        caps=axle_capacities(v,speed,ay,0,rc_front,rc_rear,aero_on)
        feasible=all(caps[i][1]>=v.mass*ay*(v.front_fraction if i==0 else 1-v.front_fraction) for i in (0,1))
        if feasible: lo=speed
        else: hi=speed
    return lo


def acceleration(v,speed,curvature,grade,rc_front,rc_rear,aero_on,torque_scale=1.,braking=False,curve=None):
    ay=speed*speed*abs(curvature)
    drag=.5*v.rho*speed*speed*v.cd*v.frontal_area if aero_on else 0.
    resist=drag+v.rolling_coefficient*v.mass*G+v.mass*G*grade
    # Find the greatest feasible acceleration. Merely summing the remaining
    # longitudinal force can falsely accelerate when an axle has already lost
    # the lateral force needed for the corner because of longitudinal transfer.
    if not braking:
        drive=motor_forces(v,speed,torque_scale,curve)
        motor_limits=[drive["front_per_wheel_force_N"]]*2+[drive["rear_per_wheel_force_N"]]*2
    def budget(magnitude):
        ax=-magnitude if braking else magnitude
        caps=wheel_capacities(v,speed,ay,ax,rc_front,rc_rear,aero_on)
        lateral_ok=all(caps["lateral"][i]+1e-5>=v.mass*ay*(v.front_fraction if i==0 else 1-v.front_fraction) for i in (0,1))
        if braking:
            front=math.inf if v.brake_front==0 else sum(caps["longitudinal"][:2])/v.brake_front
            rear=math.inf if v.brake_front==1 else sum(caps["longitudinal"][2:])/(1-v.brake_front)
            available=min(front,rear)
            required=v.mass*magnitude-resist
        else:
            available=drive_force_limit(motor_limits,caps["longitudinal"])
            required=v.mass*magnitude+resist
        return lateral_ok and available+1e-5>=required,available
    feasible,available=budget(0.)
    if not feasible:
        return max(0,resist/v.mass) if braking else min(0,(available-resist)/v.mass)
    lo,hi=0.,4*G
    for _ in range(20):
        mid=(lo+hi)/2
        if budget(mid)[0]:lo=mid
        else:hi=mid
    return lo


def simulate_lap(v,track,rc_front=.05,rc_rear=.08,step=5.,closed=True,start_speed=0.,
                 aero_on=True,torque_scale=1.,curve=None):
    v.validate()
    # UI bounds apply to the baseline. Relative sensitivity cases may extend
    # past them (1% -10% = 0.9%; 500% +10% = 550%).
    if not math.isfinite(torque_scale) or torque_scale<=0:
        raise ValueError("Motor torque scale must be finite and > 0")
    if start_speed<0 or not math.isfinite(start_speed): raise ValueError("Invalid starting speed")
    cells=discretize(track,step)
    curve=read_motor_curve(v.motor_curve_csv) if curve is None else curve
    max_speed=max_vehicle_speed(v,curve)
    vmax=[lateral_limit(v,c["curvature"],rc_front,rc_rear,aero_on,max_speed) for c in cells]
    n=len(cells)
    # Node speed is constrained by BOTH adjoining segments. This prevents a
    # braking pass from crossing into a slower corner at excessive speed.
    limits=[min(vmax[i],vmax[(i-1)%n]) if closed or i else vmax[0] for i in range(n)]
    speeds=limits+[limits[0] if closed else vmax[-1]]
    if not closed: speeds[0]=min(start_speed,limits[0])
    converged=False
    for passes in range(80):
        before=list(speeds)
        for i,c in enumerate(cells):
            accel=acceleration(v,speeds[i],c["curvature"],c["grade"],rc_front,rc_rear,aero_on,torque_scale,curve=curve)
            speeds[i+1]=min(speeds[i+1],math.sqrt(max(0,speeds[i]**2+2*accel*c["ds"])))
        if closed: speeds[0]=min(speeds[0],speeds[-1]); speeds[-1]=speeds[0]
        for i in range(n-1,-1,-1):
            c=cells[i]
            decel=acceleration(v,speeds[i+1],c["curvature"],c["grade"],rc_front,rc_rear,aero_on,torque_scale,True,curve)
            speeds[i]=min(speeds[i],math.sqrt(max(0,speeds[i+1]**2+2*decel*c["ds"])))
        if closed: speeds[-1]=min(speeds[-1],speeds[0]); speeds[0]=speeds[-1]
        else: speeds[0]=min(start_speed,speeds[0])
        if max(abs(a-b) for a,b in zip(before,speeds))<1e-5:
            converged=True
            break
    if not converged: raise ValueError("Lap speed passes did not converge; reduce spatial step or check course")
    rows=[]; elapsed=0.
    for i,c in enumerate(cells):
        speed=(speeds[i]+speeds[i+1])/2
        if speeds[i]+speeds[i+1]<1e-7: raise ValueError("Vehicle cannot advance on this track (insufficient traction or power)")
        dt=2*c["ds"]/(speeds[i]+speeds[i+1])
        drive=motor_forces(v,speed,torque_scale,curve)
        ax=(speeds[i+1]**2-speeds[i]**2)/(2*c["ds"])
        elapsed+=dt
        q=.5*v.rho*speed*speed if aero_on else 0
        rows.append({"distance_m":c["distance_m"],"time_s":elapsed-dt,"dt_s":dt,"speed_kmh":speed*3.6,
            "entry_kmh":speeds[i]*3.6,"exit_kmh":speeds[i+1]*3.6,**drive,
            "long_g":ax/G,"lat_g":speed*speed*c["curvature"]/G,"segment":c["segment"],
            "x_m":c["x_m"],"y_m":c["y_m"],"elevation_m":c["elevation_m"],
            "front_ride_mm":-q*v.cla_front/(2*v.wheel_rate_front)*1000,
            "rear_ride_mm":-q*v.cla_rear/(2*v.wheel_rate_rear)*1000})
    return rows,{"lap_s":elapsed,"distance_m":sum(c["ds"] for c in cells),
        "top_kmh":max(speeds)*3.6,"min_kmh":min(speeds)*3.6,"passes":passes+1,
        "periodic_error_m_s":abs(speeds[-1]-speeds[0]) if closed else None,
        "end_kmh":speeds[-1]*3.6,"motor_rpm_ceiling":curve.max_rpm,
        "motor_csv_source":curve.source,"motor_csv_sha256":curve.sha256,
        "motor_csv_rows":len(curve.rpm),"motor_csv_negative_samples":sum(t<0 for t in curve.torque),
        "motor_speed_ceiling_kmh":max_speed*3.6}


def sensitivities(v,track,rc_front,rc_rear,**settings):
    settings={**settings,"curve":settings.get("curve") or read_motor_curve(v.motor_curve_csv)}
    _,baseline=simulate_lap(v,track,rc_front,rc_rear,**settings)
    result=[{"change":"baseline","lap_s":baseline["lap_s"],"delta_s":0.}]
    for name,updates,override in [
        ("Motor torque +10%",{}, {"torque_scale":1.1}), ("Motor torque -10%",{}, {"torque_scale":.9}),
        ("Grip +10%",{"mu_lat":v.mu_lat*1.1,"mu_long":v.mu_long*1.1},{}),
        ("Front bar +50%",{"arb_front":v.arb_front*1.5},{}),
        ("Front bar -50%",{"arb_front":v.arb_front*.5},{}),
        ("Rear bar +50%",{"arb_rear":v.arb_rear*1.5},{}),
        ("Rear bar -50%",{"arb_rear":v.arb_rear*.5},{}),
        ("Aero toggle",{}, {"aero_on":not settings.get("aero_on",True)})]:
        options={**settings,**override}
        if "torque_scale" in override:
            options["torque_scale"]=settings.get("torque_scale",1.)*override["torque_scale"]
        _,out=simulate_lap(replace(v,**updates),track,rc_front,rc_rear,**options)
        result.append({"change":name,"lap_s":out["lap_s"],"delta_s":out["lap_s"]-baseline["lap_s"]})
    return result
