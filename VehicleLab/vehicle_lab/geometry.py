"""FSAE double-wishbone 3-D kinematics with explicit distance constraints.

Axes: x forward, y vehicle left, z up. Front axle x=0, rear x=-wheelbase.
Hardpoints are metres; the GUI displays millimetres.
"""
import copy
import math
from .models import CORNERS, G
from .linalg import add, sub, scale, dot, norm, unit, cross, rotate, transform, solve_constraints, line_intersection
from .physics import distribution, contact_patch, linspace

TYPES={"double_wishbone": "雙 A 臂"}
HARDPOINTS=("UFI","URI","UBJ","LFI","LRI","LBJ","ROD_IN","ROD_OUT")


def default_geometry(v):
    geo={"types": {"front":"double_wishbone","rear":"double_wishbone"},"corners":{}}
    for corner in CORNERS:
        front=corner.startswith("F"); side=1 if corner.endswith("L") else -1
        x=0 if front else -v.wheelbase
        half=(v.track_front if front else v.track_rear)/2
        radius=(v.tire_diameter_front if front else v.tire_diameter_rear)/2
        caster=math.tan(math.radians(6.5 if front else .2))
        lower=[x+.008,side*(half-.045),radius-.13]
        upper=[lower[0]-.25*caster,side*(half-.08),radius+.12]
        upper_front_z,upper_rear_z=(.1182,.1527) if front else (.0809,.1852)
        geo["corners"][corner]={
            "UFI":[x+.14,side*(half-.32),radius+upper_front_z],"URI":[x-.17,side*(half-.32),radius+upper_rear_z],"UBJ":upper,
            "LFI":[x+.18,side*(half-.34),radius-.12],"LRI":[x-.18,side*(half-.34),radius-.10],"LBJ":lower,
            "ROD_IN":[x-.11,side*(half-.34),radius-.035],"ROD_OUT":[x-.115,side*(half-.055),radius-.045]}
    return geo


def validate_geometry(geo):
    for axle in ("front","rear"):
        if geo["types"][axle] != "double_wishbone": raise ValueError("Only double-wishbone suspension is supported")
    if "solid" in geo:raise ValueError("This FSAE geometry schema contains only double-wishbone corners")
    for corner in CORNERS:
        points=geo["corners"][corner]
        if set(points)!=set(HARDPOINTS):raise ValueError(f"{corner}: expected the eight double-wishbone hardpoints")
        for name in HARDPOINTS:
            if name not in geo["corners"][corner]: raise ValueError(f"Missing {corner}/{name}")
            p=points[name]
            if len(p)!=3 or not all(isinstance(x,(int,float)) and math.isfinite(x) and abs(x)<50 for x in p):
                raise ValueError(f"Invalid hardpoint: {corner}/{name}")


def wheel_basis(v,corner):
    front=corner.startswith("F"); side=1 if corner.endswith("L") else -1
    axle="front" if front else "rear"
    camber=math.radians(getattr(v,"camber_"+axle))
    yaw=-side*math.radians(getattr(v,"toe_"+axle))
    def aligned(vector): return rotate(rotate(vector,[-side*camber,0,0]),[0,0,yaw])
    radius=getattr(v,"tire_diameter_"+axle)/2
    center=[0 if front else -v.wheelbase,side*getattr(v,"track_"+axle)/2,radius*math.cos(camber)]
    return center,aligned([1,0,0]),aligned([0,0,1]),radius


def wheel_contact(center,forward,up,radius,normal=(0,0,1)):
    """Lowest point of the ideal tire center circle along a plane normal."""
    fz,uz=dot(forward,normal),dot(up,normal)
    return add(center,scale(add(scale(forward,fz),scale(up,uz)),-radius/max(math.hypot(fz,uz),1e-9)))


def wheel_state(v,corner,pose,center=None):
    original,forward,up,radius=wheel_basis(v,corner)
    pivot=original if center is None else center
    wc=transform(original,pivot,pose)
    f=rotate(forward,pose[3:]); u=rotate(up,pose[3:])
    contact=wheel_contact(wc,f,u,radius)
    side=1 if corner.endswith("L") else -1
    yaw=math.degrees(math.atan2(f[1],f[0]))
    return {"center":wc,"forward":f,"up":u,"contact":contact,"radius":radius,
            "camber_deg":math.degrees(math.atan2(side*u[1],u[2])),"steer_deg":yaw,"toe_deg":-side*yaw}


def solve_corner(v,geo,corner,travel=0.,rack=0.,initial=None,*,ground_plane=None):
    front=corner.startswith("F"); axle="front" if front else "rear"
    if geo["types"][axle]!="double_wishbone":raise ValueError("Only double-wishbone suspension is supported")
    points=geo["corners"][corner]
    center=wheel_basis(v,corner)[0]
    base_contact=wheel_state(v,corner,[0]*6)["contact"][2]
    inner=add(points["ROD_IN"],[0,rack,0])
    arm_pairs=[("LFI","LBJ"),("LRI","LBJ"),("UFI","UBJ"),("URI","UBJ")]
    lengths=[norm(sub(points[a],points[b])) for a,b in arm_pairs]
    rod_length=norm(sub(points["ROD_IN"],points["ROD_OUT"]))
    if rod_length<.005 or min(lengths)<.005: raise ValueError("Link lengths must exceed 5 mm")
    def residual(pose):
        moved=lambda key:transform(points[key],center,pose)
        result=[norm(sub(points[a],moved(b)))-length for (a,b),length in zip(arm_pairs,lengths)]
        result.append(norm(sub(inner,moved("ROD_OUT")))-rod_length)
        wheel=wheel_state(v,corner,pose)
        if ground_plane is None:
            result.append(wheel["contact"][2]-base_contact-travel)
        else:
            normal,height=ground_plane
            contact=wheel_contact(wheel["center"],wheel["forward"],wheel["up"],wheel["radius"],normal)
            result.append(dot(contact,normal)-height)
        return result
    pose,error=solve_constraints(residual,initial or [0,0,travel,0,0,0])
    moving={"UBJ","LBJ","ROD_OUT"}
    moved={key:transform(value,center,pose) if key in moving else list(value) for key,value in points.items()}
    moved["ROD_IN"]=inner
    wheel=wheel_state(v,corner,pose)
    axis=unit(sub(moved["UBJ"],moved["LBJ"]))
    side=1 if corner.endswith("L") else -1
    wheel.update({"caster_deg":math.degrees(math.atan2(-axis[0],axis[2])),
        "kpi_deg":math.degrees(math.atan2(-side*axis[1],axis[2])),"pose":pose,
        "points":moved,"axis_origin":moved["LBJ"],"axis":axis,"residual_m":error})
    return wheel





def solve_axle(v,geo,axle,left_travel=0.,right_travel=0.):
    """Solve the two independent double-wishbone corners on one axle."""
    if axle not in ("front","rear"):raise ValueError("Unknown axle")
    corners=("FL","FR") if axle=="front" else ("RL","RR")
    return {c:solve_corner(v,geo,c,t) for c,t in zip(corners,(left_travel,right_travel))}


def solve_vehicle(v,geo,travels=None,rack=0.,rear_ratio=0.):
    travels=travels or {}
    return {corner:solve_corner(v,geo,corner,travels.get(corner,0.),
            rack if corner.startswith("F") else rack*rear_ratio) for corner in CORNERS}


def solve_body_pose(v,geo,heave=0.,roll=0.,pitch=0.,pivot=None,rack=0.,rear_ratio=0.):
    """Display kinematics in the road frame, with each tire center circle grounded.

    Metres/radians; positive heave lowers the body, roll lowers its right side,
    and pitch lowers the nose. Roll/pitch form a Rodrigues rotation vector.
    Link constraints are solved in the body frame against the transformed road
    plane before moving all hardpoints and wheel directions into the road frame.
    """
    pivot=[-v.wheelbase/2,0.,0.] if pivot is None else pivot
    pose=[0.,0.,-heave,roll,pitch,0.]
    normal=rotate([0.,0.,1.],[-roll,-pitch,0.])
    height=dot(normal,pivot)-pivot[2]+heave
    world=lambda point:transform(point,pivot,pose)
    wheels={}
    for corner in CORNERS:
        base=wheel_state(v,corner,[0.]*6)
        contact=wheel_contact(base["center"],base["forward"],base["up"],base["radius"],normal)
        travel=(height-dot(normal,contact))/normal[2]
        w=solve_corner(v,geo,corner,travel,rack if corner.startswith("F") else rack*rear_ratio,
                       ground_plane=(normal,height))
        w["points"]={key:world(point) for key,point in w["points"].items()}
        w["center"]=world(w["center"])
        w["axis_origin"]=world(w["axis_origin"])
        for key in ("forward","up","axis"):w[key]=rotate(w[key],pose[3:])
        w["contact"]=wheel_contact(w["center"],w["forward"],w["up"],w["radius"])
        side=1 if corner.endswith("L") else -1
        w["camber_deg"]=math.degrees(math.atan2(side*w["up"][1],w["up"][2]))
        w["steer_deg"]=math.degrees(math.atan2(w["forward"][1],w["forward"][0]))
        w["toe_deg"]=-side*w["steer_deg"]
        w["caster_deg"]=math.degrees(math.atan2(-w["axis"][0],w["axis"][2]))
        w["kpi_deg"]=math.degrees(math.atan2(-side*w["axis"][1],w["axis"][2]))
        wheels[corner]=w
    return wheels


def body_scene(v,geo,wheels):
    """Suspension plus a schematic chassis frame and fixed road reference."""
    result=segments(v,geo,wheels)
    nodes={}
    for c,w in wheels.items():
        points=w["points"]
        for level,a,b in (("upper","UFI","URI"),("lower","LFI","LRI")):
            nodes[c,level]=scale(add(points[a],points[b]),.5)
        result.append((nodes[c,"upper"],nodes[c,"lower"],"chassis"))
    for level in ("upper","lower"):
        for a,b in (("FL","FR"),("FR","RR"),("RR","RL"),("RL","FL")):
            result.append((nodes[a,level],nodes[b,level],"chassis"))
    half=max(v.track_front,v.track_rear)/2+.18
    front=max(v.tire_diameter_front,v.tire_diameter_rear)/2+.12
    road=[[front,-half,0.],[front,half,0.],[-v.wheelbase-front,half,0.],[-v.wheelbase-front,-half,0.]]
    result.extend((road[i],road[(i+1)%4],"ground") for i in range(4))
    return result


def plane_line(points,a,b,c,view,fixed):
    """Intersection of an arm plane with a front or side projection plane."""
    normal=cross(sub(points[b],points[a]),sub(points[c],points[a]))
    constant=dot(normal,points[a])
    if view=="front":
        n=[normal[1],normal[2]]; constant-=normal[0]*fixed
    else:
        n=[normal[0],normal[2]]; constant-=normal[1]*fixed
    if norm(n)<1e-10: return None
    p=scale(n,constant/dot(n,n))
    return p,add(p,[-n[1],n[0]])


def instant_center(points,view,fixed):
    lower=plane_line(points,"LFI","LRI","LBJ",view,fixed)
    upper=plane_line(points,"UFI","URI","UBJ",view,fixed)
    return None if lower is None or upper is None else line_intersection(*lower,*upper)


def roll_center(v,geo,axle,wheels=None):
    wheels=wheels or solve_vehicle(v,geo)
    corners=("FL","FR") if axle=="front" else ("RL","RR")
    lines=[]
    for corner in corners:
        wheel=wheels[corner]
        ic=instant_center(wheel["points"],"front",wheel["contact"][0])
        if ic is None:return None
        lines.append(([wheel["contact"][1],wheel["contact"][2]],ic))
    return line_intersection(*lines[0],*lines[1])


def anti_percent(v,geo,axle,wheels=None):
    wheels=wheels or solve_vehicle(v,geo)
    corner="FL" if axle=="front" else "RL"
    wheel=wheels[corner]
    ic=instant_center(wheel["points"],"side",wheel["contact"][1])
    if ic is None:return None
    contact=wheel["contact"]
    if abs(ic[0]-contact[0])<1e-9:return None
    at=-v.wheelbase*(1-v.front_fraction) if axle=="front" else 0.
    height=(ic[1]-contact[2])*(at-contact[0])/(ic[0]-contact[0])
    return height/v.cg_height*100


def geometry_sweep(v,geo,travel=.04,count=33):
    rows=[]
    for t in linspace(-travel,travel,count):
        row={"travel_mm":t*1000}
        try:
            wheels=solve_vehicle(v,geo,{c:t for c in CORNERS})
            for c,w in wheels.items():
                for name in ("camber_deg","caster_deg","toe_deg"): row[c+"_"+name]=w[name]
            for axle in ("front","rear"):
                rc=roll_center(v,geo,axle,wheels)
                row[axle+"_RC_mm"]=None if rc is None else (rc[1]-t)*1000
            row["status"]="ok"
        except ValueError as exc:
            row["status"]=str(exc)
        rows.append(row)
    return rows


def steering_metrics(v,geo,rack=0.,rear_ratio=0.,ax_g=0.,ay_g=0.,rc_front=.05,rc_rear=.08):
    wheels=solve_vehicle(v,geo,rack=rack,rear_ratio=rear_ratio)
    loads=distribution(v,ay_g,rc_front,rc_rear)["loads"]
    longitudinal=v.mass*ax_g*G*v.cg_height/v.wheelbase
    loads=[load+(longitudinal/2 if i<2 else -longitudinal/2) for i,load in enumerate(loads)]
    rows=[]
    for i,c in enumerate(CORNERS):
        w=wheels[c]; front=i<2; side=1 if c.endswith("L") else -1
        axis=w["axis"]; origin=w["axis_origin"]
        if abs(axis[2])<1e-8: raise ValueError("Kingpin is parallel to road")
        ground=add(origin,scale(axis,-origin[2]/axis[2]))
        scrub=side*(w["contact"][1]-ground[1]); trail=ground[0]-w["contact"][0]
        inner=w["points"]["ROD_IN"]; outer=w["points"]["ROD_OUT"]
        lever=abs(dot(axis,cross(sub(outer,origin),unit(sub(inner,outer)))))
        fraction=v.front_fraction if front else 1-v.front_fraction
        axle_sum=sum(max(0,x) for x in loads[:2 if front else 4]) if front else sum(max(0,x) for x in loads[2:])
        fy=v.mass*G*ay_g*fraction*max(0,loads[i])/max(axle_sum,1e-9)
        if ax_g>=0: drive_fraction=v.brake_front if front else 1-v.brake_front
        else:
            gain_front=v.reduction_front*v.reduction_efficiency_front/(v.tire_diameter_front/2)
            gain_rear=v.reduction_rear*v.reduction_efficiency_rear/(v.tire_diameter_rear/2)
            drive_fraction=(gain_front if front else gain_rear)/(gain_front+gain_rear)
        fx=-v.mass*G*ax_g*drive_fraction/2
        patch=contact_patch(loads[i],v.pressure_front if front else v.pressure_rear,
            v.tire_width_front if front else v.tire_width_rear,w["radius"])
        pneumatic=patch["length_m"]/6
        moment=fy*(trail+pneumatic)+fx*scrub
        rows.append({"corner":c,"steer_deg":w["steer_deg"],"camber_deg":w["camber_deg"],
            "toe_deg":w["toe_deg"],"scrub_mm":scrub*1000,"trail_mm":trail*1000,
            "pneumatic_mm":pneumatic*1000,"load_N":loads[i],"kingpin_Nm":moment,
            "lever_mm":lever*1000,"rod_N":None if lever<1e-5 else moment/lever})
    ack=ackermann_percent(rows[0]["steer_deg"],rows[1]["steer_deg"],v.wheelbase,v.track_front)
    return rows,wheels,ack


def ackermann_percent(left,right,wheelbase,track):
    if left*right<=0 or min(abs(left),abs(right))<.1:return None
    inside,outside=(abs(left),abs(right)) if left+right>0 else (abs(right),abs(left))
    return (1/math.tan(math.radians(outside))-1/math.tan(math.radians(inside)))*wheelbase/track*100


def tune_geometry(v,geo,axle,rc_target,anti_target):
    """Retune upper-arm mounts toward RC (m) and geometric anti (%)."""
    base=copy.deepcopy(geo)
    corners=("FL","FR") if axle=="front" else ("RL","RR")
    def candidate(q):
        out=copy.deepcopy(base)
        for corner in corners:
            points=out["corners"][corner]
            for name in ("UFI","URI"):
                points[name][2]+=q[0]+q[1]*(points[name][0]-points["UBJ"][0])
        return out
    def residual(q):
        out=candidate(q);w=solve_vehicle(v,out)
        rc=roll_center(v,out,axle,w);anti=anti_percent(v,out,axle,w)
        if rc is None or anti is None:raise ValueError("Goal trial has parallel instant-center projections")
        return [rc[1]-rc_target,(anti-anti_target)*v.cg_height/100]
    q,error=solve_constraints(residual,[0.,0.],tolerance=1e-6,iterations=45)
    if max(map(abs,q))>.5:raise ValueError("Goals require excessive mount movement; edit hardpoints manually")
    out=candidate(q);validate_geometry(out);solve_vehicle(v,out)
    return out


def set_kingpin_goals(v,geo,axle,scrub,trail):
    """Set static scrub/trail by changing the upper ball joint."""
    out=copy.deepcopy(geo)
    corners=("FL","FR") if axle=="front" else ("RL","RR")
    wheels=solve_vehicle(v,out)
    for corner in corners:
        side=1 if corner.endswith("L") else -1
        points=out["corners"][corner];lower=points["LBJ"];top=points["UBJ"]
        if abs(lower[2])<1e-6:raise ValueError("Lower joint is at ground height")
        ground=[wheels[corner]["contact"][0]+trail,wheels[corner]["contact"][1]-side*scrub,0]
        for axis in (0,1):top[axis]=lower[axis]+(ground[axis]-lower[axis])*(top[2]-lower[2])/(-lower[2])
    solve_vehicle(v,out)
    return out


def segments(v,geo,wheels=None):
    wheels=wheels or solve_vehicle(v,geo)
    result=[]
    pairs=[("LFI","LBJ","lower"),("LRI","LBJ","lower"),("LBJ","UBJ","upright"),
           ("ROD_IN","ROD_OUT","rod"),("UFI","UBJ","upper"),("URI","UBJ","upper")]
    for corner,wheel in wheels.items():
        points=wheel["points"]
        result.extend((points[a],points[b],color) for a,b,color in pairs)
        forward,up=wheel["forward"],wheel["up"]
        axle=unit(cross(up,forward))
        half_width=(v.tire_width_front if corner.startswith("F") else v.tire_width_rear)/2
        # Keep the center ring and place side rings along the wheel's posed axle.
        for offset in (0.,-half_width,half_width):
            center=add(wheel["center"],scale(axle,offset))
            circle=[add(center,scale(add(scale(forward,math.cos(t)),scale(up,math.sin(t))),wheel["radius"]))
                    for t in linspace(0,2*math.pi,25)]
            result.extend((a,b,"wheel") for a,b in zip(circle,circle[1:]))
    return result


def construction_segments(v,wheels,projection,projection_lines=True,kingpin_lines=True):
    """Display-only arm projections and steering axes for a solved wheel pose.

    Arm planes are sliced at each contact patch, exactly as in ``roll_center``
    and ``anti_percent``. Distant instant centers are left intact; callers must
    fit the view to the physical suspension and clip these construction lines.
    """
    if projection not in ("Front","Rear","Side","Top","Iso"):return []
    corners=("FL","FR") if projection=="Front" else ("RL","RR") if projection=="Rear" else CORNERS
    result=[]
    def emit(a,b,kind):
        if all(math.isfinite(x) for point in (a,b) for x in point) and norm(sub(a,b))>1e-9:
            result.append((list(a),list(b),kind))
    views=("front","side") if projection=="Iso" else ("front",) if projection in ("Front","Rear") else ("side",) if projection=="Side" else ()
    for view in views if projection_lines else ():
        centers={}
        for corner in corners:
            if corner not in wheels:continue
            wheel=wheels[corner];contact=wheel["contact"]
            fixed=contact[0] if view=="front" else contact[1]
            centers[corner]=instant_center(wheel["points"],view,fixed)
        for corner in corners:
            if corner not in wheels:continue
            wheel=wheels[corner];points=wheel["points"];contact=wheel["contact"]
            fixed=contact[0] if view=="front" else contact[1]
            lift=lambda p:[fixed,p[0],p[1]] if view=="front" else [p[0],fixed,p[1]]
            flatten=lambda p:[p[1],p[2]] if view=="front" else [p[0],p[2]]
            ic=centers[corner]
            span=max(.15,wheel["radius"]*1.5)
            for a,b,joint,kind in (("UFI","URI","UBJ","projection_upper"),("LFI","LRI","LBJ","projection_lower")):
                line=plane_line(points,a,b,joint,view,fixed)
                if line is None:continue
                p,q=line;direction=unit(sub(q,p))
                anchor=add(p,scale(direction,dot(sub(flatten(points[joint]),p),direction)))
                distance=None if ic is None else dot(sub(ic,anchor),direction)
                lo=min(-span,distance) if distance is not None else -span
                hi=max(span,distance) if distance is not None else span
                emit(lift(add(anchor,scale(direction,lo))),lift(add(anchor,scale(direction,hi))),kind)
            if ic is None:continue
            origin=flatten(contact);delta=sub(ic,origin)
            if norm(delta)<1e-9:continue
            # Include the RC or anti-evaluation point even when it is on the
            # opposite ray from the instant center through the contact patch.
            target=None
            if view=="front":
                other={"FL":"FR","FR":"FL","RL":"RR","RR":"RL"}[corner]
                if other in wheels and centers.get(other) is not None:
                    target=line_intersection(origin,ic,flatten(wheels[other]["contact"]),centers[other])
            elif abs(delta[0])>1e-9:
                at=-v.wheelbase*(1-v.front_fraction) if corner.startswith("F") else 0.
                target=add(origin,scale(delta,(at-origin[0])/delta[0]))
            t=None if target is None else dot(sub(target,origin),delta)/dot(delta,delta)
            lo=min(0.,t) if t is not None else 0.
            hi=max(1.,t) if t is not None else 1.
            emit(lift(add(origin,scale(delta,lo))),lift(add(origin,scale(delta,hi))),"contact_line")
    if kingpin_lines:
        for corner in corners:
            if corner not in wheels:continue
            wheel=wheels[corner];lower=wheel["points"]["LBJ"];upper=wheel["points"]["UBJ"]
            delta=sub(upper,lower);length=norm(delta)
            if length<1e-9:continue
            direction=scale(delta,1/length);extension=max(.05,wheel["radius"]*.3)
            if abs(direction[2])>1e-8:
                ground=(wheel["contact"][2]-lower[2])/direction[2]
                lo=min(0.,ground);hi=max(length+extension,ground)
            else:
                lo=-wheel["radius"];hi=length+extension
            emit(add(lower,scale(direction,lo)),add(lower,scale(direction,hi)),"kingpin")
    return result


