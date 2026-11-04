"""Independent engineering models; no network calls or third party packages."""
import math
from .models import CORNERS, G


def linspace(start, stop, count):
    return [start + (stop - start) * i / (count - 1) for i in range(count)]


def interpolate(x, points):
    if x <= points[0][0]:
        return points[0][1]
    for (a, b), (c, d) in zip(points, points[1:]):
        if x <= c:
            return b + (d - b) * (x - a) / (c - a)
    return points[-1][1]


def frequency(rate, mass):
    return math.sqrt(rate / mass) / (2 * math.pi)


def damper_force(velocity, low, high, knee):
    """Continuous bilinear curve. Positive compression, negative rebound."""
    speed = abs(velocity)
    force = low * min(speed, knee) + high * max(0, speed - knee)
    return math.copysign(force, velocity)


def damper_curves(comp_low, comp_high, comp_knee, reb_low, reb_high, reb_knee, end, count=101):
    return [{"shaft_m_s": s, "compression_N": damper_force(s, comp_low, comp_high, comp_knee),
        "rebound_N": damper_force(-s, reb_low, reb_high, reb_knee)} for s in linspace(0, end, count)]


def step_response(mass, rate, comp, rebound, step=.05, duration=3., delay=0., count=1201):
    """Physical base-step SDOF: m*x''+c*x'+k*(x-road)=0 after the step.

    Exact displacement initial condition x=0 and velocity=0 at arrival; the
    impulsive road-velocity term of an ideal base step is deliberately excluded.
    Compression is selected by relative suspension velocity < 0.
    """
    if min(mass, rate, duration) <= 0 or min(comp, rebound, delay) < 0:
        raise ValueError("Invalid step parameters")
    if count < 3 or count > 200001:
        raise ValueError("Step sample count must be 3..200001")
    dt = duration / (count - 1)
    omega = math.sqrt(rate / mass)
    substeps = max(1, math.ceil(dt * max(omega, max(comp, rebound) / mass) / .15))
    if substeps > 2000:
        raise ValueError("Step model too stiff; reduce duration or coefficients")
    def derivative(z, speed):
        c = comp if speed < 0 else rebound
        return speed, -(c * speed + rate * (z - step)) / mass
    def advance(z, speed, h):
        a, b = derivative(z, speed)
        c, d = derivative(z + h*a/2, speed + h*b/2)
        e, f = derivative(z + h*c/2, speed + h*d/2)
        g, j = derivative(z + h*e, speed + h*f)
        return z + h*(a+2*c+2*e+g)/6, speed + h*(b+2*d+2*f+j)/6
    z, speed = 0., 0.
    rows = []
    for i in range(count):
        rows.append({"time_s": delay + i*dt, "body_m": z, "velocity_m_s": speed,
                     "road_m": step, "phase": "compression" if speed < 0 else "rebound"})
        if i < count - 1:
            for _ in range(substeps):
                z, speed = advance(z, speed, dt/substeps)
    if step:
        peak = max(r["body_m"] / step for r in rows)
        outside = [i for i, r in enumerate(rows) if abs(r["body_m"] - step) > abs(step)*.02]
        settle = None if outside and outside[-1] == count-1 else rows[outside[-1]+1]["time_s"] if outside else delay
    else:
        peak, settle = 1, delay
    critical = 2 * math.sqrt(rate * mass)
    return rows, {"frequency_Hz": frequency(rate, mass), "critical_N_s_m": critical,
        "zeta_compression": comp / critical, "zeta_rebound": rebound / critical,
        "overshoot_percent": max(0, (peak-1)*100), "settle_absolute_s": settle}


def transmissibility(rate, mass, comp, rebound):
    fn = frequency(rate, mass)
    zeta = (comp+rebound) / (4*math.sqrt(rate*mass))
    rows = []
    for f in linspace(.05, max(15, fn*6), 301):
        r = f/fn
        den = (1-r*r)**2 + (2*zeta*r)**2
        rows.append({"frequency_Hz": f, "body_road_ratio": math.sqrt((1+(2*zeta*r)**2)/max(den, 1e-20))})
    return rows


def roll_stiffness(v):
    front = v.wheel_rate_front*v.track_front**2/2
    rear = v.wheel_rate_rear*v.track_rear**2/2
    return {"spring_front": front, "spring_rear": rear,
        "front": front+v.arb_front, "rear": rear+v.arb_rear,
        "total": front+rear+v.arb_front+v.arb_rear}


def distribution(v, ay_g, rc_front=.05, rc_rear=.08, speed=0., aero=False):
    """Three-path lateral moment balance. Delta is transferred load, not R-L."""
    ay = ay_g*G
    mf, mr = v.sprung(True), v.sprung(False)
    uf, ur = 2*v.unsprung_front, 2*v.unsprung_rear
    hs = (v.mass*v.cg_height - uf*v.unsprung_height_front - ur*v.unsprung_height_rear)/(mf+mr)
    roll_axis = (mf*rc_front+mr*rc_rear)/(mf+mr)
    stiffness = roll_stiffness(v)
    elastic = (mf+mr)*ay*(hs-roll_axis)
    sf = stiffness["front"]/stiffness["total"]
    paths = {
        "unsprung": [uf*ay*v.unsprung_height_front, ur*ay*v.unsprung_height_rear],
        "geometric": [mf*ay*rc_front, mr*ay*rc_rear],
        "elastic": [elastic*sf, elastic*(1-sf)]}
    moments = [sum(row[i] for row in paths.values()) for i in (0, 1)]
    df, dr = moments[0]/v.track_front, moments[1]/v.track_rear
    static = [v.mass*G*v.front_fraction/2]*2 + [v.mass*G*(1-v.front_fraction)/2]*2
    q = .5*v.rho*speed*speed if aero else 0
    loads = [static[0]-df+q*v.cla_front/2, static[1]+df+q*v.cla_front/2,
             static[2]-dr+q*v.cla_rear/2, static[3]+dr+q*v.cla_rear/2]
    total_moment = v.mass*ay*v.cg_height
    return {"paths": paths, "moments": moments, "delta": [df, dr],
        "static": static, "loads": loads, "sprung_cg": hs, "roll_axis": roll_axis,
        "gradient_deg_g": math.degrees((mf+mr)*G*(hs-roll_axis)/stiffness["total"]),
        "roll_deg": math.degrees(elastic/stiffness["total"]),
        "front_moment_fraction": moments[0]/total_moment if abs(total_moment)>1e-10 else None,
        "stiffness_front_fraction": sf, "moment_error": sum(moments)-total_moment}


def tire_peak(v, load, lateral=True, camber=0.):
    if load <= 0:
        return 0.
    mu = v.mu_lat if lateral else v.mu_long
    factor = max(.05, 1-v.load_sensitivity*(load/v.reference_tire_load-1))
    alignment = max(.1, 1-v.camber_sensitivity*abs(camber)) if lateral else 1.
    return mu*load*factor*alignment


def magic_force(alpha, peak, b=8., c=1.3, e=-1.):
    x = b*alpha
    return peak*math.sin(c*math.atan(x-e*(x-math.atan(x))))


def slip_for_force(force, peak, b=8., c=1.3, e=-1.):
    # Only use the stable rising branch; saturation is reported explicitly.
    samples = [(i*.5/300, magic_force(i*.5/300, peak, b, c, e)) for i in range(301)]
    imax = max(range(len(samples)), key=lambda i: samples[i][1])
    if abs(force)>samples[imax][1]+1e-8:
        return None
    lo, hi = 0., samples[imax][0]
    for _ in range(40):
        mid=(lo+hi)/2
        if magic_force(mid, peak, b, c, e)<abs(force): lo=mid
        else: hi=mid
    return math.copysign((lo+hi)/2, force)


def steady_balance(v, ay_g, radius, rc_front, rc_rear, speed=0., aero=False):
    d = distribution(v, ay_g, rc_front, rc_rear, speed, aero)
    slip, remaining, demands = [], [], []
    for i, front in enumerate((True, False)):
        pair=d["loads"][2*i:2*i+2]
        frac=v.front_fraction if front else 1-v.front_fraction
        demand=v.mass*ay_g*G*frac
        caps=[tire_peak(v, load) for load in pair]
        base=[tire_peak(v, load) for load in d["static"][2*i:2*i+2]]
        remaining.append(sum(caps)/sum(base)*100)
        axle_peak=sum(caps)
        angle=slip_for_force(demand, axle_peak, v.tire_b, v.tire_c, v.tire_e)
        slip.extend([None if angle is None else math.degrees(angle)]*2)
        demands.extend([demand*x/max(sum(caps),1e-9) for x in caps])
    road = math.degrees(math.atan(v.wheelbase/radius))* (1 if ay_g>=0 else -1)
    steer = None if any(x is None for x in slip) else road+slip[0]-slip[2]
    return {**d, "slip_deg": slip, "forces_N": demands, "grip_percent": remaining,
        "steer_deg": steer, "understeer_deg_g": None if abs(ay_g)<1e-9 or steer is None else (steer-road)/ay_g}


def rigid_transfer(v, ax_g, ay_g):
    front=v.mass*G*v.front_fraction + v.mass*ax_g*G*v.cg_height/v.wheelbase
    rear=v.mass*G-front
    df=front*ay_g*v.cg_height/v.track_front
    dr=rear*ay_g*v.cg_height/v.track_rear
    return {"static": [v.mass*G*v.front_fraction/2]*2+[v.mass*G*(1-v.front_fraction)/2]*2,
        "loads": [front/2-df, front/2+df, rear/2-dr, rear/2+dr],
        "axles": [front, rear], "longitudinal_delta_N": v.mass*ax_g*G*v.cg_height/v.wheelbase}


def contact_patch(load, pressure, section_width, radius):
    if load <= 0:
        return {"area_m2": 0., "length_m": 0., "width_m": 0., "lifted": True}
    area=load/pressure
    width=min(section_width*.88, math.sqrt(4*area/math.pi))
    length=4*area/(math.pi*width)
    return {"area_m2": area, "length_m": length, "width_m": width, "lifted": False}


def air_density(altitude, temp_c):
    if not -100 <= temp_c <= 100 or not -500 <= altitude <= 11000:
        raise ValueError("Density helper: altitude -500..11000 m, temperature -100..100 C")
    pressure=101325*(1-2.25577e-5*altitude)**5.25588
    return pressure/(287.058*(temp_c+273.15))


def wing_coefficients(angle, chord, span, camber=.02, camber_at=.4, thickness=.12):
    # Thin-airfoil zero-lift shift numerically integrated over a NACA mean line.
    integral=0.
    count=500
    for j in range(count):
        theta=(j+.5)*math.pi/count
        x=(1-math.cos(theta))/2
        slope=2*camber/(camber_at**2)*(camber_at-x) if x<camber_at else 2*camber/(1-camber_at)**2*(camber_at-x)
        integral += slope*(math.cos(theta)-1)/count
    alpha=math.radians(angle)+integral
    ar=span/chord
    cl=2*math.pi*alpha*ar/(ar+2)
    cd0=.008+.08*thickness**2
    return cl, cd0+cl*cl/(math.pi*ar*.8)


def aero(v, speed, chord=.22, span=1.1, angle=8., camber=.02, camber_at=.4,
         thickness=.12, position=1.08, estimate=True, cl=.8, wing_cd=.06):
    if estimate:
        cl, wing_cd=wing_coefficients(angle,chord,span,camber,camber_at,thickness)
    q=.5*v.rho*speed*speed
    wing_down=q*chord*span*cl
    wing_drag=q*chord*span*wing_cd
    body_drag=q*v.cd*v.frontal_area
    front=q*v.cla_front+wing_down*(1-position)
    rear=q*v.cla_rear+wing_down*position
    return {"speed_kmh": speed*3.6, "body_drag_N": body_drag, "body_power_kW": body_drag*speed/1000,
        "body_front_N": q*v.cla_front, "body_rear_N": q*v.cla_rear,
        "wing_down_N": wing_down, "wing_drag_N": wing_drag, "total_power_kW": (body_drag+wing_drag)*speed/1000,
        "front_down_N": front, "rear_down_N": rear,
        "front_ride_mm": -front/(2*v.wheel_rate_front)*1000,
        "rear_ride_mm": -rear/(2*v.wheel_rate_rear)*1000,
        "Cl": cl, "wing_Cd": wing_cd, "aspect_ratio": span/chord,
        "front_load_N": v.mass*G*v.front_fraction+front, "rear_load_N": v.mass*G*(1-v.front_fraction)+rear}


def lift_limit(v, rc_front, rc_rear):
    d=distribution(v,1.,rc_front,rc_rear)
    limits=[d["static"][2*i]/abs(d["delta"][i]) if abs(d["delta"][i])>1e-10 else math.inf for i in (0,1)]
    return min(limits), "front" if limits[0]<limits[1] else "rear"
