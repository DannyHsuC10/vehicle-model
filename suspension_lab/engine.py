"""Backend-independent kinematics seeded exclusively by StaticWireframe.

FV: circle-intersection linkage seeded by the package's static geometry.
SV: fixed Au/Al tangent paths seeded by the same static geometry.
WE: Rodrigues steering around the combined static FV/SV kingpin.
No Matplotlib, SciPy, global mutable solvers, or car.py design defaults.
"""
import copy
import math

import numpy as np

from .project import static_objects, validate, travel_limits
from .static.fv.wireframe_geometry import build_wireframe_scene
from .wheel_end_geometry import rotation_matrix
from . import hardpoints
from .anti import axle_anti

COLORS = {"upper": "#fbbf24", "lower": "#38bdf8", "upright": "#a78bfa",
          "wheel": "#e2e8f0", "ground": "#64748b", "kingpin": "#fb7185", "force": "#34d399"}


class Unreachable(ValueError):
    pass


def check_pose_travel(values, bounds):
    low, high = bounds
    if any(value < low-1e-5 or value > high+1e-5 for value in values):
        raise Unreachable(f"此姿態需要行程 {values[0]:.5g} / {values[1]:.5g} mm，超出下垂 {abs(low):.5g} ～ 上壓 {high:.5g} mm。")


def intersection(a, b, c, d):
    a, b, c, d = (np.asarray(p, dtype=float) for p in (a, b, c, d))
    u, v = b - a, d - c
    det = u[0] * v[1] - u[1] * v[0]
    if abs(det) <= 1e-12 * max(np.linalg.norm(u) * np.linalg.norm(v), 1e-12):
        return None
    w = c - a
    return a + (w[0] * v[1] - w[1] * v[0]) / det * u


def rotate2(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def scalar_solve(function, target, origin, span):
    """Find the nearest reachable branch, bisect and verify the actual residual."""
    def evaluate(x):
        try:
            value = float(function(x)) - target
            return value if math.isfinite(value) else None
        except (Unreachable, FloatingPointError):
            return None
    zero = evaluate(origin)
    if zero is not None and abs(zero) < 1e-11:
        return origin
    samples = [(x, evaluate(x)) for x in np.linspace(origin - span, origin + span, 81)]
    brackets = [(a, b) for (a, fa), (b, fb) in zip(samples, samples[1:])
                if fa is not None and fb is not None and fa * fb <= 0]
    brackets.sort(key=lambda pair: abs(sum(pair) / 2 - origin))
    for lo, hi in brackets:
        flo = evaluate(lo)
        for _ in range(45):
            mid = (lo + hi) / 2
            fm = evaluate(mid)
            if fm is None:
                break
            if abs(fm) < 1e-11:
                return mid
            if flo * fm <= 0:
                hi = mid
            else:
                lo, flo = mid, fm
        else:
            if abs(evaluate(mid)) < 1e-8:
                return mid
    raise Unreachable(f"要求的行程 {target * 1000:.3f} mm 在目前幾何分支不可達。")


def line(points, name, construction=False, color=None, dash=(), fit=True, width=1):
    return {"points": np.asarray(points, dtype=float).tolist(), "name": name,
            "color": color or COLORS.get(name, "#34d399"),
            "construction": construction, "dash": dash or ((6, 4) if construction else ()), "fit": fit, "width": width}


def contact_frame(first, second):
    """Midpoint origin, x' along the contacts, y' its upward CCW normal."""
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    delta = second-first
    length = np.linalg.norm(delta)
    if not np.isfinite(length) or length < 1e-12:
        raise Unreachable("兩個接地點重合，無法建立接地座標系。")
    tangent = delta/length
    return {"origin": ((first+second)/2).tolist(), "x_axis": tangent.tolist(),
            "y_axis": [-float(tangent[1]), float(tangent[0])],
            "angle_deg": math.degrees(math.atan2(tangent[1], tangent[0]))}


def to_contact_frame(point, frame):
    delta = np.asarray(point, dtype=float)-frame["origin"]
    return np.array([np.dot(delta, frame["x_axis"]), np.dot(delta, frame["y_axis"])])


def center_metrics(prefix, point, frame):
    local = None if point is None else to_contact_frame(point, frame)
    return {f"{prefix}_x_prime_mm": None if local is None else float(local[0]*1000),
            f"{prefix}_y_prime_mm": None if local is None else float(local[1]*1000),
            f"{prefix}_world_x_mm": None if point is None else float(point[0]*1000),
            f"{prefix}_world_z_mm": None if point is None else float(point[1]*1000)}


def frame_axes(frame, span, height):
    """Visible reference arrows; excluded from fitting and force-line toggles."""
    origin = np.asarray(frame["origin"])
    lines, annotations = [], []
    for name, direction, start, end, color, offset in (
            ("x′", frame["x_axis"], -.58*span, .58*span, "#2dd4bf", [8, -12]),
            ("y′", frame["y_axis"], -.08, height, "#fb923c", [12, -10])):
        direction = np.asarray(direction)
        tip = origin+end*direction
        lines.append(line([origin+start*direction, tip], name, color=color, fit=False, width=2))
        normal = np.array([-direction[1], direction[0]])
        arrow_size = min(span*.025, .025)
        lines.append(line([tip-arrow_size*direction+arrow_size*.45*normal, tip,
                           tip-arrow_size*direction-arrow_size*.45*normal], name, color=color, fit=False, width=2))
        annotations.append({"position": tip.tolist(), "start": origin.tolist(),
                            "name": name, "color": color, "offset": offset})
    return lines, annotations


class FVMotion:
    def __init__(self, plane):
        # Frozen zero-travel source. Never mutate the authoritative scene.
        self.zero = {key: None if value is None else np.array(value, copy=True) for key, value in plane.static_pts.items()
                     if key in ("kua", "lak", "kg", "gnd", "wheel_center", "cpb_rl", "cpb_ru", "IC", "cg", "RC")}
        z = self.zero
        self.lower = np.linalg.norm(z["lak"] - z["cpb_rl"])
        self.upper = np.linalg.norm(z["kua"] - z["cpb_ru"])
        self.upright = np.linalg.norm(z["kua"] - z["lak"])
        v = z["lak"] - z["cpb_rl"]
        self.theta0 = math.atan2(v[1], v[0])
        self.width, self.radius = plane.helper.tire_width, plane.helper.free_radius
        self.cache = {}
        self._roll_bounds_cache = {}

    def at_angle(self, theta):
        z = self.zero
        lower = z["cpb_rl"] + self.lower * np.array([math.cos(theta), math.sin(theta)])
        delta = lower - z["cpb_ru"]
        d = np.linalg.norm(delta)
        if d < 1e-12 or d > self.upper + self.upright or d < abs(self.upper - self.upright):
            raise Unreachable("FV 上臂與羊角無交點。")
        a = (self.upper**2 - self.upright**2 + d*d) / (2*d)
        h = math.sqrt(max(0, self.upper**2 - a*a))
        p = z["cpb_ru"] + a * delta / d
        normal = np.array([-delta[1], delta[0]]) / d
        upper = min((p + h*normal, p - h*normal), key=lambda q: np.linalg.norm(q - z["kua"]))
        old, new = z["kua"] - z["lak"], upper - lower
        angle = math.atan2(new[1], new[0]) - math.atan2(old[1], old[0])
        matrix = rotate2(angle)
        result = {key: lower + matrix @ (z[key] - z["lak"])
                  for key in ("kg", "gnd", "wheel_center")}
        result.update(kua=upper, lak=lower, cpb_rl=z["cpb_rl"], cpb_ru=z["cpb_ru"], angle=angle)
        result["IC"] = intersection(z["cpb_rl"], lower, z["cpb_ru"], upper)
        wc = result["wheel_center"]
        offsets = np.array([[-self.width/2, -self.radius], [self.width/2, -self.radius],
                            [self.width/2, self.radius], [-self.width/2, self.radius], [-self.width/2, -self.radius]])
        result["wheel"] = offsets @ matrix.T + wc
        return result

    def side(self, travel_mm, left=False):
        key = round(float(travel_mm), 9)
        if key not in self.cache:
            target = travel_mm / 1000 + self.zero["gnd"][1]
            theta = scalar_solve(lambda t: self.at_angle(t)["gnd"][1], target, self.theta0, 1.15)
            self.cache[key] = self.at_angle(theta)
        state = dict(self.cache[key])
        if left:
            state = {k: v * [-1, 1] if isinstance(v, np.ndarray) else -v for k, v in state.items() if v is not None}
            state.setdefault("IC", None)
        return state

    def solve(self, left_mm, right_mm, gains=True):
        left, right = self.side(left_mm, True), self.side(right_mm)
        def cambers(l, r):
            g = r["gnd"] - l["gnd"]
            roll = math.atan2(g[1], g[0])
            return math.degrees(l["angle"] - roll), -math.degrees(r["angle"] - roll), math.degrees(roll)
        cl, cr, roll = cambers(left, right)
        frame = contact_frame(left["gnd"], right["gnd"])
        rc = None if left["IC"] is None or right["IC"] is None else intersection(left["gnd"], left["IC"], right["gnd"], right["IC"])
        metrics = {"travel_left_mm": float(left_mm), "travel_right_mm": float(right_mm),
                   "heave_mm": (left_mm + right_mm)/2, "roll_deg": roll,
                   "camber_left_deg": cl, "camber_right_deg": cr,
                   **center_metrics("cg", self.zero["cg"], frame),
                   **center_metrics("rc", rc, frame)}
        metrics["cg_height_mm"] = metrics["cg_y_prime_mm"]
        if gains:
            for label, is_left in (("left", True), ("right", False)):
                try:
                    values = []
                    for delta in (-.01, .01):
                        l = self.side(left_mm + delta, True) if is_left else left
                        r = right if is_left else self.side(right_mm + delta)
                        values.append(cambers(l, r)[0 if is_left else 1])
                    metrics[f"camber_gain_{label}_deg_per_mm"] = (values[1]-values[0])/.02
                except Unreachable:
                    metrics[f"camber_gain_{label}_deg_per_mm"] = None
        lines = []
        points = {"CG": self.zero["cg"].tolist()}
        if self.zero["RC"] is not None:
            points["RC0"] = self.zero["RC"].tolist()
        for label, state in (("L", left), ("R", right)):
            for a, b, kind in (("cpb_ru", "kua", "upper"), ("cpb_rl", "lak", "lower"),
                               ("kua", "lak", "upright"), ("kg", "kua", "kingpin")):
                lines.append(line([state[a], state[b]], kind))
            lines.append(line(state["wheel"], "wheel"))
            if state["IC"] is not None:
                for key in ("gnd", "kua", "lak"):
                    lines.append(line([state[key], state["IC"]], "force", True))
            for name in ("kua", "lak", "kg", "IC", "gnd", "wheel_center", "cpb_ru", "cpb_rl"):
                if state[name] is not None:
                    points[label + "_" + name] = state[name].tolist()
        lines.append(line([left["gnd"], right["gnd"]], "ground"))
        if rc is not None:
            points["RC"] = rc.tolist()
        axes, annotations = frame_axes(frame, np.linalg.norm(right["gnd"]-left["gnd"]),
                                       max(self.zero["cg"][1]+.09, .4))
        lines.extend(axes)
        return {"metrics": metrics, "lines": lines, "points": points,
                "frame": frame, "annotations": annotations}

    def roll_bounds(self, heave_mm):
        """Roll attainable at this heave before either wheel reaches its stroke.

        Mirror symmetry gives opposite angles at opposite differential travels.
        If the linkage itself cannot reach the requested stroke, approach its
        continuous reachable boundary from the equal-travel pose.
        """
        key = float(heave_mm)
        if key not in self._roll_bounds_cache:
            low, high = self.travel_bounds
            check_pose_travel((heave_mm, heave_mm), (low, high))
            self.solve(heave_mm, heave_mm, gains=False)
            delta = max(0., min(heave_mm-low, high-heave_mm))
            def at_delta(value):
                return abs(self.solve(heave_mm-value, heave_mm+value, gains=False)["metrics"]["roll_deg"])
            try:
                angle = at_delta(delta)
            except Unreachable:
                reachable, unreachable = 0., delta
                angle = 0.
                for _ in range(30):
                    mid = (reachable+unreachable)/2
                    try:
                        candidate = at_delta(mid)
                    except Unreachable:
                        unreachable = mid
                    else:
                        reachable, angle = mid, candidate
            self._roll_bounds_cache[key] = (-float(angle), float(angle))
        return self._roll_bounds_cache[key]

    def pose(self, heave_mm, roll_deg):
        # Contact X moves during travel; iterate the actual track span.
        dz = self.zero["gnd"][0] * 2 * math.tan(math.radians(roll_deg)) * 1000
        for _ in range(20):
            l, r = heave_mm - dz/2, heave_mm + dz/2
            state = self.solve(l, r, gains=False)
            if abs(state["metrics"]["roll_deg"]-roll_deg) < 1e-7:
                if hasattr(self, "travel_bounds"):
                    check_pose_travel((l, r), self.travel_bounds)
                return self.solve(l, r)
            span = state["points"]["R_gnd"][0] - state["points"]["L_gnd"][0]
            dz = span * math.tan(math.radians(roll_deg))*1000
        raise Unreachable("Heave / roll 反算未收斂。")


class SVMotion:
    def __init__(self, template):
        self.template = template
        self.au, self.al = template.kingpin_Au.copy(), template.kingpin_Al.copy()
        self.length = np.linalg.norm(self.au - self.al)
        self.directions = getattr(template, "motion_directions", None)
        if self.directions is None:
            self.directions = []
            for p in (self.au, self.al):
                radial = template.initial_ic - p
                radial /= np.linalg.norm(radial)
                self.directions.append(np.array([-radial[1], radial[0]]))
        self.cache = {}

    def at_al(self, offset):
        du, dl = self.directions
        lower = self.al + offset * dl
        delta = self.au - lower
        b, c = 2*np.dot(du, delta), np.dot(delta, delta)-self.length**2
        disc = b*b-4*c
        if disc < -1e-12:
            raise Unreachable("SV 切線路徑與羊角無交點。")
        root = math.sqrt(max(0, disc))
        upper = min((self.au+(-b+root)/2*du, self.au+(-b-root)/2*du), key=lambda p: np.linalg.norm(p-self.au))
        old, new = self.au-self.al, upper-lower
        angle = math.atan2(new[1], new[0])-math.atan2(old[1], old[0])
        matrix = rotate2(angle)
        t = self.template
        transform = lambda p: lower + matrix @ (p-self.al)
        nu, nl = np.array([-du[1], du[0]]), np.array([-dl[1], dl[0]])
        return {"au": upper, "al": lower, "contact": transform(t.contact_point),
                "center": transform(t.wheel_center), "kg": transform(t.kingpin_ground),
                "kt": transform(t.kingpin_top), "ic": intersection(upper, upper+nu, lower, lower+nl),
                "angle": angle}

    def side(self, travel_mm):
        key = round(float(travel_mm), 9)
        if key not in self.cache:
            offset = scalar_solve(lambda x: self.at_al(x)["contact"][1], travel_mm/1000,
                                  0., .5)
            self.cache[key] = self.at_al(offset)
        return self.cache[key]


class Engine:
    def __init__(self, project):
        project = validate(project)
        self.project = copy.deepcopy(project)
        self.travel_bounds = travel_limits(project)
        self._pitch_angle_limit = None
        self.car, sf, sr = static_objects(project)
        if hardpoints.enabled(project):
            self.scene, self.derived_parameters = hardpoints.build_scene(project, self.car, sf, sr)
        else:
            self.scene = build_wireframe_scene(self.car, sf, sr, mount_x=project.get("arm_mount_x"))
            self.derived_parameters = {axle: copy.deepcopy(project[axle]) for axle in ("front", "rear")}
        self.fv = {axle: FVMotion(self.scene.geometry[axle]) for axle in ("front", "rear")}
        for solver in self.fv.values():
            solver.travel_bounds = self.travel_bounds
        sv = self.scene.geometry["side_view"]
        self.sv = {"front": SVMotion(sv.front), "rear": SVMotion(sv.rear)}
        for axle, solver in self.sv.items():
            t = solver.template
            values = axle_anti(axle, t.initial_ic, t.contact_point, t.wheel_center,
                               self.car.l, self.car.h_cog, project["vehicle"]["brake_bias_front"],
                               project["vehicle"]["drive_force_front"])
            self.derived_parameters[axle].update({key: values.get(key) for key in ("anti_dive", "anti_lift", "anti_squat")})
            self.derived_parameters[axle]["svsa"] = (None if t.initial_ic is None else
                float(t.direction_sign*(t.initial_ic[0]-t.contact_point[0])))

    def pitch_angle_limit(self):
        """Reference pitch extent from reachable opposing travel endpoints.

        Smaller endpoint fractions handle mechanisms that cannot reach the
        requested full stroke. Heave/angle combinations are checked separately.
        """
        if self._pitch_angle_limit is None:
            low, high = self.travel_bounds
            angles = [0.]
            for fraction in (1., .75, .5, .25, .1):
                for first, second in ((low*fraction, high*fraction), (high*fraction, low*fraction)):
                    try:
                        state = self.solve_sv(first, second)
                        angles.append(abs(state["metrics"]["pitch_deg"]))
                    except Unreachable:
                        continue
            self._pitch_angle_limit = float(max(angles))
        return self._pitch_angle_limit

    def solve_sv(self, front_mm, rear_mm):
        states = {"front": self.sv["front"].side(front_mm), "rear": self.sv["rear"].side(rear_mm)}
        f, r = states["front"], states["rear"]
        ground = r["contact"] - f["contact"]
        frame = contact_frame(f["contact"], r["contact"])
        pitch = math.atan2(ground[1], ground[0])
        basis = rotate2(-pitch)
        wheelbase = float(np.linalg.norm(ground))
        # CG belongs to the rigid body. Its road-relative position changes when
        # the body is rendered in the current contact frame, just like pickups.
        cg = np.array([self.sv["front"].template.wheel_center[0]+self.car.lf, self.car.h_cog])
        cg_height = float(to_contact_frame(cg, frame)[1])
        pc = None if f["ic"] is None or r["ic"] is None else intersection(f["contact"], f["ic"], r["contact"], r["ic"])
        metrics = {"travel_front_mm": front_mm, "travel_rear_mm": rear_mm,
                   "ride_height_front_mm": self.car.bottom*1000-front_mm,
                   "ride_height_rear_mm": self.car.bottom*1000-rear_mm,
                   "heave_mm": (front_mm+rear_mm)/2, "pitch_deg": math.degrees(pitch),
                   "wheelbase_mm": wheelbase*1000, "cg_height_mm": cg_height*1000,
                   **center_metrics("cg", cg, frame),
                   **center_metrics("pc", pc, frame)}
        initial_pc = self.scene.geometry["side_view"].local_pitch_center
        lines, points = [], {"CG": cg.tolist()}
        if initial_pc is not None:
            points["PC0"] = initial_pc.tolist()
        for axle, s in states.items():
            t = self.sv[axle].template
            v = basis @ (s["kt"]-s["kg"])
            metrics[f"caster_{axle}_deg"] = t.direction_sign * math.degrees(math.atan2(v[0], v[1]))
            values = axle_anti(axle, s["ic"], s["contact"], s["center"], wheelbase, cg_height,
                               self.project["vehicle"]["brake_bias_front"], self.project["vehicle"]["drive_force_front"], basis)
            metrics.update({f"{key}_{axle}_pct": None if value is None else 100*value for key, value in values.items()})
            metrics[f"svsa_{axle}_mm"] = None if s["ic"] is None else float(t.direction_sign*(s["ic"][0]-s["contact"][0])*1000)
            for a, b, kind in (("au", "al", "upright"), ("kg", "kt", "kingpin")):
                lines.append(line([s[a], s[b]], kind))
            if s["ic"] is not None:
                for k in ("au", "al", "center", "contact"):
                    lines.append(line([s[k], s["ic"]], "force", True))
            angles = np.linspace(0, 2*math.pi, 49)
            circle = s["center"] + self.car.free_radius*np.column_stack((np.cos(angles), np.sin(angles)))
            lines.append(line(circle, "Front wheel" if axle == "front" else "Rear wheel", color="#38bdf8" if axle == "front" else "#fb923c"))
            for k in ("au", "al", "kg", "kt", "contact", "center", "ic"):
                if s[k] is not None:
                    points[axle+"_"+k] = s[k].tolist()
        lines.append(line([f["contact"], r["contact"]], "ground"))
        if pc is not None:
            points["PC"] = pc.tolist()
        axes, annotations = frame_axes(frame, np.linalg.norm(ground), max(self.car.h_cog+.09, .4))
        lines.extend(axes)
        return {"metrics": metrics, "lines": lines, "points": points,
                "frame": frame, "annotations": annotations}

    def sv_pose(self, heave_mm, pitch_deg):
        dz = self.car.l*math.tan(math.radians(pitch_deg))*1000
        for _ in range(20):
            f, r = heave_mm-dz/2, heave_mm+dz/2
            state = self.solve_sv(f, r)
            if abs(state["metrics"]["pitch_deg"]-pitch_deg) < 1e-7:
                check_pose_travel((f, r), self.travel_bounds)
                return state
            span = state["points"]["rear_contact"][0]-state["points"]["front_contact"][0]
            dz = span*math.tan(math.radians(pitch_deg))*1000
        raise Unreachable("Heave / pitch 反算未收斂。")

    def solve_we(self, steer_deg, axle="front", caster_deg=None):
        corner = "FR" if axle == "front" else "RR"
        kingpin = self.scene.geometry["kingpins"][corner]
        origin = kingpin["ground"]
        axis = kingpin["UBJ"]-kingpin["LBJ"]
        if caster_deg is not None:
            # Same projection convention as static SV (+front, mirrored rear).
            axis = axis.copy()
            axis[0] = (-1 if axle == "front" else 1) * axis[2]*math.tan(math.radians(caster_deg))
        axis = axis/np.linalg.norm(axis)
        matrix = rotation_matrix(axis, math.radians(steer_deg))
        center0 = self.scene.geometry["wheel_projection_geometry"][corner]["center"]
        center = origin+matrix@(center0-origin)
        radius = float(self.car.load_Radius[0 if axle == "front" else 1])
        bottom0 = center0 - [0, 0, radius]
        reference = origin+matrix@(bottom0-origin)
        normal = matrix@np.array([0., 1., 0.])
        down = np.array([0., 0., 1.])-normal[2]*normal
        contact = center-radius*down/np.linalg.norm(down)
        projection = origin+axis*np.dot(contact-origin, axis)
        metrics = {"steer_deg": float(steer_deg),
                   "caster_deg": math.degrees(math.atan2(axis[0]*(-1 if axle == "front" else 1), axis[2])),
                   "wheel_tilt_deg": math.degrees(math.atan2(normal[2], np.linalg.norm(normal[:2]))),
                   "contact_radius_mm": float(np.linalg.norm(contact-projection)*1000),
                   "material_radius_mm": float(np.linalg.norm(np.cross(reference-origin, axis))*1000)}
        for label, point in (("contact", contact), ("material", reference), ("center", center)):
            for k, value in zip("xyz", point):
                metrics[f"{label}_{k}_mm"] = float(value*1000)
        for k, force in zip("xyz", np.eye(3)):
            metrics[f"moment_arm_f{k}_mm"] = float(np.dot(np.cross(contact-origin, force), axis)*1000)
        angles = np.linspace(0, 2*math.pi, 49)
        lines = [line([origin-axis*.08, origin+axis*.6], "kingpin"),
                 line([center-normal*self.car.tire_width/2, center+normal*self.car.tire_width/2], "upright"),
                 line([projection, contact], "Contact radius", color="#34d399")]
        for width in (-self.car.tire_width/2, 0, self.car.tire_width/2):
            circle = center0 + np.column_stack((self.car.free_radius*np.sin(angles), np.full(len(angles), width), self.car.free_radius*np.cos(angles)))
            lines.append(line((circle-origin)@matrix.T+origin, "wheel"))
        lines.append(line([origin, reference], "Material reference", True, "#fb923c"))
        return {"metrics": metrics, "lines": lines,
                "points": {"contact": contact.tolist(), "material": reference.tolist(), "center": center.tolist(),
                           "kingpin_ground": origin.tolist()}}
