"""Stroke and pose sweeps with explicit invalid cells and cached side solves."""
import numpy as np

from .engine import Unreachable, line
from .project import travel_limits

PAGES = {"front": "前 FV", "rear": "後 FV", "sv": "SV 側視", "we": "WE3D 輪端", "static": "靜態全車"}
PLOTS = {
    "front": [("motion", "FV 機構動態"), ("rc_map", "Roll center 位置分布（x′–y′）"),
              ("rc_y_prime_mm", "RC 高度曲面（y′）"), ("rc_x_prime_mm", "RC 位置曲面（x′）"),
              ("camber_left_deg", "左輪 Camber 曲面"),
              ("camber_right_deg", "右輪 Camber 曲面"), ("camber_gain_left_deg_per_mm", "左輪 Camber gain 曲面"),
              ("camber_gain_right_deg_per_mm", "右輪 Camber gain 曲面")],
    "sv": [("motion", "SV 機構動態"), ("caster_front_deg", "前 Caster 曲面"), ("caster_rear_deg", "後 Caster 曲面"),
           ("anti_dive_front_pct", "前 Anti-dive（煞車）曲面"), ("anti_lift_rear_pct", "後 Anti-lift（煞車）曲面"),
           ("anti_lift_front_pct", "前 Anti-lift（加速）曲面"), ("anti_squat_rear_pct", "後 Anti-squat（加速）曲面"),
           ("pc_y_prime_mm", "Pitch center 高度曲面（y′）"), ("pc_x_prime_mm", "Pitch center 位置曲面（x′）"),
           ("pc_map", "Pitch center 位置分布（x′–y′）")],
    "we": [("motion", "輪端 3D 轉向動態"), ("caster_surface", "Steering × Caster → 輪軸傾角曲面"),
           ("radius", "接地點至主銷半徑變化"), ("xyz", "接地點 XYZ vs Steering"),
           ("path", "接地點 3D 軌跡"), ("tilt", "Caster / 輪軸傾角 vs Steering"),
           ("arms", "Fx / Fy / Fz 轉向力臂")],
    "static": [("iso", "全車 3D"), ("front", "全車前視"), ("side", "全車側視"), ("top", "全車上視")],
}
PLOTS["rear"] = PLOTS["front"]
NOTES = {
    "front": "動態圖使用接地座標：原點為兩接地點中點，x′ 左→右，y′ 垂直向上。CG 與車身接點一同轉換，隨升降／roll 移動。RC0 為車身零位參考；gain 為局部偏導 deg/mm。",
    "rear": "後軸使用獨立設計。動態圖與 RC 均使用當前接地座標；CG 與車身接點一同轉換。RC0 為車身零位參考，圖中隨車身姿態移動。",
    "sv": "當前接地座標 x′ 前→後、y′ 向上，CG 隨姿態移動。輪端煞車：接地點→IC，前 Anti-dive／後 Anti-lift 使用煞車比例；加速：輪心→IC，前 Anti-lift／後 Anti-squat 使用獨立驅動力比例。未解輪胎極限與車身動力平衡。",
    "we": "繞靜態全車主銷轉向；座標 X 前／Y 左／Z 上。接地點=負載半徑中心圓最低點，未解輪胎變形或頂升平衡；material 為原接地材料點。固定主銷 caster 不隨 steering 改變；曲面掃描不同設計 caster。",
    "static": "完整靜態構造直接來自 StaticWireframe；X 前／Y 左／Z 上，單位 m。FV/SV 為參考平面，非 3D 懸吊連桿運動求解。",
}


def calculate_mapping(engine, page, mode="travel", axle="front", cancelled=lambda: False):
    a = engine.project["analysis"]
    n = int(a["samples"])
    fv_pose = mode == "pose" and page in ("front", "rear")
    low, high = travel_limits(engine.project)
    travel = np.linspace(low, high, n)
    if low < 0 < high:
        # Retain both endpoints and include the static pose for asymmetric ranges.
        travel[1+int(np.argmin(np.abs(travel[1:-1])))] = 0.
    if page == "we":
        xs = np.linspace(-a["steer_deg"], a["steer_deg"], n)
        ys = np.linspace(a["caster_min_deg"], a["caster_max_deg"], n)
        labels = ("Steering (deg)", "Design caster (deg)")
    elif mode == "pose":
        # FV scans the full left/right stroke square, then projects it into
        # Heave/Roll. No independent angular extent clips the reachable domain.
        xs = travel
        ys = travel if fv_pose else np.linspace(-engine.pitch_angle_limit(), engine.pitch_angle_limit(), n)
        labels = ("Heave (mm)", "Pitch (deg)" if page == "sv" else "Roll (deg)")
    else:
        xs, ys = travel, travel
        labels = ("Front ride height (mm)", "Rear ride height (mm)") if page == "sv" else ("Left travel (mm)", "Right travel (mm)")
    grid, rows = [], []
    for y in ys:
        if cancelled():
            return None
        row = []
        for x in xs:
            entry = {"sample_kind": "surface", "axis_x": float(x), "axis_y": float(y)}
            if fv_pose:
                entry.update(axis_x=float((x+y)/2), axis_y=None,
                             travel_left_mm=float(x), travel_right_mm=float(y))
            try:
                if page == "we":
                    metrics = engine.solve_we(float(x), axle, float(y))["metrics"]
                elif page == "sv":
                    metrics = (engine.sv_pose(float(x), float(y)) if mode == "pose"
                               else engine.solve_sv(float(x), float(y)))["metrics"]
                else:
                    metrics = engine.fv[page].solve(float(x), float(y))["metrics"]
                    if fv_pose:
                        entry.update(axis_x=metrics["heave_mm"], axis_y=metrics["roll_deg"])
                entry.update(metrics, valid=True, error="")
            except Unreachable as exc:
                entry.update(valid=False, error=str(exc))
            row.append(entry)
            rows.append(entry)
        grid.append(row)
    if page == "sv" and mode == "travel":
        # Display front/rear ride heights while retaining actual travel columns.
        for entry in rows:
            entry["axis_x"] = engine.car.bottom*1000-entry["axis_x"]
            entry["axis_y"] = engine.car.bottom*1000-entry["axis_y"]
    sweep = []
    if page == "we":
        for x in xs:
            entry = {"sample_kind": "steering_sweep", "axis_x": float(x), "axis_y": engine.derived_parameters[axle]["caster_deg"],
                     "valid": True, "error": "", **engine.solve_we(float(x), axle)["metrics"]}
            sweep.append(entry)
        rows.extend(sweep)
    result = {"grid": grid, "rows": rows, "sweep": sweep, "xlabel": labels[0], "ylabel": labels[1],
              "valid": sum(r["valid"] for r in rows), "total": len(rows), "mode": mode, "axle": axle}
    if fv_pose:
        result["travel_bounds"] = [low, high]
    return result


def motion_markers(page, points):
    if page == "we":
        return []
    center = "PC" if page == "sv" else "RC"
    markers = []
    for key, label, color, radius, hollow, offset in (
            (center+"0", center+"0", "#f8fafc", 10, True, [-32, 24]),
            (center, center, "#fb7185", 6, False, [12, -15]),
            ("CG", "CG", "#fbbf24", 7, False, [12, -15]),
            ("front_ic" if page == "sv" else "L_IC", "IC F" if page == "sv" else "IC L", "#2dd4bf", 7, False, [-40, -16]),
            ("rear_ic" if page == "sv" else "R_IC", "IC R", "#c4b5fd", 7, False, [12, -16])):
        if key in points:
            markers.append({"position": points[key], "name": label, "color": color,
                            "radius": radius, "hollow": hollow, "offset": offset})
    return markers


def with_current_position(spec, page, plot, state, mapping):
    """Annotate solved pose values, independently of the cached sweep grid."""
    spec = {**spec, "markers": [], "current_status": ""}
    metrics = (state or {}).get("metrics")
    if not metrics:
        spec["current_status"] = "目前姿態無有效解"
        return spec

    def finite(values):
        return all(value is not None and np.isfinite(value) for value in values)

    def fmt(value):
        return f"{0 if abs(value) < 1e-9 else value:.5g}"

    if page == "we":
        axes = [metrics.get("steer_deg"), metrics.get("caster_deg")]
    elif mapping["mode"] == "pose":
        axes = [metrics.get("heave_mm"), metrics.get("pitch_deg" if page == "sv" else "roll_deg")]
    elif page == "sv":
        axes = [metrics.get("ride_height_front_mm"), metrics.get("ride_height_rear_mm")]
    else:
        axes = [metrics.get("travel_left_mm"), metrics.get("travel_right_mm")]
    if not finite(axes):
        spec["current_status"] = "目前座標無有效定義"
        return spec
    if "travel_bounds" in mapping:
        low, high = mapping["travel_bounds"]
        travels = [metrics.get("travel_left_mm"), metrics.get("travel_right_mm")]
        if not finite(travels) or any(value < low-1e-5 or value > high+1e-5 for value in travels):
            spec["current_status"] = "目前位置超出懸吊行程範圍"
            return spec
    dimensions = 1 if page == "we" and plot != "caster_surface" else 2
    for axis in range(dimensions):
        key = ("axis_x", "axis_y")[axis]
        samples = [entry[key] for row in mapping["grid"] for entry in row if finite([entry[key]])]
        if not samples:
            spec["current_status"] = "掃描範圍沒有有效座標"
            return spec
        low, high = min(samples), max(samples)
        tolerance = 1e-6*max(1, abs(low), abs(high))
        if axes[axis] < low-tolerance or axes[axis] > high+tolerance:
            spec["current_status"] = "目前位置超出掃描範圍"
            return spec

    def marker(position, labels, name="目前位置", offset=None, color="#fb7185"):
        return {"position": [float(v) for v in position], "name": name, "label_lines": labels,
                "color": color, "radius": 7, "offset": offset or [12, -24]}

    axis_labels = [f"{mapping['xlabel']}: {fmt(axes[0])}", f"{mapping['ylabel']}: {fmt(axes[1])}"]
    if page == "sv" and mapping["mode"] == "travel":
        axis_labels.append(f"前／後行程: {fmt(metrics['travel_front_mm'])} / {fmt(metrics['travel_rear_mm'])} mm")
    if "mesh" in spec:
        key = "wheel_tilt_deg" if plot == "caster_surface" else plot
        value = metrics.get(key)
        if finite([value]):
            spec["markers"] = [marker([*axes, value], ["目前位置", *axis_labels, f"{spec['zlabel']}: {fmt(value)}"])]
    elif plot in ("rc_map", "pc_map"):
        prefix = "rc" if plot == "rc_map" else "pc"
        position = [metrics.get(prefix+"_x_prime_mm"), metrics.get(prefix+"_y_prime_mm")]
        if finite(position):
            spec["markers"] = [marker(position, ["目前位置", f"x′: {fmt(position[0])} mm, y′: {fmt(position[1])} mm", *axis_labels])]
    elif page == "we" and plot == "path":
        xyz = [metrics.get(f"contact_{axis}_mm") for axis in "xyz"]
        if finite(xyz):
            spec["markers"] = [marker(np.array(xyz)/1000, ["目前位置", f"Steering: {fmt(axes[0])} deg",
                                  " / ".join(f"{axis.upper()}: {fmt(value)}" for axis, value in zip("xyz", xyz))+" mm"])]
    else:
        colors = ("#38bdf8", "#fbbf24", "#a78bfa")
        for index, series in enumerate(spec.get("series", [])):
            key = series["name"]
            value = metrics.get(key)
            if not finite([value]):
                continue
            labels = [f"{key}: {fmt(value)} {spec['ylabel']}"]
            if index == 0:
                labels.insert(0, f"目前 Steering: {fmt(axes[0])} deg")
            spec["markers"].append(marker([axes[0], value], labels, "目前 "+key,
                                           [12, -30 if index == 0 else index*30-6], colors[index % len(colors)]))
    if not spec["markers"]:
        spec["current_status"] = "目前數值無有限定義（N/A）"
    return spec


def make_plot(page, plot, state=None, mapping=None, engine=None, groups=None, show_static_references=True):
    if page == "static":
        visible = groups or {"front": True, "rear": True, "sv": True}
        lines = [line(l.points, l.group, color=l.color, dash=l.dash)
                 for l in engine.scene.lines if visible.get(l.group, True)
                 and (show_static_references or not l.reference)]
        markers = [{"position": point, "name": corner+"_"+name, "color": "#fbbf24", "radius": 4}
                   for corner, points in engine.scene.geometry.get("hardpoints", {}).items() for name, point in points.items()]
        return {"title": "StaticWireframe · 靜態全車", "world_lines": lines, "dimension": 3, "markers": markers,
                "projection": {"iso": "Iso", "front": "Front", "side": "Side", "top": "Top"}[plot],
                "xlabel": "m", "ylabel": "m", "equal": True}
    if plot == "motion":
        paths = state["lines"]
        markers = motion_markers(page, state["points"])
        annotations = state.get("annotations", [])
        contact_view = page != "we" and "frame" in state
        if contact_view:
            frame = state["frame"]
            origin = np.asarray(frame["origin"])
            basis = np.asarray([frame["x_axis"], frame["y_axis"]])
            transform = lambda points: ((np.asarray(points)-origin)@basis.T).tolist()
            paths = [{**path, "points": transform(path["points"])} for path in paths]
            def transform_annotation(annotation):
                return {key: transform(value) if key in ("position", "start") else value
                        for key, value in annotation.items()}
            markers = [transform_annotation(marker) for marker in markers]
            annotations = [transform_annotation(annotation) for annotation in annotations]
        return {"title": PAGES[page] + " · 幾何動態" + (" · 接地座標" if contact_view else ""), "world_lines": paths,
                "dimension": 3 if page == "we" else 2, "projection": "Iso" if page == "we" else "2D",
                "xlabel": "X (m)" if page == "we" else "x′ (m) · Current contact frame",
                "ylabel": "Z (m)" if page == "we" else "y′ (m)", "equal": True,
                "coordinate_system": "contact" if contact_view else "vehicle",
                "markers": markers, "annotations": annotations}
    if mapping is None:
        return None
    title = dict(PLOTS[page])[plot]
    if page == "we" and plot != "caster_surface":
        rows = mapping["sweep"]
        if plot == "path":
            points = [[r[f"contact_{k}_mm"]/1000 for k in "xyz"] for r in rows]
            return with_current_position({"title": title, "world_lines": [line(points, "Contact path", color="#38bdf8")],
                    "dimension": 3, "projection": "Iso", "equal": True, "xlabel": "m", "ylabel": "m"}, page, plot, state, mapping)
        fields, unit = {
            "radius": (["contact_radius_mm", "material_radius_mm"], "mm"),
            "xyz": ([f"contact_{k}_mm" for k in "xyz"], "mm"),
            "tilt": (["caster_deg", "wheel_tilt_deg"], "deg"),
            "arms": ([f"moment_arm_f{k}_mm" for k in "xyz"], "mm"),
        }[plot]
        return with_current_position({"title": title, "xlabel": "Steering (deg)", "ylabel": unit,
                "series": [{"name": k, "x": [r["steer_deg"] for r in rows], "y": [r[k] for r in rows]} for k in fields]}, page, plot, state, mapping)
    if plot in ("rc_map", "pc_map"):
        xkey, ykey = ("rc_x_prime_mm", "rc_y_prime_mm") if plot == "rc_map" else ("pc_x_prime_mm", "pc_y_prime_mm")
        paths = list(mapping["grid"]) + list(zip(*mapping["grid"]))
        return with_current_position({"title": title, "xlabel": "x′ (mm)", "ylabel": "y′ (mm)", "equal": True,
                "series": [{"name": "Mapping rows" if i < len(mapping["grid"]) else "Mapping columns",
                            "x": [r.get(xkey) for r in row], "y": [r.get(ykey) for r in row],
                            "color": "#38bdf8" if i < len(mapping["grid"]) else "#a78bfa"} for i, row in enumerate(paths)]}, page, plot, state, mapping)
    metric = "wheel_tilt_deg" if plot == "caster_surface" else plot
    mesh = [[[r["axis_x"], r["axis_y"], r.get(metric)] for r in row] for row in mapping["grid"]]
    zlabel = {"wheel_tilt_deg": "Wheel tilt (deg)", "rc_y_prime_mm": "RC y′ (mm)", "rc_x_prime_mm": "RC x′ (mm)",
              "pc_y_prime_mm": "PC y′ (mm)", "pc_x_prime_mm": "PC x′ (mm)",
              "camber_left_deg": "Camber L (deg)", "camber_right_deg": "Camber R (deg)",
              "camber_gain_left_deg_per_mm": "Camber gain L (deg/mm)",
              "camber_gain_right_deg_per_mm": "Camber gain R (deg/mm)"}.get(metric, metric.replace("_", " "))
    return with_current_position({"title": title, "mesh": mesh, "projection": "Iso", "xlabel": mapping["xlabel"],
            "ylabel": mapping["ylabel"], "zlabel": zlabel}, page, plot, state, mapping)


def static_point_rows(engine):
    rows = []
    def add(name, point):
        rows.append({"name": name, **{k+"_m": float(v) for k, v in zip("xyz", point)}})
    if "hardpoints" in engine.scene.geometry:
        from .hardpoints import CORNERS, POINTS
        for corner in CORNERS:
            for key in POINTS:
                add(corner+"_"+key, engine.scene.geometry["hardpoints"][corner][key])
        return rows
    for corner, data in engine.scene.geometry["kingpins"].items():
        for key in ("ground", "UBJ", "LBJ", "axis_top"):
            add(corner+"_"+key, data[key])
    for corner, wheel in engine.scene.geometry["wheel_projection_geometry"].items():
        add(corner+"_wheel_center", wheel["center"])
    for group in ("upper_arm_mount_geometry", "lower_arm_mount_geometry", "rear_upper_arm_mount_geometry", "rear_lower_arm_mount_geometry"):
        for corner, geometry in engine.scene.geometry[group].items():
            for key, point in geometry["mounts"].items():
                add(corner+"_"+key, point)
    return rows
