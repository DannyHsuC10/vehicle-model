"""Rectangular parameter sweeps with explicit invalid cells and cached side solves."""
import numpy as np

from .engine import Unreachable, line

PAGES = {"front": "前 FV", "rear": "後 FV", "sv": "SV 側視", "we": "WE3D 輪端", "static": "靜態全車"}
PLOTS = {
    "front": [("motion", "FV 機構動態"), ("rc_map", "Roll center 位置分布（x′–y′）"),
              ("rc_y_prime_mm", "RC 高度曲面（y′）"), ("rc_x_prime_mm", "RC 位置曲面（x′）"),
              ("camber_left_deg", "左輪 Camber 曲面"),
              ("camber_right_deg", "右輪 Camber 曲面"), ("camber_gain_left_deg_per_mm", "左輪 Camber gain 曲面"),
              ("camber_gain_right_deg_per_mm", "右輪 Camber gain 曲面")],
    "sv": [("motion", "SV 機構動態"), ("caster_front_deg", "前 Caster 曲面"), ("caster_rear_deg", "後 Caster 曲面"),
           ("anti_dive_front_pct", "前 Anti-dive 曲面"), ("anti_dive_rear_pct", "後 Anti-dive 曲面"),
           ("anti_lift_front_pct", "前 Anti-lift 曲面"), ("anti_lift_rear_pct", "後 Anti-lift 曲面"),
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
    "front": "RC 使用接地座標：原點為兩接地點中點，x′ 左→右，y′ 垂直向上。RC0 為固定車身零位參考。Camber 相對接地線，向外為正；gain 為局部偏導 deg/mm。",
    "rear": "後軸使用獨立設計。RC 使用接地中點與隨接地線旋轉的 x′／y′；RC0 為固定車身零位參考。圓點標示 RC、RC0、CG、IC L／R。",
    "sv": "PC 使用接地中點座標：x′ 前→後，y′ 垂直向上。沿用 Au/Al 固定切線路徑；正行程=輪側上壓。Anti 沿用原公式，兩軸 dive 均以前煞車比例正規化，非完整制動受力分析。",
    "we": "繞靜態全車主銷轉向；座標 X 前／Y 左／Z 上。接地點=負載半徑中心圓最低點，未解輪胎變形或頂升平衡；material 為原接地材料點。固定主銷 caster 不隨 steering 改變；曲面掃描不同設計 caster。",
    "static": "完整靜態構造直接來自 StaticWireframe；X 前／Y 左／Z 上，單位 m。FV/SV 為參考平面，非 3D 懸吊連桿運動求解。",
}


def calculate_mapping(engine, page, mode="travel", axle="front", cancelled=lambda: False):
    a = engine.project["analysis"]
    n = int(a["samples"])
    travel = np.linspace(-a["travel_mm"], a["travel_mm"], n)
    if page == "we":
        xs = np.linspace(-a["steer_deg"], a["steer_deg"], n)
        ys = np.linspace(a["caster_min_deg"], a["caster_max_deg"], n)
        labels = ("Steering (deg)", "Design caster (deg)")
    elif mode == "pose":
        xs = travel
        angle_key = "pitch_deg" if page == "sv" else "roll_deg"
        ys = np.linspace(-a[angle_key], a[angle_key], n)
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
            try:
                if page == "we":
                    metrics = engine.solve_we(float(x), axle, float(y))["metrics"]
                elif page == "sv":
                    metrics = (engine.sv_pose(float(x), float(y)) if mode == "pose"
                               else engine.solve_sv(float(x), float(y)))["metrics"]
                else:
                    metrics = (engine.fv[page].pose(float(x), float(y)) if mode == "pose"
                               else engine.fv[page].solve(float(x), float(y)))["metrics"]
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
            entry = {"sample_kind": "steering_sweep", "axis_x": float(x), "axis_y": engine.project[axle]["caster_deg"],
                     "valid": True, "error": "", **engine.solve_we(float(x), axle)["metrics"]}
            sweep.append(entry)
        rows.extend(sweep)
    return {"grid": grid, "rows": rows, "sweep": sweep, "xlabel": labels[0], "ylabel": labels[1],
            "valid": sum(r["valid"] for r in rows), "total": len(rows), "mode": mode, "axle": axle}


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


def make_plot(page, plot, state=None, mapping=None, engine=None, groups=None, show_static_references=True):
    if page == "static":
        visible = groups or {"front": True, "rear": True, "sv": True}
        lines = [line(l.points, l.group, color=l.color, dash=l.dash)
                 for l in engine.scene.lines if visible.get(l.group, True)
                 and (show_static_references or not l.reference)]
        return {"title": "StaticWireframe · 靜態全車", "world_lines": lines, "dimension": 3,
                "projection": {"iso": "Iso", "front": "Front", "side": "Side", "top": "Top"}[plot],
                "xlabel": "m", "ylabel": "m", "equal": True}
    if plot == "motion":
        return {"title": PAGES[page] + " · 幾何動態", "world_lines": state["lines"],
                "dimension": 3 if page == "we" else 2, "projection": "Iso" if page == "we" else "2D",
                "xlabel": "X (m)" if page == "we" else "Body X (m) · x′ / y′ rotate with contact line",
                "ylabel": "Z (m)", "equal": True,
                "markers": motion_markers(page, state["points"]),
                "annotations": state.get("annotations", [])}
    if mapping is None:
        return None
    title = dict(PLOTS[page])[plot]
    if page == "we" and plot != "caster_surface":
        rows = mapping["sweep"]
        if plot == "path":
            points = [[r[f"contact_{k}_mm"]/1000 for k in "xyz"] for r in rows]
            return {"title": title, "world_lines": [line(points, "Contact path", color="#38bdf8")],
                    "dimension": 3, "projection": "Iso", "equal": True, "xlabel": "m", "ylabel": "m"}
        fields, unit = {
            "radius": (["contact_radius_mm", "material_radius_mm"], "mm"),
            "xyz": ([f"contact_{k}_mm" for k in "xyz"], "mm"),
            "tilt": (["caster_deg", "wheel_tilt_deg"], "deg"),
            "arms": ([f"moment_arm_f{k}_mm" for k in "xyz"], "mm"),
        }[plot]
        return {"title": title, "xlabel": "Steering (deg)", "ylabel": unit,
                "series": [{"name": k, "x": [r["steer_deg"] for r in rows], "y": [r[k] for r in rows]} for k in fields]}
    if plot in ("rc_map", "pc_map"):
        xkey, ykey = ("rc_x_prime_mm", "rc_y_prime_mm") if plot == "rc_map" else ("pc_x_prime_mm", "pc_y_prime_mm")
        paths = list(mapping["grid"]) + list(zip(*mapping["grid"]))
        return {"title": title, "xlabel": "x′ (mm)", "ylabel": "y′ (mm)", "equal": True,
                "series": [{"name": "Mapping rows" if i < len(mapping["grid"]) else "Mapping columns",
                            "x": [r.get(xkey) for r in row], "y": [r.get(ykey) for r in row],
                            "color": "#38bdf8" if i < len(mapping["grid"]) else "#a78bfa"} for i, row in enumerate(paths)]}
    metric = "wheel_tilt_deg" if plot == "caster_surface" else plot
    mesh = [[[r["axis_x"], r["axis_y"], r.get(metric)] for r in row] for row in mapping["grid"]]
    zlabel = {"wheel_tilt_deg": "Wheel tilt (deg)", "rc_y_prime_mm": "RC y′ (mm)", "rc_x_prime_mm": "RC x′ (mm)",
              "pc_y_prime_mm": "PC y′ (mm)", "pc_x_prime_mm": "PC x′ (mm)",
              "camber_left_deg": "Camber L (deg)", "camber_right_deg": "Camber R (deg)",
              "camber_gain_left_deg_per_mm": "Camber gain L (deg/mm)",
              "camber_gain_right_deg_per_mm": "Camber gain R (deg/mm)"}.get(metric, metric.replace("_", " "))
    return {"title": title, "mesh": mesh, "projection": "Iso", "xlabel": mapping["xlabel"],
            "ylabel": mapping["ylabel"], "zlabel": zlabel}


def static_point_rows(engine):
    rows = []
    def add(name, point):
        rows.append({"name": name, **{k+"_m": float(v) for k, v in zip("xyz", point)}})
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
