"""Choose chassis X on an arm plane, bounded by the original clearance rays."""
import copy

import numpy as np


def position_arm_mounts(mounts, arm_planes, requests, *, arm):
    """Resolve each mirrored arm pair, falling back as a pair on any violation.

    Missing/None X requests use the original extrema. The original clearance
    construction is retained under limit_mounts/limit_legs for inspection.
    Requests and all checks use metres; excess_mm is for UI/report display.
    """
    result = copy.deepcopy(mounts)
    prefix = "U" if arm == "upper" else "L"
    names = (prefix + "FI", prefix + "RI")
    keys = (arm + "_front_x", arm + "_rear_x")
    for corner, data in result.items():
        limit = copy.deepcopy(data["mounts"])
        data["limit_mounts"] = limit
        data["limit_legs"] = copy.deepcopy(data["legs"])
        checks = {}
        data["position_checks"] = checks
        if not all(name in limit for name in names):
            for key, name in zip(keys, names):
                checks[key] = {"requested_x": requests.get(key),
                               "actual_x": None if name not in limit else float(limit[name][0]),
                               "limit_min_x": None, "limit_max_x": None, "excess_mm": 0.,
                               "fallback": True, "message": "無法取得兩個有限極限接點；保留原幾何。"}
            continue
        low, high = float(limit[names[1]][0]), float(limit[names[0]][0])
        values = [float(limit[name][0]) if requests.get(key) is None else float(requests[key])
                  for key, name in zip(keys, names)]
        excess = [max(0., low - x, x - high) for x in values]
        invalid_range = any(distance > 1e-12 for distance in excess)
        invalid_order = values[1] >= values[0] - 1e-12
        fallback = invalid_range or invalid_order
        points = []
        plane = arm_planes[corner + "_" + arm]
        normal, origin = plane["normal"], plane["vertices"][0]
        cannot_solve = False
        if not fallback:
            for key, name, x in zip(keys, names, values):
                if requests.get(key) is None:
                    points.append(limit[name].copy())
                    continue
                if abs(normal[2]) < 1e-12:
                    cannot_solve = True
                    break
                x = min(high, max(low, x))
                y = data["target_y"]
                z = origin[2] - (normal[0]*(x-origin[0]) + normal[1]*(y-origin[1]))/normal[2]
                point = np.array([x, y, z], dtype=float)
                if not np.isfinite(point).all():
                    cannot_solve = True
                    break
                points.append(point)
        fallback = fallback or cannot_solve
        if not fallback:
            data["mounts"] = dict(zip(names, points))
            for leg in data["legs"]:
                name = leg["name"]
                if name in data["mounts"] and requests.get(keys[names.index(name)]) is not None:
                    point = data["mounts"][name]
                    joint = data["UBJ"] if arm == "upper" else data["LBJ"]
                    leg.update(endpoint=point.copy(), point=point.copy(), direction=point-joint,
                               status="specified_x", is_extension=True, t=1., pose=None, poses=[], angles_deg=[])
        for key, name, distance in zip(keys, names, excess):
            message = ""
            if distance > 1e-12:
                message = f"超過極限範圍 {distance*1000:.6g} mm；同一 A 臂已使用極限接點。"
            elif invalid_order:
                difference = max(0., values[1]-values[0])*1000
                message = (f"前後順序錯誤：後接點 X 必須小於前接點 X（超前 {difference:.6g} mm）；"
                           "同一 A 臂已使用極限接點。")
            elif invalid_range:
                message = "同一 A 臂的另一個接點超限；已使用極限接點。"
            elif cannot_solve:
                message = "固定 X、Y 後無法由此平面唯一求 Z；同一 A 臂已使用極限接點。"
            checks[key] = {"requested_x": requests.get(key), "actual_x": float(data["mounts"][name][0]),
                           "limit_min_x": low, "limit_max_x": high, "excess_mm": float(distance*1000),
                           "fallback": bool(fallback), "message": message}
    return result
