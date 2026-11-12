"""Portable setup files, validation and import of the design lab's linear settings."""

import csv
import json
from pathlib import Path
from uuid import uuid4

from .components import KCSystem, MAX_CURVE_POINTS, damper_points, number, points, spring_points
from .road_profiles import ISO_FREQUENCY_BAND, ISO_ROUGHNESS


APP = "SuspensionSetupLab"
CORNERS = ("FL", "FR", "RL", "RR")
CHANNELS = {
    "heave_f": "前軸升沉", "heave_r": "後軸升沉", "roll_f": "前軸側傾", "roll_r": "後軸側傾",
    "roll_c": "前後同向側傾", "warp": "對角扭擺", **{c: f"{c} 直推" for c in CORNERS},
}
MODELS = {"rigid": "全車 7 DOF｜剛性車架", "flex": "全車 8 DOF｜可扭轉車架",
          "quarter": "1/4 車｜2 DOF", "half_lr": "1/2 左右｜4 DOF", "half_fr": "1/2 前後｜4 DOF"}
PAGE_MODELS = {"full": ("flex", "rigid"), "half_lr": ("half_lr",), "half_fr": ("half_fr",), "quarter": ("quarter",)}
MODEL_PAGE = {model: page for page, models in PAGE_MODELS.items() for model in models}
MODEL_SOURCES = {"quarter": {c: c for c in CORNERS}, "half_lr": {"front": "前軸", "rear": "後軸"},
                 "half_fr": {"left": "左側", "right": "右側"}}
DEFAULT_SOURCES = {model: next(iter(choices)) for model, choices in MODEL_SOURCES.items()}
AXES = {"rigid": (True, True), "flex": (True, True), "quarter": (False, False),
        "half_lr": (False, True), "half_fr": (True, False)}
SCENARIOS = {"bump": "凸塊／凹坑", **{key: f"ISO 8608 Class {key[-1].upper()}" for key in ISO_ROUGHNESS},
             "cornering": "側向加速度（轉彎）", "maneuver": "煞車／加速／轉彎",
             "combined": "凸塊＋加速度", "csv": "量測 CSV 回放"}
BUMP_CASES = ("bump", "combined")
MANEUVER_CASES = ("maneuver", "combined")
CASE_DEFAULTS = {"seed": 0, "iso_lateral": False}
DEFAULT_DAMPER_SWITCH_SPEED = .1
AERO_DEFAULTS = {"enabled": False, "reference_speed_kmh": 100., "reference_force_n": 500.}
AERO_FIELDS = {"reference_speed_kmh": ("參考車速 (km/h)", .1, 540),
               "reference_force_n": ("參考下壓力 (N)", 0, 1e6)}
TARGET_WHEELS = {**{c: (c,) for c in CORNERS}, "front": CORNERS[:2], "rear": CORNERS[2:],
                 "left": CORNERS[::2], "right": CORNERS[1::2], "all": CORNERS}
ROAD_HEADER = ("time_s", "z_fl_mm", "z_fr_mm", "z_rl_mm", "z_rr_mm", "ax_mps2", "ay_mps2")
SPRING_FORCE_HEADER = ("travel_mm", "force_n")
DAMPER_FORCE_HEADER = ("speed_mps", "compression_n", "rebound_n")
MR_HEADER = ("travel_mm", "mr")
DAMPER_FORCE_LABELS = {"compression": "壓縮阻尼", "rebound": "回彈阻尼"}
VEHICLE_FIELDS = {
    "m": ("總質量 (kg)", 1, 1e5), "lf": ("重心至前軸 (m)", .01, 30), "lr": ("重心至後軸 (m)", .01, 30),
    "tf": ("前輪距 (m)", .01, 30), "tr": ("後輪距 (m)", .01, 30),
    "h_cg": ("重心高度 (m)", 0, 20), "Ixx": ("簧上側傾慣量 (kg·m²)", .01, 1e8),
    "Iyy": ("簧上俯仰慣量 (kg·m²)", .01, 1e8), "h_rc_f": ("前側傾中心高度 (m)", -10, 10),
    "h_rc_r": ("後側傾中心高度 (m)", -10, 10),
    "dive_front": ("煞車前 Anti-dive (比例)", -2, 2), "lift_rear": ("煞車後 Anti-lift (比例)", -2, 2),
    "lift_front": ("加速前 Anti-lift (比例)", -2, 2), "squat_rear": ("加速後 Anti-squat (比例)", -2, 2),
    "body_twist_k": ("車架扭轉剛性 (N·m/rad)", .01, 1e9),
    "body_twist_c": ("車架扭轉阻尼 (N·m·s/rad)", 0, 1e7),
}
TIRE_FIELDS = {"mu": ("簧下質量 (kg)", .01, 1e4), "k": ("輪端剛性 (N/mm)", .01, 1e5),
               "c": ("輪端阻尼 (N·s/m)", 0, 1e6)}
CASE_FIELDS = {
    "duration": ("模擬時間 (s)", .001, 120), "dt": ("輸出間隔 (s)", .0001, .05),
    "start": ("輸入開始 (s)", 0, 120), "height": ("凸塊高度 (mm)", -1000, 1000),
    "period": ("通過凸塊時間 (s)", .002, 120), "speed": ("車速 (m/s)", 0, 150),
    "seed": ("路面隨機種子", 0, 2**32 - 1),
    "ax": ("縱向加速度 ax (m/s²)", -100, 100), "ay": ("側向加速度 ay (m/s²)", -100, 100),
    "ramp": ("加速度上升時間 (s)", .002, 120),
}
COMPONENT_FIELDS = {
    "installed_mm": "安裝預壓行程 (mm)", "main_rate": "彈簧／主簧剛性 (N/mm)",
    "tender_rate": "Tender 剛性 (N/mm)", "tender_travel": "Tender 併圈前行程 (mm)",
    "jounce_mm": "壓縮限位起點 (mm)", "jounce_rate": "壓縮限位後總剛性 (N/mm)",
    "rebound_mm": "伸長限位起點 (mm)", "rebound_rate": "伸長限位後總剛性 (N/mm)",
}


def linear(k, c, ratio=1.0):
    return {"enabled": bool(k or c), "spring_kind": "single", "spring": [[0, k / 1000]],
            "damper_kind": "stages", "compression": [[0, c]], "rebound": [[0, c]], "mr": [[0, ratio]],
            "installed_mm": 0., "main_rate": k / 1000,
            "tender_rate": 25., "tender_travel": 4.3, "jounce_mm": 50., "jounce_rate": 0.,
            "rebound_mm": 50., "rebound_rate": 0., "limit_rate_mode": "total"}


def _four_way_channel(k, c, *, spring_kind="single"):
    return dict(linear(k, c), spring_kind=spring_kind, damper_kind="bilinear",
                compression=[[0, c], [DEFAULT_DAMPER_SWITCH_SPEED, c]],
                rebound=[[0, c], [DEFAULT_DAMPER_SWITCH_SPEED, c]])


def with_total_limit_rates(project):
    """Convert legacy increments using the spring slope just outside each stop."""
    channels = {}
    for key, settings in project["channels"].items():
        if settings.get("limit_rate_mode") == "total":
            channels[key] = settings
            continue
        converted = {**settings, "limit_rate_mode": "total"}
        if settings["jounce_rate"] or settings["rebound_rate"]:
            component = KCSystem(settings)
            if component.jounce_rate:
                converted["jounce_rate"] += component.spring_slope(component.jounce, from_right=True)
            if component.rebound_rate:
                converted["rebound_rate"] += component.spring_slope(-component.rebound, from_right=False)
        channels[key] = converted
    return {**project, "channels": channels}


def model_corners(model, sources):
    if model in MODEL_SOURCES:
        return TARGET_WHEELS[sources.get(model, DEFAULT_SOURCES[model])]
    return CORNERS


def model_channels(model, corners):
    if model not in MODEL_SOURCES:
        return tuple(CHANNELS)
    axles = tuple(dict.fromkeys(corner[0].lower() for corner in corners))
    heave = tuple(f"heave_{axle}" for axle in axles)
    roll = tuple(f"roll_{axle}" for axle in axles) if model == "half_lr" else ()
    return (*heave, *roll, *corners)


def case_axes(scenario, axes, iso_lateral):
    if scenario in MANEUVER_CASES:
        return axes
    return False, axes[1] and (scenario == "cornering" or (scenario in ISO_ROUGHNESS and iso_lateral))


def analysis_page_cases(project):
    stored = project.get("page_cases", {})
    if not isinstance(stored, dict):
        raise ValueError("分析頁工況的格式有誤。")
    cases = stored.copy()
    if "half" in cases:
        legacy = cases.pop("half")
        if not isinstance(legacy, dict) or legacy.get("model") not in ("half_lr", "half_fr"):
            raise ValueError("半車分析頁的模型設定有誤。")
        cases.setdefault(MODEL_PAGE[legacy["model"]], legacy)
    return cases


def default_project():
    return {
        "application": APP, "schema_version": 1, "name": "全車懸吊調校", "model": "flex", "notes": [],
        "model_sources": DEFAULT_SOURCES.copy(),
        "aero": AERO_DEFAULTS.copy(),
        "vehicle": dict(m=321., lf=.765, lr=.765, tf=1.28, tr=1.24, h_cg=.3, Ixx=60., Iyy=100.,
                        h_rc_f=.025, h_rc_r=.025, dive_front=.15, lift_rear=.15, lift_front=.15,
                        squat_rear=.15, body_twist_k=180000., body_twist_c=300.),
        "tires": {c: dict(mu=15., k=80., c=300.) for c in CORNERS},
        "channels": {"heave_f": _four_way_channel(35000, 3350, spring_kind="tender"),
                     "heave_r": _four_way_channel(31500, 3175, spring_kind="tender"),
                     "roll_f": _four_way_channel(20000, 2500), "roll_r": _four_way_channel(18000, 2400),
                     "roll_c": _four_way_channel(0, 0),
                     **{key: linear(0, 0) for key in ("warp", *CORNERS)}},
        "case": dict(scenario="bump", target="FL", duration=2., dt=.002, start=.5, height=15.,
                     period=.12, speed=12., ax=-5., ay=5., ramp=.2, rear_delay=False,
                     recording=[], recording_name="", **CASE_DEFAULTS),
    }


def validate_recording(rows):
    if not isinstance(rows, list) or not 2 <= len(rows) <= 100001:
        raise ValueError("量測 CSV 需要 2–100,001 筆資料。")
    previous = None
    for index, row in enumerate(rows, 2):
        if not isinstance(row, list) or len(row) != len(ROAD_HEADER):
            raise ValueError(f"量測 CSV 第 {index} 列需要七欄。")
        time = number(row[0], f"CSV 第 {index} 列時間", 0)
        if previous is not None and time - previous < .00001 - 1e-12:
            raise ValueError(f"CSV 第 {index} 列時間須遞增至少 0.00001 s。")
        for field, value in zip(ROAD_HEADER[1:], row[1:]):
            number(value, f"CSV 第 {index} 列 {field}", -1000, 1000)
        previous = time
    if rows[-1][0] - rows[0][0] > 120:
        raise ValueError("量測 CSV 的時間範圍須小於或等於 120 s。")


def read_recording(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream)
        if tuple(next(reader, ())) != ROAD_HEADER:
            raise ValueError("CSV 欄名須依序為 " + ",".join(ROAD_HEADER))
        rows = []
        for index, row in enumerate(reader, 2):
            if not row:
                continue
            if len(rows) >= 100001:
                raise ValueError("量測 CSV 超過 100,001 筆資料。")
            try:
                rows.append([float(value) for value in row])
            except ValueError as exc:
                raise ValueError(f"CSV 第 {index} 列包含非數值欄位。") from exc
    validate_recording(rows)
    return rows


def read_force_csv(path, table):
    """Load the selected force or motion ratio curve."""
    header = {"spring": SPRING_FORCE_HEADER, "dampers": DAMPER_FORCE_HEADER, "mr": MR_HEADER}[table]
    rows = []
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream, strict=True)
        first = True
        try:
            for row in reader:
                if not row or (len(row) == 1 and not row[0].strip()):
                    continue
                is_header = first and tuple(cell.strip().lower() for cell in row) == header
                first = False
                if is_header:
                    continue
                if len(rows) == MAX_CURVE_POINTS:
                    raise ValueError(f"曲線 CSV 最多可匯入 {MAX_CURVE_POINTS} 筆資料。")
                try:
                    if len(row) != len(header):
                        raise ValueError()
                    rows.append([float(cell) for cell in row])
                except ValueError as exc:
                    raise ValueError(f"曲線 CSV 第 {reader.line_num} 列須有 {len(header)} 個數值。"
                                     f"欄名可省略，若有欄名請使用 {','.join(header)}。") from exc
        except csv.Error as exc:
            raise ValueError(f"曲線 CSV 第 {reader.line_num} 列格式有誤。") from exc

    if table == "spring":
        return {table: spring_points(rows, stages=False)}
    if table == "mr":
        return {table: points(rows, "運動比曲線", minimum_y=1e-6)}
    return {key: damper_points([[row[0], row[index]] for row in rows], label, stages=False)
            for index, (key, label) in enumerate(DAMPER_FORCE_LABELS.items(), 1)}


def validate_aero(settings):
    if not isinstance(settings, dict) or type(settings.get("enabled")) is not bool:
        raise ValueError("空力開關設定有誤。")
    for key, (label, low, high) in AERO_FIELDS.items():
        number(settings.get(key), label, low, high)


def downforce(settings, speed_kmh):
    return settings["reference_force_n"] * (speed_kmh / settings["reference_speed_kmh"]) ** 2


def compile_project(project):
    if project.get("application") != APP or project.get("schema_version") != 1:
        raise ValueError("無法辨識調校專案的格式。")
    if not isinstance(project["name"], str) or not project["name"].strip():
        raise ValueError("專案名稱不可空白。")
    if not isinstance(project["notes"], list) or any(not isinstance(note, str) for note in project["notes"]):
        raise ValueError("專案備註格式有誤。")
    model = project["model"]
    if model not in MODELS:
        raise ValueError("無法辨識懸吊模型。")
    sources = project.get("model_sources", {})
    if not isinstance(sources, dict) or any(key not in MODEL_SOURCES or not isinstance(value, str)
                                             or value not in MODEL_SOURCES[key] for key, value in sources.items()):
        raise ValueError("1/4 車輪位或 1/2 車的車軸／側別設定有誤。")
    vehicle = project["vehicle"]
    for key, (label, low, high) in VEHICLE_FIELDS.items():
        number(vehicle[key], label, 0 if model != "flex" and key == "body_twist_k" else low, high)
    for corner in CORNERS:
        for key, (label, low, high) in TIRE_FIELDS.items():
            number(project["tires"][corner][key], f"{corner} {label}", low, high)
    if vehicle["m"] <= sum(project["tires"][c]["mu"] for c in CORNERS):
        raise ValueError("總質量必須大於四輪簧下質量總和。")

    validate_aero(project.get("aero", AERO_DEFAULTS))
    validate_case(model, sources, project["case"])
    for page, setup in analysis_page_cases(project).items():
        if page not in PAGE_MODELS or not isinstance(setup, dict) or setup.get("model") not in PAGE_MODELS[page]:
            raise ValueError("分析頁的模型設定有誤。")
        validate_case(setup["model"], sources, setup["case"])

    compiled = []
    for key, label in CHANNELS.items():
        try:
            compiled.append(KCSystem(project["channels"][key]))
        except (ValueError, KeyError, TypeError) as exc:
            raise ValueError(f"{label}：{exc}") from exc
    return compiled


def validate_case(model, sources, case):
    for key, (label, low, high) in CASE_FIELDS.items():
        number(case.get(key, CASE_DEFAULTS.get(key)), label, low, high)
    scenario = case["scenario"]
    if scenario not in SCENARIOS or case["target"] not in TARGET_WHEELS or type(case["rear_delay"]) is not bool:
        raise ValueError("工況、作用輪位或後軸延遲設定有誤。")
    if scenario in MANEUVER_CASES and not any(AXES[model]):
        raise ValueError("1/4 車請使用凸塊／凹坑、ISO 路面或路面 CSV 回放。")
    if scenario == "cornering" and not AXES[model][1]:
        raise ValueError("側向加速度工況適用於全車及 1/2 左右模型。")
    if type(case.get("iso_lateral", CASE_DEFAULTS["iso_lateral"])) is not bool:
        raise ValueError("ISO 路面側向加速度開關設定有誤。")
    if scenario in BUMP_CASES or scenario in ISO_ROUGHNESS:
        if not set(model_corners(model, sources)).intersection(TARGET_WHEELS[case["target"]]):
            raise ValueError("路面作用輪位不在選定模型的輪位中。")
    delayed_bump = scenario in BUMP_CASES and case["rear_delay"] and AXES[model][0]
    if (scenario in ISO_ROUGHNESS or delayed_bump) and case["speed"] < .1:
        raise ValueError("ISO 路面及凸塊後輪延遲需要至少 0.1 m/s 的車速。")
    if scenario in ISO_ROUGHNESS:
        if not float(case.get("seed", CASE_DEFAULTS["seed"])).is_integer():
            raise ValueError("ISO 路面隨機種子須為整數。")
        limit = .5 / (ISO_FREQUENCY_BAND[1] * case["speed"])
        if case["dt"] >= limit:
            raise ValueError(f"以目前車速取樣 ISO 路面時，輸出間隔須小於 {limit:.6g} s。")
    duration = case["duration"]
    if case["recording"] or case["scenario"] == "csv":
        validate_recording(case["recording"])
    if case["scenario"] == "csv":
        duration = case["recording"][-1][0] - case["recording"][0][0]
    elif case["start"] >= duration:
        raise ValueError("輸入開始時間須早於模擬結束時間。")
    if duration / case["dt"] > 30000:
        raise ValueError("輸出上限為 30,001 筆，請縮短時間或增加輸出間隔。")
    if not isinstance(case["recording_name"], str):
        raise ValueError("量測檔名格式有誤。")


def import_linear(source):
    full = source.get("parameters", {}).get("full")
    if full is None and source.get("application") == "VehicleInitialSizingLab-DynamicsExport":
        full = {**source["vehicle"], **source["geometry"], **source["dynamics"]}
        if source.get("schema_version") == 3:
            # Convert the physical main springs to the model's axle-sum coordinate.
            for key in ("heave_f_k", "heave_r_k"):
                if type(full.get(key)) in (int, float):
                    full[key] /= 4
    if not isinstance(full, dict) or not all(k in full for k in ("m", "lf", "lr", "heave_f_k", "heave_r_k")):
        raise ValueError("找不到可匯入的全車參數。")
    setup = default_project()
    setup["name"] = source.get("name") or setup["name"]
    setup["model"] = source.get("analysis", {}).get("full", {}).get("model", "flex")
    vehicle = setup["vehicle"]
    for key in vehicle.keys() & full.keys():
        vehicle[key] = full[key]
    for end, names in (("front", ("dive_front", "lift_front")), ("rear", ("lift_rear", "squat_rear"))):
        for name in names:
            vehicle[name] = full.get(f"anti_{end}", vehicle[name])

    def rates(key, fallback=None):
        prefix = key if f"{key}_k" in full or fallback is None else fallback
        return (full.get(f"{prefix}_k", 0), full.get(f"{prefix}_c", 0), full.get(f"{prefix}_mr", 1))

    for channel in CHANNELS:
        if channel in CORNERS:
            fallback = "f" if channel.startswith("F") else "r"
            k, c, mr = rates(f"corner_{channel}", f"corner_{fallback}")
        else:
            k, c, mr = rates(channel)
        setup["channels"][channel] = linear(k, c, mr)
    for corner in CORNERS:
        end = "f" if corner.startswith("F") else "r"
        k, c, mr = rates(f"tire_{corner}", f"tire_{end}")
        setup["tires"][corner].update(mu=full.get(f"mu_{corner}", full.get(f"mu_{end}", 15.)),
                                      k=k * mr**2 / 1000, c=c * mr**2)
    setup["notes"].append("已匯入全車線性元件與車輛參數。工況使用新工具預設值。")
    if full.get("tender_enabled"):
        setup["notes"].append("來源標記使用 Tender，但缺少完整併圈行程與安裝條件。請在元件頁輸入實際值，匯入目前保留來源的線性升沉剛性。")
    return setup


def open_project(path):
    with Path(path).open(encoding="utf-8-sig") as stream:
        source = json.load(stream)
    try:
        if not isinstance(source, dict):
            raise ValueError("專案須為 JSON 物件。")
        native = source.get("application") == APP
        setup = source if native else import_linear(source)
        compile_project(setup)
        setup = with_total_limit_rates(setup)
    except (KeyError, TypeError, AttributeError, OverflowError) as exc:
        raise ValueError(f"專案欄位不完整或格式有誤：{exc}") from exc
    return setup, path if native else None


def save_project(path, project):
    compile_project(project)
    path = Path(path)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    created = False
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            created = True
            json.dump(project, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    finally:
        if created:
            temporary.unlink(missing_ok=True)
