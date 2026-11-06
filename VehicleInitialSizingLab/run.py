"""Launch the sizing workbench in a local browser."""

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from inspect import signature
import json
import math
from pathlib import Path
import webbrowser

from vehicle_initial_sizing import solve
from vehicle_initial_sizing_report import render_markdown_report


WEB = Path(__file__).with_name("web")
ASSETS = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/lab.css": ("lab.css", "text/css; charset=utf-8"),
    "/lab.js": ("lab.js", "text/javascript; charset=utf-8"),
    "/vendor/marked.js": ("vendor/marked.js", "text/javascript; charset=utf-8"),
    "/vendor/tex-svg.js": ("vendor/tex-svg.js", "text/javascript; charset=utf-8"),
}

# Numeric fields are positive by default. Bounds are inclusive unless positive is set.
GROUPS = (
    ("車輛與底盤", True, (
        ("ms", "全車簧上質量", "kg"),
        ("mu", "單輪簧下質量", "kg"),
        ("h_cog", "重心高度", "m"),
        ("lf", "重心到前軸距離", "m"),
        ("lr", "重心到後軸距離", "m"),
        ("tf", "前輪距", "m"),
        ("tr", "後輪距", "m"),
        ("sprung_roll_inertia_kg_m2", "簧上側傾慣量 Ixx", "kg·m²"),
    )),
    ("性能與設計目標", True, (
        ("ax", "縱向加速度 ax", "g"),
        ("ay", "側向加速度 ay", "g", {"positive": False, "nonzero": True}),
        ("a_acc", "傳動加速目標", "g"),
        ("brake_ax", "煞車減速度", "g", {"hint": "留空時沿用 ax"}),
        ("target_lap_time_s", "Skidpad 目標圈時", "s", {"min": 4, "max": 6}),
        ("drive_target_speed_kmh", "目標極速", "km/h"),
        ("cg_safety_factor", "重心檢查安全係數", "", {"min": 1}),
    )),
    ("懸吊與轉向", False, (
        ("use_tender", "使用 Tender 彈簧", ""),
        ("MR", "運動比 MR", ""),
        ("sus_travel", "懸吊壓縮行程", "m", {"max": 0.05}),
        ("target_pitch_angle_deg", "允許俯仰角", "°", {"min": 0.1, "max": 3}),
        ("target_roll_angle_deg", "允許側傾角", "°", {"min": 0.1, "max": 3}),
        ("kf", "前軸剛性權重 kf", ""),
        ("kr", "後軸剛性權重 kr", ""),
        ("N", "Tender 頻率除數 N", "", {"hint": "僅在啟用 Tender 時使用"}),
        ("min_turn_radius_m", "最小轉彎半徑", "m"),
        ("kpi_deg", "主銷內傾角 KPI", "°", {"positive": False}),
        ("caster_direction_check_deg", "方向驗證 Caster", "°", {"positive": False}),
        ("rc_chassis_stiffness_ratio", "前 RC 車體／懸吊剛性比", ""),
        ("anti_front_chassis_stiffness_N_per_m", "前 Anti 車體等效剛性", "N/m"),
        ("anti_rear_chassis_stiffness_N_per_m", "後 Anti 車體等效剛性", "N/m"),
    )),
    ("輪胎與空力", False, (
        ("Kt", "單輪垂直剛性 Kt", "N/m"),
        ("wl_mu_y", "橫向摩擦係數", ""),
        ("mu_w", "縱向摩擦係數", ""),
        ("R_tire", "輪胎有效滾動半徑", "m"),
        ("A_air", "空力有效面積", "m²"),
        ("air_density_kg_per_m3", "空氣密度", "kg/m³"),
        ("F_ref_ratio", "下壓力施加比例", "", {"positive": False, "min": 0, "max": 1}),
        ("wl_aero_front_ratio", "下壓力前軸分配", "", {"positive": False, "min": 0, "max": 1}),
    )),
    ("馬達與煞車", False, (
        ("drive_motor_torque_Nm", "單顆馬達最大扭矩", "N·m"),
        ("drive_motor_rpm", "馬達最高轉速", "rpm"),
        ("brake_driver_force_N", "車手煞車踏力", "N"),
    )),
)


def form_schema():
    defaults = signature(solve).parameters
    groups = []
    for title, expanded, entries in GROUPS:
        fields = []
        for name, label, unit, *options in entries:
            default = defaults[name].default
            fields.append({
                "name": name, "label": label, "unit": unit, "default": default,
                "kind": "checkbox" if isinstance(default, bool) else "number",
                "positive": True, **(options[0] if options else {}),
            })
        groups.append({"title": title, "expanded": expanded, "fields": fields})
    return groups


SCHEMA = form_schema()
FIELDS = {field["name"]: field for group in SCHEMA for field in group["fields"]}


class FieldError(ValueError):
    def __init__(self, field, message):
        self.field = field["name"]
        super().__init__(f"{field['label']}{message}")


def validate_inputs(values):
    if not isinstance(values, dict) or values.keys() - FIELDS.keys():
        raise ValueError("參數格式有誤。")

    for name, value in values.items():
        field = FIELDS[name]
        if field["kind"] == "checkbox":
            if type(value) is not bool:
                raise FieldError(field, "須為開啟或關閉。")
            continue
        if value is None and field["default"] is None:
            continue
        if type(value) not in (int, float) or not math.isfinite(value):
            raise FieldError(field, "須填入有限數值。")
        if field["positive"] and value <= 0:
            raise FieldError(field, "須大於 0。")
        if field.get("nonzero") and value == 0:
            raise FieldError(field, "須為非零值。")
        minimum, maximum = field.get("min"), field.get("max")
        if minimum is not None and value < minimum:
            raise FieldError(field, f"須至少為 {minimum:g}。")
        if maximum is not None and value > maximum:
            raise FieldError(field, f"須小於或等於 {maximum:g}。")
    return values


def solve_report(values):
    snapshot = solve(**validate_inputs(values))
    p = snapshot["parameters"]
    return {
        "markdown": render_markdown_report(snapshot),
        "summary": [
            {"label": "整車總質量", "value": f"{snapshot['frequency_mass']['m_total_vehicle_kg']:.2f}", "unit": "kg"},
            {"label": "前軸靜態重量分配", "value": f"{p['wf'] * 100:.1f}", "unit": "%"},
            {"label": "Skidpad 所需下壓力", "value": f"{snapshot['skidpad']['skidpad_required_aero_downforce_N']:.1f}", "unit": "N"},
            {"label": "Roll gradient（含輪胎）",
             "value": f"{snapshot['gradients']['roll_gradient_with_tires_deg_per_g']:.3f}", "unit": "°/g"},
        ],
    }


class Handler(BaseHTTPRequestHandler):
    def log_request(self, code="-", size="-"):
        pass

    def respond(self, status, data, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def respond_json(self, status, payload):
        self.respond(status, json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def local_request(self):
        expected = f"127.0.0.1:{self.server.server_port}"
        return self.headers.get("Host") == expected and self.headers.get("Origin") in (None, f"http://{expected}")

    def do_GET(self):
        if not self.local_request():
            self.send_error(HTTPStatus.FORBIDDEN)
        elif self.path == "/api/fields":
            self.respond_json(HTTPStatus.OK, SCHEMA)
        elif self.path in ASSETS:
            filename, content_type = ASSETS[self.path]
            self.respond(HTTPStatus.OK, (WEB / filename).read_bytes(), content_type)
        elif self.path == "/favicon.ico":
            self.respond(HTTPStatus.NO_CONTENT, b"", "image/x-icon")
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        if not self.local_request():
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if self.path != "/api/solve":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if self.headers.get_content_type() != "application/json":
            self.send_error(HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
            return
        try:
            size = int(self.headers.get("Content-Length", 0))
            if not 0 < size <= 16_384:
                raise ValueError("參數資料大小有誤。")
            values = json.loads(self.rfile.read(size))
        except (ValueError, RecursionError):
            self.respond_json(HTTPStatus.BAD_REQUEST, {"error": "無法讀取參數資料。"})
            return

        try:
            result = solve_report(values)
        except FieldError as error:
            self.respond_json(HTTPStatus.BAD_REQUEST, {"error": str(error), "field": error.field})
        except ValueError as error:
            self.respond_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
        except ArithmeticError:
            self.respond_json(HTTPStatus.BAD_REQUEST, {"error": "數值超出可計算範圍，請檢查參數與單位。"})
        else:
            self.respond_json(HTTPStatus.OK, result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=0, help="Local port (default: choose a free port)")
    parser.add_argument("--no-browser", action="store_true", help="Print the address without opening a browser")
    args = parser.parse_args()

    with ThreadingHTTPServer(("127.0.0.1", args.port), Handler) as server:
        url = f"http://127.0.0.1:{server.server_port}/"
        print(f"Vehicle Initial Sizing Lab\n{url}\nPress Ctrl+C in this window to stop.", flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
