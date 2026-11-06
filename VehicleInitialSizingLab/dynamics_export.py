"""Export full-vehicle Dynamics parameters from one sizing result."""

import json
import math
from pathlib import Path


EXPORT_FIELDS = {
    "zeta": ("目標阻尼比 ζ", "", False),
    "body_twist_k": ("車架扭轉剛性 body_twist_k", "N·m/rad", False),
    "body_twist_c": ("車架扭轉阻尼 body_twist_c", "N·m·s/rad", False),
    "k_frame": ("車架傳力剛性 k_frame", "N/m", True),
    "c_frame": ("車架傳力阻尼 c_frame", "N·s/m", False),
    "m_frame": ("車架傳力虛擬質量 m_frame", "kg", True),
}


def load_template():
    path = Path(__file__).with_name("dynamics_template.json")
    return json.loads(path.read_text(encoding="utf-8-sig"))


def default_export_settings():
    full = load_template()
    return {key: full[key] for key in EXPORT_FIELDS}


def validate_export_settings(settings):
    if not isinstance(settings, dict) or settings.keys() != EXPORT_FIELDS.keys():
        raise ValueError("Dynamics 匯出設定的欄位不完整或無法辨識。")
    for key, (label, _, positive) in EXPORT_FIELDS.items():
        value = settings[key]
        if type(value) not in (float, int) or not math.isfinite(value) or value < 0 or (positive and value == 0):
            limit = "大於 0" if positive else "大於或等於 0"
            raise ValueError(f"{label}須為{limit}的有限數值。")
    return settings


def build_dynamics_export(snapshot, settings, name):
    settings = validate_export_settings(settings)
    full = load_template()
    full.update(settings)
    p = snapshot["parameters"]
    anti = snapshot["anti"]["heave_only"]
    zeta = settings["zeta"]
    mr = p["MR"]
    full.update(
        m=p["ms"] + 4 * p["mu"], mu_f=p["mu"], mu_r=p["mu"],
        lf=p["lf"], lr=p["lr"], tf=p["tf"], tr=p["tr"], h_cg=p["h_cog"],
        Ixx=p["sprung_roll_inertia_kg_m2"],
        h_rc_f=snapshot["front_rc"]["rc_normalized_optimal_height_m"],
        h_rc_r=snapshot["rear_rc_height_m"],
        brake_front=snapshot["brake"]["brake_ideal_front_ratio"], drive_front=0.5,
        dive_front=anti["front_anti_dive_ratio"], lift_front=anti["front_anti_lift_ratio"],
        squat_rear=anti["rear_anti_squat_ratio"], lift_rear=anti["rear_anti_lift_ratio"],
    )

    def channel(prefix, stiffness, damping, ratio):
        full.update({f"{prefix}_k": stiffness, f"{prefix}_c": damping, f"{prefix}_mr": ratio})

    for end, axle in (("f", "front"), ("r", "rear")):
        heave = snapshot["damping"]["pure_heave"][axle]
        roll = snapshot["damping"]["roll"][axle]
        # Dynamics uses the sum of the two wheel displacements for axle heave,
        # so its channel gives four times the coefficient at equal wheel travel.
        channel(f"heave_{end}", heave["spring_stiffness_N_per_m"] / 4,
                zeta * heave["damper_critical_damping_Ns_per_m"] / 4, mr)
        channel(f"roll_{end}", roll["spring_stiffness_N_per_m"],
                zeta * roll["damper_critical_damping_Ns_per_m"], mr)
    channel("roll_c", 0, 0, 1)
    tire_damping = 2 * zeta * math.sqrt(p["Kt"] * p["mu"])
    for corner in ("FL", "FR", "RL", "RR"):
        channel(f"corner_{corner}", 0, 0, 1)
        channel(f"tire_{corner}", p["Kt"], tire_damping, 1)

    tender = snapshot["tender"]
    full.update(
        tender_enabled=p["use_tender"],
        tender_stiffness_N_per_m=tender["K_tender_required_N_per_m"] if tender is not None else None,
        heave_configuration="main_spring",
    )
    return {
        "application": "VehicleInitialSizingLab-DynamicsExport",
        "schema_version": 1,
        "name": name,
        "parameters": {"full": full},
    }
