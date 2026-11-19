import csv
from pathlib import Path
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
FILTER_DIR = Path(__file__).resolve().parent

def save_to_csv(
    sim,
    filename="output",
    eps=1e-12,
    digits=6
):

    data_dict = sim.logger.data

    keys = list(data_dict.keys())
    length = len(data_dict[keys[0]])

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    filename_with_time = f"{filename}_{timestamp}.csv"

    file_path = FILTER_DIR / "output" / filename_with_time

    with open(file_path, "w", newline="", encoding="utf-8") as f:

        writer = csv.writer(f)

        writer.writerow(keys)

        for i in range(length):

            row = []

            for k in keys:

                val = data_dict[k][i]

                # =====================================
                # Clean tiny noise
                # =====================================

                if isinstance(val, (float, np.floating)):

                    if abs(val) < eps:
                        val = 0.0

                    # =====================================
                    # Format precision
                    # =====================================

                    val = f"{val:.{digits}f}"

                row.append(val)

            writer.writerow(row)

    print(f"[CSV saved] {file_path}")

def print_out(sim,key):
    print("\n===== TARGET DATA (LAST STEP) =====")
    data = sim.logger.data[key]
    print(data)

class Plotter:
    def __init__(self, data):
        self.data = data
        self.eps = 1e-6

    def plot(self, key, show=True, trim=True):
        t = self.data["t"]
        y = self.data[key]

        # ===== 去頭尾兩筆（for acceleration）=====
        if trim and len(t) > 4:
            t = t[2:-2]# << 防2階導數初值
            y = y[2:-2]
            y = np.where(np.abs(y) < self.eps, 0.0, y)

        fig = plt.figure()
        plt.plot(t, y)

        plt.xlabel("Time (s)")
        plt.ylabel(key)
        plt.title(key)

        plt.grid()
        if show:
            plt.show()
        
        return fig
    
def save_report(report, filename="static_report"):# 靜態報告存檔
    # 紀錄時間
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename_with_time = f"{filename}_{timestamp}.txt"

    # 完整路徑
    file_path =  FILTER_DIR / "output" / filename_with_time


    # 寫入文字
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(report)

    print(f"Report saved to: {file_path}")

    return file_path

def save_rod_forces_csv(rods, filename="rod_forces"):

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename_with_time = f"{filename}_{timestamp}.csv"

    file_path = FILTER_DIR / "output" / filename_with_time

    with open(file_path, "w", newline="", encoding="utf-8") as f:

        writer = csv.writer(f)

        # =========================
        # header（工程完整版）
        # =========================
        writer.writerow([
            "rod_name",
            "force_N",
            "state",

            # geometry
            "length_m",
            "radius_m",
            "area_m2",

            # material
            "E_Pa",
            "max_stress_Pa",

            # mechanics
            "stress_Pa",
            "strain",
            "delta_L_m",
            "stiffness_Nm",

            # energy
            "energy_J",

            # safety
            "safety_ratio",
            "fail"
        ])

        # =========================
        # data
        # =========================
        for r in rods.values():

            # force state
            if r.force > 0.01:
                state = "tension"
            elif r.force < -0.01:
                state = "compression"
            else:
                state = "zero"
                r.force = 0

            # safety check (保險再算一次，避免 state 沒更新)
            stress = r.stress if r.stress is not None else 0
            max_s = r.max_stress if hasattr(r, "max_stress") else 0

            safety_ratio = abs(stress) / max_s if max_s else None
            fail = (abs(stress) > max_s) if max_s else False

            writer.writerow([
                r.name,
                r.force,
                state,

                # geometry
                r.L0,
                r.radius,
                r.area,

                # material
                r.E,
                max_s,

                # mechanics
                r.stress,
                r.strain,
                r.delta_L,
                r.k,

                # energy
                r.energy,

                # safety
                safety_ratio,
                fail
            ])
            
def build_report(state):

    lines = []

    # ==================================================
    # Rod Forces
    # ==================================================
    lines.append("")
    lines.append("==================================================")
    lines.append("Rod Forces")
    lines.append("==================================================")

    for r in state.active_rods.values():

        lines.append(
            f"{r.name:20s} | "
            f"F = {r.force:12.3f} N"
        )

    # ==================================================
    # Material Results
    # ==================================================
    lines.append("")
    lines.append("==================================================")
    lines.append("Material Results")
    lines.append("==================================================")

    for r in state.active_rods.values():

        if r.max_stress and r.max_stress > 0:
            status = "FAIL" if r.fail else "OK"
            sf = r.safety_ratio
            sf_str = f"{sf:6.2f}" if sf is not None else "  -  "
        else:
            status = "N/A"
            sf_str = "  -  "

        lines.append(
            f"{r.name:20s} | "
            f"σ = {r.stress:10.3f} MPa | "
            f"ε = {r.strain:12.6e} | "
            f"ΔL = {r.delta_L:10.6f} mm | "
            f"U = {r.energy:10.3f} | "
            f"max = {r.max_stress:8.2f} | "
            f"S.F. = {sf_str} | "
            f"{status}"
        )

    # ==================================================
    # Node Forces
    # ==================================================
    lines.append("")
    lines.append("==================================================")
    lines.append("Node Equilibrium")
    lines.append("==================================================")

    for name, F in state.node_forces.items():

        node = state.nodes[name]

        tag = "Reaction" if node.fixed else "Residual"

        lines.append(
            f"{name:20s} | "
            f"{tag:10s} | "
            f"Fx = {F[0]:10.3f} | "
            f"Fy = {F[1]:10.3f} | "
            f"Fz = {F[2]:10.3f}"
        )

    report = "\n".join(lines)
    print(report)

    return report

