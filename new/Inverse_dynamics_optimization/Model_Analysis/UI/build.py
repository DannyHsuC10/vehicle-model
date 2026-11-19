# build.py

from Model_Analysis import simulator
import Model_Analysis.Geometry as geo
import Model_Analysis.motion as mot
import json

# =========================================================
# CONFIG
# =========================================================
class Config:
    def __init__(self, filename):
        self.path = filename

        if not self.path.exists():
            raise FileNotFoundError(f"找不到 config: {self.path}")

        with open(self.path, "r", encoding="utf-8") as f:
            self.data = json.load(f)

        self._validate()

    def get(self, *keys, default=None):
        d = self.data
        for k in keys:
            if k not in d:
                return default
            d = d[k]
        return d

    def require(self, *keys):
        d = self.data
        for k in keys:
            if k not in d:
                raise ValueError(f"缺少必要欄位: {' -> '.join(keys)}")
            d = d[k]
        return d

    def _validate(self):
        self.require("simulation")
        self.require("inputs")
        self.require("targets")

        sim = self.data["simulation"]
        if "SIM_TIME" not in sim or "DT" not in sim:
            raise ValueError("simulation 必須包含 SIM_TIME, DT")

# =========================================================
# BUILDERS dynamic
# =========================================================
def build_inputs(sim, cfg, SIM_TIME):# 輸入設定
    inputs = cfg.require("inputs")

    for icfg in inputs:
        if not icfg.get("enable", True):
            continue

        node = sim.nodes[icfg["node"]]
        P_init = node.pos.copy()

        ctrl = simulator.PositionConstraint(node)
        sim.solver.add_constraint(ctrl)

        axis = geo.axis_unitization(
            sim.nodes[icfg["axis_nodes"][0]].pos,
            sim.nodes[icfg["axis_nodes"][1]].pos
        )

        mcfg = icfg["motion"]
        times = mcfg.get("times", 1)

        motion = mot.MotionConfig(
            motion_type=mcfg["motion_type"],
            input_type=mcfg["input_type"],
            x_max=mcfg["x_max"],
            x_min=mcfg["x_min"],
            times=times,
            normal_vector=mcfg.get("normal_vector", None)
        )

        def make_input(ctrl, motion, P_init, axis):
            def input_func(t):
                x = motion.get_input(t, SIM_TIME)
                new_pos = motion.apply_motion(P_init, axis, x)
                ctrl.set_target(new_pos)
                return x
            return input_func

        sim.add_input(make_input(ctrl, motion, P_init, axis))


def build_forces(sim, cfg):# 力量設定
    f_cfg = cfg.get("forces", default=[])
    f_targets = []

    for f in f_cfg:
        axis = geo.axis_unitization(
            sim.nodes[f["axis_nodes"][0]].pos,
            sim.nodes[f["axis_nodes"][1]].pos
        )

        f_targets.append({
            "name": f["name"],
            "in_node": sim.nodes[f["node"]],
            "in_axis": axis[0],
            "in_force": f["force"]
        })

    return f_targets


def build_torques(sim, cfg):# 扭矩設定
    t_cfg = cfg.get("torques", default=[])
    t_targets = []

    for t in t_cfg:
        axis = geo.axis_unitization(
            sim.nodes[t["axis_nodes"][0]].pos,
            sim.nodes[t["axis_nodes"][1]].pos
        )

        t_targets.append({
            "name": t["name"],
            "node": t["node"],
            "axis": axis
        })

    return t_targets


def build_targets(sim, cfg):
    sim.set_targets(
        node_targets=cfg.require("targets", "node_targets"),
        rod_targets=cfg.require("targets", "rod_targets"),
        force_targets=build_forces(sim, cfg),
        torque_targets=build_torques(sim, cfg)
    )


# =========================================================
# BUILDERS static
# =========================================================

