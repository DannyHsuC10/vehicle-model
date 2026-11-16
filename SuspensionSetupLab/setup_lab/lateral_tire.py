"""Adapt the shared Pacejka solver to the setup lab's lateral convention."""

from functools import lru_cache
import math
from pathlib import Path
import sys

import numpy as np


DEFAULT_TIR = "Hoosier_43075_R20_W8_candidate.tir"


@lru_cache(maxsize=1)
def _load_model():
    # The lab also runs with its own directory as the working directory.
    shared = Path(__file__).resolve().parents[2] / "new"
    if (shared / "Pacejka_tire_model").is_dir() and str(shared) not in sys.path:
        sys.path.append(str(shared))
    try:
        from Pacejka_tire_model import tir_io
        from Pacejka_tire_model.model import api

    except ImportError as exc:
        raise ValueError(f"無法載入 Pacejka 輪胎模型：{exc}") from exc
    return api, tir_io


def _load_tire(source):
    api, tir_io = _load_model()
    name = source["name"] if source else DEFAULT_TIR
    try:
        if source:
            doc = tir_io.TirDocument.from_text(source["text"])
        else:
            doc = tir_io.read_tir(Path(tir_io.__file__).with_name("tir") / DEFAULT_TIR)
        tir_io.validate_tir(doc)
        params = doc.model_parameters()
        limit = min(math.pi / 2, -float(params.get("ALPMIN", -math.pi / 2)),
                    float(params.get("ALPMAX", math.pi / 2)))
        if limit <= 0.:
            raise ValueError("滑角範圍須涵蓋零度兩側。")
        side = str(params.get("TYRESIDE", "left")).strip().lower()
        if side not in ("left", "right", "symmetric"):
            raise ValueError(f"不支援的 TYRESIDE：{side!r}")
        sample = api.CalculationInput(FZ=params["FNOMIN"], SA=[-1., 0., 1.], tir_params=params)
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            if not np.isfinite(api.calculate_fy0(sample)).all():
                raise ValueError("側向力係數在標準輪荷與胎壓下無法計算有限值。")
    except (OSError, ValueError, KeyError, AttributeError, TypeError) as exc:
        raise ValueError(f"無法載入 {name}：{exc}") from exc
    return api, params, math.degrees(limit), side


def import_tir(path):
    """Validate the selected file and keep its content for portable projects."""
    path = Path(path)
    source = {"name": path.name, "text": path.read_text(encoding="utf-8-sig")}
    _load_tire(source)
    return source


class LateralTire:
    def __init__(self, pressure_f, pressure_r, source=None):
        self.api, self.params, limit, side = _load_tire(source)
        self.pressures = 1000 * np.repeat([pressure_f, pressure_r], 2)
        # Mirroring the fitted tire reverses the model slip input and Fy output.
        self.angles = np.linspace(-limit, limit, 2 * math.ceil(limit / .1) + 1)
        self.sides = np.array([1., -1., 1., -1.])
        if side == "right":
            self.sides *= -1.

    def curves(self, loads, baseline):
        """Sample pure Fy at the actual and reference Fz in one call."""
        fz = np.maximum(0., np.stack((loads, baseline)))[:, :, None]
        sides = self.sides[:, None]
        conditions = self.api.CalculationInput(
            FZ=fz, SA=-sides * self.angles, SL=0., IA=0.,
            pressure=self.pressures[:, None], tir_params=self.params,
        )
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            fy = np.where(fz > 0., sides * self.api.calculate_fy0(conditions), 0.)
        if not np.isfinite(fy).all():
            raise ValueError("目前輪荷與胎壓下，輪胎模型無法計算有限側向力。")
        return fy

    def axle_response(self, wheels, demand):
        """Return peak and the equilibrium on the rising branch around zero Fy."""
        total = wheels.sum(axis=0)
        zeros = np.flatnonzero((total[:-1] <= 0) & (total[1:] >= 0) & (np.diff(total) > 0))
        if zeros.size == 0:
            return 0., None, None
        start = zeros[np.argmin(abs(self.angles[zeros] + self.angles[zeros + 1]))]
        if demand < 0.:
            branch = slice(start + 1, None, -1)
            force = -total[branch]
        else:
            branch = slice(start, None)
            force = total[branch]
        # Discard post-peak samples so a sliding solution never replaces the
        # low-slip equilibrium. The first sample brackets zero force.
        folds = np.flatnonzero(np.diff(force) <= 0.)
        end = int(folds[0] + 1) if folds.size else len(force)
        force = force[:end]
        peak = float(force[-1])
        if abs(demand) > peak:
            return peak, None, None
        slip = float(np.interp(abs(demand), force, self.angles[branch][:end]))
        forces = [float(np.interp(abs(demand), force, curve[branch][:end])) for curve in wheels]
        return peak, slip, forces
