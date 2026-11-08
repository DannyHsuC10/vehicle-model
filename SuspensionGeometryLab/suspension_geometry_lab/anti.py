"""Contact-patch Anti ratios for outboard brakes and upright-mounted motors."""
import numpy as np


def anti_ratio(ic, contact, wheelbase, cg_height, basis=None):
    """Return L/h times the signed CP-to-IC slope, with positive X forward."""
    if cg_height <= 1e-12 or ic is None:
        return None
    delta = np.asarray(ic)-contact
    if basis is not None:
        delta = basis@delta
    # The SV solver and contact basis use rearward-positive horizontal X.
    distance = -delta[0]
    if abs(distance) < 1e-12:
        return None
    return float(wheelbase/cg_height*delta[1]/distance)


def axle_anti(axle, ic, contact, wheelbase, cg_height, brake_front, drive_front, basis=None):
    """Report the axle's inward-positive Anti, using the same sign after weighting."""
    ratio = anti_ratio(ic, contact, wheelbase, cg_height, basis)
    if axle == "front" and ratio is not None:
        ratio = -ratio
    values = {"anti": ratio}
    for key, front_share in (("anti_brake", brake_front), ("anti_drive", drive_front)):
        share = front_share if axle == "front" else 1-front_share
        if share == 0 and cg_height > 1e-12:
            values[key] = 0.
        else:
            values[key] = None if ratio is None else ratio*share
    return values


def anti_percentages(axle, ratios):
    return {f"{key}_{axle}_pct": None if value is None else 100*value for key, value in ratios.items()}
