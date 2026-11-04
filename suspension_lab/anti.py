"""Outboard-brake and independent-drive anti ratios in one road frame."""
import numpy as np


def anti_ratio(ic, origin, wheelbase, cg_height, fraction, sign, basis=None):
    if cg_height <= 1e-12:
        return None
    if fraction == 0:
        return 0.
    if ic is None:
        return None
    delta = np.asarray(ic)-origin
    if basis is not None:
        delta = basis@delta
    distance = sign*delta[0]
    if abs(distance) < 1e-12:
        return None
    return float(fraction*wheelbase/cg_height*delta[1]/distance)


def axle_anti(axle, ic, contact, center, wheelbase, cg_height, brake_front, drive_front, basis=None):
    """Only return physically named channels for this axle; values are ratios.

    CP -> IC reacts outboard braking; WC -> IC reacts drive. Signed distance
    points toward the other axle, so reversing IC direction reverses the effect.
    """
    front = axle == "front"
    sign = 1 if front else -1
    brake = anti_ratio(ic, contact, wheelbase, cg_height, brake_front if front else 1-brake_front, sign, basis)
    drive = anti_ratio(ic, center, wheelbase, cg_height, drive_front if front else 1-drive_front, sign, basis)
    return {"anti_dive": brake, "anti_lift": drive} if front else {"anti_lift": brake, "anti_squat": drive}
