"""Angular motion of suspension links, expressed in chassis coordinates."""

from dataclasses import dataclass

import numpy as np

from .geometry import ARM_LINKS, LINKS, POINTS, UPRIGHT_LINKS


@dataclass(frozen=True)
class RodMotion:
    """Direction angles of A → B and angular vectors in rad/s and rad/s²."""

    azimuth_deg: float | None
    elevation_deg: float
    from_static_deg: float
    angular_velocity: np.ndarray
    angular_acceleration: np.ndarray
    rotation_model: str


@dataclass(frozen=True)
class HingeMotion:
    """Signed A-arm motion about the inboard axis from front to rear."""

    angle_deg: float
    velocity: float
    acceleration: float
    axis: np.ndarray


LINK_INDICES = np.array([(POINTS.index(a), POINTS.index(b)) for a, b in LINKS])
HINGES = {
    "UCA": ("upper_arm", *(POINTS.index(p) for p in ("UCA_F", "UCA_R", "UBJ"))),
    "LCA": ("lower_arm", *(POINTS.index(p) for p in ("LCA_F", "LCA_R", "LBJ"))),
}
ROTATION_MODELS = {
    **{link: HINGES[link[1][:3]][0] for link in ARM_LINKS},
    **dict.fromkeys(UPRIGHT_LINKS, "upright"),
    ("TOE", "RACK"): "axis_only",
}


def hinge_motion(reference, points, velocity, acceleration, front, rear, joint):
    axis = reference[rear] - reference[front]
    axis /= np.linalg.norm(axis)
    initial = reference[joint] - reference[front]
    current = points[joint] - points[front]
    initial -= axis * (axis @ initial)
    current -= axis * (axis @ current)
    angle = np.degrees(np.arctan2(axis @ np.cross(initial, current), initial @ current))
    tangent = np.cross(axis, current)
    scale = tangent @ tangent
    radial_velocity = velocity[joint] - velocity[front]
    rate = (tangent @ radial_velocity) / scale
    # The projection can change radius when a leg is replaced by a spring.
    rate_derivative = ((tangent @ (acceleration[joint] - acceleration[front]))
                       - 2 * (current @ radial_velocity) * rate) / scale
    axis.setflags(write=False)
    return HingeMotion(float(angle), float(rate), float(rate_derivative), axis)


def axis_angular_rates(direction, velocity, acceleration):
    """Rotation of the axis direction, with its changing length allowed."""
    length_squared = direction @ direction
    omega = np.cross(direction, velocity) / length_squared
    alpha = np.cross(direction, acceleration) / length_squared
    alpha -= (2 * (direction @ velocity) / length_squared) * omega
    omega.setflags(write=False)
    alpha.setflags(write=False)
    return omega, alpha


def axis_motion(reference, direction, velocity, acceleration):
    """Orientation and transverse angular rates for a two-point element."""
    horizontal = np.hypot(direction[0], direction[1])
    azimuth = (float(np.degrees(np.arctan2(direction[1], direction[0])))
               if horizontal > 1e-12 * np.linalg.norm(direction) else None)
    elevation = float(np.degrees(np.arctan2(direction[2], horizontal)))
    offset = float(np.degrees(np.arctan2(np.linalg.norm(np.cross(reference, direction)), reference @ direction)))
    omega, alpha = axis_angular_rates(direction, velocity, acceleration)
    return RodMotion(azimuth, elevation, offset, omega, alpha, "axis_only")


def compute_rod_motion(reference, points, velocity, acceleration, upright_omega, upright_alpha, rigid_links=None):
    """Derive link and A-arm motion from a solved node state."""
    arms = {}
    rigid_rates = {"upright": (upright_omega, upright_alpha)}
    for name, (model, front, rear, joint) in HINGES.items():
        motion = hinge_motion(reference, points, velocity, acceleration, front, rear, joint)
        arms[name] = motion
        rigid_rates[model] = (motion.axis * motion.velocity, motion.axis * motion.acceleration)
    if rigid_links is not None:
        active = {frozenset(link) for link in rigid_links}
        rigid_rates = {
            model: rates for model, rates in rigid_rates.items()
            if all(frozenset(link) in active for link, group in ROTATION_MODELS.items() if group == model)
        }

    start, end = LINK_INDICES.T
    reference_axes = reference[end] - reference[start]
    axes = points[end] - points[start]
    horizontal = np.hypot(axes[:, 0], axes[:, 1])
    azimuth = np.degrees(np.arctan2(axes[:, 1], axes[:, 0]))
    elevation = np.degrees(np.arctan2(axes[:, 2], horizontal))
    from_static = np.degrees(np.arctan2(
        np.linalg.norm(np.cross(reference_axes, axes), axis=1),
        np.einsum("ij,ij->i", reference_axes, axes),
    ))

    rods = {}
    for i, link in enumerate(LINKS):
        r = axes[i]
        length_squared = r @ r
        model = ROTATION_MODELS[link]
        if model not in rigid_rates:
            v = velocity[end[i]] - velocity[start[i]]
            a = acceleration[end[i]] - acceleration[start[i]]
            # Two points determine only the changing direction, not axial spin.
            omega, alpha = axis_angular_rates(r, v, a)
            model = "axis_only"
        else:
            omega, alpha = rigid_rates[model]
        omega.setflags(write=False)
        alpha.setflags(write=False)
        angle = float(azimuth[i]) if horizontal[i] > 1e-12 * np.sqrt(length_squared) else None
        rods[link] = RodMotion(angle, float(elevation[i]), float(from_static[i]), omega, alpha, model)
    return rods, arms
