"""Static side-view geometry, independent of any plotting backend."""

import numpy as np

from .. import geometry as ge
from .. import Geometric_Analysis as ga
from ..car import Car, SuspensionF, SuspensionR
from ..analysis_settings import AnalysisSettings


# Colors identify geometric feature types, not front/rear axles.
STYLE = {
    "wheel": "0.35",
    "kingpin": "tab:blue",
    "svsa": "black",
    "ic_links": "tab:orange",
    "tangent_lines": "tab:purple",
    "hard_points": "tab:blue",
    "contact": "tab:red",
    "ic": "tab:green",
    "pitch_center": "tab:pink",
}


car = Car()
suspension_f = SuspensionF()
suspension_r = SuspensionR()
analysis_settings = AnalysisSettings()


class SideSuspension:
    """One wheel/upright assembly at its initial side-view geometry."""

    def __init__(self, name, suspension, axle_index, direction_sign, center_x, color,
                 *, settings=None, vehicle=None):
        self.name = name
        self.suspension = suspension
        self.car = car if vehicle is None else vehicle
        self.settings = analysis_settings if settings is None else settings
        self.axle_index = axle_index
        self.direction_sign = direction_sign
        self.color = color

        self.contact_point = np.asarray(self.settings.contact_point_sv, dtype=float).copy()
        self.contact_point += np.array([center_x, 0.0])
        self.wheel_center = np.array(
            [center_x, self.car.load_Radius[axle_index]], dtype=float
        )

        distance = suspension.svsa
        if not np.isfinite(distance) or abs(distance) < 1e-8:
            raise ValueError(f"{name}：Anti 設定無法產生有效的有限 SVSA。")
        rearward_distance = direction_sign * distance
        inward_slope = suspension.target_anti_sv * (self.car.h_cog / self.car.l)
        height = distance * inward_slope
        if not np.isfinite(height):
            raise ValueError(f"{name}：Anti 設定無法產生有限的瞬心高度。")
        self.initial_ic = self.contact_point + np.array([rearward_distance, height])

        self.kingpin_ground = np.array(
            [center_x - direction_sign * suspension.mechanical_trail_sv, 0.0]
        )
        caster = np.deg2rad(suspension.caster_sv_deg)
        self.kingpin_direction = np.array(
            [direction_sign * np.sin(caster), np.cos(caster)]
        )
        self.kingpin_top = (
            self.kingpin_ground
            + self.settings.kingpin_length_sv * self.kingpin_direction
        )

        self.kingpin_wc = self._point_on_kingpin(self.wheel_center[1])
        self.kingpin_Au = self._point_on_kingpin(
            self.wheel_center[1] + suspension.Au
        )
        self.kingpin_Al = self._point_on_kingpin(
            self.wheel_center[1] - suspension.Al
        )

        self.wc_pt = ge.Point(self.wheel_center, f"{name} Wheel Center")
        self.contact_pt = ge.Point(self.contact_point, f"{name} Contact")
        self.au_pt = ge.Point(self.kingpin_Au, f"{name} Au")
        self.al_pt = ge.Point(self.kingpin_Al, f"{name} Al")
        self.ic_pt = ge.Point(self.initial_ic, f"{name} IC")
        self.kg_pt = ge.Point(self.kingpin_ground, f"{name} Kingpin Ground")
        self.kt_pt = ge.Point(self.kingpin_top, f"{name} Kingpin Top")
        self.kwc_pt = ge.Point(self.kingpin_wc, f"{name} Kingpin @ WC")

        wheel_angles = np.linspace(
            0.0,
            2.0 * np.pi,
            self.settings.side_view_wheel_outline_points,
            endpoint=False,
        )
        self.wheel_outline_pts = [
            ge.Point(
                self.wheel_center + self.car.free_radius * np.array([np.cos(a), np.sin(a)]),
                f"{name} Wheel {i}",
            )
            for i, a in enumerate(wheel_angles)
        ]
        self.wheel_polygon = ge.Polygon(self.wheel_outline_pts, name=f"{name} Wheel")
        self.au_tangent = self._make_tangent_reference(self.au_pt, f"{name} Au tangent")
        self.al_tangent = self._make_tangent_reference(self.al_pt, f"{name} Al tangent")

        self.kingpin_link = ge.Link(self.kg_pt, self.kt_pt, f"{name} Kingpin")
        self.svsa_link = ge.Link(self.contact_pt, self.ic_pt, f"{name} SVSA")
        self.au_ic_link = ge.Link(self.au_pt, self.ic_pt, f"{name} Au to IC")
        self.al_ic_link = ge.Link(self.al_pt, self.ic_pt, f"{name} Al to IC")

    def _point_on_kingpin(self, target_y):
        t = (target_y - self.kingpin_ground[1]) / self.kingpin_direction[1]
        return self.kingpin_ground + t * self.kingpin_direction

    def _make_tangent_reference(self, point, label):
        """Draw a fixed tangent perpendicular to the point-to-IC reference."""
        radial = self.initial_ic - point.pos
        length = np.linalg.norm(radial)
        if not np.isfinite(length) or length <= 1e-12:
            raise ValueError(f"{self.name}：瞬心與 {point.name} 重合或無有限力線。")
        radial /= length
        tangent = np.array([-radial[1], radial[0]])
        half_length = self.settings.side_view_tangent_half_length
        start = ge.Point(point.pos - half_length * tangent, f"{label} start")
        end = ge.Point(point.pos + half_length * tangent, f"{label} end")
        return ge.Link(start, end, label)

    def add_to_scene(self, scene):
        scene.add(
            self.wheel_polygon,
            facecolor="none",
            edgecolor=STYLE["wheel"],
            linewidth=2,
        )
        for link, style in (
            (self.kingpin_link, {"color": STYLE["kingpin"], "linewidth": 2.2}),
            (self.svsa_link, {"color": STYLE["svsa"], "linewidth": 2.6}),
            (self.au_ic_link, {"color": STYLE["ic_links"], "linestyle": "--"}),
            (self.al_ic_link, {"color": STYLE["ic_links"], "linestyle": "--"}),
            (self.au_tangent, {"color": STYLE["tangent_lines"], "linestyle": "-."}),
            (self.al_tangent, {"color": STYLE["tangent_lines"], "linestyle": "-."}),
        ):
            scene.add(link, **style)

        for point, style in (
            (self.wc_pt, {"color": STYLE["hard_points"]}),
            (self.contact_pt, {"color": STYLE["contact"]}),
            (self.au_pt, {"color": STYLE["hard_points"]}),
            (self.al_pt, {"color": STYLE["hard_points"]}),
            (self.ic_pt, {"color": STYLE["ic"], "markersize": 10}),
            (self.kg_pt, {"color": STYLE["hard_points"]}),
            (self.kt_pt, {"color": STYLE["hard_points"]}),
        ):
            # PointPlot uses circular markers for geometric points.
            style.setdefault("markersize", 6)
            scene.add(point, **style)


def build_side_view_geometry(vehicle=None, suspension_f=None, suspension_r=None,
                             settings=None):
    """Build the original static SV geometry without creating a plot.

    Local SV X increases from the front axle toward the rear axle. The front
    axle is one wheelbase left of the rear axle, retaining the original signs.
    """
    vehicle = car if vehicle is None else vehicle
    suspension_f = SuspensionF() if suspension_f is None else suspension_f
    suspension_r = SuspensionR() if suspension_r is None else suspension_r
    settings = analysis_settings if settings is None else settings
    front_center_x = settings.wheel_center_x_sv - vehicle.l
    rear_center_x = settings.wheel_center_x_sv
    front = SideSuspension(
        "Front", suspension_f, axle_index=0, direction_sign=1.0,
        center_x=front_center_x, color=STYLE["wheel"], settings=settings, vehicle=vehicle,
    )
    rear = SideSuspension(
        "Rear", suspension_r, axle_index=1, direction_sign=-1.0,
        center_x=rear_center_x, color=STYLE["wheel"], settings=settings, vehicle=vehicle,
    )
    pitch_center = ga.SusGeometryHelper.line_intersection(
        front.contact_point,
        front.initial_ic,
        rear.contact_point,
        rear.initial_ic,
    )
    return front, rear, pitch_center


