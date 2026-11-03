import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from static import geometry as ge
from static import Geometric_Analysis as ga
from static.car import Car, SuspensionF, SuspensionR
from static.analysis_settings import AnalysisSettings
from static.visualization import LinkPlot, PointPlot, PolygonPlot


from static.sv.side_view_geometry import (
    STYLE, SideSuspension, analysis_settings, build_side_view_geometry,
    calc_anti_angle, car, suspension_f, suspension_r,
)


class Scene:
    """Static Matplotlib scene using the shared front-view plot helpers."""

    def __init__(self, ax):
        self.ax = ax
        self.objects = []

    def add(self, obj, **kwargs):
        if isinstance(obj, ge.Point):
            self.objects.append(PointPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Link):
            self.objects.append(LinkPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Polygon):
            self.objects.append(PolygonPlot(self.ax, obj, **kwargs))


if __name__ == '__main__':
    fig, ax = plt.subplots(figsize=analysis_settings.side_view_figure_size)
    scene = Scene(ax)
    front, rear, pitch_center = build_side_view_geometry(
        vehicle=car, suspension_f=suspension_f, suspension_r=suspension_r,
        settings=analysis_settings,
    )
    front_center_x = front.wheel_center[0]
    rear_center_x = rear.wheel_center[0]
    front.add_to_scene(scene)
    rear.add_to_scene(scene)

    # Connect front/rear tire contact points and extend the same line at both ends.
    contact_span = rear.contact_pt.pos - front.contact_pt.pos
    contact_axis_start = ge.Point(
        front.contact_pt.pos - contact_span, "Contact axis start"
    )
    contact_axis_end = ge.Point(
        rear.contact_pt.pos + contact_span, "Contact axis end"
    )
    contact_axis = ge.Link(
        contact_axis_start, contact_axis_end, "Extended tire-contact axis"
    )
    scene.add(contact_axis, color="0.25", linestyle="--", linewidth=1.5)

    pc_pt = ge.Point(pitch_center, "PC")
    scene.add(pc_pt, color=STYLE["pitch_center"], markersize=10)

    ax.axhline(0.0, color="black", linewidth=1)
    ax.axvline(front_center_x, color="0.5", linestyle=":", linewidth=1)
    ax.axvline(rear_center_x, color="0.5", linestyle=":", linewidth=1)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)
    ax.set_xlabel("Longitudinal X [m]")
    ax.set_ylabel("Vertical Y [m]")
    front_svsa = front.ic_pt.x - front.contact_pt.x
    rear_svsa = rear.contact_pt.x - rear.ic_pt.x
    ax.set_title(
        "Static Front and Rear Side-View Suspension Geometry\n"
        f"Front SVSA={front_svsa:.3f} m, Rear SVSA={rear_svsa:.3f} m | "
        f"PC=({pc_pt.x:.3f}, {pc_pt.y:.3f}) m"
    )

    legend_handles = [
        Line2D([0], [0], color=STYLE["wheel"], lw=2, label="Wheel outline"),
        Line2D([0], [0], color=STYLE["kingpin"], lw=2, label="Kingpin"),
        Line2D([0], [0], color=STYLE["svsa"], lw=2, label="Contact point to IC"),
        Line2D([0], [0], color=STYLE["ic_links"], linestyle="--", label="Au/Al to IC"),
        Line2D([0], [0], color=STYLE["tangent_lines"], linestyle="-.", label="Au/Al tangent references"),
        Line2D([0], [0], color=STYLE["anti_lift"], linestyle=":", label="Wheel center to IC"),
        Line2D([0], [0], color=STYLE["ic"], marker="o", linestyle="none", label="IC"),
        Line2D([0], [0], color=STYLE["contact"], marker="o", linestyle="none", label="Contact point"),
        Line2D([0], [0], color=STYLE["pitch_center"], marker="o", linestyle="none", label="PC"),
    ]
    ax.legend(handles=legend_handles, loc="best", fontsize=8)
    ax.annotate(
        "PC", xy=pc_pt.pos, xytext=(6, -10), textcoords="offset points",
        color=STYLE["pitch_center"], fontsize=9,
    )

    plt.show()
