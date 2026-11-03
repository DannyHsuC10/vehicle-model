"""Live visibility controls for the static suspension figure."""

from matplotlib.widgets import CheckButtons


def add_visibility_controls(figure, planes, arm_plane_artists, *, update_legend=None):
    """Toggle existing artists without rebuilding geometry or moving the camera."""
    panel = figure.add_axes([0.02, 0.68, 0.16, 0.20], facecolor="#f5f7fa")
    panel.set_title("View options", loc="left", fontsize=10, pad=8)
    for spine in panel.spines.values():
        spine.set_edgecolor("#cbd5e1")
    checkboxes = CheckButtons(
        panel,
        ("Filled planes", "Front FV", "Rear FV", "SV"),
        (figure.show_reference_planes, figure.view_visibility["front"],
         figure.view_visibility["rear"], figure.view_visibility["sv"]),
    )
    for label in checkboxes.labels:
        label.set_fontsize(9)

    def apply_visibility(_label=None):
        filled, front, rear, sv = map(bool, checkboxes.get_status())
        figure.show_reference_planes = filled
        figure.view_visibility.update(front=front, rear=rear, sv=sv)
        for name, plane in planes.items():
            visible = figure.view_visibility[name]
            plane.visible = visible
            for artist in plane.view_artists:
                artist.set_visible(visible)
            plane.plane_surface.set_visible(filled and visible)

        for name, artists in arm_plane_artists.items():
            if artists["surface"] is not None:
                artists["surface"].set_visible(filled)
            for kind, anchor in artists["anchors"].items():
                view = "sv" if kind == "SV_IC" else (
                    "front" if name.startswith("F") else "rear"
                )
                for key in ("marker", "label"):
                    if anchor[key] is not None:
                        anchor[key].set_visible(figure.view_visibility[view])
        if update_legend is not None:
            update_legend()
        figure.canvas.draw_idle()

    # Keep the widget alive for mouse events and expose it to Python callers.
    figure.visibility_checkboxes = checkboxes
    checkboxes.on_clicked(apply_visibility)
    apply_visibility()
    return checkboxes
