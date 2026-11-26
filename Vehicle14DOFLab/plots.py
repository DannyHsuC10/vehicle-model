"""Export self-contained SVG figures from a run's telemetry."""

from html import escape
import math

import numpy as np


BLUE = "#2059a6"
TEAL = "#087c77"
ORANGE = "#bc4b17"
TEXT = "#172b40"
MUTED = "#526678"
GRID = "#e1e8ee"
SPEED_STOPS = ((30, 55, 115), (0, 136, 148), (245, 176, 45))


def text(x, y, label, *, size=13, fill=MUTED, anchor="start", weight=400):
    return (f'<text x="{x:.1f}" y="{y:.1f}" fill="{fill}" text-anchor="{anchor}" '
            f'font-size="{size}" font-weight="{weight}">{escape(label)}</text>')


def line(x1, y1, x2, y2, color=GRID, width=1, dash=""):
    return (f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>')


def path(x, y, color, width, dash=""):
    points = " ".join(f"{a:.2f},{b:.2f}" for a, b in zip(x, y))
    return (f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width}" '
            f'stroke-dasharray="{dash}" stroke-linejoin="round" stroke-linecap="round"/>')


def svg_file(output, title, subtitle, elements, *, height=530):
    content = "\n".join((
        f'<svg xmlns="http://www.w3.org/2000/svg" width="960" height="{height}" viewBox="0 0 960 {height}">',
        f"<title>{escape(title)}</title>",
        '<rect width="100%" height="100%" fill="white"/>',
        '<g font-family="Segoe UI, system-ui, sans-serif">',
        text(28, 38, title, size=24, fill=TEXT, weight=700),
        text(28, 62, subtitle, size=14),
        *elements, "</g>", "</svg>",
    ))
    output.write_text(content, encoding="utf-8")


def ticks(low, high, count=6):
    span = (high - low) / count
    decade = 10 ** math.floor(math.log10(span))
    step = next(unit * decade for unit in (1, 2, 2.5, 5, 10) if unit * decade >= span)
    return np.arange(math.ceil(low / step), math.floor(high / step) + 1) * step


def axes(x_range, y_range, box, x_label, y_label):
    low_x, high_x = x_range
    low_y, high_y = y_range
    left, top, right, bottom = box

    def x(value):
        return left + (value - low_x) * (right - left) / (high_x - low_x)

    def y(value):
        return bottom - (value - low_y) * (bottom - top) / (high_y - low_y)

    elements = []
    for value in ticks(low_x, high_x, 8):
        pos = x(value)
        elements.extend((line(pos, top, pos, bottom), text(pos, bottom + 20, f"{value:g}", anchor="middle")))
    for value in ticks(low_y, high_y):
        pos = y(value)
        elements.extend((line(left, pos, right, pos), text(left - 10, pos + 4, f"{value:g}", anchor="end")))
    elements.extend((line(left, bottom, right, bottom, MUTED), line(left, top, left, bottom, MUTED),
                     text((left + right) / 2, bottom + 43, x_label, anchor="middle"),
                     text(left, top - 12, y_label)))
    return x, y, elements


def time_plot(output, title, subtitle, time, y_label, series, *, timed_start=0.):
    lower = min(0., *(float(values.min()) for _, values, _ in series))
    upper = max(0., *(float(values.max()) for _, values, _ in series))
    padding = max(.1, .08 * (upper - lower))
    x, y, elements = axes((0., time[-1]), (lower - padding, upper + padding),
                          (80, 112, 932, 466), "Simulation time (s)", y_label)
    elements.append(line(80, y(0.), 932, y(0.), MUTED))
    if timed_start > 0.:
        boundary = x(timed_start)
        elements.extend((line(boundary, 113, boundary, 466, MUTED, 1, "4 4"),
                         text(boundary, 104, "Timing starts", size=12, anchor="middle")))
    for i, (label, values, color) in enumerate(series):
        legend_x = 934 - 142 * len(series) + 142 * i
        elements.extend((path(x(time), y(values), color, 2.3),
                         line(legend_x, 91, legend_x + 22, 91, color, 2.5),
                         text(legend_x + 28, 95, label, fill=TEXT)))
    svg_file(output, title, subtitle, elements)


def speed_map(output, subtitle, track, position, speed):
    left, top, right, bottom = 80, 113, 804, 590
    combined = np.vstack((track, position))
    lo, hi = combined.min(axis=0), combined.max(axis=0)
    scale = 1.14 * max((hi[0] - lo[0]) / (right - left), (hi[1] - lo[1]) / (bottom - top))
    midpoint = (lo + hi) * .5
    span = scale * np.array([right - left, bottom - top])
    lo, hi = midpoint - span / 2., midpoint + span / 2.
    x, y, elements = axes((lo[0], hi[0]), (lo[1], hi[1]), (left, top, right, bottom),
                          "World X (m)", "World Y (m)")
    elements.append(path(x(track[:, 0]), y(track[:, 1]), "#a6b2be", 4, "5 7"))
    elements.extend((line(631, 86, 657, 86, "#a6b2be", 2, "4 4"),
                     text(665, 90, "Track centerline")))

    low, high = float(speed.min()), float(speed.max())
    normalized = (speed - low) / max(1e-9, high - low)
    bins = np.minimum(39, np.floor(40 * (normalized[:-1] + normalized[1:]) / 2.)).astype(int)
    edges = np.r_[0, np.flatnonzero(np.diff(bins)) + 1, len(bins)] if bins.size else [0]
    xp, yp = x(position[:, 0]), y(position[:, 1])
    for start, end in zip(edges[:-1], edges[1:]):
        value = (bins[start] + .5) / 40
        rgb = [round(np.interp(value, (0., .5, 1.), component)) for component in zip(*SPEED_STOPS)]
        elements.append(path(xp[start:end + 1], yp[start:end + 1], "#{:02x}{:02x}{:02x}".format(*rgb), 3.5))

    legend_top, legend_bottom = 167, 480
    stops = "".join(f'<stop offset="{i / 2:g}" stop-color="#{r:02x}{g:02x}{b:02x}"/>'
                    for i, (r, g, b) in enumerate(SPEED_STOPS))
    elements.extend((
        '<defs><linearGradient id="speed-scale" x1="0" y1="1" x2="0" y2="0">'
        f'{stops}</linearGradient></defs>',
        text(833, 148, "Speed (km/h)", fill=TEXT, weight=600),
        f'<rect x="834" y="{legend_top}" width="16" height="{legend_bottom - legend_top}" fill="url(#speed-scale)"/>',
    ))
    speed_range = high - low
    decimals = 1 if speed_range >= .4 else 3
    for fraction in (np.linspace(0., 1., 5) if speed_range >= .004 else [.5]):
        py = legend_bottom - fraction * (legend_bottom - legend_top)
        elements.extend((line(851, py, 858, py, MUTED),
                         text(864, py + 4, f"{low + fraction * speed_range:.{decimals}f}")))
    for index, fill in ((0, "white"), (-1, TEXT)):
        elements.append(f'<circle cx="{xp[index]:.2f}" cy="{yp[index]:.2f}" r="4" fill="{fill}" stroke="{TEXT}"/>')
    elements.append(text(830, 515, "○ Start   ● Finish", size=12))
    svg_file(output, "Speed map", subtitle, elements, height=657)


def write_plots(output, rows, centerline, case_name, *, map_start=0.):
    def column(name):
        return np.asarray([row[name] for row in rows])

    time = column("time_s")
    speed = 3.6 * column("speed_mps")
    subtitle = f"{case_name}  |  14 DOF  |  {time[-1]:.3f} s simulated"
    figure = {name: output.with_name(f"{output.stem}_{name}.svg")
              for name in ("speed_map", "speed_time", "acceleration_time", "aero_time")}
    map_rows = time >= map_start
    speed_map(figure["speed_map"], subtitle, centerline,
              np.column_stack((column("x_m")[map_rows], column("y_m")[map_rows])), speed[map_rows])
    time_plot(figure["speed_time"], "Speed vs time", subtitle, time, "Speed (km/h)",
              (("Vehicle speed", speed, BLUE),), timed_start=map_start)
    time_plot(figure["acceleration_time"], "Acceleration vs time", subtitle, time, "Acceleration (m/s²)",
              (("Longitudinal", column("ax_mps2"), BLUE), ("Lateral", column("ay_mps2"), ORANGE)),
              timed_start=map_start)
    time_plot(figure["aero_time"], "Aerodynamic forces vs time", subtitle, time, "Force (N)",
              (("Downforce", column("downforce_n"), TEAL), ("Drag", column("drag_n"), ORANGE)),
              timed_start=map_start)
    if {"cop_x_m", "cop_y_m", "cop_z_m"} <= rows[0].keys():
        figure["aero_cop_time"] = output.with_name(f"{output.stem}_aero_cop_time.svg")
        time_plot(figure["aero_cop_time"], "Aerodynamic center of pressure vs time", subtitle, time, "Position (m)",
                  (("X fore-aft", column("cop_x_m"), BLUE), ("Y lateral", column("cop_y_m"), ORANGE),
                   ("Z above road", column("cop_z_m"), TEAL)), timed_start=map_start)
    return list(figure.values())
