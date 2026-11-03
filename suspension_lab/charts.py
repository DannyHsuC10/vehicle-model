"""Vehicle Lab Canvas charts extended with rotatable filled surface meshes."""
import math
import html
from pathlib import Path

from vehicle_lab.charts import Chart, BG


def heat_color(fraction):
    f = max(0., min(1., fraction))
    stops = [(56, 189, 248), (52, 211, 153), (251, 191, 36), (251, 113, 133)]
    index = min(int(f*3), 2)
    t = f*3-index
    return "#" + "".join(f"{round(a+(b-a)*t):02x}" for a, b in zip(stops[index], stops[index+1]))


class LabChart(Chart):
    def __init__(self, master):
        self.polygons = []
        self.circles = []
        self._plot_bounds = None
        super().__init__(master)
        self.canvas.bind("<Map>", lambda event: self.draw(), add="+")
        self.azimuth = .65
        self.elevation = .6

    def set(self, spec, *, preserve_view=False):
        """Update a pose without refitting the user's current plot viewport."""
        if preserve_view:
            self.spec = spec
            self.draw()
        else:
            self._plot_bounds = None
            super().set(spec)

    def reset(self):
        self.azimuth, self.elevation = .65, .6
        super().reset()

    def _draw_flat(self):
        """Keep equal metric scale after resize without fitting moving geometry."""
        super().draw()
        if not self.spec or not self.spec.get("equal") or self.view is None:
            return
        l, t, r, b = self.bounds
        width, height = max(r-l, 1), max(b-t, 1)
        xmin, xmax, ymin, ymax = self.view
        if not math.isclose(width/(xmax-xmin), height/(ymax-ymin), rel_tol=1e-10):
            previous = self._plot_bounds or self.bounds
            # Preserve the scale and centre chosen before the viewport resized.
            units_per_pixel = (ymax-ymin)/max(previous[3]-previous[1], 1)
            cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
            dx, dy = width*units_per_pixel/2, height*units_per_pixel/2
            self.view = [cx-dx, cx+dx, cy-dy, cy+dy]
            super().draw()
        self._plot_bounds = self.bounds

    def draw(self):
        self.polygons = []
        self.circles = []
        # Fit a new plot only after its notebook page has a real viewport.
        if (not self.canvas.winfo_ismapped() or self.canvas.winfo_width() <= 1
                or self.canvas.winfo_height() <= 1):
            return
        spec = self.spec
        if spec and "mesh" in spec:
            return self.draw_mesh(spec)
        if not spec or "world_lines" not in spec:
            return self._draw_flat()
        dimension = spec.get("dimension", 3)
        projection = spec.get("projection", "Iso")
        project = lambda p: tuple(p) if dimension == 2 else self.project(p, projection)
        physical, overlays = [], []
        for path in spec["world_lines"]:
            points = [project(p) for p in path["points"]]
            series = {"name": path["name"], "color": path["color"], "dash": path["dash"], "width": path.get("width", 1),
                      "x": [p[0] for p in points], "y": [p[1] for p in points]}
            (overlays if path["construction"] or not path.get("fit", True) else physical).append(series)
        try:
            self.spec = {**spec, "series": physical}
            self._draw_flat()
        finally:
            self.spec = spec
        if not self.view or not hasattr(self, "bounds"):
            return
        l, t, r, b = self.bounds
        xmin, xmax, ymin, ymax = self.view
        for s in overlays:
            points = [(l+(x-xmin)/(xmax-xmin)*(r-l), b-(y-ymin)/(ymax-ymin)*(b-t))
                      for x, y in zip(s["x"], s["y"])]
            for a, z in zip(points, points[1:]):
                clipped = self.clip(a, z, l, t, r, b)
                if clipped:
                    self.line(clipped, s["color"], s["width"], s["dash"])
        self.draw_annotations(spec, project)

    def draw_annotations(self, spec, project):
        l, t, r, b = self.bounds
        xmin, xmax, ymin, ymax = self.view
        for annotation in spec.get("markers", [])+spec.get("annotations", []):
            x, y = project(annotation["position"])
            px, py = l+(x-xmin)/(xmax-xmin)*(r-l), b-(y-ymin)/(ymax-ymin)*(b-t)
            radius = annotation.get("radius")
            if not (l <= px <= r and t <= py <= b):
                if radius is not None or "start" not in annotation:
                    continue
                a, z = project(annotation["start"])
                start = (l+(a-xmin)/(xmax-xmin)*(r-l), b-(z-ymin)/(ymax-ymin)*(b-t))
                clipped = self.clip(start, (px, py), l+20, t+20, r-20, b-20)
                if not clipped:
                    continue
                px, py = clipped[2:]
            color = annotation["color"]
            if radius is not None:
                hollow = annotation.get("hollow", False)
                outline = color if hollow else "#f8fafc"
                self.canvas.create_oval(px-radius, py-radius, px+radius, py+radius,
                                        fill="" if hollow else color, outline=outline, width=2)
                self.circles.append((px, py, radius, color, hollow, outline, annotation["name"]))
            dx, dy = annotation.get("offset", [12, -15])
            self.text(max(l+4, min(r-45, px+dx)), max(t+12, min(b-12, py+dy)),
                      annotation["name"], color, 11, "w")

    def draw_mesh(self, spec):
        self.canvas.delete("all")
        self.primitives = []
        w, h = max(self.canvas.winfo_width(), 300), max(self.canvas.winfo_height(), 220)
        points = [p for row in spec["mesh"] for p in row if all(self.finite(v) for v in p)]
        self.text(25, 22, spec["title"], "#f8fafc", 14, "w")
        if not points:
            self.text(w/2, h/2, "此範圍沒有可達的幾何姿態")
            self.view = None
            return
        ranges = [(min(p[i] for p in points), max(p[i] for p in points)) for i in range(3)]
        spans = [b-a if b-a > 1e-10 else max(abs(a)*.1, 1e-3) for a, b in ranges]
        norm = lambda p: [(p[i]-ranges[i][0])/spans[i]*2-1 for i in range(3)]
        l, t, r, b = 75, 65, w-90, h-85
        self.bounds = (l, t, r, b)
        if self.view is None:
            ratio = (r-l)/max(b-t, 1)
            self.view = [-1.8*max(ratio, 1), 1.8*max(ratio, 1), -1.8*max(1/ratio, 1), 1.8*max(1/ratio, 1)]
        xmin, xmax, ymin, ymax = self.view
        def screen(p):
            x, y = self.project(p, "Iso")
            return l+(x-xmin)/(xmax-xmin)*(r-l), b-(y-ymin)/(ymax-ymin)*(b-t)
        faces = []
        mesh = spec["mesh"]
        for j in range(len(mesh)-1):
            for i in range(len(mesh[j])-1):
                face = [mesh[j][i], mesh[j][i+1], mesh[j+1][i+1], mesh[j+1][i]]
                if not all(all(self.finite(v) for v in p) for p in face):
                    continue
                normalized = [norm(p) for p in face]
                # Painter's algorithm in the same orthographic camera as Chart.
                depth = sum((p[0]*math.sin(self.azimuth)+p[1]*math.cos(self.azimuth))*math.cos(self.elevation)
                            + p[2]*math.sin(self.elevation) for p in normalized)
                coords = [v for p in normalized for v in screen(p)]
                fraction = (sum(p[2] for p in face)/4-ranges[2][0])/spans[2]
                faces.append((depth, coords, heat_color(fraction)))
        for _, coords, color in sorted(faces):
            self.canvas.create_polygon(*coords, fill=color, outline="#334155", width=.6)
            self.polygons.append((coords, color))
        origin = [-1., -1., -1.]
        for axis, label, color in ((0, spec["xlabel"], "#38bdf8"), (1, spec["ylabel"], "#fbbf24"), (2, spec["zlabel"], "#fb7185")):
            end = origin.copy()
            end[axis] = 1.2
            self.line([*screen(origin), *screen(end)], color, 2)
            px, py = screen(end)
            # Z metric names can be long; keep labels inside the chart.
            self.text(max(85, min(w-85, px)), max(50, min(h-30, py+18)), label, color, 9)
            for f in (0., .5, 1.):
                tick = origin.copy()
                tick[axis] = -1+2*f
                x, y = screen(tick)
                value = ranges[axis][0]+spans[axis]*f
                self.text(x+5, y-9, f"{value:.4g}", color, 8)
        self.text(w-20, 48, f"Z: {ranges[2][0]:.5g} → {ranges[2][1]:.5g}", "#cbd5e1", 9, "e")
        self.text(w-20, h-18, "拖曳旋轉 · 滾輪縮放 · 雙擊重設；缺格表示無解 / 無有限交點", "#94a3b8", 9, "e")

    def hover(self, event):
        if self.spec and "mesh" in self.spec:
            return
        super().hover(event)

    def svg(self):
        svg = super().svg()
        polygons = []
        for coords, color in self.polygons:
            xy = " ".join(f"{coords[i]:.2f},{coords[i+1]:.2f}" for i in range(0, len(coords), 2))
            polygons.append(f'<polygon points="{xy}" fill="{color}" stroke="#334155" stroke-width="0.6"/>')
        lines = svg.splitlines()
        circles = [f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{"none" if hollow else color}" '
                   f'stroke="{outline}" stroke-width="2" data-label="{html.escape(name, quote=True)}"/>'
                   for x, y, radius, color, hollow, outline, name in self.circles]
        return "\n".join(lines[:2]+polygons+lines[2:-1]+circles+lines[-1:])

    def png(self, path):
        if not self.polygons and not self.circles:
            return super().png(path)
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError as exc:
            raise ValueError("PNG 匯出需要 Pillow；請安裝 requirements-suspension.txt。") from exc
        factor = 2
        w, h = max(self.canvas.winfo_width(), 300), max(self.canvas.winfo_height(), 220)
        image = Image.new("RGB", (w*factor, h*factor), BG)
        draw = ImageDraw.Draw(image)
        for coords, color in self.polygons:
            pairs = [(coords[i]*factor, coords[i+1]*factor) for i in range(0, len(coords), 2)]
            draw.polygon(pairs, fill=color, outline="#334155")
        font_path = Path("C:/Windows/Fonts/msjh.ttc")
        for kind, xy, color, size, *rest in self.primitives:
            scaled = [v*factor for v in xy]
            if kind == "line":
                dash = rest[0] if rest else ()
                if dash:
                    for a, b in self.dashed_segments(scaled, [d*factor for d in dash]):
                        draw.line([*a, *b], fill=color, width=max(1, round(size*factor)))
                else:
                    draw.line(scaled, fill=color, width=max(1, round(size*factor)))
            else:
                content, anchor = rest
                font = ImageFont.truetype(str(font_path), round(size*1.3*factor)) if font_path.exists() else ImageFont.load_default()
                draw.text(scaled, content, fill=color, font=font, anchor="rm" if anchor == "e" else "lm" if anchor == "w" else "mm")
        for x, y, radius, color, hollow, outline, name in self.circles:
            draw.ellipse([(x-radius)*factor, (y-radius)*factor, (x+radius)*factor, (y+radius)*factor],
                         fill=None if hollow else color, outline=outline, width=2*factor)
        image.save(path)
