"""Draw a circular section with its inner and outer diameters."""

from math import sqrt
import tkinter as tk

from .viewport import BG, FONT, INK, MUTED


class SectionSketch(tk.Canvas):
    def __init__(self, parent):
        super().__init__(parent, width=175, height=175, background=BG, highlightthickness=0)
        self.section = None
        self.bind("<Configure>", self.draw)

    def set_section(self, section):
        self.section = section
        self.draw()

    def draw(self, _event=None):
        self.delete("all")
        width, height = self.winfo_width(), self.winfo_height()
        if width < 80 or height < 100:
            return
        x, y = width / 2., height * .42
        if self.section is None:
            self.create_text(x, height / 2., text="請檢查截面尺寸", fill=MUTED, font=(FONT, 10))
            return

        outer = min(width * .30, height * .29)
        inner = outer * (self.section.inner_mm / self.section.outer_mm)
        outer_color = "#8bc6f6"
        inner_color = "#86dfce"
        self.create_oval(x - outer, y - outer, x + outer, y + outer, fill="#386178",
                         outline=outer_color, width=2, tags=("outer-circle",))
        if inner:
            self.create_oval(x - inner, y - inner, x + inner, y + inner, fill=BG,
                             outline=inner_color, width=1, tags=("inner-circle",))
        for dx, dy in ((1., 0.), (0., 1.)):
            length = outer + 8.
            self.create_line(x - dx * length, y - dy * length, x + dx * length, y + dy * length,
                             fill=MUTED, dash=(6, 4, 2, 4))

        dim_y = y + outer + 11.
        for end in (x - outer, x + outer):
            self.create_line(end, y + 4, end, dim_y + 4, fill=MUTED)
        self.create_line(x - outer, dim_y, x + outer, dim_y, fill=outer_color, arrow="both", arrowshape=(7, 8, 3))
        self.create_text(x, dim_y + 4, text="外徑 Dₒ", anchor="n", fill=INK, font=(FONT, 9))

        if inner >= 17.:
            self.create_line(x - inner, y, x + inner, y, fill=inner_color, arrow="both", arrowshape=(6, 7, 3))
            self.create_text(x, y - 2, text="Dᵢ", anchor="s", fill=inner_color, font=(FONT, 9))
        elif inner:
            at = inner / sqrt(2.)
            self.create_line(x + at, y - at, width - 18, y - outer - 2, fill=inner_color, arrow="first")
            self.create_text(width - 13, y - outer - 3, text="Dᵢ", anchor="se", fill=inner_color, font=(FONT, 9))
        else:
            self.create_text(x, y - outer / 2., text="實心", fill=INK, font=(FONT, 9))
