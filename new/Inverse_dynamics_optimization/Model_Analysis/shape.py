# 形狀計算工具(材料力學使用)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle


# =========================================================
# Hollow Rectangle Section
# =========================================================

class HollowRectangleSection:

    def __init__(self, width, height, thickness=0):

        self.b = width
        self.h = height
        self.t = thickness

        # 內部尺寸
        self.bi = max(0, self.b - 2 * self.t)
        self.hi = max(0, self.h - 2 * self.t)

    # -----------------------------------------------------
    # Area
    # -----------------------------------------------------

    def area(self):

        outer = self.b * self.h
        inner = self.bi * self.hi

        return outer - inner

    # -----------------------------------------------------
    # Ix
    # -----------------------------------------------------

    def Ix(self):

        outer = self.b * self.h**3 / 12
        inner = self.bi * self.hi**3 / 12

        return outer - inner

    # -----------------------------------------------------
    # Iy
    # -----------------------------------------------------

    def Iy(self):

        outer = self.h * self.b**3 / 12
        inner = self.hi * self.bi**3 / 12

        return outer - inner

    # -----------------------------------------------------
    # Polar Moment
    # J = Ix + Iy
    # -----------------------------------------------------

    def J(self):

        return self.Ix() + self.Iy()

    # -----------------------------------------------------
    # Section Modulus
    # -----------------------------------------------------

    def Zx(self):

        return self.Ix() / (self.h / 2)

    def Zy(self):

        return self.Iy() / (self.b / 2)

    # -----------------------------------------------------
    # Polar Section Modulus
    # -----------------------------------------------------

    def Zp(self):

        r = np.sqrt((self.b / 2)**2 + (self.h / 2)**2)

        return self.J() / r

    # -----------------------------------------------------
    # Draw
    # -----------------------------------------------------

    def draw(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots()

        # 外框
        outer = Rectangle(
            (-self.b/2, -self.h/2),
            self.b,
            self.h,
            fill=False,
            linewidth=2
        )

        ax.add_patch(outer)

        # 中空部分
        if self.t > 0:

            inner = Rectangle(
                (-self.bi/2, -self.hi/2),
                self.bi,
                self.hi,
                fill=False,
                linestyle="--"
            )

            ax.add_patch(inner)

        ax.set_aspect("equal")

        margin = max(self.b, self.h) * 0.6

        ax.set_xlim(-margin, margin)
        ax.set_ylim(-margin, margin)

        ax.set_title("Hollow Rectangle Section")

        return ax


# =========================================================
# Hollow Circle Section
# =========================================================

class HollowCircleSection:

    def __init__(self, outer_radius, thickness=0):

        self.ro = outer_radius
        self.t = thickness

        self.ri = max(0, self.ro - self.t)

    # -----------------------------------------------------
    # Area
    # -----------------------------------------------------

    def area(self):

        return np.pi * (self.ro**2 - self.ri**2)

    # -----------------------------------------------------
    # Ix = Iy
    # -----------------------------------------------------

    def Ix(self):

        return np.pi / 4 * (self.ro**4 - self.ri**4)

    def Iy(self):

        return self.Ix()

    # -----------------------------------------------------
    # Polar Moment
    # -----------------------------------------------------

    def J(self):

        return np.pi / 2 * (self.ro**4 - self.ri**4)

    # -----------------------------------------------------
    # Section Modulus
    # -----------------------------------------------------

    def Zx(self):

        return self.Ix() / self.ro

    def Zy(self):

        return self.Zx()

    # -----------------------------------------------------
    # Polar Section Modulus
    # -----------------------------------------------------

    def Zp(self):

        return self.J() / self.ro

    # -----------------------------------------------------
    # Draw
    # -----------------------------------------------------

    def draw(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots()

        # 外圓
        outer = Circle(
            (0, 0),
            self.ro,
            fill=False,
            linewidth=2
        )

        ax.add_patch(outer)

        # 內圓
        if self.t > 0:

            inner = Circle(
                (0, 0),
                self.ri,
                fill=False,
                linestyle="--"
            )

            ax.add_patch(inner)

        ax.set_aspect("equal")

        margin = self.ro * 1.3

        ax.set_xlim(-margin, margin)
        ax.set_ylim(-margin, margin)

        ax.set_title("Hollow Circle Section")

        return ax


# =========================================================
# Example Usage
# =========================================================

if __name__ == "__main__":

    # -----------------------------------------------------
    # Rectangle Tube
    # -----------------------------------------------------

    rect = HollowRectangleSection(
        width=100,
        height=60,
        thickness=5
    )

    print("===== Hollow Rectangle =====")

    print("Area =", rect.area())
    print("Ix =", rect.Ix())
    print("Iy =", rect.Iy())
    print("J =", rect.J())
    print("Zx =", rect.Zx())
    print("Zy =", rect.Zy())
    print("Zp =", rect.Zp())

    fig1, ax1 = plt.subplots()
    rect.draw(ax1)

    # -----------------------------------------------------
    # Hollow Circle
    # -----------------------------------------------------

    tube = HollowCircleSection(
        outer_radius=50,
        thickness=5
    )

    print("\n===== Hollow Circle =====")

    print("Area =", tube.area())
    print("Ix =", tube.Ix())
    print("Iy =", tube.Iy())
    print("J =", tube.J())
    print("Zx =", tube.Zx())
    print("Zy =", tube.Zy())
    print("Zp =", tube.Zp())

    fig2, ax2 = plt.subplots()
    tube.draw(ax2)

    plt.show()
# 使用完之後可參考計算數值填入CSV
