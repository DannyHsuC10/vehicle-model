"""LP02 STL playback using the full-car model's static pose as the reference."""

from pathlib import Path
import tkinter as tk
from tkinter import ttk

import numpy as np

from .project import CORNERS


STL_DIR = Path(__file__).resolve().parents[1] / "assets" / "LP02"
BACKGROUND = "#192337"


def vehicle_motion(result):
    signals = result.signals
    if result.project["model"] == "flex":
        roll_front, roll_rear = signals["前側傾 [deg]"], signals["後側傾 [deg]"]
    else:
        roll_front = roll_rear = signals["側傾 [deg]"]
    wheels = result.motion[:, :, 1]
    return (signals["升沉 [mm]"], np.deg2rad(signals["俯仰 [deg]"]),
            np.deg2rad(roll_front), np.deg2rad(roll_rear), 1000 * (wheels - wheels[0]))


class LP02Scene:
    def __init__(self, size):
        import pyvista as pv

        chassis = pv.read(STL_DIR / "LP02.stl")
        wheels = [pv.read(STL_DIR / f"W_{corner}.stl") for corner in CORNERS]
        hubs = np.array([mesh.center for mesh in wheels])
        self.front_x, self.rear_x = hubs[:2, 0].mean(), hubs[2:, 0].mean()
        self.center_y = hubs[:, 1].mean()
        self.ground_z = min(mesh.bounds[4] for mesh in wheels)
        self.vertices = np.array(chassis.points, dtype=float)
        self.chassis = chassis
        # Keep each axle's roll constant beyond the wheelbase and blend between the axles.
        self.front_weight = np.clip((self.vertices[:, 0] - self.rear_x) / (self.front_x - self.rear_x), 0., 1.)

        self.plotter = pv.Plotter(off_screen=True, window_size=size)
        self.aspect = max(1., size[1] / size[0])
        try:
            self.plotter.set_background(BACKGROUND)
            self.plotter.add_mesh(chassis, color="#c0cfde", specular=.2, render=False)
            self.wheel_actors = [self.plotter.add_mesh(mesh, color="#394754", specular=.2, render=False)
                                 for mesh in wheels]
            self.plotter.enable_parallel_projection()
            self.reset_camera()
            self.plotter.show(auto_close=False)
        except Exception:
            self.plotter.close()
            raise

    def set_reference(self, vehicle):
        # Place the simulated CG on the CAD wheelbase using its front/rear weight split.
        rear_fraction = vehicle["lr"] / (vehicle["lf"] + vehicle["lr"])
        self.cg = np.array([self.rear_x + rear_fraction * (self.front_x - self.rear_x),
                            self.center_y, self.ground_z + vehicle["h_cg"] * 1000])
        self.local = self.vertices - self.cg

    def set_pose(self, heave, pitch, roll_front, roll_rear, wheel_z):
        # CAD X is forward, Y is left, and Z is up. Positive solver pitch lifts the nose.
        roll = roll_rear + self.front_weight * (roll_front - roll_rear)
        x, y, z = self.local.T
        c, s = np.cos(roll), np.sin(roll)
        y, z = c * y - s * z, s * y + c * z
        c, s = np.cos(pitch), np.sin(pitch)
        self.chassis.points[:] = self.cg + np.column_stack((c * x - s * z, y, s * x + c * z + heave))
        for actor, height in zip(self.wheel_actors, wheel_z):
            actor.position = (0., 0., float(height))

    def reset_camera(self):
        self.plotter.view_isometric(render=False)
        self.plotter.camera.parallel_scale *= self.aspect / 1.4

    def image(self, size):
        if tuple(self.plotter.window_size) != size:
            self.plotter.window_size = size
            aspect = max(1., size[1] / size[0])
            self.plotter.camera.parallel_scale *= aspect / self.aspect
            self.aspect = aspect
        self.plotter.renderer.ResetCameraClippingRange()
        self.plotter.render()
        return self.plotter.screenshot(return_img=True, transparent_background=False)


class FullVehicleView(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        toolbar = ttk.Frame(self, padding=(6, 4))
        toolbar.pack(fill="x")
        self.reset = ttk.Button(toolbar, text="重設視角", command=self.reset_camera, state="disabled")
        self.reset.pack(side="right")
        ttk.Label(toolbar, text="拖曳旋轉 · 滾輪縮放").pack(side="left")
        self.canvas = tk.Canvas(self, bg=BACKGROUND, width=380, height=380, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.preview = self.canvas.create_image(0, 0, anchor="nw")
        self.message = self.canvas.create_text(12, 12, anchor="nw", fill="#e2e8f0", width=350)
        for event in ("<Configure>", "<Map>"):
            self.canvas.bind(event, self.schedule_draw)
        self.canvas.bind("<ButtonPress-1>", self.begin_drag)
        self.canvas.bind("<B1-Motion>", self.orbit)
        self.canvas.bind("<MouseWheel>", self.zoom)
        self.canvas.bind("<Button-4>", self.zoom)
        self.canvas.bind("<Button-5>", self.zoom)
        self.result = self.scene = self.pending = self.photo = self.drawn_index = None
        self.error = None
        self.index = 0

    def set_result(self, result):
        self.result, self.index = result, 0
        self.drawn_index = self.error = None
        self.motion = vehicle_motion(result) if result is not None else None
        if self.scene is not None and result is not None:
            self.scene.set_reference(result.project["vehicle"])
        if result is None:
            self.canvas.itemconfigure(self.preview, image="")
            self.photo = None
        self.reset.state(("!disabled",) if self.scene is not None and result is not None else ("disabled",))
        self.schedule_draw()

    def update_frame(self, index):
        if index != self.index:
            self.index = index
            self.schedule_draw()

    def schedule_draw(self, event=None):
        if self.pending is None and self.canvas.winfo_ismapped():
            self.pending = self.after_idle(self.draw)

    def draw(self):
        self.pending = None
        if not self.canvas.winfo_ismapped():
            return
        size = self.canvas.winfo_width(), self.canvas.winfo_height()
        self.canvas.itemconfigure(self.message, width=max(100, size[0] - 24))
        if self.result is None or self.error is not None:
            message = "計算後顯示 LP02 實車與四輪動態。" if self.result is None else self.error
            self.canvas.itemconfigure(self.message, text=message)
            return
        if min(size) < 80:
            return
        try:
            if self.scene is None:
                self.scene = LP02Scene(size)
                self.scene.set_reference(self.result.project["vehicle"])
                self.reset.state(("!disabled",))
            if self.drawn_index != self.index:
                self.scene.set_pose(*(values[self.index] for values in self.motion))
                self.drawn_index = self.index
            pixels = self.scene.image(size)
        except ImportError:
            self.show_error("實車 3D 需要 PyVista。請在啟動程式的 Python 環境安裝 requirements.txt。")
            return
        except (OSError, RuntimeError, ValueError) as exc:
            self.show_error(f"無法顯示 LP02 實車 3D。\n{exc}")
            return
        height, width = pixels.shape[:2]
        header = f"P6\n{width} {height}\n255\n".encode("ascii")
        self.photo = tk.PhotoImage(master=self.canvas, data=header + pixels.tobytes(), format="PPM")
        self.canvas.itemconfigure(self.preview, image=self.photo)
        self.canvas.itemconfigure(self.message, text="")

    def show_error(self, message):
        self.error = message
        self.canvas.itemconfigure(self.preview, image="")
        self.canvas.itemconfigure(self.message, text=message)
        self.reset.state(("disabled",))
        self.photo = None
        if self.scene is not None:
            self.scene.plotter.close()
            self.scene = None

    def begin_drag(self, event):
        self.drag = event.x, event.y

    def orbit(self, event):
        x, y = self.drag
        self.drag = event.x, event.y
        if self.scene is not None and self.result is not None:
            camera = self.scene.plotter.camera
            camera.Azimuth((x - event.x) * .4)
            camera.Elevation((event.y - y) * .4)
            camera.OrthogonalizeViewUp()
            self.schedule_draw()

    def zoom(self, event):
        if self.scene is not None and self.result is not None:
            camera = self.scene.plotter.camera
            direction = 1 if event.delta > 0 or event.num == 4 else -1
            camera.parallel_scale = float(np.clip(camera.parallel_scale / 1.12**direction, 150., 20_000.))
            self.schedule_draw()

    def reset_camera(self):
        if self.scene is not None:
            self.scene.reset_camera()
            self.schedule_draw()

    def destroy(self):
        if self.pending is not None:
            self.after_cancel(self.pending)
        if self.scene is not None:
            self.scene.plotter.close()
        super().destroy()
