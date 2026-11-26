"""Repeatable proving-ground inputs and measured four-wheel replay."""

from bisect import bisect_right
import math

import numpy as np

from .project import BUMP_CASES, CASE_DEFAULTS, CORNERS, TARGETED_ROAD_CASES, TARGET_WHEELS, case_axes
from .road_profiles import ISO_ROUGHNESS, IsoRoadProfile, RandomRoadProfile


class Road:
    def __init__(self, options, wheelbase, corners=CORNERS, axes=(True, True)):
        self.options = options
        self.wheel_count = len(corners)
        self.scenario = options["scenario"]
        self.duration = options["duration"]
        self.replay_start = 0.
        self.bump = self.scenario in BUMP_CASES
        self.iso = self.scenario in ISO_ROUGHNESS
        targeted_road = self.scenario in TARGETED_ROAD_CASES
        self.tire_filter = (options.get("tire_road_filter", False) and options["speed"] > 0
                            and (targeted_road or self.scenario == "csv"))
        self.filter_tracks = {}
        self.discontinuities = set()
        command_axes = case_axes(self.scenario, axes, options.get("iso_lateral", CASE_DEFAULTS["iso_lateral"]))
        self.maneuver = any(command_axes)
        self.onsets = []
        if self.scenario == "csv":
            rows = np.asarray(options["recording"], dtype=float)
            self.times = tuple(rows[:, 0] - rows[0, 0])
            self.duration = self.times[-1]
            self.heights = rows[:, [1 + CORNERS.index(corner) for corner in corners]] / 1000
            self.acceleration = np.where(axes, rows[:, 5:], 0.)
            dt = np.diff(self.times)[:, None]
            self.secants = np.diff(self.heights, axis=0) / dt
            self.slopes = np.zeros_like(self.heights)
            # Shape-preserving cubic interpolation with a stationary first/last road sample.
            left, right = self.secants[:-1], self.secants[1:]
            same = left * right > 0
            w_left = np.broadcast_to(2 * dt[1:] + dt[:-1], same.shape)[same]
            w_right = np.broadcast_to(dt[1:] + 2 * dt[:-1], same.shape)[same]
            self.slopes[1:-1][same] = (w_left + w_right) / (w_left / left[same] + w_right / right[same])
            if self.tire_filter:
                self.filter_tracks = dict.fromkeys(range(self.wheel_count), (0., None))
                self.quadratic = 3 * self.secants - 2 * self.slopes[:-1] - self.slopes[1:]
                self.cubic = self.slopes[:-1] + self.slopes[1:] - 2 * self.secants
                area = dt * (self.heights[:-1] + dt * (
                    self.slopes[:-1] / 2 + self.quadratic / 3 + self.cubic / 4))
                self.integral = np.vstack((np.zeros(self.wheel_count), np.cumsum(area, axis=0)))
            self.events = self.times
            return

        self.command = tuple(options[key] if active else 0. for key, active in zip(("ax", "ay"), command_axes))
        follow_front = (self.bump or self.iso) and options["rear_delay"] and axes[0]
        delay = wheelbase / options["speed"] if follow_front else 0.
        selected = TARGET_WHEELS[options["target"]]
        self.onsets = [(index, options["start"] + (delay if corner.startswith("R") else 0.))
                       for index, corner in enumerate(corners) if corner in selected and targeted_road]
        boundaries = {options["start"], options["start"] + options["ramp"]} if self.maneuver else set()
        for _, start in self.onsets:
            boundaries.add(start)
            if self.bump:
                boundaries.add(start + options["period"])
        if self.scenario == "step":
            self.discontinuities.update(start for _, start in self.onsets)
        elif self.scenario == "random":
            self.seed = int(options.get("seed", CASE_DEFAULTS["seed"]))
            self.random_interval = options.get("random_interval", CASE_DEFAULTS["random_interval"])
            self.random_profile = RandomRoadProfile(options["start"], self.duration, self.random_interval,
                                                    options["height"] / 1000, self.seed)
            if self.onsets:
                self.discontinuities.update(t for t in self.random_profile.times if t <= self.duration)
        boundaries.update(self.discontinuities)
        if self.iso:
            self.seed = int(options.get("seed", CASE_DEFAULTS["seed"]))
            track_corners = [corners[index] for index, _ in self.onsets]
            if follow_front:
                track_corners = ["F" + corner[1] for corner in track_corners]
            self.iso_tracks = tuple(CORNERS.index(corner) for corner in track_corners)
            profiles = {}
            self.iso_onsets = {}
            for (index, start), track in zip(self.onsets, self.iso_tracks):
                if track not in profiles:
                    profiles[track] = IsoRoadProfile(ISO_ROUGHNESS[self.scenario], self.seed, track=track)
                    if self.tire_filter:
                        profiles[track].prepare_integral()
                self.iso_onsets.setdefault(start, []).append((index, profiles[track]))
                if self.tire_filter:
                    self.filter_tracks[index] = (start, profiles[track])
        elif self.tire_filter:
            self.filter_tracks = {index: (start, None) for index, start in self.onsets}
        self.events = tuple(sorted(t for t in boundaries if 0 < t < self.duration))

    def prepend_zero_input(self, seconds):
        """Delay CSV input while keeping its original interpolation."""
        self.replay_start = seconds
        self.duration = seconds + self.times[-1]
        self.events = tuple(seconds + time for time in self.times)

    @property
    def replay_notice(self):
        if self.replay_start:
            return f"LapData 首筆無法求得靜態平衡。前 {self.replay_start:g} 秒以零輸入模擬，之後接續原始資料。"
        return ""

    def at(self, time):
        if self.scenario == "csv":
            if self.replay_start and time < self.replay_start:
                zero = np.zeros(self.wheel_count)
                return zero, zero, 0., 0.
            time -= self.replay_start
            i = min(len(self.times) - 2, max(0, bisect_right(self.times, time) - 1))
            dt = self.times[i + 1] - self.times[i]
            x = min(1., max(0., (time - self.times[i]) / dt))
            left, right = self.slopes[i:i + 2]
            gradient = self.secants[i]
            a = 3 * gradient - 2 * left - right
            b = left + right - 2 * gradient
            height = self.heights[i] + dt * x * (left + x * (a + x * b))
            speed = left + x * (2 * a + 3 * x * b)
            ax, ay = (1 - x) * self.acceleration[i] + x * self.acceleration[i + 1]
            return height, speed, ax, ay

        o = self.options
        height, speed = np.zeros(self.wheel_count), np.zeros(self.wheel_count)
        if self.iso:
            for start, tracks in self.iso_onsets.items():
                if time >= start:
                    distance = o["speed"] * (time - start)
                    for index, profile in tracks:
                        z, slope = profile.at(distance)
                        height[index] = z
                        speed[index] = o["speed"] * slope
        else:
            for index, start in self.onsets:
                elapsed = time - start
                if self.scenario == "step":
                    height[index] = o["height"] / 1000 if time >= start else 0.
                elif self.scenario == "random":
                    height[index], speed[index], _ = self.random_profile.sample(time)
                elif 0 < elapsed < o["period"]:
                    phase = math.tau * elapsed / o["period"]
                    height[index] = o["height"] * (1 - math.cos(phase)) / 2000
                    speed[index] = o["height"] * math.pi / (1000 * o["period"]) * math.sin(phase)
        ax = ay = 0.
        if self.maneuver:
            fraction = min(1., max(0., (time - o["start"]) / o["ramp"]))
            blend = (1 - math.cos(math.pi * fraction)) / 2
            ax, ay = (value * blend for value in self.command)
        return height, speed, ax, ay

    def _csv_height_and_integral(self, index, time):
        if time <= 0:
            height = self.heights[0, index]
            return height, time * height
        if time >= self.times[-1]:
            height = self.heights[-1, index]
            return height, self.integral[-1, index] + (time - self.times[-1]) * height
        i = bisect_right(self.times, time) - 1
        elapsed = time - self.times[i]
        u = elapsed / (self.times[i + 1] - self.times[i])
        height, slope = self.heights[i, index], self.slopes[i, index]
        a, b = self.quadratic[i, index], self.cubic[i, index]
        area = elapsed * (height + elapsed * (slope / 2 + u * (a / 3 + u * b / 4)))
        height += elapsed * (slope + u * (a + u * b))
        return height, self.integral[i, index] + area

    def _height_and_integral(self, index, time):
        start, profile = self.filter_tracks[index]
        if self.scenario == "csv":
            return self._csv_height_and_integral(index, time - self.replay_start)
        if self.iso:
            speed = self.options["speed"]
            height, area = profile.height_and_integral(speed * (time - start))
            return height, area / speed
        if self.scenario == "step":
            height = self.options["height"] / 1000 if time >= start else 0.
            return height, max(0., time - start) * height
        if self.scenario == "random":
            height, _, area = self.random_profile.sample(time)
            return height, area
        period = self.options["period"]
        elapsed = min(period, max(0., time - start))
        scale = self.options["height"] / 2000
        omega = math.tau / period
        phase = omega * elapsed
        height = scale * (1 - math.cos(phase))
        return height, scale * (elapsed - math.sin(phase) / omega)

    def average(self, index, time, half_window):
        """Return the centered box average and its velocity at fixed window width."""
        left, left_area = self._height_and_integral(index, time - half_window)
        right, right_area = self._height_and_integral(index, time + half_window)
        width = 2 * half_window
        return (right_area - left_area) / width, (right - left) / width

    def signature(self):
        o = self.options
        if self.scenario == "csv":
            return (self.scenario, o["dt"], self.replay_start,
                    self.times, self.heights.tobytes(), self.acceleration.tobytes())
        track = ()
        if self.bump:
            track = (tuple(self.onsets), o["height"], o["period"])
        elif self.scenario == "step":
            track = (tuple(self.onsets), o["height"])
        elif self.scenario == "random":
            track = (tuple(self.onsets), o["height"], self.seed, self.random_interval)
        elif self.iso:
            track = (tuple(self.onsets), o["speed"], self.seed, self.iso_tracks)
        maneuver = (o["start"], o["ramp"], *self.command) if self.maneuver and any(self.command) else ()
        return self.scenario, self.duration, o["dt"], track, maneuver
