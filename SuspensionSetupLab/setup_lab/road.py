"""Repeatable proving-ground inputs and measured four-wheel replay."""

from bisect import bisect_right
import math

import numpy as np

from .project import BUMP_CASES, CASE_DEFAULTS, CORNERS, TARGET_WHEELS, case_axes
from .road_profiles import ISO_ROUGHNESS, IsoRoadProfile


class Road:
    def __init__(self, options, wheelbase, corners=CORNERS, axes=(True, True)):
        self.options = options
        self.wheel_count = len(corners)
        self.scenario = options["scenario"]
        self.duration = options["duration"]
        self.bump = self.scenario in BUMP_CASES
        self.iso = self.scenario in ISO_ROUGHNESS
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
            self.events = self.times
            return

        self.command = tuple(options[key] if active else 0. for key, active in zip(("ax", "ay"), command_axes))
        follow_front = options["rear_delay"] and axes[0]
        delay = wheelbase / options["speed"] if follow_front else 0.
        selected = TARGET_WHEELS[options["target"]]
        self.onsets = [(index, options["start"] + (delay if corner.startswith("R") else 0.))
                       for index, corner in enumerate(corners) if corner in selected and (self.bump or self.iso)]
        boundaries = {options["start"], options["start"] + options["ramp"]} if self.maneuver else set()
        for _, start in self.onsets:
            boundaries.add(start)
            if self.bump:
                boundaries.add(start + options["period"])
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
                self.iso_onsets.setdefault(start, []).append((index, profiles[track]))
        self.events = tuple(sorted(t for t in boundaries if 0 < t < self.duration))

    def at(self, time):
        if self.scenario == "csv":
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
                if 0 < elapsed < o["period"]:
                    phase = math.tau * elapsed / o["period"]
                    height[index] = o["height"] * (1 - math.cos(phase)) / 2000
                    speed[index] = o["height"] * math.pi / (1000 * o["period"]) * math.sin(phase)
        ax = ay = 0.
        if self.maneuver:
            fraction = min(1., max(0., (time - o["start"]) / o["ramp"]))
            blend = (1 - math.cos(math.pi * fraction)) / 2
            ax, ay = (value * blend for value in self.command)
        return height, speed, ax, ay

    def signature(self):
        o = self.options
        if self.scenario == "csv":
            return (self.scenario, o["dt"], self.times, self.heights.tobytes(), self.acceleration.tobytes())
        track = ()
        if self.bump:
            track = (tuple(self.onsets), o["height"], o["period"])
        elif self.iso:
            track = (tuple(self.onsets), o["speed"], self.seed, self.iso_tracks)
        maneuver = (o["start"], o["ramp"], *self.command) if self.maneuver and any(self.command) else ()
        return self.scenario, self.duration, o["dt"], track, maneuver
