"""Seeded random and ISO 8608 longitudinal road profiles."""

from bisect import bisect_right
import math

import numpy as np


ISO_ROUGHNESS = {"iso_a": 16e-6, "iso_b": 64e-6, "iso_c": 256e-6}
ISO_REFERENCE_FREQUENCY = .1  # cycles/m
ISO_FREQUENCY_BAND = (.011, 2.83)  # cycles/m
PROFILE_LENGTH = 4096.  # m, spatial synthesis period
PROFILE_SAMPLE_COUNT = 2**18


class RandomRoadProfile:
    """Gaussian height knots joined by straight segments."""

    def __init__(self, start, duration, interval, height_std, seed):
        count = math.ceil((duration - start) / interval) + 2
        self.times = tuple(start + i * interval for i in range(count))
        self.height = np.random.default_rng(seed).normal(0., height_std, count)
        self.height[0] = 0.
        dt = np.diff(self.times)
        self.velocity = np.diff(self.height) / dt
        area = dt * (self.height[:-1] + self.height[1:]) / 2
        self.integral = np.concatenate(([0.], np.cumsum(area)))

    def sample(self, time):
        """Return height, velocity, and integrated height at a time."""
        index = bisect_right(self.times, time) - 1
        if index < 0:
            return 0., 0., 0.
        elapsed = time - self.times[index]
        initial_height = self.height[index]
        slope = self.velocity[index] if index < len(self.velocity) else 0.
        height = initial_height + slope * elapsed
        area = self.integral[index] + elapsed * (initial_height + height) / 2
        return height, slope, area


class IsoRoadProfile:
    """One deterministic track, sampled in metres independently of vehicle speed."""

    def __init__(self, roughness, seed, *, track=0):
        self.spacing = PROFILE_LENGTH / PROFILE_SAMPLE_COUNT
        frequencies = np.fft.rfftfreq(PROFILE_SAMPLE_COUNT, self.spacing)
        lower, upper = ISO_FREQUENCY_BAND
        active = (frequencies >= lower) & (frequencies <= upper)
        n = frequencies[active]
        density = roughness * (n / ISO_REFERENCE_FREQUENCY) ** -2
        # Keep the original FL stream and derive reproducible phases for the other tracks.
        track_seed = (int(seed), track) if track else int(seed)
        phase = np.random.default_rng(track_seed).uniform(0., 2 * np.pi, n.size)
        # A one-sided PSD bin contributes G(n) Δn to mean square height.
        spectrum = np.zeros(frequencies.size, dtype=complex)
        spectrum[active] = PROFILE_SAMPLE_COUNT * np.sqrt(density / (2 * PROFILE_LENGTH)) * np.exp(1j * phase)
        self.height = np.fft.irfft(spectrum, n=PROFILE_SAMPLE_COUNT)
        self.height -= self.height[0]
        self.slope = np.fft.irfft(2j * np.pi * frequencies * spectrum, n=PROFILE_SAMPLE_COUNT)

    def prepare_integral(self):
        next_height = np.roll(self.height, -1)
        next_slope = np.roll(self.slope, -1)
        area = self.spacing * ((self.height + next_height) / 2
                               + self.spacing * (self.slope - next_slope) / 12)
        self.integral = np.concatenate(([0.], np.cumsum(area)))

    def _segment(self, distance):
        position = (distance % PROFILE_LENGTH) / self.spacing
        u = position % 1
        index = int(position) % PROFILE_SAMPLE_COUNT
        next_index = (index + 1) % PROFILE_SAMPLE_COUNT
        height = self.height[index]
        change = self.height[next_index] - height
        left_slope = self.slope[index] * self.spacing
        right_slope = self.slope[next_index] * self.spacing
        quadratic = 3 * change - 2 * left_slope - right_slope
        cubic = -2 * change + left_slope + right_slope
        return index, u, height, left_slope, quadratic, cubic

    def at(self, distance):
        """Return height and spatial slope using the same cubic interpolant."""
        _, u, height, left_slope, quadratic, cubic = self._segment(distance)
        value = height + u * (left_slope + u * (quadratic + u * cubic))
        slope = (left_slope + u * (2 * quadratic + u * 3 * cubic)) / self.spacing
        return value, slope

    def height_and_integral(self, distance):
        """Return height and the integral from the start of the track."""
        if distance <= 0:
            return 0., 0.
        index, u, height, left_slope, quadratic, cubic = self._segment(distance)
        value = height + u * (left_slope + u * (quadratic + u * cubic))
        area = self.spacing * u * (height + u * (left_slope / 2 + u * (quadratic / 3 + u * cubic / 4)))
        return value, self.integral[index] + area + (distance // PROFILE_LENGTH) * self.integral[-1]
