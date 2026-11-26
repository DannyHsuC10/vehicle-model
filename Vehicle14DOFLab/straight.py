"""Open-course full-throttle acceleration using the same 14 DOF equations."""

import numpy as np

from .lap import DRIVER_PERIOD, LapIncomplete, advance_state
from .model import Controls, flat_road


def simulate_straight(vehicle, length, *, initial_speed=0., step=.005, time_limit=240., on_position=None):
    if not np.isfinite(length) or length <= 0.:
        raise ValueError("Straight length must be finite and positive")
    if not np.isfinite([step, time_limit]).all() or not .0001 <= step <= DRIVER_PERIOD or time_limit <= 0.:
        raise ValueError("Integration step must be 0.0001–0.02 s and the time limit must be positive")
    controls = Controls(throttle=1.)
    state = vehicle.initial_state(speed=initial_speed)
    substeps = int(np.ceil(DRIVER_PERIOD / step))
    time, records = 0., []

    def sample():
        records.append({
            "time_s": time, "distance_m": state[0], "centerline_error_m": state[1],
            "throttle": controls.throttle, "brake": controls.brake, "steer_command_deg": 0.,
            **vehicle.evaluate(state, controls, flat_road(time), telemetry=True),
        })

    while time < time_limit:
        sample()
        if on_position is not None:
            on_position(1, time, state[0])
        end_time = min(time + DRIVER_PERIOD, time_limit)
        next_state = advance_state(vehicle, state, controls, flat_road, time, end_time, substeps)
        if not np.isfinite(next_state).all():
            raise LapIncomplete(f"Non-finite vehicle state at {end_time:.3f} s. Reduce the integration step.")
        if next_state[0] >= length:
            fraction = (length - state[0]) / (next_state[0] - state[0])
            state = state + fraction * (next_state - state)
            state[0] = length
            time += fraction * (end_time - time)
            sample()
            return records
        time, state = end_time, next_state
    raise LapIncomplete(f"No finish before {time_limit:g} s of simulation, x = {state[0]:.1f} m")
