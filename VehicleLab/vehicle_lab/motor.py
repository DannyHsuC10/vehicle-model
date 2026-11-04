"""CSV-backed four independent wheel motors with symmetric axle reductions.

The table contains ONE motor's speed (RPM) and torque (N m). Relative paths
are resolved against the application root, independent of the shell CWD.
"""
from bisect import bisect_right
import csv
from dataclasses import dataclass
import hashlib
import io
import math
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parents[1]


def resolve_motor_path(path):
    candidate = Path(path).expanduser()
    return (candidate if candidate.is_absolute() else APP_ROOT / candidate).resolve()


@dataclass(frozen=True)
class MotorCurve:
    rpm: tuple
    torque: tuple
    source: str
    sha256: str

    @property
    def max_rpm(self):
        return self.rpm[-1]

    @property
    def points(self):
        return tuple(zip(self.rpm, self.torque))

    def torque_at(self, rpm):
        """Linear interpolation; low-speed endpoint hold, no overspeed torque."""
        if not math.isfinite(rpm) or rpm < 0:
            raise ValueError("Motor speed must be finite and >= 0 RPM")
        if rpm > self.max_rpm:
            return 0.0
        if rpm <= self.rpm[0]:
            return self.torque[0]
        index = bisect_right(self.rpm, rpm) - 1
        if index == len(self.rpm) - 1:
            return self.torque[-1]
        lo, hi = self.rpm[index], self.rpm[index + 1]
        return self.torque[index] + (self.torque[index + 1] - self.torque[index]) * (rpm - lo) / (hi - lo)


def read_motor_curve(path):
    """Read and freeze a validated speed,torque CSV; never use a fallback map."""
    source = resolve_motor_path(path)
    try:
        raw = source.read_bytes()
    except OSError as exc:
        raise ValueError(f"Cannot read motor torque CSV: {source} ({exc.strerror or exc})") from exc
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ValueError(f"Motor torque CSV must be UTF-8: {source}") from exc
    reader = csv.DictReader(io.StringIO(text))
    if not reader.fieldnames:
        raise ValueError(f"Motor torque CSV has no header: {source}")
    headers = [name.strip().lower() for name in reader.fieldnames]
    if len(headers) != len(set(headers)) or not {"speed", "torque"}.issubset(headers):
        raise ValueError(f"Motor torque CSV requires unique speed,torque columns: {source}")
    reader.fieldnames = headers
    points = []
    for line, row in enumerate(reader, start=2):
        if all(value is None or not str(value).strip() for value in row.values()):
            continue
        try:
            rpm, torque = float(row["speed"]), float(row["torque"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Motor torque CSV line {line}: speed and torque must be numbers") from exc
        if not math.isfinite(rpm) or not math.isfinite(torque) or rpm < 0:
            raise ValueError(f"Motor torque CSV line {line}: RPM must be nonnegative and RPM/N m must be finite")
        points.append((rpm, torque))
    if len(points) < 2:
        raise ValueError("Motor torque CSV requires at least two data rows")
    points.sort()
    if any(a[0] == b[0] for a, b in zip(points, points[1:])):
        raise ValueError("Motor torque CSV speed values must be unique")
    if points[-1][0] <= 0:
        raise ValueError("Motor torque CSV maximum speed must be > 0 RPM")
    return MotorCurve(tuple(p[0] for p in points), tuple(p[1] for p in points), str(source), hashlib.sha256(raw).hexdigest())


def max_vehicle_speed(v, curve):
    """The lower of front/rear motor-RPM ceilings; all motors stay connected."""
    return min(curve.max_rpm * 2 * math.pi * getattr(v, "tire_diameter_" + axle) / 2 /
               (60 * getattr(v, "reduction_" + axle)) for axle in ("front", "rear"))


def motor_forces(v, speed, torque_scale=1.0, curve=None):
    """Motor -> fixed reducer -> per-wheel force, then two motors per axle.

    These are drivetrain capabilities before individual tire limits. All four
    motors use the same CSV; the front and rear ratios/efficiencies can differ.
    """
    if not math.isfinite(speed) or speed < 0:
        raise ValueError("Vehicle speed must be finite and >= 0 m/s")
    if not math.isfinite(torque_scale) or torque_scale < 0:
        raise ValueError("Motor torque scale must be finite and >= 0")
    curve = read_motor_curve(v.motor_curve_csv) if curve is None else curve
    result = {}
    for axle in ("front", "rear"):
        radius = getattr(v, "tire_diameter_" + axle) / 2
        ratio = getattr(v, "reduction_" + axle)
        efficiency = getattr(v, "reduction_efficiency_" + axle)
        if not all(math.isfinite(x) and x > 0 for x in (radius, ratio, efficiency)) or efficiency > 1:
            raise ValueError("Wheel radius and reduction must be positive; reduction efficiency must be 0..1")
        wheel_rpm = speed / radius * 60 / (2 * math.pi)
        motor_rpm = wheel_rpm * ratio
        # Preserve signed CSV samples and interpolate FIRST. This is maximum
        # propulsion, so negative interpolated torque supplies no drive force;
        # regenerative braking is outside this model.
        lookup_torque = curve.torque_at(motor_rpm)
        motor_torque = max(0.0, lookup_torque) * torque_scale
        wheel_torque = motor_torque * ratio * efficiency
        per_wheel_force = wheel_torque / radius
        result.update({axle + "_wheel_rpm": wheel_rpm, axle + "_rpm": motor_rpm,
            axle + "_lookup_torque_Nm": lookup_torque,
            axle + "_motor_torque_Nm": motor_torque, axle + "_wheel_torque_Nm": wheel_torque,
            axle + "_per_wheel_force_N": per_wheel_force, axle + "_force_N": 2 * per_wheel_force})
    result["total_force_N"] = result["front_force_N"] + result["rear_force_N"]
    return result


def drive_force_limit(motor_limits, tire_limits):
    """Sum each independent motor/tire minimum; never couple wheel capacities."""
    if len(motor_limits) != 4 or len(tire_limits) != 4:
        raise ValueError("Four motor and four tire force limits are required")
    if not all(math.isfinite(x) and x >= 0 for x in (*motor_limits, *tire_limits)):
        raise ValueError("Motor and tire force limits must be finite and nonnegative")
    return sum(min(motor, tire) for motor, tire in zip(motor_limits, tire_limits))
