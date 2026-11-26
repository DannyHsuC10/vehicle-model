"""Load the split LP03 STL assembly for the canvas replay."""

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import struct

import numpy as np

from ._vendor.suspension.project import CORNERS


STL_DIRECTORY = Path(__file__).with_name("car_stl")
STL_FACE = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])


def triangles(path):
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0] if len(raw) >= 84 else 0
    if count and len(raw) == 84 + STL_FACE.itemsize * count:
        data = np.frombuffer(raw, dtype=STL_FACE, count=count, offset=84)["vertices"]
    else:
        try:
            vertices = [line.split()[1:] for line in raw.decode("ascii").splitlines()
                        if line.lstrip().startswith("vertex ")]
            data = np.asarray(vertices, dtype=float).reshape((-1, 3, 3))
        except (ValueError, UnicodeError) as exc:
            raise ValueError(f"無法讀取 STL：{path.name}") from exc
    if not len(data) or not np.isfinite(data).all():
        raise ValueError(f"STL 沒有有效的三角面：{path.name}")
    return np.asarray(data, dtype=float) / 1000.


@dataclass(frozen=True)
class Mesh:
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray

    @classmethod
    def build(cls, triangles_m, *, cell=0.):
        vertices, indices = np.unique(triangles_m.reshape(-1, 3), axis=0, return_inverse=True)
        faces = indices.reshape(-1, 3)
        if cell:
            # Merge nearby vertices only in the replay mesh. Keep CAD files unchanged.
            bins = np.floor((vertices - vertices.min(axis=0)) / cell).astype(int)
            _, groups, counts = np.unique(bins, axis=0, return_inverse=True, return_counts=True)
            groups = groups.ravel()
            vertices = np.column_stack([np.bincount(groups, weights=vertices[:, axis]) / counts for axis in range(3)])
            faces = groups[faces]
            distinct = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 2] != faces[:, 0]))
            faces = faces[distinct]
            # Keep each orientation for thin parts that collapse onto one plane.
            start = np.argmin(faces, axis=1)[:, None]
            rotated = np.take_along_axis(faces, (start + np.arange(3)) % 3, axis=1)
            _, first = np.unique(rotated, axis=0, return_index=True)
            faces = faces[np.sort(first)]
        p = vertices[faces]
        normals = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
        length = np.linalg.norm(normals, axis=1)
        valid = length > 1e-12
        if not valid.any():
            if cell:
                return cls.build(triangles_m)
            raise ValueError("STL 沒有可繪製的三角面。")
        return cls(vertices, faces[valid], normals[valid] / length[valid, None])


@dataclass(frozen=True)
class Assembly:
    body: Mesh
    wheels: tuple[Mesh, ...]
    hubs: np.ndarray
    ground: float

    def reference(self, axles, cg_height):
        front, rear = self.hubs[:2, 0].mean(), self.hubs[2:, 0].mean()
        lf, lr = axles[0, 0], -axles[2, 0]
        return np.array([front - (front - rear) * lf / (lf + lr), self.hubs[:, 1].mean(), self.ground + cg_height])


@lru_cache(maxsize=1)
def read_assembly(files):
    body, *wheels = map(triangles, files)
    mins = np.array([wheel.min(axis=(0, 1)) for wheel in wheels])
    maxes = np.array([wheel.max(axis=(0, 1)) for wheel in wheels])
    hubs = (mins + maxes) / 2.
    if np.any(maxes <= mins) or not (hubs[:2, 0].mean() > hubs[2:, 0].mean()
                                    and np.all(hubs[::2, 1] > hubs[1::2, 1])):
        raise ValueError("STL 輪心座標與車輛座標方向不符。")
    body_mesh = Mesh.build(body, cell=.060)
    wheel_meshes = tuple(Mesh.build(wheel - hub, cell=.030) for wheel, hub in zip(wheels, hubs))
    return Assembly(body_mesh, wheel_meshes, hubs, float(mins[:, 2].min()))


def load_assembly(directory=STL_DIRECTORY):
    if not directory.is_dir():
        return None
    entries = tuple(path for path in directory.iterdir() if path.is_file())
    suffixes = ("body.stl", *(f"wheel_{corner.lower()}.stl" for corner in CORNERS))
    candidates = [tuple(path for path in entries if path.name.lower().endswith(suffix)) for suffix in suffixes]
    if not all(len(group) == 1 for group in candidates):
        raise ValueError("car_stl 需各有一個 body.stl 與 wheel_FL／FR／RL／RR.stl 分件。")
    return read_assembly(tuple(group[0] for group in candidates))
