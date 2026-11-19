"""Axle-level push-rod geometry and migration of independently saved mounts."""

import copy
import json
import unittest

import numpy as np

from MultibodyGeometryLab.deformable import build_dynamics_model, sample_dampers, with_axle_pushrod_mounts
from MultibodyGeometryLab.geometry import POINTS, build_vehicle
from MultibodyGeometryLab.project import default_project, validate
from MultibodyGeometryLab.solver import CornerSolver, VehicleResult


UBJ = POINTS.index("UBJ")
ZERO = (0., 0., 0.)


class AxlePushrodTests(unittest.TestCase):
    def test_four_axle_coordinates_set_mirrored_body_mounts(self):
        source = default_project()
        source["dynamics"]["pushrod_mounts"] = {
            "front": {"y_mm": 430., "z_mm": 520.},
            "rear": {"y_mm": 410., "z_mm": 540.},
        }
        project = validate(source)
        geometry = build_vehicle(project)
        model = build_dynamics_model(geometry, project["geometry"]["vehicle"], project["dynamics"])
        references = {corner.name: corner.reference[UBJ] for corner in geometry.corners}
        expected_yz = {"FL": (.43, .52), "FR": (-.43, .52), "RL": (.41, .54), "RR": (-.41, .54)}

        for name, (y, z) in expected_yz.items():
            with self.subTest(corner=name):
                spec = model.corners[name]
                self.assertEqual(spec.attachment_index, UBJ)
                np.testing.assert_allclose(spec.fixed_point, (references[name][0], y, z), atol=1e-12, rtol=0)
        np.testing.assert_allclose(model.corners["FL"].fixed_point * [1, -1, 1],
                                   model.corners["FR"].fixed_point, atol=1e-12, rtol=0)
        np.testing.assert_allclose(model.corners["RL"].fixed_point * [1, -1, 1],
                                   model.corners["RR"].fixed_point, atol=1e-12, rtol=0)

        solvers = {corner.name: CornerSolver(corner) for corner in geometry.corners}
        static = {name: solver.solve(ZERO, ZERO) for name, solver in solvers.items()}
        moved = {**static, "FL": solvers["FL"].solve((.025, .04, 0.), ZERO)}
        rest, jounce = [sample_dampers(model, VehicleResult(poses))["FL"] for poses in (static, moved)]
        for state, poses in ((rest, static), (jounce, moved)):
            np.testing.assert_array_equal(state.fixed_point, model.corners["FL"].fixed_point)
            np.testing.assert_array_equal(state.moving_point, poses["FL"].position[UBJ])
            axis = state.fixed_point - state.moving_point
            np.testing.assert_allclose(state.force_on_chassis, state.force * axis / np.linalg.norm(axis),
                                       atol=1e-9, rtol=0)
        self.assertGreater(abs(rest.inclination_deg - jounce.inclination_deg), .5)
        self.assertGreater(abs(jounce.moving_point[0] - jounce.fixed_point[0]), 1e-5)

    def test_legacy_left_mounts_migrate_to_four_persisted_axle_values(self):
        source = default_project()
        source["dynamics"].pop("pushrod_mounts", None)
        geometry = build_vehicle(source)
        references = {corner.name: np.asarray(corner.reference[UBJ]) for corner in geometry.corners}
        starting = source["dynamics"]
        defaults = with_axle_pushrod_mounts(geometry, starting)
        for axle, left in (("front", "FL"), ("rear", "RL")):
            mount = defaults["pushrod_mounts"][axle]
            self.assertAlmostEqual(mount["y_mm"], references[left][1] * 1000)
            self.assertAlmostEqual(mount["z_mm"], references[left][2] * 1000 + 250)

        starting["corners"]["FL"].update(mount_reference="body_coordinates", mount_mm=[70., 430., 520.])
        starting["corners"]["FR"].update(mount_reference="body_coordinates", mount_mm=[110., -470., 580.],
                                         spring_n_m=34500.)
        starting["corners"]["RL"].update(mount_reference="attachment", mount_mm=[55., -80., 245.])
        original = copy.deepcopy(starting)
        converted = with_axle_pushrod_mounts(geometry, starting)
        self.assertEqual(starting, original)
        self.assertEqual(converted["pushrod_mounts"]["front"], {"y_mm": 430., "z_mm": 520.})
        rear = converted["pushrod_mounts"]["rear"]
        self.assertAlmostEqual(rear["y_mm"], references["RL"][1] * 1000 - 80.)
        self.assertAlmostEqual(rear["z_mm"], references["RL"][2] * 1000 + 245.)
        self.assertEqual(converted["corners"]["FR"]["spring_n_m"], 34500.)
        for corner in converted["corners"].values():
            self.assertEqual(set(corner), {"mass_kg", "spring_n_m", "damper_ns_m"})

        wire_data = json.loads(json.dumps({**source, "dynamics": converted}))
        loaded = validate(wire_data)
        self.assertEqual(loaded["dynamics"], converted)
        self.assertEqual(with_axle_pushrod_mounts(geometry, loaded["dynamics"]), converted)
        model = build_dynamics_model(geometry, loaded["geometry"]["vehicle"], loaded["dynamics"])
        np.testing.assert_allclose(model.corners["FL"].fixed_point, (references["FL"][0], .43, .52))
        np.testing.assert_allclose(model.corners["FR"].fixed_point, (references["FR"][0], -.43, .52))


if __name__ == "__main__":
    unittest.main()
