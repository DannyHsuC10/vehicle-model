import copy
from dataclasses import asdict
import unittest

import numpy as np

from MultibodyGeometryLab.deformable import build_dynamics_model
from MultibodyGeometryLab.elements import DAMPER_POINT, Member, PUSH_ROD, default_elements, validate_elements
from MultibodyGeometryLab.geometry import build_vehicle
from MultibodyGeometryLab.project import default_project, validate
from MultibodyGeometryLab.rigid_body import simulate_rigid_road
from MultibodyGeometryLab.solver import CornerSolver


class EditableElementTests(unittest.TestCase):
    def setUp(self):
        self.project = default_project()
        self.project["dynamics"]["simulation"].update(
            target="FL", bump_mm=15., bump_seconds=.20, duration_seconds=.50)

    @staticmethod
    def model(project):
        source = validate(project)
        return build_dynamics_model(build_vehicle(source), source["geometry"]["vehicle"], source["dynamics"])

    @classmethod
    def run_road(cls, project):
        return simulate_rigid_road(cls.model(project), project["dynamics"], project["geometry"]["vehicle"]["mass"])

    def test_added_spring_changes_road_response_without_adding_a_rigid_constraint(self):
        edited = copy.deepcopy(self.project)
        edited["dynamics"]["elements"] = default_elements()
        edited["dynamics"]["elements"]["FL"].append(asdict(Member(
            "輔助彈簧", DAMPER_POINT, "WC", "spring_damper", 30000., 500.)))
        model = self.model(edited)
        corner = next(corner for corner in model.geometry.corners if corner.name == "FL")
        solver = CornerSolver(corner)
        self.assertFalse(solver.locked)
        self.assertNotIn((DAMPER_POINT, "WC"), solver.rigid_links)
        before = self.run_road(self.project)
        after = self.run_road(edited)
        difference = np.array([a.translation - b.translation for a, b in zip(before, after)])
        self.assertGreater(np.max(np.abs(difference)), 5e-5)
        self.assertGreater(np.ptp([frame.element_springs["FL"]["輔助彈簧"].force for frame in after]), 10.)
        for frame in after:
            self.assertLess(max(abs(frame.result.corners["FL"].length_error)), 1e-7)

    def test_all_rigid_keeps_suspension_fixed_while_tires_and_chassis_respond(self):
        elements = default_elements()
        for records in elements.values():
            for member in records:
                member["kind"] = "rigid"
        self.project["dynamics"]["elements"] = elements
        frames = self.run_road(self.project)
        self.assertGreater(max(frame.translation[2] for frame in frames), .001)
        self.assertGreater(max(frame.dampers["FL"].tire_force for frame in frames)
                           - frames[0].dampers["FL"].tire_force, 100.)
        for frame in frames:
            for name, state in frame.result.corners.items():
                np.testing.assert_allclose(state.position, frames[0].result.corners[name].position, atol=1e-10)
                self.assertAlmostEqual(frame.dampers[name].deflection, 0.)
                self.assertEqual(frame.dampers[name].force, 0.)

    def test_role_change_round_trips_and_incomplete_constraints_are_reported(self):
        elements = default_elements()
        tie_rod = next(member for member in elements["FL"] if {member["a"], member["b"]} == {"TOE", "RACK"})
        tie_rod.update(kind="spring_damper", damping_ns_m=750.)
        self.project["dynamics"]["elements"] = elements
        self.assertEqual(validate(self.project)["dynamics"]["elements"], validate_elements(elements))
        with self.assertRaisesRegex(ValueError, "FL"):
            self.model(self.project)
        elements["FL"].append(asdict(Member("替代剛性桿", "RACK", "WC", "rigid")))
        model = self.model(self.project)
        front_left = next(corner for corner in model.geometry.corners if corner.name == "FL")
        solver = CornerSolver(front_left)
        self.assertNotIn(("TOE", "RACK"), solver.rigid_links)
        self.assertIn(("RACK", "WC"), solver.rigid_links)
        pose = solver.solve((.010, 0., 0.), (0., 0., 0.))
        self.assertLess(max(abs(pose.length_error)), 1e-7)


if __name__ == "__main__":
    unittest.main()
