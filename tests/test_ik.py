"""Tests for the analytic single-leg Two-Bone IK solver."""

from __future__ import annotations

import math
import random
import unittest
from dataclasses import FrozenInstanceError

import pygame

from spider_sim.ik import IKReachState, TwoBoneIKSolver


class TwoBoneIKSolverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.solver = TwoBoneIKSolver()

    def assert_bone_lengths(
        self,
        result: object,
        upper_length: float,
        lower_length: float,
    ) -> None:
        hip = pygame.Vector2(result.hip)  # type: ignore[attr-defined]
        knee = pygame.Vector2(result.knee)  # type: ignore[attr-defined]
        foot = pygame.Vector2(result.foot)  # type: ignore[attr-defined]
        self.assertAlmostEqual(
            hip.distance_to(knee),
            upper_length,
            places=7,
        )
        self.assertAlmostEqual(
            knee.distance_to(foot),
            lower_length,
            places=7,
        )

    def test_reachable_target_is_solved_exactly(self) -> None:
        result = self.solver.solve(
            (10.0, 20.0),
            (130.0, 40.0),
            100.0,
            80.0,
            (0.0, 1.0),
        )

        self.assertEqual(result.reach_state, IKReachState.REACHABLE)
        self.assertEqual(result.foot, result.requested_target)
        self.assertAlmostEqual(result.target_error, 0.0)
        self.assert_bone_lengths(result, 100.0, 80.0)

    def test_too_far_target_is_clamped_to_maximum_reach(self) -> None:
        result = self.solver.solve(
            (0.0, 0.0),
            (300.0, 0.0),
            100.0,
            80.0,
            (0.0, 1.0),
        )

        self.assertEqual(result.reach_state, IKReachState.TOO_FAR)
        self.assertAlmostEqual(pygame.Vector2(result.foot).length(), 180.0)
        self.assertAlmostEqual(result.target_error, 120.0)
        self.assert_bone_lengths(result, 100.0, 80.0)

    def test_too_close_target_is_clamped_to_minimum_reach(self) -> None:
        result = self.solver.solve(
            (0.0, 0.0),
            (10.0, 0.0),
            120.0,
            70.0,
            (0.0, 1.0),
        )

        self.assertEqual(result.reach_state, IKReachState.TOO_CLOSE)
        self.assertAlmostEqual(pygame.Vector2(result.foot).length(), 50.0)
        self.assertAlmostEqual(result.target_error, 40.0)
        self.assert_bone_lengths(result, 120.0, 70.0)

    def test_target_at_hip_is_handled_without_nan(self) -> None:
        result = self.solver.solve(
            (25.0, 40.0),
            (25.0, 40.0),
            100.0,
            100.0,
            (0.0, 1.0),
        )

        self.assertEqual(result.reach_state, IKReachState.DEGENERATE)
        values = (
            *result.hip,
            *result.knee,
            *result.foot,
            result.target_error,
        )
        self.assertTrue(all(math.isfinite(value) for value in values))
        self.assert_bone_lengths(result, 100.0, 100.0)

    def test_epsilon_cannot_be_used_as_a_geometric_scale(self) -> None:
        solver = TwoBoneIKSolver(epsilon=1000.0)

        with self.assertRaises(ValueError):
            solver.solve(
                (0.0, 0.0),
                (0.0, 0.0),
                1.0,
                1.0,
                (0.0, 1.0),
            )

    def test_bend_hint_selects_expected_knee_side(self) -> None:
        downward = self.solver.solve(
            (0.0, 0.0),
            (150.0, 0.0),
            100.0,
            100.0,
            (0.0, 1.0),
        )
        upward = self.solver.solve(
            (0.0, 0.0),
            (150.0, 0.0),
            100.0,
            100.0,
            (0.0, -1.0),
        )

        self.assertGreater(downward.knee[1], 0.0)
        self.assertLess(upward.knee[1], 0.0)
        self.assertNotEqual(downward.bend_branch, upward.bend_branch)

    def test_previous_knee_resolves_ambiguous_hint_continuously(self) -> None:
        first = self.solver.solve(
            (0.0, 0.0),
            (150.0, 0.0),
            100.0,
            100.0,
            (1.0, 0.0),
        )
        alternate_knee = first.knee_candidates[1]
        continued = self.solver.solve(
            (0.0, 0.0),
            (150.0, 0.0),
            100.0,
            100.0,
            (1.0, 0.0),
            previous_knee_world=alternate_knee,
        )

        self.assertEqual(continued.knee, alternate_knee)

    def test_exact_reach_boundaries_are_stable(self) -> None:
        fully_extended = self.solver.solve(
            (0.0, 0.0),
            (180.0, 0.0),
            100.0,
            80.0,
            (0.0, 1.0),
        )
        fully_folded = self.solver.solve(
            (0.0, 0.0),
            (20.0, 0.0),
            100.0,
            80.0,
            (0.0, 1.0),
        )

        self.assertEqual(
            fully_extended.reach_state,
            IKReachState.REACHABLE,
        )
        self.assertEqual(
            fully_folded.reach_state,
            IKReachState.REACHABLE,
        )
        self.assert_bone_lengths(fully_extended, 100.0, 80.0)
        self.assert_bone_lengths(fully_folded, 100.0, 80.0)

    def test_nearly_equal_lengths_are_stable_at_inner_boundary(self) -> None:
        solver = TwoBoneIKSolver(epsilon=1e-15)
        upper = 100.0
        lower = 100.0 + 1e-12
        inner_radius = abs(upper - lower)
        result = solver.solve(
            (0.0, 0.0),
            (inner_radius, 0.0),
            upper,
            lower,
            (0.0, 1.0),
        )

        self.assertEqual(result.reach_state, IKReachState.REACHABLE)
        self.assert_bone_lengths(result, upper, lower)

    def test_bend_hint_magnitude_does_not_change_direction(self) -> None:
        normal = self.solver.solve(
            (0.0, 0.0),
            (150.0, 0.0),
            100.0,
            100.0,
            (0.0, 1.0),
        )
        tiny = self.solver.solve(
            (0.0, 0.0),
            (150.0, 0.0),
            100.0,
            100.0,
            (0.0, 1e-12),
        )

        self.assertEqual(tiny.knee, normal.knee)

    def test_states_change_immediately_outside_physical_boundaries(self) -> None:
        cases = (
            (19.999999, IKReachState.TOO_CLOSE),
            (20.0, IKReachState.REACHABLE),
            (20.000001, IKReachState.REACHABLE),
            (179.999999, IKReachState.REACHABLE),
            (180.0, IKReachState.REACHABLE),
            (180.000001, IKReachState.TOO_FAR),
        )

        for distance, expected_state in cases:
            with self.subTest(distance=distance):
                result = self.solver.solve(
                    (0.0, 0.0),
                    (distance, 0.0),
                    100.0,
                    80.0,
                    (0.0, 1.0),
                )
                self.assertEqual(result.reach_state, expected_state)
                self.assert_bone_lengths(result, 100.0, 80.0)

    def test_knee_candidates_are_mirrored_about_target_axis(self) -> None:
        result = self.solver.solve(
            (0.0, 0.0),
            (150.0, 0.0),
            100.0,
            100.0,
            (0.0, 1.0),
        )
        positive = pygame.Vector2(result.knee_candidates[0])
        negative = pygame.Vector2(result.knee_candidates[1])

        self.assertAlmostEqual(positive.x, negative.x, places=9)
        self.assertAlmostEqual(positive.y, -negative.y, places=9)

    def test_previous_knee_prevents_flip_around_full_circle(self) -> None:
        solver = TwoBoneIKSolver(continuity_weight=3.0)
        previous_knee: tuple[float, float] | None = None
        maximum_jump = 0.0

        for step in range(721):
            angle = math.tau * step / 720.0
            target = (
                160.0 * math.cos(angle),
                160.0 * math.sin(angle),
            )
            result = solver.solve(
                (0.0, 0.0),
                target,
                145.0,
                125.0,
                (0.0, 1.0),
                previous_knee,
            )
            if previous_knee is not None:
                maximum_jump = max(
                    maximum_jump,
                    pygame.Vector2(result.knee).distance_to(previous_knee),
                )
            previous_knee = result.knee

        self.assertLess(maximum_jump, 2.0)

    def test_target_error_matches_requested_to_solved_distance(self) -> None:
        result = self.solver.solve(
            (20.0, 30.0),
            (900.0, -400.0),
            100.0,
            80.0,
            (0.0, 1.0),
        )

        expected = pygame.Vector2(result.requested_target).distance_to(
            result.solved_target
        )
        self.assertAlmostEqual(result.target_error, expected, places=12)

    def test_random_targets_preserve_bone_lengths_and_finite_output(self) -> None:
        generator = random.Random(20260728)

        for _ in range(500):
            upper = generator.uniform(10.0, 250.0)
            lower = generator.uniform(10.0, 250.0)
            target = (
                generator.uniform(-600.0, 600.0),
                generator.uniform(-600.0, 600.0),
            )
            result = self.solver.solve(
                (0.0, 0.0),
                target,
                upper,
                lower,
                (0.0, 1.0),
            )

            self.assert_bone_lengths(result, upper, lower)
            self.assertTrue(
                all(
                    math.isfinite(value)
                    for value in (
                        *result.knee,
                        *result.foot,
                        result.target_error,
                    )
                )
            )

    def test_invalid_inputs_are_rejected(self) -> None:
        cases = (
            ((0.0, 0.0), (10.0, 0.0), 0.0, 10.0, (0.0, 1.0)),
            ((0.0, 0.0), (10.0, 0.0), 10.0, math.inf, (0.0, 1.0)),
            ((0.0, 0.0), (math.nan, 0.0), 10.0, 10.0, (0.0, 1.0)),
            ((0.0, 0.0), (10.0, 0.0), 10.0, 10.0, (0.0, 0.0)),
        )

        for arguments in cases:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    self.solver.solve(*arguments)

        with self.assertRaises(ValueError):
            self.solver.solve(
                (0.0, 0.0),
                (100.0, 0.0),
                100.0,
                80.0,
                (0.0, 1.0),
                previous_knee_world=(math.inf, 0.0),
            )

        with self.assertRaises(ValueError):
            self.solver.solve(
                (0.0, 0.0),
                (1.0, 0.0),
                1e308,
                1e308,
                (0.0, 1.0),
            )

    def test_result_is_immutable(self) -> None:
        result = self.solver.solve(
            (0.0, 0.0),
            (100.0, 0.0),
            100.0,
            80.0,
            (0.0, 1.0),
        )

        with self.assertRaises(FrozenInstanceError):
            result.bend_branch = -1  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
