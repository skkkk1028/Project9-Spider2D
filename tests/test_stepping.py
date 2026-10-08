"""Acceptance tests for Milestone 5 foot locking and single-leg stepping."""

from __future__ import annotations

import math
import random
import unittest

import pygame

from spider_sim.config import AppConfig, FootStepConfig
from spider_sim.spider import BodySnapshot
from spider_sim.stepping import (
    FootState,
    SingleLegStepAssembly,
    SteppingAssemblySnapshot,
)


def _body(
    position: tuple[float, float],
    heading: float = 0.0,
) -> BodySnapshot:
    return BodySnapshot(position, (0.0, 0.0), heading)


def _leg(snapshot: SteppingAssemblySnapshot, leg_id: str):
    return next(leg for leg in snapshot.legs if leg.mount.leg_id == leg_id)


def _active_count(snapshot: SteppingAssemblySnapshot) -> int:
    return sum(leg.state is not FootState.PLANTED for leg in snapshot.legs)


def _all_ik_numbers(snapshot: SteppingAssemblySnapshot) -> list[float]:
    numbers: list[float] = []
    for leg in snapshot.legs:
        result = leg.ik
        for point in (
            result.hip,
            result.knee,
            result.foot,
            result.requested_target,
            result.solved_target,
            *result.knee_candidates,
        ):
            numbers.extend(point)
        numbers.extend(
            (
                result.target_error,
                result.upper_length,
                result.lower_length,
                result.minimum_reach,
                result.maximum_reach,
                leg.foot_height,
                leg.distance_to_default,
                leg.extension_ratio,
            )
        )
    return numbers


class FootLockTests(unittest.TestCase):
    def test_planted_feet_keep_exact_world_coordinates(self) -> None:
        config = AppConfig()
        step_config = FootStepConfig(
            trigger_distance=1000.0,
            max_extension_ratio=1.0,
        )
        initial_body = _body((600.0, 400.0))
        assembly = SingleLegStepAssembly(
            config.body,
            config.leg,
            step_config,
            initial_body,
        )
        initial = assembly.solve(initial_body)
        initial_feet = {
            leg.mount.leg_id: leg.foot_world
            for leg in initial.legs
        }

        moved_body = _body((610.0, 404.0), math.radians(4.0))
        moved = assembly.update(moved_body, 1.0 / 60.0)

        self.assertIsNone(moved.active_leg_id)
        self.assertEqual(_active_count(moved), 0)
        for leg in moved.legs:
            with self.subTest(leg=leg.mount.leg_id):
                self.assertIs(leg.state, FootState.PLANTED)
                self.assertEqual(
                    leg.foot_world,
                    initial_feet[leg.mount.leg_id],
                )
                self.assertEqual(
                    leg.ik.requested_target,
                    initial_feet[leg.mount.leg_id],
                )

    def test_extension_ratio_can_trigger_without_distance_threshold(self) -> None:
        config = AppConfig()
        assembly = SingleLegStepAssembly(
            config.body,
            config.leg,
            FootStepConfig(
                trigger_distance=1000.0,
                max_extension_ratio=0.70,
            ),
            _body((600.0, 400.0)),
        )

        snapshot = assembly.update(
            _body((635.0, 400.0)),
            1.0 / 120.0,
        )

        self.assertIsNotNone(snapshot.active_leg_id)
        self.assertEqual(_active_count(snapshot), 1)


class FootStateMachineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = AppConfig()
        self.step_config = FootStepConfig(
            trigger_distance=5.0,
            max_extension_ratio=1.0,
            lift_duration=0.125,
            swing_duration=0.25,
            lower_duration=0.125,
            lift_height=20.0,
        )
        self.initial_body = _body((600.0, 400.0))
        self.moved_body = _body((612.0, 400.0))
        self.assembly = SingleLegStepAssembly(
            self.config.body,
            self.config.leg,
            self.step_config,
            self.initial_body,
        )
        self.initial = self.assembly.solve(self.initial_body)

    def test_lift_swing_lower_cycle_is_continuous_and_complete(self) -> None:
        initial_foot = _leg(self.initial, "L0").foot_world

        lift_mid = self.assembly.update(self.moved_body, 0.03125)
        active_id = lift_mid.active_leg_id
        self.assertIsNotNone(active_id)
        assert active_id is not None
        active_lift = _leg(lift_mid, active_id)
        self.assertIs(active_lift.state, FootState.LIFT)
        self.assertEqual(active_lift.foot_world, initial_foot)
        self.assertGreater(active_lift.foot_height, 0.0)
        self.assertLess(active_lift.foot_height, self.step_config.lift_height)

        swing_start = self.assembly.update(self.moved_body, 0.09375)
        active_swing_start = _leg(swing_start, active_id)
        self.assertIs(active_swing_start.state, FootState.SWING)
        self.assertEqual(active_swing_start.foot_world, initial_foot)
        self.assertEqual(
            active_swing_start.foot_height,
            self.step_config.lift_height,
        )

        swing_mid = self.assembly.update(self.moved_body, 0.125)
        active_swing_mid = _leg(swing_mid, active_id)
        self.assertIs(active_swing_mid.state, FootState.SWING)
        self.assertNotEqual(active_swing_mid.foot_world, initial_foot)
        self.assertNotEqual(
            active_swing_mid.foot_world,
            active_swing_mid.step_target_world,
        )

        lower_start = self.assembly.update(self.moved_body, 0.125)
        active_lower_start = _leg(lower_start, active_id)
        self.assertIs(active_lower_start.state, FootState.LOWER)
        self.assertEqual(
            active_lower_start.foot_world,
            active_lower_start.step_target_world,
        )
        self.assertEqual(
            active_lower_start.foot_height,
            self.step_config.lift_height,
        )

        lower_mid = self.assembly.update(self.moved_body, 0.0625)
        active_lower_mid = _leg(lower_mid, active_id)
        self.assertIs(active_lower_mid.state, FootState.LOWER)
        self.assertEqual(
            active_lower_mid.foot_world,
            active_lower_start.foot_world,
        )
        self.assertGreater(active_lower_mid.foot_height, 0.0)
        self.assertLess(
            active_lower_mid.foot_height,
            self.step_config.lift_height,
        )

        planted = self.assembly.update(self.moved_body, 0.0625)
        completed = _leg(planted, active_id)
        self.assertIs(completed.state, FootState.PLANTED)
        self.assertEqual(completed.foot_height, 0.0)
        self.assertEqual(completed.step_count, 1)
        self.assertIsNone(planted.active_leg_id)
        self.assertEqual(_active_count(planted), 0)
        self.assertEqual(
            completed.foot_world,
            completed.default_target_world,
        )

        for snapshot in (
            lift_mid,
            swing_start,
            swing_mid,
            lower_start,
            lower_mid,
            planted,
        ):
            self.assertLessEqual(_active_count(snapshot), 1)

    def test_only_active_leg_changes_world_xy_during_step(self) -> None:
        before = {
            leg.mount.leg_id: leg.foot_world
            for leg in self.initial.legs
        }

        snapshot = self.assembly.update(self.moved_body, 0.25)

        self.assertEqual(_active_count(snapshot), 1)
        for leg in snapshot.legs:
            if leg.mount.leg_id == snapshot.active_leg_id:
                continue
            with self.subTest(leg=leg.mount.leg_id):
                self.assertIs(leg.state, FootState.PLANTED)
                self.assertEqual(
                    leg.foot_world,
                    before[leg.mount.leg_id],
                )


class FrameRateConsistencyTests(unittest.TestCase):
    def _completed_step_length(self, frames_per_second: int) -> float:
        config = AppConfig()
        step_config = FootStepConfig(
            trigger_distance=5.0,
            max_extension_ratio=1.0,
            lift_duration=0.12,
            swing_duration=0.24,
            lower_duration=0.12,
        )
        initial_body = _body((600.0, 400.0))
        moved_body = _body((620.0, 400.0))
        assembly = SingleLegStepAssembly(
            config.body,
            config.leg,
            step_config,
            initial_body,
        )
        initial = assembly.solve(initial_body)
        initial_foot = _leg(initial, "L0").foot_world

        for _ in range(frames_per_second * 2):
            snapshot = assembly.update(
                moved_body,
                1.0 / frames_per_second,
            )
            completed = _leg(snapshot, "L0")
            if completed.step_count == 1:
                return pygame.Vector2(initial_foot).distance_to(
                    completed.foot_world
                )

        self.fail("L0 did not complete a step")

    def test_step_length_is_consistent_across_frame_rates(self) -> None:
        lengths = {
            fps: self._completed_step_length(fps)
            for fps in (30, 60, 120, 240)
        }

        for fps, length in lengths.items():
            with self.subTest(fps=fps):
                self.assertAlmostEqual(length, 20.0, places=9)
        self.assertLess(max(lengths.values()) - min(lengths.values()), 1e-9)


class RandomMotionRobustnessTests(unittest.TestCase):
    def test_one_thousand_random_body_moves_remain_finite(self) -> None:
        config = AppConfig()
        randomizer = random.Random(20260728)
        position = pygame.Vector2(600.0, 400.0)
        heading = 0.0
        initial_body = _body((position.x, position.y), heading)
        assembly = SingleLegStepAssembly(
            config.body,
            config.leg,
            config.foot_step,
            initial_body,
        )
        previous = assembly.solve(initial_body)
        completed_steps = 0

        for _ in range(1000):
            position += pygame.Vector2(
                randomizer.uniform(-2.0, 2.0),
                randomizer.uniform(-2.0, 2.0),
            )
            heading += randomizer.uniform(-0.025, 0.025)
            current_body = _body(
                (position.x, position.y),
                heading,
            )
            snapshot = assembly.update(current_body, 1.0 / 120.0)

            self.assertLessEqual(_active_count(snapshot), 1)
            self.assertTrue(all(math.isfinite(value) for value in _all_ik_numbers(snapshot)))
            for old_leg, new_leg in zip(
                previous.legs,
                snapshot.legs,
                strict=True,
            ):
                if (
                    old_leg.state is FootState.PLANTED
                    and new_leg.state is FootState.PLANTED
                    and new_leg.step_count == old_leg.step_count
                ):
                    self.assertEqual(old_leg.foot_world, new_leg.foot_world)
            previous = snapshot
            completed_steps = sum(leg.step_count for leg in snapshot.legs)

        self.assertGreater(completed_steps, 0)

    def test_invalid_delta_is_rejected(self) -> None:
        config = AppConfig()
        body = _body((600.0, 400.0))
        assembly = SingleLegStepAssembly(
            config.body,
            config.leg,
            config.foot_step,
            body,
        )

        for invalid_delta in (0.0, -0.1, math.nan, True):
            with self.subTest(invalid_delta=invalid_delta):
                with self.assertRaises(ValueError):
                    assembly.update(body, invalid_delta)


if __name__ == "__main__":
    unittest.main()
