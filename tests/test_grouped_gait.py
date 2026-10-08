"""Acceptance tests for Milestone 6 fixed four-phase grouped gait."""

from __future__ import annotations

import math
import unittest
from dataclasses import replace

import pygame

from spider_sim.config import AppConfig, FourPhaseGaitConfig
from spider_sim.leg import LegSide
from spider_sim.simulation import Simulation, split_delta_time
from spider_sim.spider import BodySnapshot
from spider_sim.stepping import (
    FootState,
    FourPhaseGaitAssembly,
    GroupedGaitSnapshot,
)


def _body(
    position: tuple[float, float],
    heading: float = 0.0,
) -> BodySnapshot:
    return BodySnapshot(position, (0.0, 0.0), heading)


def _active_legs(snapshot: GroupedGaitSnapshot):
    return [
        leg
        for leg in snapshot.legs
        if leg.state is not FootState.PLANTED
    ]


def _assert_support_constraints(
    test_case: unittest.TestCase,
    snapshot: GroupedGaitSnapshot,
) -> None:
    active = _active_legs(snapshot)
    test_case.assertLessEqual(len(active), 2)
    test_case.assertGreaterEqual(snapshot.support_count, 6)
    test_case.assertLessEqual(
        sum(leg.mount.side is LegSide.LEFT for leg in active),
        1,
    )
    test_case.assertLessEqual(
        sum(leg.mount.side is LegSide.RIGHT for leg in active),
        1,
    )
    test_case.assertEqual(
        {leg.mount.leg_id for leg in active},
        set(snapshot.active_leg_ids),
    )


def _assert_finite_ik(
    test_case: unittest.TestCase,
    snapshot: GroupedGaitSnapshot,
) -> None:
    for leg in snapshot.legs:
        result = leg.ik
        values = (
            *result.hip,
            *result.knee,
            *result.foot,
            *result.requested_target,
            *result.solved_target,
            *result.knee_candidates[0],
            *result.knee_candidates[1],
            result.target_error,
            leg.foot_height,
            leg.distance_to_default,
            leg.extension_ratio,
        )
        test_case.assertTrue(all(math.isfinite(value) for value in values))


class GroupDefinitionTests(unittest.TestCase):
    def test_groups_are_opposite_side_non_adjacent_pairs(self) -> None:
        config = AppConfig()
        assembly = FourPhaseGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            _body((600.0, 400.0)),
        )
        mounts = {mount.leg_id: mount for mount in assembly.mounts}

        self.assertEqual(
            assembly.groups,
            (
                ("L0", "R2"),
                ("R0", "L2"),
                ("L1", "R3"),
                ("R1", "L3"),
            ),
        )
        for group in assembly.groups:
            first, second = (mounts[leg_id] for leg_id in group)
            with self.subTest(group=group):
                self.assertIsNot(first.side, second.side)
                self.assertGreaterEqual(abs(first.row - second.row), 2)

    def test_invalid_anatomical_groups_fail_during_assembly(self) -> None:
        config = AppConfig()
        invalid_group_sets = (
            (
                ("L0", "L2"),
                ("R0", "R2"),
                ("L1", "R3"),
                ("R1", "L3"),
            ),
            (
                ("L0", "R1"),
                ("R0", "L1"),
                ("L2", "R3"),
                ("R2", "L3"),
            ),
        )
        for groups in invalid_group_sets:
            with self.subTest(groups=groups):
                with self.assertRaises(ValueError):
                    FourPhaseGaitAssembly(
                        config.body,
                        config.leg,
                        config.foot_step,
                        FourPhaseGaitConfig(groups=groups),
                        _body((600.0, 400.0)),
                    )


class SchedulerConstraintTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = AppConfig()
        self.initial_body = _body((600.0, 400.0))
        self.assembly = FourPhaseGaitAssembly(
            self.config.body,
            self.config.leg,
            self.config.foot_step,
            self.config.four_phase_gait,
            self.initial_body,
        )

    def _collect_group_starts(
        self,
        body: BodySnapshot,
        count: int,
    ) -> list[int]:
        starts: list[int] = []
        previous_active = self.assembly.active_group_index

        for _ in range(5000):
            snapshot = self.assembly.update(body, 1.0 / 120.0)
            _assert_support_constraints(self, snapshot)
            _assert_finite_ik(self, snapshot)
            if (
                snapshot.active_group_index is not None
                and snapshot.active_group_index != previous_active
            ):
                starts.append(snapshot.active_group_index)
                if len(starts) == count:
                    return starts
            previous_active = snapshot.active_group_index

        self.fail(f"only observed group starts {starts}")

    def test_all_eligible_groups_receive_fair_round_robin_turns(self) -> None:
        first_cycle = self._collect_group_starts(
            _body((635.0, 400.0)),
            4,
        )
        second_cycle = self._collect_group_starts(
            _body((670.0, 400.0)),
            4,
        )

        self.assertEqual(first_cycle, [0, 1, 2, 3])
        self.assertEqual(second_cycle, [0, 1, 2, 3])
        snapshot = self.assembly.solve(_body((670.0, 400.0)))
        self.assertEqual(snapshot.group_start_counts, (2, 2, 2, 2))

    def test_stationary_body_does_not_step_in_place(self) -> None:
        for _ in range(2000):
            snapshot = self.assembly.update(
                self.initial_body,
                1.0 / 120.0,
            )
            self.assertIsNone(snapshot.active_group_index)
            self.assertEqual(snapshot.support_count, 8)

        self.assertEqual(snapshot.group_start_counts, (0, 0, 0, 0))
        self.assertEqual(sum(leg.step_count for leg in snapshot.legs), 0)

    def test_settled_body_stops_after_finishing_pending_rotation(self) -> None:
        settled_body = _body((635.0, 400.0))
        for _ in range(3000):
            snapshot = self.assembly.update(
                settled_body,
                1.0 / 120.0,
            )
            if (
                snapshot.active_group_index is None
                and snapshot.group_start_counts == (1, 1, 1, 1)
            ):
                break
        else:
            self.fail("group rotation did not settle")

        settled_counts = snapshot.group_start_counts
        settled_steps = sum(leg.step_count for leg in snapshot.legs)
        for _ in range(2000):
            snapshot = self.assembly.update(
                settled_body,
                1.0 / 120.0,
            )
            self.assertIsNone(snapshot.active_group_index)

        self.assertEqual(snapshot.group_start_counts, settled_counts)
        self.assertEqual(
            sum(leg.step_count for leg in snapshot.legs),
            settled_steps,
        )

    def test_low_speed_translation_and_turn_never_activate_all_legs(self) -> None:
        position = pygame.Vector2(600.0, 400.0)
        heading = 0.0
        saw_active_group = False

        for _ in range(2400):
            position.x += 6.0 / 120.0
            heading += math.radians(8.0) / 120.0
            snapshot = self.assembly.update(
                _body((position.x, position.y), heading),
                1.0 / 120.0,
            )
            _assert_support_constraints(self, snapshot)
            _assert_finite_ik(self, snapshot)
            saw_active_group |= snapshot.active_group_index is not None

        self.assertTrue(saw_active_group)


class MousePathRobustnessTests(unittest.TestCase):
    def _run_path(
        self,
        target_at_time,
        duration: float,
    ) -> tuple[set[int], int]:
        config = AppConfig()
        body_config = replace(
            config.body,
            max_speed=config.four_phase_gait.demo_max_speed,
            max_acceleration=config.four_phase_gait.demo_max_acceleration,
            heading_speed_threshold=2.0,
        )
        simulation = Simulation(
            body_config,
            initial_position=(600.0, 400.0),
        )
        assembly = FourPhaseGaitAssembly(
            body_config,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            simulation.snapshot().body,
        )
        seen_groups: set[int] = set()
        same_group_active_seconds = 0.0
        previous_group: int | None = None
        maximum_active_seconds = (
            config.foot_step.lift_duration
            + config.foot_step.swing_duration
            + config.foot_step.lower_duration
            + config.timing.max_substep_seconds * 2.0
        )
        frame_delta = 1.0 / 60.0
        frame_count = math.ceil(duration / frame_delta)

        for frame in range(frame_count):
            elapsed = frame * frame_delta
            simulation.set_target(target_at_time(elapsed))
            for delta in split_delta_time(frame_delta, config.timing):
                simulation.update(delta)
                snapshot = assembly.update(
                    simulation.snapshot().body,
                    delta,
                )
                _assert_support_constraints(self, snapshot)
                _assert_finite_ik(self, snapshot)
                active_group = snapshot.active_group_index
                if active_group is not None:
                    seen_groups.add(active_group)
                if (
                    active_group is not None
                    and active_group == previous_group
                ):
                    same_group_active_seconds += delta
                else:
                    same_group_active_seconds = (
                        delta if active_group is not None else 0.0
                    )
                self.assertLessEqual(
                    same_group_active_seconds,
                    maximum_active_seconds,
                )
                previous_group = active_group

        final_body = simulation.snapshot().body
        for _ in range(240):
            snapshot = assembly.update(final_body, 1.0 / 120.0)
            _assert_support_constraints(self, snapshot)
            _assert_finite_ik(self, snapshot)

        return (
            seen_groups,
            sum(leg.step_count for leg in snapshot.legs),
        )

    def test_circle_mouse_path_does_not_lock_state_machine(self) -> None:
        seen, completed = self._run_path(
            lambda elapsed: (
                600.0 + math.cos(elapsed * 1.2) * 190.0,
                400.0 + math.sin(elapsed * 1.2) * 190.0,
            ),
            7.0,
        )

        self.assertEqual(seen, {0, 1, 2, 3})
        self.assertGreater(completed, 0)

    def test_polyline_mouse_path_does_not_lock_state_machine(self) -> None:
        points = (
            pygame.Vector2(350.0, 250.0),
            pygame.Vector2(900.0, 260.0),
            pygame.Vector2(850.0, 620.0),
            pygame.Vector2(420.0, 570.0),
        )

        def polyline_target(elapsed: float) -> tuple[float, float]:
            segment = int(elapsed) % len(points)
            progress = elapsed - math.floor(elapsed)
            point = points[segment].lerp(
                points[(segment + 1) % len(points)],
                progress,
            )
            return point.x, point.y

        seen, completed = self._run_path(polyline_target, 7.0)

        self.assertEqual(seen, {0, 1, 2, 3})
        self.assertGreater(completed, 0)

    def test_sudden_mouse_jumps_do_not_lock_state_machine(self) -> None:
        points = (
            (250.0, 180.0),
            (950.0, 650.0),
            (950.0, 180.0),
            (250.0, 650.0),
        )

        seen, completed = self._run_path(
            lambda elapsed: points[int(elapsed / 0.7) % len(points)],
            7.0,
        )

        self.assertEqual(seen, {0, 1, 2, 3})
        self.assertGreater(completed, 0)


if __name__ == "__main__":
    unittest.main()
