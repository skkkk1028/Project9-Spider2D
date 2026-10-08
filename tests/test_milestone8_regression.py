"""Milestone 8 end-to-end replay and long-run delivery regression tests."""

from __future__ import annotations

import math
import unittest
from dataclasses import dataclass, replace

import pygame

from spider_sim.config import AppConfig, BodyConfig
from spider_sim.ik import IKReachState
from spider_sim.landing import ConstrainedGaitAssembly, ConstrainedGaitSnapshot
from spider_sim.leg import LegSide
from spider_sim.math2d import wrap_angle
from spider_sim.simulation import Simulation, split_delta_time
from spider_sim.stepping import FootState


@dataclass(frozen=True, slots=True)
class ReplayContract:
    """Public tolerances for the deterministic cross-FPS replay."""

    frames_per_second: tuple[int, ...] = (30, 60, 144)
    reference_frames_per_second: int = 120
    motion_seconds: float = 8.0
    settle_seconds: float = 16.0
    body_position_tolerance: float = 0.05
    body_velocity_tolerance: float = 0.05
    heading_tolerance: float = math.radians(0.1)
    # Different render-frame target samples can reach equivalent settled
    # poses through a nearby, but not identical, discrete gait sequence.
    discrete_step_count_tolerance: int = 4


@dataclass(frozen=True, slots=True)
class SoakContract:
    """Workload definition for the ten-minute simulated-time soak."""

    simulated_seconds: float = 600.0
    input_frames_per_second: int = 60
    x_radius: float = 50.0
    y_radius: float = 40.0
    x_frequency: float = 0.06
    y_frequency: float = 0.08


@dataclass(frozen=True, slots=True)
class ReplayResult:
    """Stable final values compared between replay frame rates."""

    body_position: tuple[float, float]
    body_velocity: tuple[float, float]
    body_heading: float
    feet: tuple[tuple[float, float], ...]
    group_start_counts: tuple[int, int, int, int]
    leg_step_counts: tuple[int, ...]
    support_count: int
    active_count: int
    spacing_violation_count: int
    bone_intersection_count: int
    bend_anomaly_count: int


REPLAY_CONTRACT = ReplayContract()
SOAK_CONTRACT = SoakContract()


def _constrained_body_config(
    config: AppConfig,
    *,
    heading_speed_threshold: float,
) -> BodyConfig:
    return replace(
        config.body,
        max_speed=config.landing.demo_max_speed,
        max_acceleration=config.landing.demo_max_acceleration,
        heading_speed_threshold=heading_speed_threshold,
    )


def _replay_target(elapsed: float) -> tuple[float, float]:
    return (
        600.0 + 120.0 * math.sin(elapsed * 0.5),
        400.0 + 90.0 * math.sin(elapsed * 0.8),
    )


def _soak_target(
    elapsed: float,
    contract: SoakContract,
) -> tuple[float, float]:
    return (
        600.0
        + contract.x_radius
        * math.cos(elapsed * contract.x_frequency),
        400.0
        + contract.y_radius
        * math.sin(elapsed * contract.y_frequency),
    )


class Milestone8RegressionTests(unittest.TestCase):
    def _assert_snapshot_invariants(
        self,
        snapshot: ConstrainedGaitSnapshot,
        config: AppConfig,
    ) -> None:
        self.assertEqual(len(snapshot.legs), 8)
        self.assertEqual(
            len({leg.mount.leg_id for leg in snapshot.legs}),
            8,
        )
        active = [
            leg
            for leg in snapshot.legs
            if leg.state is not FootState.PLANTED
        ]
        self.assertLessEqual(len(active), 2)
        self.assertEqual(snapshot.support_count, 8 - len(active))
        self.assertGreaterEqual(snapshot.support_count, 6)
        self.assertLessEqual(
            sum(leg.mount.side is LegSide.LEFT for leg in active),
            1,
        )
        self.assertLessEqual(
            sum(leg.mount.side is LegSide.RIGHT for leg in active),
            1,
        )

        for leg in snapshot.legs:
            values = (
                *leg.foot_world,
                leg.foot_height,
                leg.distance_to_default,
                leg.extension_ratio,
                leg.state_progress,
                *leg.ik.hip,
                *leg.ik.knee,
                *leg.ik.foot,
                *leg.ik.requested_target,
                *leg.ik.solved_target,
                *leg.ik.knee_candidates[0],
                *leg.ik.knee_candidates[1],
                leg.ik.target_error,
                leg.ik.upper_length,
                leg.ik.lower_length,
                leg.ik.minimum_reach,
                leg.ik.maximum_reach,
            )
            self.assertTrue(all(math.isfinite(value) for value in values))
            self.assertGreaterEqual(leg.state_progress, 0.0)
            self.assertLessEqual(leg.state_progress, 1.0)
            self.assertIn(leg.ik.bend_branch, (-1, 1))
            self.assertAlmostEqual(
                pygame.Vector2(leg.ik.hip).distance_to(leg.ik.knee),
                config.leg.upper_length,
                delta=1e-6,
            )
            self.assertAlmostEqual(
                pygame.Vector2(leg.ik.knee).distance_to(leg.ik.foot),
                config.leg.lower_length,
                delta=1e-6,
            )

    def _run_replay(self, frames_per_second: int) -> ReplayResult:
        config = AppConfig()
        body_config = _constrained_body_config(
            config,
            heading_speed_threshold=2.0,
        )
        simulation = Simulation(
            body_config,
            initial_position=(600.0, 400.0),
        )
        assembly = ConstrainedGaitAssembly(
            body_config,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            simulation.snapshot().body,
        )
        frame_delta = 1.0 / frames_per_second
        total_seconds = (
            REPLAY_CONTRACT.motion_seconds
            + REPLAY_CONTRACT.settle_seconds
        )

        for frame in range(round(total_seconds * frames_per_second)):
            elapsed = frame * frame_delta
            target_time = min(
                elapsed,
                REPLAY_CONTRACT.motion_seconds,
            )
            simulation.set_target(_replay_target(target_time))
            for delta in split_delta_time(frame_delta, config.timing):
                simulation.update(delta)
                snapshot = assembly.update(
                    simulation.snapshot().body,
                    delta,
                )
                self._assert_snapshot_invariants(snapshot, config)

        simulation_snapshot = simulation.snapshot()
        self.assertAlmostEqual(
            simulation_snapshot.elapsed_seconds,
            total_seconds,
            places=9,
        )
        return ReplayResult(
            body_position=simulation_snapshot.body.position,
            body_velocity=simulation_snapshot.body.velocity,
            body_heading=simulation_snapshot.body.heading,
            feet=tuple(leg.foot_world for leg in snapshot.legs),
            group_start_counts=snapshot.group_start_counts,
            leg_step_counts=tuple(
                leg.step_count for leg in snapshot.legs
            ),
            support_count=snapshot.support_count,
            active_count=sum(
                leg.state is not FootState.PLANTED
                for leg in snapshot.legs
            ),
            spacing_violation_count=len(snapshot.spacing_violations),
            bone_intersection_count=len(snapshot.bone_intersections),
            bend_anomaly_count=snapshot.total_bend_anomalies,
        )

    def test_30_60_144_fps_replays_stay_within_contract(self) -> None:
        config = AppConfig()
        reference = self._run_replay(
            REPLAY_CONTRACT.reference_frames_per_second
        )

        for frames_per_second in REPLAY_CONTRACT.frames_per_second:
            with self.subTest(frames_per_second=frames_per_second):
                actual = self._run_replay(frames_per_second)
                self.assertLessEqual(
                    pygame.Vector2(actual.body_position).distance_to(
                        reference.body_position
                    ),
                    REPLAY_CONTRACT.body_position_tolerance,
                )
                self.assertLessEqual(
                    pygame.Vector2(actual.body_velocity).distance_to(
                        reference.body_velocity
                    ),
                    REPLAY_CONTRACT.body_velocity_tolerance,
                )
                self.assertLessEqual(
                    abs(
                        wrap_angle(
                            actual.body_heading
                            - reference.body_heading
                        )
                    ),
                    REPLAY_CONTRACT.heading_tolerance,
                )
                for actual_foot, reference_foot in zip(
                    actual.feet,
                    reference.feet,
                    strict=True,
                ):
                    self.assertLessEqual(
                        pygame.Vector2(actual_foot).distance_to(
                            reference_foot
                        ),
                        config.foot_step.trigger_distance,
                    )
                for actual_count, reference_count in zip(
                    actual.group_start_counts,
                    reference.group_start_counts,
                    strict=True,
                ):
                    self.assertLessEqual(
                        abs(actual_count - reference_count),
                        REPLAY_CONTRACT.discrete_step_count_tolerance,
                    )
                for actual_count, reference_count in zip(
                    actual.leg_step_counts,
                    reference.leg_step_counts,
                    strict=True,
                ):
                    self.assertLessEqual(
                        abs(actual_count - reference_count),
                        REPLAY_CONTRACT.discrete_step_count_tolerance,
                    )
                self.assertEqual(actual.support_count, 8)
                self.assertEqual(actual.active_count, 0)
                self.assertEqual(actual.spacing_violation_count, 0)
                self.assertEqual(actual.bone_intersection_count, 0)
                self.assertEqual(actual.bend_anomaly_count, 0)

    def test_ten_simulated_minutes_remain_finite_and_recover_reach(
        self,
    ) -> None:
        config = AppConfig()
        body_config = _constrained_body_config(
            config,
            heading_speed_threshold=1.0,
        )
        simulation = Simulation(
            body_config,
            initial_position=(600.0, 400.0),
        )
        assembly = ConstrainedGaitAssembly(
            body_config,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            simulation.snapshot().body,
        )
        frame_delta = 1.0 / SOAK_CONTRACT.input_frames_per_second
        frame_count = round(
            SOAK_CONTRACT.simulated_seconds
            * SOAK_CONTRACT.input_frames_per_second
        )
        phase_duration = (
            config.foot_step.lift_duration
            + config.foot_step.swing_duration
            + config.foot_step.lower_duration
        )
        maximum_unreachable_duration = (
            phase_duration * len(config.four_phase_gait.groups)
            + config.landing.deferred_retry_duration
            + config.timing.max_frame_seconds
        )
        unreachable_durations = {
            mount.leg_id: 0.0 for mount in assembly.mounts
        }
        observed_maximum_unreachable = 0.0

        for frame in range(frame_count):
            elapsed = frame * frame_delta
            simulation.set_target(
                _soak_target(elapsed, SOAK_CONTRACT)
            )
            for delta in split_delta_time(frame_delta, config.timing):
                simulation.update(delta)
                snapshot = assembly.update(
                    simulation.snapshot().body,
                    delta,
                )
                self._assert_snapshot_invariants(snapshot, config)
                self.assertEqual(snapshot.bend_anomalies, ())
                for leg in snapshot.legs:
                    leg_id = leg.mount.leg_id
                    if leg.ik.reach_state is IKReachState.REACHABLE:
                        unreachable_durations[leg_id] = 0.0
                    else:
                        unreachable_durations[leg_id] += delta
                    observed_maximum_unreachable = max(
                        observed_maximum_unreachable,
                        unreachable_durations[leg_id],
                    )
                    self.assertLessEqual(
                        unreachable_durations[leg_id],
                        maximum_unreachable_duration,
                    )

        final = simulation.snapshot()
        self.assertAlmostEqual(
            final.elapsed_seconds,
            SOAK_CONTRACT.simulated_seconds,
            places=6,
        )
        self.assertEqual(final.update_count, 72_000)
        self.assertTrue(
            all(count > 0 for count in snapshot.group_start_counts)
        )
        self.assertEqual(snapshot.total_bend_anomalies, 0)
        self.assertTrue(
            all(
                leg.ik.reach_state is IKReachState.REACHABLE
                for leg in snapshot.legs
            )
        )
        self.assertLessEqual(
            observed_maximum_unreachable,
            maximum_unreachable_duration,
        )


if __name__ == "__main__":
    unittest.main()
