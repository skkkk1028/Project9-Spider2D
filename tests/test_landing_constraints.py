"""Acceptance tests for Milestone 7 constrained landing selection."""

from __future__ import annotations

import math
import unittest
from dataclasses import replace

import pygame

from spider_sim.config import AppConfig, BodyConfig
from spider_sim.landing import (
    ConstrainedGaitAssembly,
    LandingDecision,
    _proper_segment_intersection,
    _segment_closest_points,
)
from spider_sim.leg import LegSide
from spider_sim.math2d import heading_axes, world_to_local
from spider_sim.simulation import Simulation, split_delta_time
from spider_sim.spider import BodySnapshot
from spider_sim.stepping import (
    FootState,
    FourPhaseGaitAssembly,
    GroupedGaitSnapshot,
)


def _body(
    position: tuple[float, float] = (600.0, 400.0),
    heading: float = 0.0,
) -> BodySnapshot:
    return BodySnapshot(position, (0.0, 0.0), heading)


def _cross(first: pygame.Vector2, second: pygame.Vector2) -> float:
    return first.x * second.y - first.y * second.x


def _segments_cross_properly(
    first_start: pygame.Vector2,
    first_end: pygame.Vector2,
    second_start: pygame.Vector2,
    second_end: pygame.Vector2,
    epsilon: float,
) -> bool:
    first_direction = first_end - first_start
    second_direction = second_end - second_start
    denominator = _cross(first_direction, second_direction)
    if abs(denominator) <= epsilon:
        return False
    offset = second_start - first_start
    first_parameter = _cross(offset, second_direction) / denominator
    second_parameter = _cross(offset, first_direction) / denominator
    return (
        epsilon < first_parameter < 1.0 - epsilon
        and epsilon < second_parameter < 1.0 - epsilon
    )


def _independent_diagnostic_counts(
    snapshot: GroupedGaitSnapshot,
    minimum_spacing: float,
    epsilon: float,
) -> tuple[int, int]:
    spacing_count = 0
    for first_index, first in enumerate(snapshot.legs):
        first_foot = pygame.Vector2(first.foot_world)
        for second in snapshot.legs[first_index + 1 :]:
            if first_foot.distance_to(second.foot_world) < minimum_spacing:
                spacing_count += 1

    segments: list[
        tuple[str, pygame.Vector2, pygame.Vector2]
    ] = []
    for leg in snapshot.legs:
        segments.extend(
            (
                (
                    leg.mount.leg_id,
                    pygame.Vector2(leg.ik.hip),
                    pygame.Vector2(leg.ik.knee),
                ),
                (
                    leg.mount.leg_id,
                    pygame.Vector2(leg.ik.knee),
                    pygame.Vector2(leg.ik.foot),
                ),
            )
        )

    intersection_count = 0
    for first_index, first in enumerate(segments):
        for second in segments[first_index + 1 :]:
            if first[0] == second[0]:
                continue
            intersection_count += _segments_cross_properly(
                first[1],
                first[2],
                second[1],
                second[2],
                epsilon,
            )
    return spacing_count, intersection_count


def _assert_finite_snapshot(
    test_case: unittest.TestCase,
    snapshot: GroupedGaitSnapshot,
) -> None:
    for leg in snapshot.legs:
        values = (
            *leg.foot_world,
            leg.foot_height,
            leg.distance_to_default,
            leg.extension_ratio,
            *leg.ik.hip,
            *leg.ik.knee,
            *leg.ik.foot,
            *leg.ik.knee_candidates[0],
            *leg.ik.knee_candidates[1],
            leg.ik.target_error,
        )
        test_case.assertTrue(all(math.isfinite(value) for value in values))


def _mouse_body_samples(
    config: AppConfig,
    target_at_time,
    *,
    path_seconds: float = 7.0,
    settle_seconds: float = 4.0,
) -> tuple[
    BodyConfig,
    BodySnapshot,
    tuple[tuple[BodySnapshot, float], ...],
]:
    body_config = replace(
        config.body,
        max_speed=config.landing.demo_max_speed,
        max_acceleration=config.landing.demo_max_acceleration,
        heading_speed_threshold=2.0,
    )
    simulation = Simulation(
        body_config,
        initial_position=(600.0, 400.0),
    )
    initial_body = simulation.snapshot().body
    frame_delta = 1.0 / 60.0
    samples: list[tuple[BodySnapshot, float]] = []
    for frame in range(math.ceil(path_seconds / frame_delta)):
        simulation.set_target(target_at_time(frame * frame_delta))
        for delta in split_delta_time(frame_delta, config.timing):
            simulation.update(delta)
            samples.append((simulation.snapshot().body, delta))

    final_body = simulation.snapshot().body
    samples.extend(
        (final_body, 1.0 / 120.0)
        for _ in range(round(settle_seconds * 120.0))
    )
    return body_config, initial_body, tuple(samples)


class SegmentIntersectionTests(unittest.TestCase):
    def test_only_strict_internal_crossings_are_reported(self) -> None:
        crossing = _proper_segment_intersection(
            pygame.Vector2(0.0, 0.0),
            pygame.Vector2(10.0, 10.0),
            pygame.Vector2(0.0, 10.0),
            pygame.Vector2(10.0, 0.0),
            1e-6,
        )

        self.assertIsNotNone(crossing)
        self.assertAlmostEqual(crossing.x, 5.0)
        self.assertAlmostEqual(crossing.y, 5.0)
        self.assertIsNone(
            _proper_segment_intersection(
                pygame.Vector2(0.0, 0.0),
                pygame.Vector2(10.0, 0.0),
                pygame.Vector2(10.0, 0.0),
                pygame.Vector2(10.0, 10.0),
                1e-6,
            )
        )
        self.assertIsNone(
            _proper_segment_intersection(
                pygame.Vector2(0.0, 0.0),
                pygame.Vector2(10.0, 0.0),
                pygame.Vector2(0.0, 2.0),
                pygame.Vector2(10.0, 2.0),
                1e-6,
            )
        )

    def test_parallel_segments_report_true_clearance(self) -> None:
        first, second = _segment_closest_points(
            pygame.Vector2(0.0, 0.0),
            pygame.Vector2(10.0, 0.0),
            pygame.Vector2(3.0, 4.0),
            pygame.Vector2(8.0, 4.0),
            1e-6,
        )

        self.assertAlmostEqual(first.distance_to(second), 4.0)

    def test_collinear_overlap_has_zero_clearance(self) -> None:
        first, second = _segment_closest_points(
            pygame.Vector2(0.0, 0.0),
            pygame.Vector2(10.0, 0.0),
            pygame.Vector2(5.0, 0.0),
            pygame.Vector2(15.0, 0.0),
            1e-6,
        )

        self.assertAlmostEqual(first.distance_to(second), 0.0)


class ExtendedGeometryDiagnosticTests(unittest.TestCase):
    def test_default_pose_has_no_extended_geometry_violation(self) -> None:
        config = AppConfig()
        initial_body = _body()
        snapshot = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        ).solve(initial_body)

        self.assertEqual(snapshot.bone_clearance_violations, ())
        self.assertEqual(snapshot.body_penetrations, ())
        self.assertEqual(snapshot.anatomy_violations, ())

    def test_half_turn_is_detected_and_counted_without_nan(self) -> None:
        config = AppConfig()
        initial_body = _body()
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )

        snapshot = assembly.update(
            _body(heading=math.pi),
            1.0 / 120.0,
        )

        self.assertTrue(snapshot.bone_clearance_violations)
        self.assertTrue(snapshot.body_penetrations)
        self.assertEqual(len(snapshot.anatomy_violations), 16)
        self.assertEqual(
            snapshot.total_bone_clearance_violation_samples,
            len(snapshot.bone_clearance_violations),
        )
        self.assertEqual(
            snapshot.total_body_penetration_samples,
            len(snapshot.body_penetrations),
        )
        self.assertEqual(
            snapshot.total_anatomy_violation_samples,
            len(snapshot.anatomy_violations),
        )
        _assert_finite_snapshot(self, snapshot)

    def test_faster_constrained_cycle_reduces_anatomy_crossings(self) -> None:
        config = AppConfig()
        body_config = replace(
            config.body,
            max_speed=config.landing.demo_max_speed,
            max_acceleration=config.landing.demo_max_acceleration,
            heading_speed_threshold=4.5,
        )
        simulation = Simulation(
            body_config,
            initial_position=(600.0, 400.0),
        )
        initial_body = simulation.snapshot().body
        tuned = ConstrainedGaitAssembly(
            body_config,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )
        slow = ConstrainedGaitAssembly(
            body_config,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            replace(config.landing, phase_duration_scale=1.0),
            initial_body,
        )
        tuned_knee_crossings = 0
        slow_knee_crossings = 0
        tuned_foot_crossings = 0
        slow_foot_crossings = 0

        for frame in range(2400):
            elapsed = frame / 120.0
            simulation.set_target(
                (
                    600.0 + 180.0 * math.cos(0.55 * elapsed),
                    400.0 + 130.0 * math.sin(0.55 * elapsed),
                )
            )
            simulation.update(1.0 / 120.0)
            body = simulation.snapshot().body
            tuned_snapshot = tuned.update(body, 1.0 / 120.0)
            slow_snapshot = slow.update(body, 1.0 / 120.0)
            for tuned_leg, slow_leg in zip(
                tuned_snapshot.legs,
                slow_snapshot.legs,
                strict=True,
            ):
                tuned_knee_local = world_to_local(
                    tuned_leg.ik.knee,
                    body.position,
                    body.heading,
                )
                slow_knee_local = world_to_local(
                    slow_leg.ik.knee,
                    body.position,
                    body.heading,
                )
                tuned_foot_local = world_to_local(
                    tuned_leg.foot_world,
                    body.position,
                    body.heading,
                )
                slow_foot_local = world_to_local(
                    slow_leg.foot_world,
                    body.position,
                    body.heading,
                )
                side = float(tuned_leg.mount.side)
                tuned_knee_crossings += side * tuned_knee_local.y < 0.0
                slow_knee_crossings += side * slow_knee_local.y < 0.0
                tuned_foot_crossings += side * tuned_foot_local.y < 0.0
                slow_foot_crossings += side * slow_foot_local.y < 0.0

        self.assertLess(
            tuned_knee_crossings,
            slow_knee_crossings * 0.60,
        )
        self.assertLess(
            tuned_foot_crossings,
            slow_foot_crossings * 0.50,
        )
        self.assertEqual(tuned_snapshot.total_bend_anomalies, 0)


class LandingRegionTests(unittest.TestCase):
    def test_landing_prefers_available_bone_clearance(self) -> None:
        config = AppConfig()
        initial_body = _body()
        moved_body = _body((635.0, 400.0))
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )

        started = assembly.update(moved_body, 1.0 / 120.0)
        self.assertIs(started.landing_decision, LandingDecision.ACCEPTED)
        selected_ids = set(started.active_leg_ids)
        landed = assembly.update(moved_body, 0.2)

        self.assertIsNone(landed.active_group_index)
        self.assertEqual(landed.bone_intersections, ())
        self.assertFalse(
            any(
                violation.first_leg_id in selected_ids
                or violation.second_leg_id in selected_ids
                for violation in landed.bone_clearance_violations
            )
        )

    def test_clearance_preference_does_not_block_all_landings(self) -> None:
        config = AppConfig()
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            replace(config.landing, bone_clearance=200.0),
            _body(),
        )

        started = assembly.update(_body((635.0, 400.0)), 1.0 / 120.0)

        self.assertIs(started.landing_decision, LandingDecision.ACCEPTED)
        self.assertEqual(len(started.active_leg_ids), 2)
        self.assertTrue(started.bone_clearance_violations)

    def test_selected_targets_stay_in_safe_disjoint_body_local_channels(
        self,
    ) -> None:
        config = AppConfig()
        headings = (
            0.0,
            math.pi / 2.0,
            math.pi - 1e-8,
            -math.pi + 1e-8,
        )

        for heading in headings:
            with self.subTest(heading=heading):
                initial_body = _body(heading=heading)
                forward, _ = heading_axes(heading)
                moved_body = _body(
                    (
                        initial_body.position[0] + forward.x * 35.0,
                        initial_body.position[1] + forward.y * 35.0,
                    ),
                    heading,
                )
                assembly = ConstrainedGaitAssembly(
                    config.body,
                    config.leg,
                    config.foot_step,
                    config.four_phase_gait,
                    config.landing,
                    initial_body,
                )
                snapshot = assembly.update(moved_body, 1.0 / 120.0)
                mounts = {
                    mount.leg_id: mount
                    for mount in assembly.mounts
                }
                selected = [
                    candidate
                    for candidate in snapshot.candidate_points
                    if candidate.selected
                ]

                self.assertIs(
                    snapshot.landing_decision,
                    LandingDecision.ACCEPTED,
                )
                self.assertEqual(len(selected), 2)
                for candidate in selected:
                    mount = mounts[candidate.leg_id]
                    selected_local = world_to_local(
                        candidate.world_position,
                        moved_body.position,
                        moved_body.heading,
                    )
                    self.assertAlmostEqual(
                        selected_local.x,
                        candidate.local_position[0],
                    )
                    self.assertAlmostEqual(
                        selected_local.y,
                        candidate.local_position[1],
                    )
                    self.assertTrue(candidate.valid)
                    self.assertEqual(candidate.rejection_reasons, ())
                    self.assertLessEqual(
                        abs(
                            candidate.local_position[0]
                            - mount.default_foot_local[0]
                        ),
                        config.landing.channel_half_width,
                    )
                    signed_lateral = (
                        float(mount.side)
                        * candidate.local_position[1]
                    )
                    self.assertGreaterEqual(
                        signed_lateral,
                        config.body.half_width
                        + config.landing.safe_midline_margin,
                    )
                    self.assertLessEqual(
                        signed_lateral,
                        config.body.half_width
                        + config.leg.stance_lateral
                        + config.landing.safe_outer_padding,
                    )
                    extension = pygame.Vector2(
                        mount.hip_local
                    ).distance_to(candidate.local_position)
                    self.assertLessEqual(
                        extension
                        / (
                            config.leg.upper_length
                            + config.leg.lower_length
                        ),
                        config.foot_step.max_extension_ratio,
                    )

                regions = {
                    region.leg_id: region
                    for region in snapshot.activity_regions
                }
                for side in (LegSide.LEFT, LegSide.RIGHT):
                    side_mounts = sorted(
                        (
                            mount
                            for mount in assembly.mounts
                            if mount.side is side
                        ),
                        key=lambda mount: mount.row,
                    )
                    for front, rear in zip(
                        side_mounts,
                        side_mounts[1:],
                    ):
                        self.assertGreater(
                            regions[front.leg_id].local_x_min,
                            regions[rear.leg_id].local_x_max,
                        )

    def test_no_legal_pair_is_explicitly_deferred_without_moving_feet(
        self,
    ) -> None:
        config = AppConfig()
        landing_config = replace(
            config.landing,
            candidate_longitudinal_offsets=(0.0,),
            candidate_lateral_offsets=(0.0,),
            min_foot_spacing=30.0,
        )
        initial_body = _body()
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            landing_config,
            initial_body,
        )
        original_feet = {
            leg.mount.leg_id: leg.foot_world
            for leg in assembly.solve(initial_body).legs
        }
        moved_body = _body((635.0, 400.0))

        first = assembly.update(moved_body, 1.0 / 120.0)

        self.assertIs(first.landing_decision, LandingDecision.DEFERRED)
        self.assertIsNone(first.active_group_index)
        self.assertEqual(first.support_count, 8)
        self.assertEqual(first.deferred_group_indices, (0, 1, 2, 3))
        self.assertEqual(first.total_deferred_groups, 4)
        self.assertEqual(len(first.candidate_points), 8)
        self.assertTrue(
            all(
                not candidate.valid
                and candidate.rejection_reasons
                for candidate in first.candidate_points
            )
        )
        for leg in first.legs:
            self.assertIs(leg.state, FootState.PLANTED)
            self.assertIsNone(leg.step_target_world)
            self.assertEqual(
                leg.foot_world,
                original_feet[leg.mount.leg_id],
            )

        second = assembly.update(
            _body((636.0, 401.0), 0.01),
            1.0 / 120.0,
        )
        self.assertIs(second.landing_decision, LandingDecision.DEFERRED)
        self.assertEqual(
            second.total_deferred_groups,
            first.total_deferred_groups,
        )
        self.assertEqual(second.candidate_points, first.candidate_points)
        self.assertEqual(
            second.candidate_region_body,
            first.candidate_region_body,
        )
        self.assertEqual(
            second.activity_regions,
            first.activity_regions,
        )
        _assert_finite_snapshot(self, second)

    def test_intersection_fallback_is_selected_only_as_explicit_degraded(
        self,
    ) -> None:
        config = AppConfig()
        initial_body = _body()
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )

        snapshot = assembly.update(
            _body(heading=math.pi),
            1.0 / 120.0,
        )
        selected = [
            candidate
            for candidate in snapshot.candidate_points
            if candidate.selected
        ]

        self.assertIs(
            snapshot.landing_decision,
            LandingDecision.DEGRADED,
        )
        self.assertEqual(len(selected), 2)
        for candidate in selected:
            self.assertTrue(candidate.degraded)
            self.assertFalse(candidate.valid)
            self.assertEqual(
                set(candidate.rejection_reasons),
                {
                    "bone_intersection",
                    "explicit_degraded_intersection",
                },
            )
        self.assertEqual(snapshot.total_degraded_groups, 1)


class ConstraintEffectivenessTests(unittest.TestCase):
    def test_real_mouse_paths_detect_record_and_reduce_violations(
        self,
    ) -> None:
        config = AppConfig()
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

        paths = {
            "circle": lambda elapsed: (
                600.0 + math.cos(elapsed * 1.2) * 190.0,
                400.0 + math.sin(elapsed * 1.2) * 190.0,
            ),
            "polyline": polyline_target,
        }

        for path_name, target_at_time in paths.items():
            with self.subTest(path=path_name):
                (
                    body_config,
                    initial_body,
                    samples,
                ) = _mouse_body_samples(config, target_at_time)
                baseline = FourPhaseGaitAssembly(
                    body_config,
                    config.leg,
                    config.foot_step,
                    config.four_phase_gait,
                    initial_body,
                )
                constrained = ConstrainedGaitAssembly(
                    body_config,
                    config.leg,
                    config.foot_step,
                    config.four_phase_gait,
                    config.landing,
                    initial_body,
                )
                baseline_spacing = 0
                baseline_crossings = 0
                constrained_spacing = 0
                constrained_crossings = 0
                saw_degraded_candidate = False
                saw_landing_violation = False
                maximum_knee_jump = 0.0
                observed_bend_anomalies = 0
                previous_knees = {
                    leg.mount.leg_id: pygame.Vector2(leg.ik.knee)
                    for leg in constrained.solve(initial_body).legs
                }

                for body, delta in samples:
                    baseline_snapshot = baseline.update(body, delta)
                    constrained_snapshot = constrained.update(
                        body,
                        delta,
                    )
                    baseline_counts = _independent_diagnostic_counts(
                        baseline_snapshot,
                        config.landing.min_foot_spacing,
                        config.landing.intersection_epsilon,
                    )
                    constrained_counts = (
                        _independent_diagnostic_counts(
                            constrained_snapshot,
                            config.landing.min_foot_spacing,
                            config.landing.intersection_epsilon,
                        )
                    )
                    baseline_spacing += baseline_counts[0]
                    baseline_crossings += baseline_counts[1]
                    constrained_spacing += constrained_counts[0]
                    constrained_crossings += constrained_counts[1]

                    self.assertEqual(
                        len(constrained_snapshot.spacing_violations),
                        constrained_counts[0],
                    )
                    self.assertEqual(
                        len(constrained_snapshot.bone_intersections),
                        constrained_counts[1],
                    )
                    active = [
                        leg
                        for leg in constrained_snapshot.legs
                        if leg.state is not FootState.PLANTED
                    ]
                    self.assertLessEqual(len(active), 2)
                    self.assertGreaterEqual(
                        constrained_snapshot.support_count,
                        6,
                    )
                    self.assertLessEqual(
                        sum(
                            leg.mount.side is LegSide.LEFT
                            for leg in active
                        ),
                        1,
                    )
                    self.assertLessEqual(
                        sum(
                            leg.mount.side is LegSide.RIGHT
                            for leg in active
                        ),
                        1,
                    )
                    for candidate in (
                        constrained_snapshot.candidate_points
                    ):
                        if not candidate.selected:
                            continue
                        if candidate.valid:
                            self.assertFalse(candidate.degraded)
                        else:
                            self.assertTrue(candidate.degraded)
                            self.assertEqual(
                                set(candidate.rejection_reasons),
                                {
                                    "bone_intersection",
                                    (
                                        "explicit_degraded_"
                                        "intersection"
                                    ),
                                },
                            )
                            saw_degraded_candidate = True
                    if (
                        constrained_snapshot.landing_violation
                        is not None
                    ):
                        self.assertIs(
                            constrained_snapshot.landing_decision,
                            LandingDecision.DEGRADED,
                        )
                        self.assertTrue(
                            constrained_snapshot.landing_violation
                            .bone_intersections
                        )
                        saw_landing_violation = True
                    for leg in constrained_snapshot.legs:
                        knee = pygame.Vector2(leg.ik.knee)
                        knee_jump = knee.distance_to(
                            previous_knees[leg.mount.leg_id]
                        )
                        maximum_knee_jump = max(
                            maximum_knee_jump,
                            knee_jump,
                        )
                        if (
                            knee_jump
                            > config.landing.max_knee_jump
                        ):
                            self.assertIn(
                                leg.mount.leg_id,
                                {
                                    anomaly.leg_id
                                    for anomaly in (
                                        constrained_snapshot
                                        .bend_anomalies
                                    )
                                },
                            )
                        previous_knees[leg.mount.leg_id] = knee
                    observed_bend_anomalies += len(
                        constrained_snapshot.bend_anomalies
                    )
                    _assert_finite_snapshot(
                        self,
                        constrained_snapshot,
                    )

                self.assertGreater(baseline_spacing, 0)
                self.assertGreater(baseline_crossings, 0)
                self.assertLessEqual(
                    constrained_spacing,
                    baseline_spacing * 0.50,
                )
                self.assertLessEqual(
                    constrained_crossings,
                    baseline_crossings * 0.75,
                )
                self.assertEqual(
                    constrained_snapshot
                    .total_spacing_violation_samples,
                    constrained_spacing,
                )
                self.assertEqual(
                    constrained_snapshot
                    .total_bone_intersection_samples,
                    constrained_crossings,
                )
                self.assertTrue(
                    all(
                        start_count > 0
                        for start_count in (
                            constrained_snapshot.group_start_counts
                        )
                    )
                )
                self.assertTrue(saw_degraded_candidate)
                self.assertTrue(saw_landing_violation)
                self.assertEqual(
                    constrained_snapshot.total_bend_anomalies,
                    observed_bend_anomalies,
                )
                if (
                    maximum_knee_jump
                    > config.landing.max_knee_jump
                ):
                    self.assertGreater(observed_bend_anomalies, 0)
                self.assertEqual(
                    constrained_snapshot.spacing_violations,
                    (),
                )
                self.assertEqual(
                    constrained_snapshot.bone_intersections,
                    (),
                )

    def test_sudden_mouse_jumps_do_not_lock_constrained_state_machine(
        self,
    ) -> None:
        config = AppConfig()
        targets = (
            (250.0, 180.0),
            (950.0, 650.0),
            (950.0, 180.0),
            (250.0, 650.0),
        )
        body_config, initial_body, samples = _mouse_body_samples(
            config,
            lambda elapsed: targets[
                int(elapsed / 0.7) % len(targets)
            ],
        )
        assembly = ConstrainedGaitAssembly(
            body_config,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )
        previous_group: int | None = None
        same_group_active_seconds = 0.0
        maximum_active_seconds = (
            config.foot_step.lift_duration
            + config.foot_step.swing_duration
            + config.foot_step.lower_duration
            + config.timing.max_substep_seconds * 2.0
        )

        for body, delta in samples:
            snapshot = assembly.update(body, delta)
            if (
                snapshot.active_group_index is not None
                and snapshot.active_group_index == previous_group
            ):
                same_group_active_seconds += delta
            else:
                same_group_active_seconds = (
                    delta
                    if snapshot.active_group_index is not None
                    else 0.0
                )
            self.assertLessEqual(
                same_group_active_seconds,
                maximum_active_seconds,
            )
            self.assertGreaterEqual(snapshot.support_count, 6)
            _assert_finite_snapshot(self, snapshot)
            previous_group = snapshot.active_group_index

        self.assertIsNone(snapshot.active_group_index)
        self.assertEqual(snapshot.support_count, 8)
        self.assertTrue(
            all(count > 0 for count in snapshot.group_start_counts)
        )
        self.assertEqual(snapshot.spacing_violations, ())
        self.assertEqual(snapshot.bone_intersections, ())


class BendContinuityTests(unittest.TestCase):
    def test_fast_continuous_rotation_has_bounded_physical_knee_motion(
        self,
    ) -> None:
        config = AppConfig()
        initial_body = _body()
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )
        previous_knees = {
            leg.mount.leg_id: pygame.Vector2(leg.ik.knee)
            for leg in assembly.solve(initial_body).legs
        }
        maximum_jump = 0.0

        for index in range(48):
            snapshot = assembly.update(
                _body(heading=math.radians(7.5) * (index + 1)),
                1.0 / 120.0,
            )
            for leg in snapshot.legs:
                knee = pygame.Vector2(leg.ik.knee)
                maximum_jump = max(
                    maximum_jump,
                    knee.distance_to(
                        previous_knees[leg.mount.leg_id]
                    ),
                )
                previous_knees[leg.mount.leg_id] = knee
            self.assertEqual(snapshot.bend_anomalies, ())
            _assert_finite_snapshot(self, snapshot)

        self.assertLess(maximum_jump, config.landing.max_knee_jump)
        self.assertEqual(snapshot.total_bend_anomalies, 0)

    def test_discontinuous_half_turn_is_marked_as_a_bend_anomaly(
        self,
    ) -> None:
        config = AppConfig()
        initial_body = _body()
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )

        snapshot = assembly.update(
            _body(heading=math.pi),
            1.0 / 120.0,
        )

        self.assertTrue(snapshot.bend_anomalies)
        self.assertEqual(
            snapshot.total_bend_anomalies,
            len(snapshot.bend_anomalies),
        )
        self.assertTrue(
            all(
                anomaly.knee_jump > config.landing.max_knee_jump
                for anomaly in snapshot.bend_anomalies
            )
        )
        _assert_finite_snapshot(self, snapshot)


if __name__ == "__main__":
    unittest.main()
