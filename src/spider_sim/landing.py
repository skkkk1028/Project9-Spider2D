"""Constrained landing selection and overlap diagnostics for Milestone 7."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, replace
from enum import Enum

import pygame

from spider_sim.config import (
    BodyConfig,
    FootStepConfig,
    FourPhaseGaitConfig,
    LandingConstraintConfig,
    LegConfig,
)
from spider_sim.ik import IKReachState, IKResult, TwoBoneIKSolver
from spider_sim.leg import LegMount
from spider_sim.math2d import heading_axes, local_to_world, world_to_local
from spider_sim.spider import BodySnapshot
from spider_sim.stepping import (
    FootState,
    FourPhaseGaitAssembly,
    SteppingLegSnapshot,
    _LegRuntime,
)


class LandingDecision(str, Enum):
    """Result of the latest M7 group-start attempt."""

    IDLE = "idle"
    ACCEPTED = "accepted"
    DEGRADED = "degraded"
    DEFERRED = "deferred"


@dataclass(frozen=True, slots=True)
class ActivityRegion:
    """One leg's body-local landing channel and current world polygon."""

    leg_id: str
    local_x_min: float
    local_x_max: float
    local_abs_y_min: float
    local_abs_y_max: float
    polygon_world: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ]


@dataclass(frozen=True, slots=True)
class CandidatePoint:
    """Debug record for one generated landing candidate."""

    leg_id: str
    local_position: tuple[float, float]
    world_position: tuple[float, float]
    valid: bool
    selected: bool
    degraded: bool
    rejection_reasons: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class FootSpacingViolation:
    """A pair of requested foot positions below the configured spacing."""

    first_leg_id: str
    second_leg_id: str
    distance: float


@dataclass(frozen=True, slots=True)
class BoneIntersection:
    """A proper intersection between bone segments from different legs."""

    first_leg_id: str
    first_segment: str
    second_leg_id: str
    second_segment: str
    point_world: tuple[float, float]


@dataclass(frozen=True, slots=True)
class BoneClearanceViolation:
    """Two bones from different legs closer than the visual clearance."""

    first_leg_id: str
    first_segment: str
    second_leg_id: str
    second_segment: str
    distance: float
    point_world: tuple[float, float]


@dataclass(frozen=True, slots=True)
class BodyPenetration:
    """A bone segment entering the body's clearance ellipse."""

    leg_id: str
    segment: str
    normalized_distance: float
    point_world: tuple[float, float]


@dataclass(frozen=True, slots=True)
class BendAnomaly:
    """A branch flip or unexpectedly large knee jump."""

    leg_id: str
    branch_changed: bool
    knee_jump: float
    alternate_knee_jump: float


@dataclass(frozen=True, slots=True)
class AnatomyViolation:
    """A knee inside the body corridor or a foot across its body side."""

    leg_id: str
    point_kind: str
    signed_lateral: float
    required_lateral: float
    point_world: tuple[float, float]


@dataclass(frozen=True, slots=True)
class LandingViolation:
    """A completed landing whose actual IK still intersects support bones."""

    group_index: int
    leg_ids: tuple[str, str]
    bone_intersections: tuple[BoneIntersection, ...]


@dataclass(frozen=True, slots=True)
class ConstrainedGaitSnapshot:
    """M6 scheduler state plus M7 landing and geometry diagnostics."""

    body: BodySnapshot
    legs: tuple[SteppingLegSnapshot, ...]
    active_group_index: int | None
    active_leg_ids: tuple[str, ...]
    next_group_index: int
    group_start_counts: tuple[int, int, int, int]
    activity_regions: tuple[ActivityRegion, ...]
    candidate_region_body: BodySnapshot | None
    candidate_points: tuple[CandidatePoint, ...]
    spacing_violations: tuple[FootSpacingViolation, ...]
    bone_intersections: tuple[BoneIntersection, ...]
    bone_clearance_violations: tuple[BoneClearanceViolation, ...]
    body_penetrations: tuple[BodyPenetration, ...]
    landing_decision: LandingDecision
    deferred_group_indices: tuple[int, ...]
    total_deferred_groups: int
    total_degraded_groups: int
    landing_violation: LandingViolation | None
    total_landing_violations: int
    total_spacing_violation_samples: int
    total_bone_intersection_samples: int
    total_bone_clearance_violation_samples: int
    total_body_penetration_samples: int
    bend_anomalies: tuple[BendAnomaly, ...]
    total_bend_anomalies: int
    anatomy_violations: tuple[AnatomyViolation, ...]
    total_anatomy_violation_samples: int

    @property
    def support_count(self) -> int:
        """Return the number of planted support legs."""
        return sum(leg.state is FootState.PLANTED for leg in self.legs)


@dataclass(slots=True)
class _CandidateEvaluation:
    leg_id: str
    local_position: pygame.Vector2
    world_position: pygame.Vector2
    individual_reasons: set[str]
    pair_reasons: set[str]
    selected_reasons: set[str]
    ik_result: IKResult | None = None
    support_intersections: tuple[BoneIntersection, ...] = ()
    selected: bool = False
    degraded: bool = False


def _cross(first: pygame.Vector2, second: pygame.Vector2) -> float:
    return first.x * second.y - first.y * second.x


def _proper_segment_intersection(
    first_start: pygame.Vector2,
    first_end: pygame.Vector2,
    second_start: pygame.Vector2,
    second_end: pygame.Vector2,
    epsilon: float,
) -> pygame.Vector2 | None:
    first_direction = first_end - first_start
    second_direction = second_end - second_start
    denominator = _cross(first_direction, second_direction)
    if abs(denominator) <= epsilon:
        return None

    offset = second_start - first_start
    first_parameter = _cross(offset, second_direction) / denominator
    second_parameter = _cross(offset, first_direction) / denominator
    if (
        epsilon < first_parameter < 1.0 - epsilon
        and epsilon < second_parameter < 1.0 - epsilon
    ):
        return first_start + first_direction * first_parameter
    return None


def _closest_point_on_segment(
    point: pygame.Vector2,
    start: pygame.Vector2,
    end: pygame.Vector2,
) -> pygame.Vector2:
    direction = end - start
    length_squared = direction.length_squared()
    if length_squared <= 1e-18:
        return start.copy()
    parameter = max(
        0.0,
        min(1.0, (point - start).dot(direction) / length_squared),
    )
    return start + direction * parameter


def _segment_closest_points(
    first_start: pygame.Vector2,
    first_end: pygame.Vector2,
    second_start: pygame.Vector2,
    second_end: pygame.Vector2,
    epsilon: float,
) -> tuple[pygame.Vector2, pygame.Vector2]:
    intersection = _proper_segment_intersection(
        first_start,
        first_end,
        second_start,
        second_end,
        epsilon,
    )
    if intersection is not None:
        return intersection.copy(), intersection.copy()

    candidates = (
        (
            first_start,
            _closest_point_on_segment(
                first_start,
                second_start,
                second_end,
            ),
        ),
        (
            first_end,
            _closest_point_on_segment(
                first_end,
                second_start,
                second_end,
            ),
        ),
        (
            _closest_point_on_segment(
                second_start,
                first_start,
                first_end,
            ),
            second_start,
        ),
        (
            _closest_point_on_segment(
                second_end,
                first_start,
                first_end,
            ),
            second_end,
        ),
    )
    return min(
        candidates,
        key=lambda pair: pair[0].distance_squared_to(pair[1]),
    )


class ConstrainedGaitAssembly(FourPhaseGaitAssembly):
    """Choose legal group landings and expose overlap diagnostics."""

    def __init__(
        self,
        body_config: BodyConfig,
        leg_config: LegConfig,
        step_config: FootStepConfig,
        gait_config: FourPhaseGaitConfig,
        landing_config: LandingConstraintConfig,
        initial_body: BodySnapshot,
        solver: TwoBoneIKSolver | None = None,
    ) -> None:
        continuity_solver = solver or TwoBoneIKSolver()
        constrained_step_config = replace(
            step_config,
            lift_duration=(
                step_config.lift_duration
                * landing_config.phase_duration_scale
            ),
            swing_duration=(
                step_config.swing_duration
                * landing_config.phase_duration_scale
            ),
            lower_duration=(
                step_config.lower_duration
                * landing_config.phase_duration_scale
            ),
        )
        super().__init__(
            body_config,
            leg_config,
            constrained_step_config,
            gait_config,
            initial_body,
            continuity_solver,
        )
        self._landing_config = landing_config
        self._validate_channels()
        self._last_candidates: tuple[CandidatePoint, ...] = ()
        self._last_candidate_body: BodySnapshot | None = None
        self._last_decision = LandingDecision.IDLE
        self._last_deferred_groups: tuple[int, ...] = ()
        self._total_deferred_groups = 0
        self._total_degraded_groups = 0
        self._last_landing_violation: LandingViolation | None = None
        self._total_landing_violations = 0
        self._total_spacing_violation_samples = 0
        self._total_bone_intersection_samples = 0
        self._total_bone_clearance_violation_samples = 0
        self._total_body_penetration_samples = 0
        self._previous_knees: dict[str, tuple[float, float]] = {}
        self._previous_branches: dict[str, int] = {}
        self._branch_preferences: dict[str, int] = {}
        self._last_bend_anomalies: tuple[BendAnomaly, ...] = ()
        self._total_bend_anomalies = 0
        self._total_anatomy_violation_samples = 0
        self._deferred_cooldowns = [0.0, 0.0, 0.0, 0.0]
        self._started_group_this_update: int | None = None
        self._collecting_history = True
        self.solve(initial_body)
        self._collecting_history = False

    def _validate_channels(self) -> None:
        by_side: dict[object, list[LegMount]] = {}
        for mount in self.mounts:
            by_side.setdefault(mount.side, []).append(mount)

        for mounts in by_side.values():
            ordered = sorted(mounts, key=lambda mount: mount.row)
            for first, second in zip(ordered, ordered[1:]):
                gap = (
                    first.default_foot_local[0]
                    - second.default_foot_local[0]
                )
                if 2.0 * self._landing_config.channel_half_width >= gap:
                    raise ValueError(
                        "landing channels overlap or reverse row order"
                    )

        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        if self._landing_config.min_foot_spacing >= maximum_reach:
            raise ValueError(
                "min_foot_spacing must be smaller than maximum leg reach"
            )

    def update(
        self,
        body: BodySnapshot,
        delta_seconds: float,
    ) -> ConstrainedGaitSnapshot:
        """Advance M6 phases while recording M7 bend continuity."""
        if (
            isinstance(delta_seconds, bool)
            or not isinstance(delta_seconds, (int, float))
            or not math.isfinite(delta_seconds)
            or delta_seconds <= 0.0
        ):
            raise ValueError("delta_seconds must be finite and positive")
        for index, cooldown in enumerate(self._deferred_cooldowns):
            self._deferred_cooldowns[index] = max(
                0.0,
                cooldown - float(delta_seconds),
            )
        previously_active_group = self._active_group_index
        self._started_group_this_update = None
        self._last_landing_violation = None
        self._last_bend_anomalies = ()
        self._collecting_history = True
        try:
            snapshot = super().update(body, delta_seconds)
        finally:
            self._collecting_history = False
        constrained = snapshot  # type: ignore[assignment]
        completed_group: int | None = None
        if (
            previously_active_group is not None
            and constrained.active_group_index is None
        ):
            completed_group = previously_active_group
        elif (
            previously_active_group is None
            and self._started_group_this_update is not None
            and constrained.active_group_index is None
        ):
            completed_group = self._started_group_this_update

        if completed_group is not None:
            landed_ids = tuple(
                self._runtimes[index].mount.leg_id
                for index in self._group_indices[completed_group]
            )
            landed_id_set = set(landed_ids)
            landing_intersections = tuple(
                intersection
                for intersection in constrained.bone_intersections
                if (
                    intersection.first_leg_id in landed_id_set
                    or intersection.second_leg_id in landed_id_set
                )
            )
            if landing_intersections:
                self._last_decision = LandingDecision.DEGRADED
                self._last_landing_violation = LandingViolation(
                    group_index=completed_group,
                    leg_ids=landed_ids,  # type: ignore[arg-type]
                    bone_intersections=landing_intersections,
                )
                self._total_landing_violations += 1

        self._total_spacing_violation_samples += len(
            constrained.spacing_violations
        )
        self._total_bone_intersection_samples += len(
            constrained.bone_intersections
        )
        self._total_bone_clearance_violation_samples += len(
            constrained.bone_clearance_violations
        )
        self._total_body_penetration_samples += len(
            constrained.body_penetrations
        )
        self._total_anatomy_violation_samples += len(
            constrained.anatomy_violations
        )
        return replace(
            constrained,
            total_spacing_violation_samples=(
                self._total_spacing_violation_samples
            ),
            total_bone_intersection_samples=(
                self._total_bone_intersection_samples
            ),
            total_bone_clearance_violation_samples=(
                self._total_bone_clearance_violation_samples
            ),
            total_body_penetration_samples=(
                self._total_body_penetration_samples
            ),
            landing_violation=self._last_landing_violation,
            total_landing_violations=self._total_landing_violations,
            landing_decision=self._last_decision,
            total_anatomy_violation_samples=(
                self._total_anatomy_violation_samples
            ),
        )

    def _try_start_group(self, body: BodySnapshot) -> None:
        group_count = len(self._group_indices)
        candidate_records: list[CandidatePoint] = []
        deferred_groups: list[int] = []
        self._last_decision = LandingDecision.IDLE
        base_results = self._solve_targets(body)
        current_intersections = self._find_bone_intersections(
            base_results
        )
        crossing_leg_ids = {
            leg_id
            for intersection in current_intersections
            for leg_id in (
                intersection.first_leg_id,
                intersection.second_leg_id,
            )
        }
        for offset in range(group_count):
            group_index = (
                self._next_group_index + offset
            ) % group_count
            runtime_indices = self._group_indices[group_index]
            if not any(
                (
                      self._needs_step(self._runtimes[index], body)
                      or self._runtimes[index].mount.leg_id
                      in crossing_leg_ids
                  )
                for index in runtime_indices
            ):
                continue
            if self._deferred_cooldowns[group_index] > 0.0:
                deferred_groups.append(group_index)
                continue

            targets, candidates, decision = self._plan_group_landing(
                group_index,
                body,
                base_results,
                current_intersections,
            )
            self._last_candidate_body = body
            candidate_records.extend(candidates)
            if targets is None:
                deferred_groups.append(group_index)
                self._total_deferred_groups += 1
                self._deferred_cooldowns[group_index] = (
                    self._landing_config.deferred_retry_duration
                )
                continue

            for index, target in zip(
                runtime_indices,
                targets,
                strict=True,
            ):
                self._start_runtime_at(
                    self._runtimes[index],
                    target,
                )
            self._active_group_index = group_index
            self._started_group_this_update = group_index
            self._next_group_index = (group_index + 1) % group_count
            self._group_start_counts[group_index] += 1
            self._last_decision = decision
            if decision is LandingDecision.DEGRADED:
                self._total_degraded_groups += 1
            self._last_candidates = tuple(candidate_records)
            self._last_deferred_groups = tuple(deferred_groups)
            return

        if deferred_groups:
            self._last_decision = LandingDecision.DEFERRED
        if candidate_records:
            self._last_candidates = tuple(candidate_records)
        elif not deferred_groups:
            self._last_candidates = ()
            self._last_candidate_body = None
        self._last_deferred_groups = tuple(deferred_groups)

    def _start_runtime_at(
        self,
        runtime: _LegRuntime,
        target_world: pygame.Vector2,
    ) -> None:
        if runtime.state is not FootState.PLANTED:
            raise RuntimeError("cannot start an already active constrained leg")
        runtime.state = FootState.LIFT
        runtime.state_elapsed = 0.0
        runtime.foot_height = 0.0
        runtime.swing_start_world = runtime.foot_world.copy()
        runtime.step_target_world = target_world.copy()

    def _plan_group_landing(
        self,
        group_index: int,
        body: BodySnapshot,
        base_results: dict[str, IKResult],
        current_intersections: tuple[BoneIntersection, ...],
    ) -> tuple[
        tuple[pygame.Vector2, pygame.Vector2] | None,
        tuple[CandidatePoint, ...],
        LandingDecision,
    ]:
        runtime_indices = self._group_indices[group_index]
        active_ids = {
            self._runtimes[index].mount.leg_id
            for index in runtime_indices
        }
        support_results = {
            leg_id: result
            for leg_id, result in base_results.items()
            if leg_id not in active_ids
        }
        current_relevant_count = sum(
            (
                intersection.first_leg_id in active_ids
                or intersection.second_leg_id in active_ids
            )
            for intersection in current_intersections
        )
        evaluations = [
            self._generate_candidates(
                self._runtimes[index].mount,
                body,
            )
            for index in runtime_indices
        ]

        for leg_evaluations in evaluations:
            for evaluation in leg_evaluations:
                if evaluation.individual_reasons:
                    continue
                result = self._solve_leg_target(
                    evaluation.leg_id,
                    evaluation.world_position,
                    body,
                )
                if (
                    result.reach_state is not IKReachState.REACHABLE
                    or result.target_error
                    > self._landing_config.intersection_epsilon
                ):
                    evaluation.individual_reasons.add("unreachable")
                    continue
                evaluation.ik_result = result
                candidate_with_supports = dict(support_results)
                candidate_with_supports[evaluation.leg_id] = result
                evaluation.support_intersections = tuple(
                    intersection
                    for intersection in self._find_bone_intersections(
                        candidate_with_supports
                    )
                    if (
                        intersection.first_leg_id
                        == evaluation.leg_id
                        or intersection.second_leg_id
                        == evaluation.leg_id
                    )
                )
        selected_pair: tuple[
            _CandidateEvaluation,
            _CandidateEvaluation,
        ] | None = None
        selected_decision = LandingDecision.DEFERRED
        best_degraded_pair: tuple[
            _CandidateEvaluation,
            _CandidateEvaluation,
        ] | None = None
        best_degraded_count = math.inf

        for first, second in itertools.product(
            evaluations[0],
            evaluations[1],
        ):
            if first.individual_reasons or second.individual_reasons:
                continue
            targets = {
                first.leg_id: first.world_position,
                second.leg_id: second.world_position,
            }
            spacing = self._spacing_violations_for_targets(targets)
            if spacing:
                first.pair_reasons.add("min_spacing")
                second.pair_reasons.add("min_spacing")
                continue

            if first.ik_result is None or second.ik_result is None:
                raise RuntimeError("candidate IK preview was not initialized")
            pair_intersections = self._find_bone_intersections(
                {
                    first.leg_id: first.ik_result,
                    second.leg_id: second.ik_result,
                }
            )
            intersection_count = (
                len(first.support_intersections)
                + len(second.support_intersections)
                + len(pair_intersections)
            )
            if intersection_count:
                first.pair_reasons.add("bone_intersection")
                second.pair_reasons.add("bone_intersection")
                if intersection_count < best_degraded_count:
                    best_degraded_pair = first, second
                    best_degraded_count = intersection_count
                continue

            selected_pair = first, second
            selected_decision = LandingDecision.ACCEPTED
            first.selected = True
            second.selected = True
            break

        if (
            selected_pair is None
            and best_degraded_pair is not None
            and current_relevant_count > 0
            and best_degraded_count <= current_relevant_count
        ):
            selected_pair = best_degraded_pair
            selected_decision = LandingDecision.DEGRADED
            for candidate in selected_pair:
                candidate.selected = True
                candidate.degraded = True
                candidate.selected_reasons.add("bone_intersection")

        records = tuple(
            self._candidate_record(
                evaluation,
                no_pair_found=selected_pair is None,
            )
            for leg_evaluations in evaluations
            for evaluation in leg_evaluations
        )
        if selected_pair is None:
            return None, records, LandingDecision.DEFERRED
        return (
            (
                selected_pair[0].world_position.copy(),
                selected_pair[1].world_position.copy(),
            ),
            records,
            selected_decision,
        )

    def _generate_candidates(
        self,
        mount: LegMount,
        body: BodySnapshot,
    ) -> list[_CandidateEvaluation]:
        candidates: list[_CandidateEvaluation] = []
        for longitudinal_offset in (
            self._landing_config.candidate_longitudinal_offsets
        ):
            for lateral_offset in (
                self._landing_config.candidate_lateral_offsets
            ):
                local_position = pygame.Vector2(
                    mount.default_foot_local[0] + longitudinal_offset,
                    mount.default_foot_local[1]
                    + float(mount.side) * lateral_offset,
                )
                reasons = self._individual_rejection_reasons(
                    mount,
                    local_position,
                )
                candidates.append(
                    _CandidateEvaluation(
                        leg_id=mount.leg_id,
                        local_position=local_position,
                        world_position=local_to_world(
                            local_position,
                            body.position,
                            body.heading,
                        ),
                        individual_reasons=reasons,
                        pair_reasons=set(),
                        selected_reasons=set(),
                    )
                )
        return candidates

    def _individual_rejection_reasons(
        self,
        mount: LegMount,
        local_position: pygame.Vector2,
    ) -> set[str]:
        reasons: set[str] = set()
        center_x = mount.default_foot_local[0]
        if not (
            center_x - self._landing_config.channel_half_width
            <= local_position.x
            <= center_x + self._landing_config.channel_half_width
        ):
            reasons.add("channel")

        side_distance = float(mount.side) * local_position.y
        minimum_side_distance = (
            self._body_config.half_width
            + self._landing_config.safe_midline_margin
        )
        maximum_side_distance = (
            self._body_config.half_width
            + self._leg_config.stance_lateral
            + self._landing_config.safe_outer_padding
        )
        if not minimum_side_distance <= side_distance <= maximum_side_distance:
            reasons.add("side_boundary")

        target_distance = pygame.Vector2(
            mount.hip_local
        ).distance_to(local_position)
        minimum_reach = abs(
            self._leg_config.upper_length
            - self._leg_config.lower_length
        )
        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        if not minimum_reach <= target_distance <= maximum_reach:
            reasons.add("unreachable")
        if (
            target_distance / maximum_reach
            > self._step_config.max_extension_ratio
        ):
            reasons.add("extension_limit")
        return reasons

    @property
    def _required_knee_lateral(self) -> float:
        return (
            self._body_config.half_width
            + self._landing_config.knee_body_clearance
        )

    def _signed_lateral(
        self,
        mount: LegMount,
        point_world: pygame.Vector2 | tuple[float, float],
        body: BodySnapshot,
    ) -> float:
        local = world_to_local(
            point_world,
            body.position,
            body.heading,
        )
        return float(mount.side) * local.y

    def _candidate_record(
        self,
        candidate: _CandidateEvaluation,
        *,
        no_pair_found: bool,
    ) -> CandidatePoint:
        reasons = set(candidate.individual_reasons)
        if candidate.degraded:
            reasons.update(candidate.selected_reasons)
            reasons.add("explicit_degraded_intersection")
        elif no_pair_found:
            reasons.update(candidate.pair_reasons)
            if not reasons:
                reasons.add("no_compatible_pair")
        return CandidatePoint(
            leg_id=candidate.leg_id,
            local_position=(
                candidate.local_position.x,
                candidate.local_position.y,
            ),
            world_position=(
                candidate.world_position.x,
                candidate.world_position.y,
            ),
            valid=not reasons,
            selected=candidate.selected,
            degraded=candidate.degraded,
            rejection_reasons=tuple(sorted(reasons)),
        )

    def _spacing_violations_for_targets(
        self,
        overrides: dict[str, pygame.Vector2],
    ) -> tuple[FootSpacingViolation, ...]:
        positions = {
            runtime.mount.leg_id: overrides.get(
                runtime.mount.leg_id,
                runtime.foot_world,
            )
            for runtime in self._runtimes
        }
        active_ids = set(overrides)
        violations: list[FootSpacingViolation] = []
        leg_ids = tuple(positions)
        for first_index, first_id in enumerate(leg_ids):
            for second_id in leg_ids[first_index + 1 :]:
                if first_id not in active_ids and second_id not in active_ids:
                    continue
                distance = positions[first_id].distance_to(
                    positions[second_id]
                )
                if distance < self._landing_config.min_foot_spacing:
                    violations.append(
                        FootSpacingViolation(
                            first_leg_id=first_id,
                            second_leg_id=second_id,
                            distance=distance,
                        )
                    )
        return tuple(violations)

    def _solve_targets(
        self,
        body: BodySnapshot,
        overrides: dict[str, pygame.Vector2] | None = None,
    ) -> dict[str, IKResult]:
        override_targets = overrides or {}
        forward, right = heading_axes(body.heading)
        results: dict[str, IKResult] = {}
        for runtime in self._runtimes:
            mount = runtime.mount
            hip_world = local_to_world(
                mount.hip_local,
                body.position,
                body.heading,
            )
            bend_local = pygame.Vector2(mount.bend_hint_local)
            bend_world = (
                forward * bend_local.x
                + right * bend_local.y
            )
            target = override_targets.get(
                mount.leg_id,
                runtime.foot_world,
            )
            result = self._solver.solve(
                hip_world,
                target,
                self._leg_config.upper_length,
                self._leg_config.lower_length,
                bend_world,
                None,
            )
            results[mount.leg_id] = self._select_latched_branch(
                mount.leg_id,
                result,
            )
        return results

    def _solve_leg_target(
        self,
        leg_id: str,
        target_world: pygame.Vector2,
        body: BodySnapshot,
    ) -> IKResult:
        runtime = next(
            runtime
            for runtime in self._runtimes
            if runtime.mount.leg_id == leg_id
        )
        mount = runtime.mount
        forward, right = heading_axes(body.heading)
        bend_local = pygame.Vector2(mount.bend_hint_local)
        bend_world = (
            forward * bend_local.x
            + right * bend_local.y
        )
        result = self._solver.solve(
            local_to_world(
                mount.hip_local,
                body.position,
                body.heading,
            ),
            target_world,
            self._leg_config.upper_length,
            self._leg_config.lower_length,
            bend_world,
            None,
        )
        return self._select_latched_branch(
            leg_id,
            result,
        )

    def _select_latched_branch(
        self,
        leg_id: str,
        result: IKResult,
    ) -> IKResult:
        """Keep the initial analytic branch stable; motion corrects anatomy."""
        preferred_branch = self._branch_preferences.get(leg_id)
        if preferred_branch is None or result.bend_branch == preferred_branch:
            return result
        selected_index = 0 if preferred_branch == 1 else 1
        return replace(
            result,
            knee=result.knee_candidates[selected_index],
            bend_branch=preferred_branch,
        )

    def _find_bone_intersections(
        self,
        results: dict[str, IKResult],
    ) -> tuple[BoneIntersection, ...]:
        segments: list[
            tuple[str, str, pygame.Vector2, pygame.Vector2]
        ] = []
        for leg_id, result in results.items():
            segments.extend(
                (
                    (
                        leg_id,
                        "upper",
                        pygame.Vector2(result.hip),
                        pygame.Vector2(result.knee),
                    ),
                    (
                        leg_id,
                        "lower",
                        pygame.Vector2(result.knee),
                        pygame.Vector2(result.foot),
                    ),
                )
            )

        intersections: list[BoneIntersection] = []
        for first_index, first in enumerate(segments):
            for second in segments[first_index + 1 :]:
                if first[0] == second[0]:
                    continue
                point = _proper_segment_intersection(
                    first[2],
                    first[3],
                    second[2],
                    second[3],
                    self._landing_config.intersection_epsilon,
                )
                if point is not None:
                    intersections.append(
                        BoneIntersection(
                            first_leg_id=first[0],
                            first_segment=first[1],
                            second_leg_id=second[0],
                            second_segment=second[1],
                            point_world=(point.x, point.y),
                        )
                    )
        return tuple(intersections)

    def _find_bone_clearance_violations(
        self,
        results: dict[str, IKResult],
    ) -> tuple[BoneClearanceViolation, ...]:
        segments = self._bone_segments(results)
        violations: list[BoneClearanceViolation] = []
        for first_index, first in enumerate(segments):
            for second in segments[first_index + 1 :]:
                if first[0] == second[0]:
                    continue
                clearance = self._landing_config.bone_clearance
                if (
                    max(
                        min(first[2].x, first[3].x),
                        min(second[2].x, second[3].x),
                    )
                    - min(
                        max(first[2].x, first[3].x),
                        max(second[2].x, second[3].x),
                    )
                    >= clearance
                    or max(
                        min(first[2].y, first[3].y),
                        min(second[2].y, second[3].y),
                    )
                    - min(
                        max(first[2].y, first[3].y),
                        max(second[2].y, second[3].y),
                    )
                    >= clearance
                ):
                    continue
                first_point, second_point = _segment_closest_points(
                    first[2],
                    first[3],
                    second[2],
                    second[3],
                    self._landing_config.intersection_epsilon,
                )
                distance = first_point.distance_to(second_point)
                if distance >= self._landing_config.bone_clearance:
                    continue
                midpoint = (first_point + second_point) * 0.5
                violations.append(
                    BoneClearanceViolation(
                        first_leg_id=first[0],
                        first_segment=first[1],
                        second_leg_id=second[0],
                        second_segment=second[1],
                        distance=distance,
                        point_world=(midpoint.x, midpoint.y),
                    )
                )
        return tuple(violations)

    @staticmethod
    def _bone_segments(
        results: dict[str, IKResult],
    ) -> tuple[
        tuple[str, str, pygame.Vector2, pygame.Vector2],
        ...,
    ]:
        segments: list[
            tuple[str, str, pygame.Vector2, pygame.Vector2]
        ] = []
        for leg_id, result in results.items():
            segments.extend(
                (
                    (
                        leg_id,
                        "upper",
                        pygame.Vector2(result.hip),
                        pygame.Vector2(result.knee),
                    ),
                    (
                        leg_id,
                        "lower",
                        pygame.Vector2(result.knee),
                        pygame.Vector2(result.foot),
                    ),
                )
            )
        return tuple(segments)

    def _find_body_penetrations(
        self,
        results: dict[str, IKResult],
        body: BodySnapshot,
    ) -> tuple[BodyPenetration, ...]:
        half_length = (
            self._body_config.half_length
            + self._landing_config.knee_body_clearance
        )
        half_width = (
            self._body_config.half_width
            + self._landing_config.knee_body_clearance
        )
        penetrations: list[BodyPenetration] = []
        for leg_id, segment, start_world, end_world in (
            self._bone_segments(results)
        ):
            start = world_to_local(
                start_world,
                body.position,
                body.heading,
            )
            end = world_to_local(
                end_world,
                body.position,
                body.heading,
            )
            scaled_start = pygame.Vector2(
                start.x / half_length,
                start.y / half_width,
            )
            scaled_end = pygame.Vector2(
                end.x / half_length,
                end.y / half_width,
            )
            direction = scaled_end - scaled_start
            minimum_parameter = 0.08 if segment == "upper" else 0.0
            if direction.length_squared() <= 1e-18:
                parameter = minimum_parameter
            else:
                parameter = max(
                    minimum_parameter,
                    min(
                        1.0,
                        -scaled_start.dot(direction)
                        / direction.length_squared(),
                    ),
                )
            scaled_point = scaled_start + direction * parameter
            normalized_distance = scaled_point.length()
            start_distance = scaled_start.length()
            if (
                normalized_distance >= 1.0
                or (
                    segment == "upper"
                    and start_distance < 1.0
                    and normalized_distance >= start_distance - 0.02
                )
            ):
                continue
            point_world = start_world.lerp(end_world, parameter)
            penetrations.append(
                BodyPenetration(
                    leg_id=leg_id,
                    segment=segment,
                    normalized_distance=normalized_distance,
                    point_world=(point_world.x, point_world.y),
                )
            )
        return tuple(penetrations)

    def _current_spacing_violations(
        self,
    ) -> tuple[FootSpacingViolation, ...]:
        positions = {
            runtime.mount.leg_id: runtime.foot_world
            for runtime in self._runtimes
        }
        violations: list[FootSpacingViolation] = []
        leg_ids = tuple(positions)
        for first_index, first_id in enumerate(leg_ids):
            for second_id in leg_ids[first_index + 1 :]:
                distance = positions[first_id].distance_to(
                    positions[second_id]
                )
                if distance < self._landing_config.min_foot_spacing:
                    violations.append(
                        FootSpacingViolation(
                            first_leg_id=first_id,
                            second_leg_id=second_id,
                            distance=distance,
                        )
                    )
        return tuple(violations)

    def _activity_regions(
        self,
        body: BodySnapshot,
    ) -> tuple[ActivityRegion, ...]:
        minimum_abs_y = (
            self._body_config.half_width
            + self._landing_config.safe_midline_margin
        )
        maximum_abs_y = (
            self._body_config.half_width
            + self._leg_config.stance_lateral
            + self._landing_config.safe_outer_padding
        )
        regions: list[ActivityRegion] = []
        for mount in self.mounts:
            center_x = mount.default_foot_local[0]
            x_min = center_x - self._landing_config.channel_half_width
            x_max = center_x + self._landing_config.channel_half_width
            near_y = float(mount.side) * minimum_abs_y
            far_y = float(mount.side) * maximum_abs_y
            world_points = tuple(
                local_to_world(
                    point,
                    body.position,
                    body.heading,
                )
                for point in (
                    (x_min, near_y),
                    (x_max, near_y),
                    (x_max, far_y),
                    (x_min, far_y),
                )
            )
            regions.append(
                ActivityRegion(
                    leg_id=mount.leg_id,
                    local_x_min=x_min,
                    local_x_max=x_max,
                    local_abs_y_min=minimum_abs_y,
                    local_abs_y_max=maximum_abs_y,
                    polygon_world=tuple(
                        (point.x, point.y)
                        for point in world_points
                    ),  # type: ignore[arg-type]
                )
            )
        return tuple(regions)

    def _anatomy_violations(
        self,
        body: BodySnapshot,
        results: dict[str, IKResult],
    ) -> tuple[AnatomyViolation, ...]:
        violations: list[AnatomyViolation] = []
        for runtime in self._runtimes:
            mount = runtime.mount
            result = results[mount.leg_id]
            knee_lateral = self._signed_lateral(
                mount,
                result.knee,
                body,
            )
            if knee_lateral < self._required_knee_lateral:
                violations.append(
                    AnatomyViolation(
                        leg_id=mount.leg_id,
                        point_kind="knee_body_corridor",
                        signed_lateral=knee_lateral,
                        required_lateral=self._required_knee_lateral,
                        point_world=result.knee,
                    )
                )
            foot_lateral = self._signed_lateral(
                mount,
                runtime.foot_world,
                body,
            )
            if foot_lateral < 0.0:
                violations.append(
                    AnatomyViolation(
                        leg_id=mount.leg_id,
                        point_kind="foot_cross_midline",
                        signed_lateral=foot_lateral,
                        required_lateral=0.0,
                        point_world=(
                            runtime.foot_world.x,
                            runtime.foot_world.y,
                        ),
                    )
                )
        return tuple(violations)

    def solve(self, body: BodySnapshot) -> ConstrainedGaitSnapshot:
        """Solve with previous-knee continuity and build M7 diagnostics."""
        results = self._solve_targets(body)
        anomalies: list[BendAnomaly] = []
        if self._collecting_history:
            for leg_id, result in results.items():
                previous_knee = self._previous_knees.get(leg_id)
                previous_branch = self._previous_branches.get(leg_id)
                knee_jump = (
                    0.0
                    if previous_knee is None
                    else pygame.Vector2(previous_knee).distance_to(
                        result.knee
                    )
                )
                branch_changed = (
                    previous_branch is not None
                    and previous_branch != result.bend_branch
                )
                alternate_candidate = (
                    result.knee_candidates[1]
                    if result.bend_branch == 1
                    else result.knee_candidates[0]
                )
                alternate_knee_jump = (
                    0.0
                    if previous_knee is None
                    else pygame.Vector2(previous_knee).distance_to(
                        alternate_candidate
                    )
                )
                if knee_jump > self._landing_config.max_knee_jump:
                    anomalies.append(
                        BendAnomaly(
                            leg_id=leg_id,
                            branch_changed=branch_changed,
                            knee_jump=knee_jump,
                            alternate_knee_jump=alternate_knee_jump,
                        )
                    )
                self._previous_knees[leg_id] = result.knee
                self._previous_branches[leg_id] = result.bend_branch
                self._branch_preferences.setdefault(
                    leg_id,
                    result.bend_branch,
                )
            self._last_bend_anomalies = tuple(anomalies)
            self._total_bend_anomalies += len(anomalies)

        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        snapshots: list[SteppingLegSnapshot] = []
        for runtime in self._runtimes:
            mount = runtime.mount
            default_target = local_to_world(
                mount.default_foot_local,
                body.position,
                body.heading,
            )
            hip_world = local_to_world(
                mount.hip_local,
                body.position,
                body.heading,
            )
            result = results[mount.leg_id]
            snapshots.append(
                SteppingLegSnapshot(
                    mount=mount,
                    state=runtime.state,
                    foot_world=(
                        runtime.foot_world.x,
                        runtime.foot_world.y,
                    ),
                    foot_height=runtime.foot_height,
                    default_target_world=(
                        default_target.x,
                        default_target.y,
                    ),
                    step_target_world=(
                        None
                        if runtime.step_target_world is None
                        else (
                            runtime.step_target_world.x,
                            runtime.step_target_world.y,
                        )
                    ),
                    state_progress=self._state_progress(runtime),
                    distance_to_default=runtime.foot_world.distance_to(
                        default_target
                    ),
                    extension_ratio=hip_world.distance_to(
                        runtime.foot_world
                    ) / maximum_reach,
                    step_count=runtime.step_count,
                    ik=result,
                )
            )

        active_leg_ids: tuple[str, ...] = ()
        if self._active_group_index is not None:
            active_leg_ids = tuple(
                self._runtimes[index].mount.leg_id
                for index in self._group_indices[
                    self._active_group_index
                ]
            )
        return ConstrainedGaitSnapshot(
            body=body,
            legs=tuple(snapshots),
            active_group_index=self._active_group_index,
            active_leg_ids=active_leg_ids,
            next_group_index=self._next_group_index,
            group_start_counts=(
                self._group_start_counts[0],
                self._group_start_counts[1],
                self._group_start_counts[2],
                self._group_start_counts[3],
            ),
            activity_regions=self._activity_regions(
                self._last_candidate_body or body
            ),
            candidate_region_body=self._last_candidate_body,
            candidate_points=self._last_candidates,
            spacing_violations=self._current_spacing_violations(),
            bone_intersections=self._find_bone_intersections(results),
            bone_clearance_violations=(
                self._find_bone_clearance_violations(results)
            ),
            body_penetrations=self._find_body_penetrations(
                results,
                body,
            ),
            landing_decision=self._last_decision,
            deferred_group_indices=self._last_deferred_groups,
            total_deferred_groups=self._total_deferred_groups,
            total_degraded_groups=self._total_degraded_groups,
            landing_violation=self._last_landing_violation,
            total_landing_violations=self._total_landing_violations,
            total_spacing_violation_samples=(
                self._total_spacing_violation_samples
            ),
            total_bone_intersection_samples=(
                self._total_bone_intersection_samples
            ),
            total_bone_clearance_violation_samples=(
                self._total_bone_clearance_violation_samples
            ),
            total_body_penetration_samples=(
                self._total_body_penetration_samples
            ),
            bend_anomalies=self._last_bend_anomalies,
            total_bend_anomalies=self._total_bend_anomalies,
            anatomy_violations=self._anatomy_violations(body, results),
            total_anatomy_violation_samples=(
                self._total_anatomy_violation_samples
            ),
        )
