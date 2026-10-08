"""Analytic two-dimensional Two-Bone IK solver."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import pygame

from spider_sim.math2d import VectorLike, finite_vector


class IKReachState(str, Enum):
    """Relationship between the requested target and the reachable annulus."""

    REACHABLE = "reachable"
    TOO_FAR = "too_far"
    TOO_CLOSE = "too_close"
    DEGENERATE = "degenerate"


@dataclass(frozen=True, slots=True)
class IKResult:
    """Immutable result of one analytic IK solve."""

    hip: tuple[float, float]
    knee: tuple[float, float]
    foot: tuple[float, float]
    requested_target: tuple[float, float]
    solved_target: tuple[float, float]
    knee_candidates: tuple[
        tuple[float, float],
        tuple[float, float],
    ]
    reach_state: IKReachState
    target_error: float
    bend_branch: int
    upper_length: float
    lower_length: float
    minimum_reach: float
    maximum_reach: float


def _validate_length(name: str, value: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


class TwoBoneIKSolver:
    """Solve a planar two-segment chain using circle intersection geometry."""

    def __init__(
        self,
        *,
        epsilon: float = 1e-6,
        continuity_weight: float = 3.0,
    ) -> None:
        self._epsilon = _validate_length("epsilon", epsilon)
        if (
            isinstance(continuity_weight, bool)
            or not isinstance(continuity_weight, (int, float))
            or not math.isfinite(continuity_weight)
            or continuity_weight < 0.0
        ):
            raise ValueError(
                "continuity_weight must be finite and non-negative"
            )
        self._continuity_weight = float(continuity_weight)

    def solve(
        self,
        hip_world: VectorLike,
        target_world: VectorLike,
        upper_length: float,
        lower_length: float,
        bend_hint_world: VectorLike,
        previous_knee_world: VectorLike | None = None,
    ) -> IKResult:
        """Return a stable analytic solution for one two-segment leg."""
        hip = finite_vector(hip_world, name="hip_world")
        requested_target = finite_vector(
            target_world,
            name="target_world",
        )
        upper = _validate_length("upper_length", upper_length)
        lower = _validate_length("lower_length", lower_length)
        if self._epsilon >= min(upper, lower):
            raise ValueError("epsilon must be smaller than both bone lengths")
        bend_hint = finite_vector(
            bend_hint_world,
            name="bend_hint_world",
        )
        bend_hint_length = math.hypot(bend_hint.x, bend_hint.y)
        if bend_hint_length == 0.0:
            raise ValueError("bend_hint_world must be non-zero")
        bend_hint /= bend_hint_length

        previous_knee = (
            None
            if previous_knee_world is None
            else finite_vector(
                previous_knee_world,
                name="previous_knee_world",
            )
        )

        to_target = requested_target - hip
        requested_distance = math.hypot(to_target.x, to_target.y)
        if not math.isfinite(requested_distance):
            raise ValueError("hip-to-target distance is too large")
        minimum_reach = abs(upper - lower)
        maximum_reach = upper + lower
        if not math.isfinite(maximum_reach):
            raise ValueError("combined bone length is too large")

        if requested_distance <= self._epsilon:
            reach_state = IKReachState.DEGENERATE
            # The target ray is undefined here. Rotating the bend hint by
            # -90 degrees gives a deterministic foot direction whose positive
            # knee candidate points toward the requested bend preference.
            direction = pygame.Vector2(bend_hint.y, -bend_hint.x)
            fallback_distance = min(self._epsilon, maximum_reach)
            solved_distance = max(minimum_reach, fallback_distance)
        else:
            direction = to_target / requested_distance
            if requested_distance > maximum_reach:
                reach_state = IKReachState.TOO_FAR
                solved_distance = maximum_reach
            elif requested_distance < minimum_reach:
                reach_state = IKReachState.TOO_CLOSE
                solved_distance = minimum_reach
            else:
                reach_state = IKReachState.REACHABLE
                solved_distance = requested_distance

        solved_target = hip + direction * solved_distance
        if not math.isfinite(solved_target.x) or not math.isfinite(
            solved_target.y
        ):
            raise ValueError("solved target coordinates are too large")

        geometry_scale = max(upper, lower, solved_distance)
        scaled_distance = solved_distance / geometry_scale
        length_difference = upper - lower
        length_sum = upper + lower
        along = 0.5 * (
            length_difference / solved_distance * length_sum
            + solved_distance
        )
        scaled_factor_1 = (length_sum - solved_distance) / geometry_scale
        scaled_factor_2 = (
            solved_distance + length_difference
        ) / geometry_scale
        scaled_factor_3 = (
            solved_distance - length_difference
        ) / geometry_scale
        scaled_factor_4 = (length_sum + solved_distance) / geometry_scale
        heron_product = max(
            scaled_factor_1
            * scaled_factor_2
            * scaled_factor_3
            * scaled_factor_4,
            0.0,
        )
        height = (
            math.sqrt(heron_product)
            / (2.0 * scaled_distance)
            * geometry_scale
        )

        base = hip + direction * along
        perpendicular = pygame.Vector2(-direction.y, direction.x)
        positive_candidate = base + perpendicular * height
        negative_candidate = base - perpendicular * height

        positive_score = self._candidate_score(
            positive_candidate,
            hip,
            bend_hint,
            previous_knee,
            upper,
        )
        negative_score = self._candidate_score(
            negative_candidate,
            hip,
            bend_hint,
            previous_knee,
            upper,
        )

        if positive_score >= negative_score:
            knee = positive_candidate
            bend_branch = 1
        else:
            knee = negative_candidate
            bend_branch = -1

        return IKResult(
            hip=(hip.x, hip.y),
            knee=(knee.x, knee.y),
            foot=(solved_target.x, solved_target.y),
            requested_target=(requested_target.x, requested_target.y),
            solved_target=(solved_target.x, solved_target.y),
            knee_candidates=(
                (positive_candidate.x, positive_candidate.y),
                (negative_candidate.x, negative_candidate.y),
            ),
            reach_state=reach_state,
            target_error=math.hypot(
                requested_target.x - solved_target.x,
                requested_target.y - solved_target.y,
            ),
            bend_branch=bend_branch,
            upper_length=upper,
            lower_length=lower,
            minimum_reach=minimum_reach,
            maximum_reach=maximum_reach,
        )

    def _candidate_score(
        self,
        candidate: pygame.Vector2,
        hip: pygame.Vector2,
        bend_hint: pygame.Vector2,
        previous_knee: pygame.Vector2 | None,
        upper_length: float,
    ) -> float:
        knee_direction = candidate - hip
        if knee_direction.length_squared() > self._epsilon**2:
            knee_direction.normalize_ip()
        hint_score = knee_direction.dot(bend_hint)

        if previous_knee is None:
            return hint_score

        normalized_jump = candidate.distance_to(previous_knee) / (
            2.0 * upper_length
        )
        return hint_score - self._continuity_weight * normalized_jump
