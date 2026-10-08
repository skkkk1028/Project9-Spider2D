"""World-locked foot trajectories for Milestones 5 and 6."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import pygame

from spider_sim.config import (
    BodyConfig,
    FootStepConfig,
    FourPhaseGaitConfig,
    LegConfig,
)
from spider_sim.ik import IKResult, TwoBoneIKSolver
from spider_sim.leg import LegMount, StaticLegAssembly
from spider_sim.math2d import heading_axes, local_to_world
from spider_sim.spider import BodySnapshot


class FootState(str, Enum):
    """Ground-contact state for one foot."""

    PLANTED = "planted"
    LIFT = "lift"
    SWING = "swing"
    LOWER = "lower"


@dataclass(slots=True)
class _LegRuntime:
    """Mutable state owned by the stepping assembly."""

    mount: LegMount
    foot_world: pygame.Vector2
    state: FootState = FootState.PLANTED
    state_elapsed: float = 0.0
    foot_height: float = 0.0
    swing_start_world: pygame.Vector2 | None = None
    step_target_world: pygame.Vector2 | None = None
    step_count: int = 0


@dataclass(frozen=True, slots=True)
class SteppingLegSnapshot:
    """Immutable debug and rendering view of one M5 leg."""

    mount: LegMount
    state: FootState
    foot_world: tuple[float, float]
    foot_height: float
    default_target_world: tuple[float, float]
    step_target_world: tuple[float, float] | None
    state_progress: float
    distance_to_default: float
    extension_ratio: float
    step_count: int
    ik: IKResult


@dataclass(frozen=True, slots=True)
class SteppingAssemblySnapshot:
    """All eight world-locked feet solved against one body pose."""

    body: BodySnapshot
    legs: tuple[SteppingLegSnapshot, ...]
    active_leg_id: str | None


@dataclass(frozen=True, slots=True)
class GroupedGaitSnapshot:
    """Immutable view of the M6 fixed four-group scheduler."""

    body: BodySnapshot
    legs: tuple[SteppingLegSnapshot, ...]
    active_group_index: int | None
    active_leg_ids: tuple[str, ...]
    next_group_index: int
    group_start_counts: tuple[int, int, int, int]

    @property
    def support_count(self) -> int:
        """Return the number of currently planted support legs."""
        return sum(leg.state is FootState.PLANTED for leg in self.legs)


def _smoothstep(progress: float) -> float:
    clamped = max(0.0, min(float(progress), 1.0))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def _validate_delta(delta_seconds: float) -> float:
    if (
        isinstance(delta_seconds, bool)
        or not isinstance(delta_seconds, (int, float))
        or not math.isfinite(delta_seconds)
        or delta_seconds <= 0.0
    ):
        raise ValueError("delta_seconds must be finite and positive")
    return float(delta_seconds)


class SingleLegStepAssembly:
    """Lock planted feet in world space and move at most one foot at a time."""

    def __init__(
        self,
        body_config: BodyConfig,
        leg_config: LegConfig,
        step_config: FootStepConfig,
        initial_body: BodySnapshot,
        solver: TwoBoneIKSolver | None = None,
    ) -> None:
        self._body_config = body_config
        self._leg_config = leg_config
        self._step_config = step_config
        self._solver = solver or TwoBoneIKSolver()
        static_assembly = StaticLegAssembly(
            body_config,
            leg_config,
            self._solver,
        )
        self._runtimes = [
            _LegRuntime(
                mount=mount,
                foot_world=local_to_world(
                    mount.default_foot_local,
                    initial_body.position,
                    initial_body.heading,
                ),
            )
            for mount in static_assembly.mounts
        ]
        self._active_index: int | None = None
        self._validate_default_extension()

    @property
    def mounts(self) -> tuple[LegMount, ...]:
        """Return M4's unchanged immutable mount definitions."""
        return tuple(runtime.mount for runtime in self._runtimes)

    @property
    def active_leg_id(self) -> str | None:
        """Return the only active leg ID, if a step is in progress."""
        if self._active_index is None:
            return None
        return self._runtimes[self._active_index].mount.leg_id

    def _validate_default_extension(self) -> None:
        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        for runtime in self._runtimes:
            default_distance = math.dist(
                runtime.mount.hip_local,
                runtime.mount.default_foot_local,
            )
            if (
                default_distance / maximum_reach
                > self._step_config.max_extension_ratio
            ):
                raise ValueError(
                    f"{runtime.mount.leg_id} default pose exceeds "
                    "max_extension_ratio"
                )

    def update(
        self,
        body: BodySnapshot,
        delta_seconds: float,
    ) -> SteppingAssemblySnapshot:
        """Advance one active foot, starting one when a planted foot is stale."""
        delta = _validate_delta(delta_seconds)
        if self._active_index is None:
            self._try_start_step(body)
        if self._active_index is not None:
            self._advance_active(delta)
        return self.solve(body)

    def _try_start_step(self, body: BodySnapshot) -> None:
        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        best_index: int | None = None
        best_urgency = 1.0

        for index, runtime in enumerate(self._runtimes):
            default_target = self._default_target(runtime.mount, body)
            hip_world = local_to_world(
                runtime.mount.hip_local,
                body.position,
                body.heading,
            )
            default_error = runtime.foot_world.distance_to(default_target)
            extension_ratio = hip_world.distance_to(
                runtime.foot_world
            ) / maximum_reach
            urgency = max(
                default_error / self._step_config.trigger_distance,
                extension_ratio / self._step_config.max_extension_ratio,
            )
            if urgency > best_urgency:
                best_urgency = urgency
                best_index = index

        if best_index is None:
            return

        runtime = self._runtimes[best_index]
        runtime.state = FootState.LIFT
        runtime.state_elapsed = 0.0
        runtime.foot_height = 0.0
        runtime.swing_start_world = runtime.foot_world.copy()
        runtime.step_target_world = self._default_target(runtime.mount, body)
        self._active_index = best_index

    def _advance_active(self, delta_seconds: float) -> None:
        if self._active_index is None:
            return

        runtime = self._runtimes[self._active_index]
        remaining = delta_seconds

        while remaining > 0.0 and runtime.state is not FootState.PLANTED:
            duration = self._state_duration(runtime.state)
            available = max(duration - runtime.state_elapsed, 0.0)
            consumed = min(remaining, available)
            runtime.state_elapsed += consumed
            remaining -= consumed
            self._apply_state(runtime, duration)

            if runtime.state_elapsed >= duration:
                self._complete_state(runtime)

        if runtime.state is FootState.PLANTED:
            self._active_index = None

    def _state_duration(self, state: FootState) -> float:
        if state is FootState.LIFT:
            return self._step_config.lift_duration
        if state is FootState.SWING:
            return self._step_config.swing_duration
        if state is FootState.LOWER:
            return self._step_config.lower_duration
        raise RuntimeError("PLANTED has no active-state duration")

    def _apply_state(
        self,
        runtime: _LegRuntime,
        duration: float,
    ) -> None:
        progress = _smoothstep(runtime.state_elapsed / duration)
        if runtime.state is FootState.LIFT:
            runtime.foot_height = self._step_config.lift_height * progress
            return
        if runtime.state is FootState.SWING:
            if (
                runtime.swing_start_world is None
                or runtime.step_target_world is None
            ):
                raise RuntimeError("SWING requires start and target positions")
            runtime.foot_world = runtime.swing_start_world.lerp(
                runtime.step_target_world,
                progress,
            )
            runtime.foot_height = self._step_config.lift_height
            return
        if runtime.state is FootState.LOWER:
            runtime.foot_height = self._step_config.lift_height * (
                1.0 - progress
            )

    def _complete_state(self, runtime: _LegRuntime) -> None:
        if runtime.state is FootState.LIFT:
            runtime.state = FootState.SWING
            runtime.state_elapsed = 0.0
            runtime.foot_height = self._step_config.lift_height
            return
        if runtime.state is FootState.SWING:
            if runtime.step_target_world is None:
                raise RuntimeError("SWING completed without a target")
            runtime.foot_world = runtime.step_target_world.copy()
            runtime.state = FootState.LOWER
            runtime.state_elapsed = 0.0
            runtime.foot_height = self._step_config.lift_height
            return
        if runtime.state is FootState.LOWER:
            if runtime.step_target_world is None:
                raise RuntimeError("LOWER completed without a target")
            runtime.foot_world = runtime.step_target_world.copy()
            runtime.state = FootState.PLANTED
            runtime.state_elapsed = 0.0
            runtime.foot_height = 0.0
            runtime.swing_start_world = None
            runtime.step_target_world = None
            runtime.step_count += 1
            return
        raise RuntimeError("cannot complete PLANTED state")

    def solve(self, body: BodySnapshot) -> SteppingAssemblySnapshot:
        """Solve every leg to its stored world-space foot request."""
        forward, right = heading_axes(body.heading)
        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        snapshots: list[SteppingLegSnapshot] = []

        for runtime in self._runtimes:
            hip_world = local_to_world(
                runtime.mount.hip_local,
                body.position,
                body.heading,
            )
            default_target = self._default_target(runtime.mount, body)
            bend_local = pygame.Vector2(runtime.mount.bend_hint_local)
            bend_world = (
                forward * bend_local.x
                + right * bend_local.y
            )
            result = self._solver.solve(
                hip_world,
                runtime.foot_world,
                self._leg_config.upper_length,
                self._leg_config.lower_length,
                bend_world,
            )
            snapshots.append(
                SteppingLegSnapshot(
                    mount=runtime.mount,
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

        return SteppingAssemblySnapshot(
            body=body,
            legs=tuple(snapshots),
            active_leg_id=self.active_leg_id,
        )

    def _default_target(
        self,
        mount: LegMount,
        body: BodySnapshot,
    ) -> pygame.Vector2:
        return local_to_world(
            mount.default_foot_local,
            body.position,
            body.heading,
        )

    def _state_progress(self, runtime: _LegRuntime) -> float:
        if runtime.state is FootState.PLANTED:
            return 0.0
        return min(
            runtime.state_elapsed / self._state_duration(runtime.state),
            1.0,
        )


class FourPhaseGaitAssembly(SingleLegStepAssembly):
    """Run one fixed diagonal pair at a time using fair round-robin order."""

    def __init__(
        self,
        body_config: BodyConfig,
        leg_config: LegConfig,
        step_config: FootStepConfig,
        gait_config: FourPhaseGaitConfig,
        initial_body: BodySnapshot,
        solver: TwoBoneIKSolver | None = None,
    ) -> None:
        super().__init__(
            body_config,
            leg_config,
            step_config,
            initial_body,
            solver,
        )
        self._gait_config = gait_config
        self._group_indices = self._resolve_groups()
        self._active_group_index: int | None = None
        self._next_group_index = 0
        self._group_start_counts = [0, 0, 0, 0]

    @property
    def groups(self) -> tuple[
        tuple[str, str],
        tuple[str, str],
        tuple[str, str],
        tuple[str, str],
    ]:
        """Return the configured fixed four-phase leg groups."""
        return self._gait_config.groups

    @property
    def active_group_index(self) -> int | None:
        """Return the active group, if either of its two legs is moving."""
        return self._active_group_index

    def _resolve_groups(
        self,
    ) -> tuple[
        tuple[int, int],
        tuple[int, int],
        tuple[int, int],
        tuple[int, int],
    ]:
        id_to_index = {
            runtime.mount.leg_id: index
            for index, runtime in enumerate(self._runtimes)
        }
        configured_ids = {
            leg_id
            for group in self._gait_config.groups
            for leg_id in group
        }
        if configured_ids != set(id_to_index):
            raise ValueError(
                "four-phase groups must reference every assembly leg exactly"
            )

        resolved: list[tuple[int, int]] = []
        for group_index, group in enumerate(self._gait_config.groups):
            first_index = id_to_index[group[0]]
            second_index = id_to_index[group[1]]
            first = self._runtimes[first_index].mount
            second = self._runtimes[second_index].mount
            if first.side is second.side:
                raise ValueError(
                    f"group {group_index} must contain opposite sides"
                )
            if abs(first.row - second.row) < 2:
                raise ValueError(
                    f"group {group_index} legs must be non-adjacent"
                )
            resolved.append((first_index, second_index))

        return (
            resolved[0],
            resolved[1],
            resolved[2],
            resolved[3],
        )

    def update(
        self,
        body: BodySnapshot,
        delta_seconds: float,
    ) -> GroupedGaitSnapshot:
        """Advance the active pair or fairly start the next eligible group."""
        delta = _validate_delta(delta_seconds)
        if self._active_group_index is None:
            self._try_start_group(body)
        if self._active_group_index is not None:
            self._advance_active_group(delta)
        return self.solve(body)

    def _try_start_group(self, body: BodySnapshot) -> None:
        group_count = len(self._group_indices)
        for offset in range(group_count):
            group_index = (
                self._next_group_index + offset
            ) % group_count
            runtime_indices = self._group_indices[group_index]
            if not any(
                self._needs_step(self._runtimes[index], body)
                for index in runtime_indices
            ):
                continue

            for index in runtime_indices:
                self._start_runtime(self._runtimes[index], body)
            self._active_group_index = group_index
            self._next_group_index = (group_index + 1) % group_count
            self._group_start_counts[group_index] += 1
            return

    def _needs_step(
        self,
        runtime: _LegRuntime,
        body: BodySnapshot,
    ) -> bool:
        default_target = self._default_target(runtime.mount, body)
        hip_world = local_to_world(
            runtime.mount.hip_local,
            body.position,
            body.heading,
        )
        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        return (
            runtime.foot_world.distance_to(default_target)
            > self._step_config.trigger_distance
            or hip_world.distance_to(runtime.foot_world) / maximum_reach
            > self._step_config.max_extension_ratio
        )

    def _start_runtime(
        self,
        runtime: _LegRuntime,
        body: BodySnapshot,
    ) -> None:
        if runtime.state is not FootState.PLANTED:
            raise RuntimeError("cannot start an already active grouped leg")
        runtime.state = FootState.LIFT
        runtime.state_elapsed = 0.0
        runtime.foot_height = 0.0
        runtime.swing_start_world = runtime.foot_world.copy()
        runtime.step_target_world = self._default_target(
            runtime.mount,
            body,
        )

    def _advance_active_group(self, delta_seconds: float) -> None:
        if self._active_group_index is None:
            return

        runtime_indices = self._group_indices[self._active_group_index]
        for index in runtime_indices:
            self._advance_runtime(
                self._runtimes[index],
                delta_seconds,
            )

        states = {
            self._runtimes[index].state
            for index in runtime_indices
        }
        if states == {FootState.PLANTED}:
            self._active_group_index = None
        elif FootState.PLANTED in states:
            raise RuntimeError("grouped legs lost phase synchronization")

    def _advance_runtime(
        self,
        runtime: _LegRuntime,
        delta_seconds: float,
    ) -> None:
        remaining = delta_seconds
        while remaining > 0.0 and runtime.state is not FootState.PLANTED:
            duration = self._state_duration(runtime.state)
            available = max(duration - runtime.state_elapsed, 0.0)
            consumed = min(remaining, available)
            runtime.state_elapsed += consumed
            remaining -= consumed
            self._apply_state(runtime, duration)
            if runtime.state_elapsed >= duration:
                self._complete_state(runtime)

    def solve(self, body: BodySnapshot) -> GroupedGaitSnapshot:
        """Solve all M6 feet while exposing scheduler state."""
        base_snapshot = super().solve(body)
        active_leg_ids: tuple[str, ...] = ()
        if self._active_group_index is not None:
            active_leg_ids = tuple(
                self._runtimes[index].mount.leg_id
                for index in self._group_indices[
                    self._active_group_index
                ]
            )
        return GroupedGaitSnapshot(
            body=body,
            legs=base_snapshot.legs,
            active_group_index=self._active_group_index,
            active_leg_ids=active_leg_ids,
            next_group_index=self._next_group_index,
            group_start_counts=(
                self._group_start_counts[0],
                self._group_start_counts[1],
                self._group_start_counts[2],
                self._group_start_counts[3],
            ),
        )
