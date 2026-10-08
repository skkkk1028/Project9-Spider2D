"""Frame-rate-independent simulation primitives."""

from __future__ import annotations

import math
from dataclasses import dataclass

from spider_sim.config import BodyConfig, TimingConfig
from spider_sim.math2d import VectorLike, finite_vector
from spider_sim.spider import BodySnapshot, SpiderBody


def split_delta_time(
    frame_seconds: float,
    timing: TimingConfig,
) -> tuple[float, ...]:
    """Clamp a frame delta and divide it into stable simulation substeps.

    The returned substeps add up to ``min(frame_seconds, max_frame_seconds)``.
    A zero delta produces no simulation step.
    """
    if isinstance(frame_seconds, bool) or not isinstance(
        frame_seconds, (int, float)
    ):
        raise ValueError("frame_seconds must be a real number")
    if not math.isfinite(frame_seconds) or frame_seconds < 0.0:
        raise ValueError("frame_seconds must be finite and non-negative")

    simulated_seconds = min(float(frame_seconds), timing.max_frame_seconds)
    if simulated_seconds == 0.0:
        return ()

    substep_count = math.ceil(
        simulated_seconds / timing.max_substep_seconds
    )
    if substep_count > timing.max_substeps_per_frame:
        raise RuntimeError("delta time exceeded the configured substep limit")

    substep_seconds = simulated_seconds / substep_count
    return (substep_seconds,) * substep_count


@dataclass(frozen=True, slots=True)
class SimulationSnapshot:
    """Read-only data exposed by the simulation to other layers."""

    elapsed_seconds: float
    update_count: int
    target_world: tuple[float, float]
    body: BodySnapshot


class Simulation:
    """Own and update the Milestone 2 spider body state."""

    def __init__(
        self,
        body_config: BodyConfig | None = None,
        initial_position: VectorLike = (0.0, 0.0),
    ) -> None:
        self._elapsed_seconds = 0.0
        self._update_count = 0
        self._body = SpiderBody(
            body_config or BodyConfig(),
            initial_position,
        )
        self._target_world = self._body.position

    def set_target(self, target_world: VectorLike) -> None:
        """Store the world-space target used by subsequent substeps."""
        self._target_world = finite_vector(
            target_world,
            name="target_world",
        )

    def update(self, delta_seconds: float) -> None:
        """Advance the simulation by one already-bounded substep."""
        if isinstance(delta_seconds, bool) or not isinstance(
            delta_seconds, (int, float)
        ):
            raise ValueError("delta_seconds must be a real number")
        if not math.isfinite(delta_seconds) or delta_seconds <= 0.0:
            raise ValueError("delta_seconds must be finite and positive")

        bounded_delta = float(delta_seconds)
        self._body.update(self._target_world, bounded_delta)
        self._elapsed_seconds += bounded_delta
        self._update_count += 1

    def snapshot(self) -> SimulationSnapshot:
        """Return an immutable view of the current state."""
        return SimulationSnapshot(
            elapsed_seconds=self._elapsed_seconds,
            update_count=self._update_count,
            target_world=(
                self._target_world.x,
                self._target_world.y,
            ),
            body=self._body.snapshot(),
        )
