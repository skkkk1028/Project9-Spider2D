"""Spider body state and movement for Milestone 2."""

from __future__ import annotations

import math
from dataclasses import dataclass

import pygame

from spider_sim.config import BodyConfig
from spider_sim.math2d import (
    VectorLike,
    finite_vector,
    move_towards,
    smooth_angle,
    wrap_angle,
)


@dataclass(frozen=True, slots=True)
class BodySnapshot:
    """Immutable numeric view of the spider body."""

    position: tuple[float, float]
    velocity: tuple[float, float]
    heading: float


class SpiderBody:
    """Acceleration-limited body that arrives smoothly at a world target."""

    def __init__(
        self,
        config: BodyConfig,
        initial_position: VectorLike,
        *,
        initial_heading: float = 0.0,
    ) -> None:
        if not math.isfinite(initial_heading):
            raise ValueError("initial_heading must be finite")

        self._config = config
        self._position = finite_vector(
            initial_position,
            name="initial_position",
        )
        self._velocity = pygame.Vector2()
        self._heading = wrap_angle(initial_heading)

    @property
    def position(self) -> pygame.Vector2:
        """Return a copy so callers cannot mutate internal body state."""
        return self._position.copy()

    def update(self, target_world: VectorLike, delta_seconds: float) -> None:
        """Advance body position, velocity, and heading by one substep."""
        if (
            isinstance(delta_seconds, bool)
            or not isinstance(delta_seconds, (int, float))
            or not math.isfinite(delta_seconds)
            or delta_seconds <= 0.0
        ):
            raise ValueError("delta_seconds must be finite and positive")

        target = finite_vector(target_world, name="target_world")
        to_target = target - self._position
        distance = to_target.length()
        desired_velocity = pygame.Vector2()

        if distance > self._config.stop_radius:
            usable_distance = distance - self._config.stop_radius
            response_speed = usable_distance * self._config.position_response
            braking_speed = math.sqrt(
                2.0 * self._config.max_acceleration * usable_distance
            )
            desired_speed = min(
                self._config.max_speed,
                response_speed,
                braking_speed,
            )
            desired_velocity = to_target * (desired_speed / distance)

        previous_velocity = self._velocity.copy()
        self._velocity = move_towards(
            previous_velocity,
            desired_velocity,
            self._config.max_acceleration * delta_seconds,
        )
        displacement = (
            previous_velocity + self._velocity
        ) * (0.5 * delta_seconds)
        next_position = self._position + displacement

        if (
            distance > 0.0
            and displacement.length_squared() >= distance * distance
            and (target - next_position).dot(to_target) <= 0.0
            and previous_velocity.length()
            <= self._config.max_acceleration * delta_seconds
        ):
            self._position = target
            self._velocity.update(0.0, 0.0)
        else:
            self._position = next_position

        if (
            self._velocity.length_squared()
            >= self._config.heading_speed_threshold**2
        ):
            target_heading = math.atan2(self._velocity.y, self._velocity.x)
            self._heading = smooth_angle(
                self._heading,
                target_heading,
                self._config.heading_response,
                self._config.max_turn_speed,
                delta_seconds,
            )

    def snapshot(self) -> BodySnapshot:
        """Return immutable body data for rendering and tests."""
        return BodySnapshot(
            position=(self._position.x, self._position.y),
            velocity=(self._velocity.x, self._velocity.y),
            heading=self._heading,
        )
