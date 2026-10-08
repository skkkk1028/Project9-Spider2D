"""Small, testable two-dimensional math helpers."""

from __future__ import annotations

import math
from typing import TypeAlias

import pygame

VectorLike: TypeAlias = pygame.Vector2 | tuple[float, float]


def finite_vector(value: VectorLike, *, name: str = "vector") -> pygame.Vector2:
    """Return a finite copy of a two-dimensional vector-like value."""
    try:
        vector = pygame.Vector2(value)
    except (TypeError, ValueError, IndexError) as error:
        raise ValueError(f"{name} must contain two real coordinates") from error

    if not math.isfinite(vector.x) or not math.isfinite(vector.y):
        raise ValueError(f"{name} coordinates must be finite")
    return vector


def move_towards(
    current: pygame.Vector2,
    target: pygame.Vector2,
    max_delta: float,
) -> pygame.Vector2:
    """Move a vector toward another without exceeding ``max_delta``."""
    if not math.isfinite(max_delta) or max_delta < 0.0:
        raise ValueError("max_delta must be finite and non-negative")

    difference = target - current
    distance_squared = difference.length_squared()
    if distance_squared == 0.0 or distance_squared <= max_delta * max_delta:
        return target.copy()
    return current + difference * (max_delta / math.sqrt(distance_squared))


def wrap_angle(angle: float) -> float:
    """Wrap an angle in radians to the half-open interval [-pi, pi)."""
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return (angle + math.pi) % math.tau - math.pi


def smooth_angle(
    current: float,
    target: float,
    response: float,
    max_angular_speed: float,
    delta_seconds: float,
) -> float:
    """Move an angle toward a target using shortest-path exponential response."""
    values = {
        "current": current,
        "target": target,
        "response": response,
        "max_angular_speed": max_angular_speed,
        "delta_seconds": delta_seconds,
    }
    if any(not math.isfinite(value) for value in values.values()):
        raise ValueError("angle smoothing inputs must be finite")
    if response <= 0.0 or max_angular_speed <= 0.0:
        raise ValueError("angle response and speed must be positive")
    if delta_seconds <= 0.0:
        raise ValueError("delta_seconds must be positive")

    difference = wrap_angle(target - current)
    exponential_step = difference * (1.0 - math.exp(-response * delta_seconds))
    max_step = max_angular_speed * delta_seconds
    limited_step = max(-max_step, min(exponential_step, max_step))
    return wrap_angle(current + limited_step)


def heading_axes(heading: float) -> tuple[pygame.Vector2, pygame.Vector2]:
    """Return body-local forward and right axes in Pygame world coordinates."""
    if not math.isfinite(heading):
        raise ValueError("heading must be finite")
    forward = pygame.Vector2(math.cos(heading), math.sin(heading))
    right = pygame.Vector2(-math.sin(heading), math.cos(heading))
    return forward, right


def local_to_world(
    local_point: VectorLike,
    origin_world: VectorLike,
    heading: float,
) -> pygame.Vector2:
    """Transform a body-local point into Pygame world coordinates."""
    local = finite_vector(local_point, name="local_point")
    origin = finite_vector(origin_world, name="origin_world")
    forward, right = heading_axes(heading)
    return origin + forward * local.x + right * local.y


def world_to_local(
    world_point: VectorLike,
    origin_world: VectorLike,
    heading: float,
) -> pygame.Vector2:
    """Transform a Pygame world point into body-local coordinates."""
    world = finite_vector(world_point, name="world_point")
    origin = finite_vector(origin_world, name="origin_world")
    forward, right = heading_axes(heading)
    offset = world - origin
    return pygame.Vector2(offset.dot(forward), offset.dot(right))

