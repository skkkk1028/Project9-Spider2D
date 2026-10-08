"""Pygame rendering and debug visualization for Milestone 2."""

from __future__ import annotations

import math
from dataclasses import dataclass

import pygame

from spider_sim.config import AppConfig
from spider_sim.math2d import heading_axes, local_to_world
from spider_sim.simulation import SimulationSnapshot

_BODY_POLYGON_SEGMENTS = 32


@dataclass(frozen=True, slots=True)
class FrameDebugInfo:
    """Per-render-frame timing values supplied by the application."""

    frames_per_second: float
    raw_delta_seconds: float
    substep_count: int


def _screen_point(point: pygame.Vector2 | tuple[float, float]) -> tuple[int, int]:
    vector = pygame.Vector2(point)
    return round(vector.x), round(vector.y)


class Renderer:
    """Draw a simulation snapshot without mutating simulation state."""

    def __init__(
        self,
        surface: pygame.Surface,
        config: AppConfig,
    ) -> None:
        self._surface = surface
        self._config = config
        self._font = pygame.font.Font(None, config.debug.font_size)

    def draw(
        self,
        snapshot: SimulationSnapshot,
        frame_info: FrameDebugInfo,
        *,
        debug_enabled: bool,
        clear_surface: bool = True,
        draw_hud: bool = True,
    ) -> None:
        """Draw the body and optional debug overlay."""
        if clear_surface:
            self._surface.fill(self._config.window.background_color)
        self._draw_target(snapshot.target_world)
        self._draw_body(snapshot)

        if debug_enabled:
            self._draw_debug_geometry(snapshot)
            if draw_hud:
                self._draw_hud(snapshot, frame_info)

    def _draw_body(self, snapshot: SimulationSnapshot) -> None:
        body = snapshot.body
        body_config = self._config.body
        points: list[tuple[int, int]] = []

        for index in range(_BODY_POLYGON_SEGMENTS):
            angle = math.tau * index / _BODY_POLYGON_SEGMENTS
            local_point = (
                math.cos(angle) * body_config.half_length,
                math.sin(angle) * body_config.half_width,
            )
            world_point = local_to_world(
                local_point,
                body.position,
                body.heading,
            )
            points.append(_screen_point(world_point))

        pygame.draw.polygon(
            self._surface,
            self._config.debug.body_fill_color,
            points,
        )
        pygame.draw.polygon(
            self._surface,
            self._config.debug.body_outline_color,
            points,
            width=2,
        )

        front = local_to_world(
            (body_config.half_length * 0.72, 0.0),
            body.position,
            body.heading,
        )
        pygame.draw.circle(
            self._surface,
            self._config.debug.body_front_color,
            _screen_point(front),
            5,
        )

    def _draw_target(self, target: tuple[float, float]) -> None:
        debug = self._config.debug
        center = _screen_point(target)
        pygame.draw.circle(
            self._surface,
            debug.target_color,
            center,
            debug.target_radius,
            width=2,
        )
        pygame.draw.line(
            self._surface,
            debug.target_color,
            (center[0] - debug.crosshair_radius, center[1]),
            (center[0] + debug.crosshair_radius, center[1]),
            width=1,
        )
        pygame.draw.line(
            self._surface,
            debug.target_color,
            (center[0], center[1] - debug.crosshair_radius),
            (center[0], center[1] + debug.crosshair_radius),
            width=1,
        )

    def _draw_debug_geometry(self, snapshot: SimulationSnapshot) -> None:
        body = snapshot.body
        debug = self._config.debug
        position = pygame.Vector2(body.position)
        velocity = pygame.Vector2(body.velocity)
        target = pygame.Vector2(snapshot.target_world)
        forward, right = heading_axes(body.heading)

        pygame.draw.line(
            self._surface,
            debug.target_line_color,
            _screen_point(position),
            _screen_point(target),
            width=1,
        )
        pygame.draw.line(
            self._surface,
            debug.forward_axis_color,
            _screen_point(position),
            _screen_point(position + forward * debug.axis_forward_length),
            width=3,
        )
        pygame.draw.line(
            self._surface,
            debug.right_axis_color,
            _screen_point(position),
            _screen_point(position + right * debug.axis_right_length),
            width=3,
        )
        pygame.draw.line(
            self._surface,
            debug.velocity_color,
            _screen_point(position),
            _screen_point(
                position + velocity * debug.velocity_vector_scale
            ),
            width=3,
        )

    def _draw_hud(
        self,
        snapshot: SimulationSnapshot,
        frame_info: FrameDebugInfo,
    ) -> None:
        body = snapshot.body
        position = pygame.Vector2(body.position)
        velocity = pygame.Vector2(body.velocity)
        target = pygame.Vector2(snapshot.target_world)
        lines = (
            "Milestone 2 | F1: debug | Esc: quit",
            f"FPS: {frame_info.frames_per_second:6.1f}",
            f"raw dt: {frame_info.raw_delta_seconds * 1000.0:6.2f} ms",
            f"substeps: {frame_info.substep_count}",
            f"sim time: {snapshot.elapsed_seconds:7.2f} s",
            f"position: ({position.x:7.2f}, {position.y:7.2f})",
            f"target:   ({target.x:7.2f}, {target.y:7.2f})",
            f"velocity: ({velocity.x:7.2f}, {velocity.y:7.2f})",
            f"speed: {velocity.length():7.2f} px/s",
            f"heading: {math.degrees(body.heading):7.2f} deg",
            f"target error: {position.distance_to(target):7.2f} px",
        )

        x = self._config.debug.hud_margin
        y = self._config.debug.hud_margin
        line_height = self._font.get_linesize()
        for line in lines:
            text_surface = self._font.render(
                line,
                True,
                self._config.debug.hud_text_color,
            )
            self._surface.blit(text_surface, (x, y))
            y += line_height
