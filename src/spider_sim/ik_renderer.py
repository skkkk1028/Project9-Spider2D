"""Debug renderer for the standalone single-leg IK demo."""

from __future__ import annotations

import pygame

from spider_sim.config import AppConfig
from spider_sim.ik import IKResult
from spider_sim.math2d import VectorLike, finite_vector


def _screen_point(point: VectorLike) -> tuple[int, int]:
    vector = pygame.Vector2(point)
    return round(vector.x), round(vector.y)


class IKRenderer:
    """Render one analytic Two-Bone IK result without changing it."""

    def __init__(
        self,
        surface: pygame.Surface,
        config: AppConfig,
    ) -> None:
        self._surface = surface
        self._config = config
        self._font = pygame.font.Font(None, config.ik_debug.font_size)

    def draw(
        self,
        result: IKResult,
        bend_hint_world: VectorLike,
        previous_knee_world: VectorLike | None,
        *,
        frames_per_second: float,
        debug_enabled: bool,
    ) -> None:
        """Draw bones, targets, joints, and optional diagnostics."""
        self._surface.fill(self._config.window.background_color)

        hip = pygame.Vector2(result.hip)
        knee = pygame.Vector2(result.knee)
        foot = pygame.Vector2(result.foot)
        requested = pygame.Vector2(result.requested_target)
        bend_hint = finite_vector(
            bend_hint_world,
            name="bend_hint_world",
        ).normalize()

        if debug_enabled:
            self._draw_workspace(result)
            self._draw_target_line(hip, requested)
            self._draw_candidates(result)
            self._draw_bend_hint(hip, bend_hint)
            if previous_knee_world is not None:
                self._draw_previous_knee(previous_knee_world)

        self._draw_bones(hip, knee, foot)
        self._draw_joints(hip, knee, foot)
        self._draw_targets(result)

        if debug_enabled:
            self._draw_hud(
                result,
                frames_per_second,
                bend_hint,
            )

    def _draw_workspace(self, result: IKResult) -> None:
        color = self._config.ik_debug.reach_circle_color
        hip = pygame.Vector2(result.hip)
        pygame.draw.circle(
            self._surface,
            color,
            _screen_point(hip),
            round(result.maximum_reach),
            width=2,
        )
        if result.minimum_reach > 0.0:
            pygame.draw.circle(
                self._surface,
                color,
                _screen_point(hip),
                round(result.minimum_reach),
                width=2,
            )

    def _draw_target_line(
        self,
        hip: pygame.Vector2,
        requested: pygame.Vector2,
    ) -> None:
        pygame.draw.line(
            self._surface,
            self._config.debug.target_line_color,
            _screen_point(hip),
            _screen_point(requested),
            width=1,
        )

    def _draw_candidates(self, result: IKResult) -> None:
        for candidate in result.knee_candidates:
            pygame.draw.circle(
                self._surface,
                self._config.ik_debug.candidate_color,
                _screen_point(candidate),
                self._config.ik_debug.candidate_radius,
                width=2,
            )

    def _draw_bend_hint(
        self,
        hip: pygame.Vector2,
        bend_hint: pygame.Vector2,
    ) -> None:
        end = (
            hip
            + bend_hint * self._config.ik_debug.bend_hint_length
        )
        pygame.draw.line(
            self._surface,
            self._config.ik_debug.bend_hint_color,
            _screen_point(hip),
            _screen_point(end),
            width=3,
        )

    def _draw_previous_knee(self, previous_knee_world: VectorLike) -> None:
        previous = finite_vector(
            previous_knee_world,
            name="previous_knee_world",
        )
        pygame.draw.circle(
            self._surface,
            self._config.ik_debug.candidate_color,
            _screen_point(previous),
            self._config.ik_debug.joint_radius + 4,
            width=1,
        )

    def _draw_bones(
        self,
        hip: pygame.Vector2,
        knee: pygame.Vector2,
        foot: pygame.Vector2,
    ) -> None:
        width = self._config.ik_debug.bone_width
        pygame.draw.line(
            self._surface,
            self._config.ik_debug.upper_bone_color,
            _screen_point(hip),
            _screen_point(knee),
            width=width,
        )
        pygame.draw.line(
            self._surface,
            self._config.ik_debug.lower_bone_color,
            _screen_point(knee),
            _screen_point(foot),
            width=width,
        )

    def _draw_targets(self, result: IKResult) -> None:
        requested = _screen_point(result.requested_target)
        solved = _screen_point(result.solved_target)
        radius = self._config.ik_debug.target_radius

        pygame.draw.circle(
            self._surface,
            self._config.ik_debug.requested_target_color,
            requested,
            radius + 3,
            width=2,
        )
        pygame.draw.line(
            self._surface,
            self._config.ik_debug.requested_target_color,
            (requested[0] - radius, requested[1]),
            (requested[0] + radius, requested[1]),
            width=1,
        )
        pygame.draw.line(
            self._surface,
            self._config.ik_debug.requested_target_color,
            (requested[0], requested[1] - radius),
            (requested[0], requested[1] + radius),
            width=1,
        )
        pygame.draw.circle(
            self._surface,
            self._config.ik_debug.solved_target_color,
            solved,
            radius,
            width=3,
        )

    def _draw_joints(
        self,
        hip: pygame.Vector2,
        knee: pygame.Vector2,
        foot: pygame.Vector2,
    ) -> None:
        for joint in (hip, knee, foot):
            pygame.draw.circle(
                self._surface,
                self._config.ik_debug.joint_color,
                _screen_point(joint),
                self._config.ik_debug.joint_radius,
            )

    def _draw_hud(
        self,
        result: IKResult,
        frames_per_second: float,
        bend_hint: pygame.Vector2,
    ) -> None:
        hip = pygame.Vector2(result.hip)
        knee = pygame.Vector2(result.knee)
        foot = pygame.Vector2(result.foot)
        upper_error = abs(
            hip.distance_to(knee) - result.upper_length
        )
        lower_error = abs(
            knee.distance_to(foot) - result.lower_length
        )
        lines = (
            "Milestone 3 | Single Leg Two-Bone IK",
            "B: flip bend | R: clear history | F1: debug | Esc: quit",
            f"FPS: {frames_per_second:6.1f}",
            f"state: {result.reach_state.value}",
            f"branch: {result.bend_branch:+d}",
            (
                "bend hint: "
                f"({bend_hint.x:+.2f}, {bend_hint.y:+.2f})"
            ),
            (
                "requested: "
                f"({result.requested_target[0]:7.2f}, "
                f"{result.requested_target[1]:7.2f})"
            ),
            (
                "solved:    "
                f"({result.solved_target[0]:7.2f}, "
                f"{result.solved_target[1]:7.2f})"
            ),
            f"target error: {result.target_error:9.5f} px",
            (
                f"bone lengths: {result.upper_length:.2f} / "
                f"{result.lower_length:.2f}"
            ),
            f"bone errors: {upper_error:.3e} / {lower_error:.3e}",
        )

        x = self._config.debug.hud_margin
        y = self._config.debug.hud_margin
        line_height = self._font.get_linesize()
        for line in lines:
            text = self._font.render(
                line,
                True,
                self._config.ik_debug.hud_text_color,
            )
            self._surface.blit(text, (x, y))
            y += line_height
