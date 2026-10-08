"""Renderer for the Milestone 4 static eight-leg assembly."""

from __future__ import annotations

import math

import pygame

from spider_sim.config import AppConfig
from spider_sim.ik import IKReachState
from spider_sim.landing import ConstrainedGaitSnapshot
from spider_sim.leg import LegSide, StaticAssemblySnapshot, StaticLegSnapshot
from spider_sim.math2d import heading_axes
from spider_sim.renderer import FrameDebugInfo, Renderer
from spider_sim.simulation import SimulationSnapshot
from spider_sim.stepping import (
    FootState,
    GroupedGaitSnapshot,
    SteppingAssemblySnapshot,
    SteppingLegSnapshot,
)


def _screen_point(
    point: pygame.Vector2 | tuple[float, float],
) -> tuple[int, int]:
    vector = pygame.Vector2(point)
    return round(vector.x), round(vector.y)


class StaticGaitRenderer:
    """Draw eight solved legs while keeping the body renderer reusable."""

    def __init__(
        self,
        surface: pygame.Surface,
        config: AppConfig,
    ) -> None:
        self._surface = surface
        self._config = config
        self._body_renderer = Renderer(surface, config)
        self._font = pygame.font.Font(None, config.gait_debug.font_size)

    def draw(
        self,
        simulation: SimulationSnapshot,
        assembly: StaticAssemblySnapshot,
        frame_info: FrameDebugInfo,
        *,
        debug_enabled: bool,
    ) -> None:
        """Draw static legs behind the body plus assembly diagnostics."""
        self._surface.fill(self._config.window.background_color)
        self._draw_legs(assembly, debug_enabled=debug_enabled)
        self._body_renderer.draw(
            simulation,
            frame_info,
            debug_enabled=debug_enabled,
            clear_surface=False,
            draw_hud=False,
        )

        if debug_enabled:
            self._draw_midline(assembly)
            self._draw_labels(assembly)
            self._draw_hud(simulation, assembly, frame_info)

    def _draw_legs(
        self,
        assembly: StaticAssemblySnapshot,
        *,
        debug_enabled: bool,
    ) -> None:
        debug = self._config.gait_debug

        if debug_enabled:
            for leg in assembly.legs:
                pygame.draw.circle(
                    self._surface,
                    debug.default_foot_color,
                    _screen_point(leg.ik.requested_target),
                    debug.foot_radius + 5,
                    width=1,
                )

        for leg in assembly.legs:
            upper_color, lower_color = self._bone_colors(leg)
            if leg.ik.reach_state is not IKReachState.REACHABLE:
                upper_color = debug.invalid_color
                lower_color = debug.invalid_color

            pygame.draw.line(
                self._surface,
                upper_color,
                _screen_point(leg.ik.hip),
                _screen_point(leg.ik.knee),
                width=debug.bone_width,
            )
            pygame.draw.line(
                self._surface,
                lower_color,
                _screen_point(leg.ik.knee),
                _screen_point(leg.ik.foot),
                width=debug.bone_width,
            )
            pygame.draw.circle(
                self._surface,
                debug.hip_color,
                _screen_point(leg.ik.hip),
                debug.joint_radius,
            )
            pygame.draw.circle(
                self._surface,
                debug.knee_color,
                _screen_point(leg.ik.knee),
                debug.joint_radius,
            )
            pygame.draw.circle(
                self._surface,
                debug.foot_color,
                _screen_point(leg.ik.foot),
                debug.foot_radius,
            )

    def _bone_colors(
        self,
        leg: StaticLegSnapshot,
    ) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        debug = self._config.gait_debug
        if leg.mount.side is LegSide.LEFT:
            return debug.left_upper_color, debug.left_lower_color
        return debug.right_upper_color, debug.right_lower_color

    def _draw_midline(self, assembly: StaticAssemblySnapshot) -> None:
        position = pygame.Vector2(assembly.body.position)
        forward, _ = heading_axes(assembly.body.heading)
        extent = self._config.gait_debug.midline_half_length
        pygame.draw.line(
            self._surface,
            self._config.gait_debug.midline_color,
            _screen_point(position - forward * extent),
            _screen_point(position + forward * extent),
            width=1,
        )

    def _draw_labels(self, assembly: StaticAssemblySnapshot) -> None:
        _, right = heading_axes(assembly.body.heading)
        debug = self._config.gait_debug

        for leg in assembly.legs:
            foot = pygame.Vector2(leg.ik.foot)
            label_position = (
                foot
                + right
                * float(leg.mount.side)
                * debug.label_offset
            )
            text = self._font.render(
                leg.mount.leg_id,
                True,
                debug.label_color,
            )
            rect = text.get_rect(center=_screen_point(label_position))
            self._surface.blit(text, rect)

    def _draw_hud(
        self,
        simulation: SimulationSnapshot,
        assembly: StaticAssemblySnapshot,
        frame_info: FrameDebugInfo,
    ) -> None:
        reachable_count = sum(
            leg.ik.reach_state is IKReachState.REACHABLE
            for leg in assembly.legs
        )
        maximum_error = max(
            (leg.ik.target_error for leg in assembly.legs),
            default=0.0,
        )
        lines = (
            "Milestone 4 | gait_demo: static",
            "Feet follow body-local defaults (no locking or stepping)",
            "F1: debug | Esc: quit",
            f"FPS: {frame_info.frames_per_second:6.1f}",
            f"raw dt: {frame_info.raw_delta_seconds * 1000.0:6.2f} ms",
            f"substeps: {frame_info.substep_count}",
            f"legs: {len(assembly.legs)}",
            f"reachable: {reachable_count}/{len(assembly.legs)}",
            f"max target error: {maximum_error:.3e} px",
            f"body heading: {math.degrees(assembly.body.heading):7.2f} deg",
            f"simulation time: {simulation.elapsed_seconds:7.2f} s",
        )

        x = self._config.debug.hud_margin
        y = self._config.debug.hud_margin
        line_height = self._font.get_linesize()
        for line in lines:
            text = self._font.render(
                line,
                True,
                self._config.gait_debug.label_color,
            )
            self._surface.blit(text, (x, y))
            y += line_height


class SingleStepGaitRenderer:
    """Draw M5 world locks, the one active foot, and state diagnostics."""

    def __init__(
        self,
        surface: pygame.Surface,
        config: AppConfig,
    ) -> None:
        self._surface = surface
        self._config = config
        self._body_renderer = Renderer(surface, config)
        self._font = pygame.font.Font(None, config.gait_debug.font_size)

    def draw(
        self,
        simulation: SimulationSnapshot,
        assembly: SteppingAssemblySnapshot,
        frame_info: FrameDebugInfo,
        *,
        debug_enabled: bool,
    ) -> None:
        """Draw locked feet and the single active step behind the body."""
        self._surface.fill(self._config.window.background_color)
        self._draw_legs(assembly, debug_enabled=debug_enabled)
        self._body_renderer.draw(
            simulation,
            frame_info,
            debug_enabled=debug_enabled,
            clear_surface=False,
            draw_hud=False,
        )

        if debug_enabled:
            self._draw_midline(assembly)
            self._draw_labels(assembly)
            self._draw_hud(simulation, assembly, frame_info)

    def _draw_legs(
        self,
        assembly: SteppingAssemblySnapshot,
        *,
        debug_enabled: bool,
    ) -> None:
        debug = self._config.gait_debug

        if debug_enabled:
            for leg in assembly.legs:
                pygame.draw.circle(
                    self._surface,
                    debug.default_foot_color,
                    _screen_point(leg.default_target_world),
                    debug.foot_radius + 5,
                    width=1,
                )
                if leg.step_target_world is not None:
                    pygame.draw.circle(
                        self._surface,
                        debug.landing_target_color,
                        _screen_point(leg.step_target_world),
                        debug.foot_radius + 8,
                        width=2,
                    )

        for leg in assembly.legs:
            upper_color, lower_color = self._bone_colors(leg)
            if leg.ik.reach_state is not IKReachState.REACHABLE:
                upper_color = debug.invalid_color
                lower_color = debug.invalid_color

            pygame.draw.line(
                self._surface,
                upper_color,
                _screen_point(leg.ik.hip),
                _screen_point(leg.ik.knee),
                width=debug.bone_width,
            )
            pygame.draw.line(
                self._surface,
                lower_color,
                _screen_point(leg.ik.knee),
                _screen_point(leg.ik.foot),
                width=debug.bone_width,
            )
            pygame.draw.circle(
                self._surface,
                debug.hip_color,
                _screen_point(leg.ik.hip),
                debug.joint_radius,
            )
            pygame.draw.circle(
                self._surface,
                debug.knee_color,
                _screen_point(leg.ik.knee),
                debug.joint_radius,
            )

            foot_color = self._state_color(leg.state)
            pygame.draw.circle(
                self._surface,
                foot_color,
                _screen_point(leg.foot_world),
                debug.foot_radius,
            )
            if leg.state is FootState.PLANTED:
                pygame.draw.circle(
                    self._surface,
                    debug.locked_foot_color,
                    _screen_point(leg.foot_world),
                    debug.foot_radius + 3,
                    width=1,
                )
            elif debug_enabled:
                lift_halo = (
                    debug.foot_radius
                    + 4
                    + round(leg.foot_height * 0.25)
                )
                pygame.draw.circle(
                    self._surface,
                    foot_color,
                    _screen_point(leg.foot_world),
                    lift_halo,
                    width=2,
                )

            if leg.ik.target_error > 1e-6:
                pygame.draw.line(
                    self._surface,
                    debug.invalid_color,
                    _screen_point(leg.ik.foot),
                    _screen_point(leg.foot_world),
                    width=2,
                )

    def _bone_colors(
        self,
        leg: SteppingLegSnapshot,
    ) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        debug = self._config.gait_debug
        if leg.mount.side is LegSide.LEFT:
            return debug.left_upper_color, debug.left_lower_color
        return debug.right_upper_color, debug.right_lower_color

    def _state_color(self, state: FootState) -> tuple[int, int, int]:
        debug = self._config.gait_debug
        if state is FootState.LIFT:
            return debug.lift_color
        if state is FootState.SWING:
            return debug.swing_color
        if state is FootState.LOWER:
            return debug.lower_color
        return debug.foot_color

    def _draw_midline(self, assembly: SteppingAssemblySnapshot) -> None:
        position = pygame.Vector2(assembly.body.position)
        forward, _ = heading_axes(assembly.body.heading)
        extent = self._config.gait_debug.midline_half_length
        pygame.draw.line(
            self._surface,
            self._config.gait_debug.midline_color,
            _screen_point(position - forward * extent),
            _screen_point(position + forward * extent),
            width=1,
        )

    def _draw_labels(self, assembly: SteppingAssemblySnapshot) -> None:
        _, right = heading_axes(assembly.body.heading)
        debug = self._config.gait_debug

        for leg in assembly.legs:
            label_position = (
                pygame.Vector2(leg.foot_world)
                + right
                * float(leg.mount.side)
                * debug.label_offset
            )
            label = leg.mount.leg_id
            if leg.state is not FootState.PLANTED:
                label = f"{label}:{leg.state.value.upper()}"
            text = self._font.render(label, True, debug.label_color)
            rect = text.get_rect(center=_screen_point(label_position))
            self._surface.blit(text, rect)

    def _draw_hud(
        self,
        simulation: SimulationSnapshot,
        assembly: SteppingAssemblySnapshot,
        frame_info: FrameDebugInfo,
    ) -> None:
        reachable_count = sum(
            leg.ik.reach_state is IKReachState.REACHABLE
            for leg in assembly.legs
        )
        planted_count = sum(
            leg.state is FootState.PLANTED
            for leg in assembly.legs
        )
        active = next(
            (
                leg
                for leg in assembly.legs
                if leg.state is not FootState.PLANTED
            ),
            None,
        )
        active_line = "active: none"
        if active is not None:
            active_line = (
                f"active: {active.mount.leg_id} {active.state.value.upper()} "
                f"{active.state_progress * 100.0:5.1f}% "
                f"height={active.foot_height:5.1f}"
            )
        lines = (
            "Milestone 5 | gait_demo: single-step",
            "PLANTED feet are world-locked | max active legs: 1",
            "F1: debug | Esc: quit",
            f"FPS: {frame_info.frames_per_second:6.1f}",
            f"raw dt: {frame_info.raw_delta_seconds * 1000.0:6.2f} ms",
            f"substeps: {frame_info.substep_count}",
            f"planted: {planted_count}/{len(assembly.legs)}",
            active_line,
            f"reachable IK: {reachable_count}/{len(assembly.legs)}",
            f"completed steps: {sum(leg.step_count for leg in assembly.legs)}",
            f"body heading: {math.degrees(assembly.body.heading):7.2f} deg",
            f"simulation time: {simulation.elapsed_seconds:7.2f} s",
        )

        x = self._config.debug.hud_margin
        y = self._config.debug.hud_margin
        line_height = self._font.get_linesize()
        for line in lines:
            text = self._font.render(
                line,
                True,
                self._config.gait_debug.label_color,
            )
            self._surface.blit(text, (x, y))
            y += line_height


class FourPhaseGaitRenderer(SingleStepGaitRenderer):
    """Reuse M5 leg drawing with M6 group scheduler diagnostics."""

    def draw(
        self,
        simulation: SimulationSnapshot,
        assembly: GroupedGaitSnapshot,
        frame_info: FrameDebugInfo,
        *,
        debug_enabled: bool,
    ) -> None:
        """Draw a maximum of one active diagonal group."""
        super().draw(
            simulation,
            assembly,  # type: ignore[arg-type]
            frame_info,
            debug_enabled=debug_enabled,
        )

    def _draw_hud(
        self,
        simulation: SimulationSnapshot,
        assembly: GroupedGaitSnapshot,
        frame_info: FrameDebugInfo,
    ) -> None:
        active = [
            leg
            for leg in assembly.legs
            if leg.state is not FootState.PLANTED
        ]
        left_active = sum(
            leg.mount.side is LegSide.LEFT
            for leg in active
        )
        right_active = len(active) - left_active
        reachable_count = sum(
            leg.ik.reach_state is IKReachState.REACHABLE
            for leg in assembly.legs
        )
        active_line = "active group: none"
        if assembly.active_group_index is not None and active:
            active_line = (
                f"active G{assembly.active_group_index}: "
                f"{','.join(assembly.active_leg_ids)} "
                f"{active[0].state.value.upper()} "
                f"{active[0].state_progress * 100.0:5.1f}%"
            )
        lines = (
            "Milestone 6 | gait_demo: four-phase",
            "Fixed fair round-robin | advanced scoring: OFF",
            "F1: debug | Esc: quit",
            f"FPS: {frame_info.frames_per_second:6.1f}",
            f"raw dt: {frame_info.raw_delta_seconds * 1000.0:6.2f} ms",
            f"substeps: {frame_info.substep_count}",
            active_line,
            (
                f"support: {assembly.support_count}/8 | active: "
                f"{len(active)} | left/right: {left_active}/{right_active}"
            ),
            f"next group: G{assembly.next_group_index}",
            f"group starts: {assembly.group_start_counts}",
            f"reachable IK: {reachable_count}/{len(assembly.legs)}",
            f"completed leg steps: {sum(leg.step_count for leg in assembly.legs)}",
            f"body heading: {math.degrees(assembly.body.heading):7.2f} deg",
            f"simulation time: {simulation.elapsed_seconds:7.2f} s",
        )

        x = self._config.debug.hud_margin
        y = self._config.debug.hud_margin
        line_height = self._font.get_linesize()
        for line in lines:
            text = self._font.render(
                line,
                True,
                self._config.gait_debug.label_color,
            )
            self._surface.blit(text, (x, y))
            y += line_height


class ConstrainedGaitRenderer(FourPhaseGaitRenderer):
    """Draw M7 regions, candidates, and live constraint diagnostics."""

    def draw(
        self,
        simulation: SimulationSnapshot,
        assembly: ConstrainedGaitSnapshot,
        frame_info: FrameDebugInfo,
        *,
        debug_enabled: bool,
    ) -> None:
        """Draw constrained gait without hiding rejected candidates."""
        self._surface.fill(self._config.window.background_color)
        if debug_enabled:
            self._draw_activity_regions(assembly)
            self._draw_candidates(assembly)
        self._draw_legs(
            assembly,  # type: ignore[arg-type]
            debug_enabled=debug_enabled,
        )
        self._body_renderer.draw(
            simulation,
            frame_info,
            debug_enabled=debug_enabled,
            clear_surface=False,
            draw_hud=False,
        )

        if debug_enabled:
            self._draw_constraint_diagnostics(assembly)
            self._draw_midline(assembly)  # type: ignore[arg-type]
            self._draw_labels(assembly)  # type: ignore[arg-type]
            self._draw_hud(simulation, assembly, frame_info)

    def _draw_activity_regions(
        self,
        assembly: ConstrainedGaitSnapshot,
    ) -> None:
        for region in assembly.activity_regions:
            pygame.draw.polygon(
                self._surface,
                self._config.gait_debug.activity_region_color,
                [_screen_point(point) for point in region.polygon_world],
                width=1,
            )

    def _draw_candidates(
        self,
        assembly: ConstrainedGaitSnapshot,
    ) -> None:
        debug = self._config.gait_debug
        for candidate in assembly.candidate_points:
            if candidate.degraded:
                color = debug.degraded_candidate_color
            elif candidate.valid:
                color = debug.valid_candidate_color
            else:
                color = debug.invalid_candidate_color
            radius = 7 if candidate.selected else 4
            width = 0 if candidate.selected else 1
            pygame.draw.circle(
                self._surface,
                color,
                _screen_point(candidate.world_position),
                radius,
                width=width,
            )

    def _draw_constraint_diagnostics(
        self,
        assembly: ConstrainedGaitSnapshot,
    ) -> None:
        debug = self._config.gait_debug
        feet = {
            leg.mount.leg_id: pygame.Vector2(leg.foot_world)
            for leg in assembly.legs
        }
        knees = {
            leg.mount.leg_id: pygame.Vector2(leg.ik.knee)
            for leg in assembly.legs
        }

        for violation in assembly.spacing_violations:
            pygame.draw.line(
                self._surface,
                debug.spacing_violation_color,
                _screen_point(feet[violation.first_leg_id]),
                _screen_point(feet[violation.second_leg_id]),
                width=3,
            )

        for intersection in assembly.bone_intersections:
            center = _screen_point(intersection.point_world)
            radius = 7
            pygame.draw.line(
                self._surface,
                debug.intersection_color,
                (center[0] - radius, center[1] - radius),
                (center[0] + radius, center[1] + radius),
                width=3,
            )
            pygame.draw.line(
                self._surface,
                debug.intersection_color,
                (center[0] - radius, center[1] + radius),
                (center[0] + radius, center[1] - radius),
                width=3,
            )

        for anomaly in assembly.bend_anomalies:
            pygame.draw.circle(
                self._surface,
                debug.bend_anomaly_color,
                _screen_point(knees[anomaly.leg_id]),
                debug.joint_radius + 8,
                width=3,
            )

        for violation in assembly.bone_clearance_violations:
            pygame.draw.circle(
                self._surface,
                debug.bone_clearance_color,
                _screen_point(violation.point_world),
                round(self._config.landing.bone_clearance),
                width=2,
            )

        for penetration in assembly.body_penetrations:
            center = _screen_point(penetration.point_world)
            radius = 6
            pygame.draw.rect(
                self._surface,
                debug.body_penetration_color,
                pygame.Rect(
                    center[0] - radius,
                    center[1] - radius,
                    radius * 2,
                    radius * 2,
                ),
                width=2,
            )

        for violation in assembly.anatomy_violations:
            pygame.draw.circle(
                self._surface,
                debug.anatomy_violation_color,
                _screen_point(violation.point_world),
                debug.joint_radius + 6,
                width=3,
            )

    def _draw_hud(
        self,
        simulation: SimulationSnapshot,
        assembly: ConstrainedGaitSnapshot,
        frame_info: FrameDebugInfo,
    ) -> None:
        active = [
            leg
            for leg in assembly.legs
            if leg.state is not FootState.PLANTED
        ]
        selected_count = sum(
            candidate.selected
            for candidate in assembly.candidate_points
        )
        invalid_count = sum(
            not candidate.valid
            for candidate in assembly.candidate_points
        )
        degraded_count = sum(
            candidate.degraded
            for candidate in assembly.candidate_points
        )
        active_line = "active group: none"
        if assembly.active_group_index is not None and active:
            active_line = (
                f"active G{assembly.active_group_index}: "
                f"{','.join(assembly.active_leg_ids)} "
                f"{active[0].state.value.upper()} "
                f"{active[0].state_progress * 100.0:5.1f}%"
            )
        landing_line = "landing check: clear"
        if assembly.landing_violation is not None:
            landing_line = (
                f"landing check: DEGRADED G"
                f"{assembly.landing_violation.group_index} | "
                f"crossings: "
                f"{len(assembly.landing_violation.bone_intersections)}"
            )
        lines = (
            "Phase 1 final | gait_demo: constrained",
            "Regions + candidates + geometry/anatomy diagnostics",
            (
                "candidate regions: "
                + (
                    "LAST PLANNING POSE"
                    if assembly.candidate_region_body is not None
                    else "CURRENT POSE"
                )
            ),
            "F1: debug | Esc: quit",
            f"FPS: {frame_info.frames_per_second:6.1f}",
            f"substeps: {frame_info.substep_count}",
            active_line,
            (
                f"decision: {assembly.landing_decision.value.upper()} | "
                f"deferred: {assembly.deferred_group_indices}"
            ),
            (
                f"candidates: {len(assembly.candidate_points)} | "
                f"selected: {selected_count} | invalid: {invalid_count} | "
                f"degraded: {degraded_count}"
            ),
            (
                f"support: {assembly.support_count}/8 | "
                f"spacing violations: {len(assembly.spacing_violations)}"
            ),
            (
                f"spacing samples: "
                f"{assembly.total_spacing_violation_samples}"
            ),
            (
                f"bone intersections: {len(assembly.bone_intersections)} | "
                f"samples: {assembly.total_bone_intersection_samples}"
            ),
            (
                "bone clearance/body penetration: "
                f"{len(assembly.bone_clearance_violations)}/"
                f"{len(assembly.body_penetrations)} | samples: "
                f"{assembly.total_bone_clearance_violation_samples}/"
                f"{assembly.total_body_penetration_samples}"
            ),
            (
                f"anatomy violations: {len(assembly.anatomy_violations)} | "
                f"samples: {assembly.total_anatomy_violation_samples}"
            ),
            (
                f"bend anomalies: {len(assembly.bend_anomalies)} | "
                f"total: {assembly.total_bend_anomalies}"
            ),
            f"group starts: {assembly.group_start_counts}",
            (
                f"deferred/degraded groups: "
                f"{assembly.total_deferred_groups}/"
                f"{assembly.total_degraded_groups}"
            ),
            (
                f"landing violations: "
                f"{assembly.total_landing_violations}"
            ),
            landing_line,
            f"simulation time: {simulation.elapsed_seconds:7.2f} s",
        )

        x = self._config.debug.hud_margin
        y = self._config.debug.hud_margin
        line_height = self._font.get_linesize()
        for line in lines:
            text = self._font.render(
                line,
                True,
                self._config.gait_debug.label_color,
            )
            self._surface.blit(text, (x, y))
            y += line_height
