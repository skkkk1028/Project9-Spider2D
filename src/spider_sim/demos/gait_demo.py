"""Milestone 4-7 static, stepping, grouped, and constrained gait demos."""

from __future__ import annotations

from dataclasses import replace

import pygame

from spider_sim.config import DEFAULT_CONFIG, AppConfig
from spider_sim.gait_renderer import (
    ConstrainedGaitRenderer,
    FourPhaseGaitRenderer,
    SingleStepGaitRenderer,
    StaticGaitRenderer,
)
from spider_sim.ik import TwoBoneIKSolver
from spider_sim.landing import ConstrainedGaitAssembly, LandingDecision
from spider_sim.leg import StaticLegAssembly
from spider_sim.renderer import FrameDebugInfo
from spider_sim.simulation import Simulation, split_delta_time
from spider_sim.stepping import (
    FourPhaseGaitAssembly,
    SingleLegStepAssembly,
)


def run(
    config: AppConfig = DEFAULT_CONFIG,
    *,
    mode: str = "static",
    max_frames: int | None = None,
) -> int:
    """Run the M4, M5, M6, or M7 gait demo mode."""
    if mode not in (
        "static",
        "single-step",
        "four-phase",
        "constrained",
    ):
        raise ValueError(
            "gait_demo mode must be 'static', 'single-step', "
            "'four-phase', or 'constrained'"
        )
    if max_frames is not None and (
        isinstance(max_frames, bool)
        or not isinstance(max_frames, int)
        or max_frames <= 0
    ):
        raise ValueError("max_frames must be a positive integer or None")

    pygame.init()
    try:
        surface = pygame.display.set_mode(config.window.size)
        pygame.display.set_caption(
            config.gait_debug.title
            if mode == "static"
            else (
                config.gait_debug.step_title
                if mode == "single-step"
                else (
                    config.gait_debug.four_phase_title
                    if mode == "four-phase"
                    else config.gait_debug.constrained_title
                )
            )
        )
        clock = pygame.time.Clock()
        body_config = config.body
        if mode == "single-step":
            body_config = replace(
                config.body,
                max_speed=min(
                    config.body.max_speed,
                    config.foot_step.demo_max_speed,
                ),
                max_acceleration=min(
                    config.body.max_acceleration,
                    config.foot_step.demo_max_acceleration,
                ),
                heading_speed_threshold=min(
                    config.body.heading_speed_threshold,
                    config.foot_step.demo_max_speed * 0.25,
                ),
            )
        elif mode == "four-phase":
            body_config = replace(
                config.body,
                max_speed=min(
                    config.body.max_speed,
                    config.four_phase_gait.demo_max_speed,
                ),
                max_acceleration=min(
                    config.body.max_acceleration,
                    config.four_phase_gait.demo_max_acceleration,
                ),
                heading_speed_threshold=min(
                    config.body.heading_speed_threshold,
                    config.four_phase_gait.demo_max_speed * 0.25,
                ),
            )
        elif mode == "constrained":
            body_config = replace(
                config.body,
                max_speed=min(
                    config.body.max_speed,
                    config.landing.demo_max_speed,
                ),
                max_acceleration=min(
                    config.body.max_acceleration,
                    config.landing.demo_max_acceleration,
                ),
                heading_speed_threshold=min(
                    config.body.heading_speed_threshold,
                    config.landing.demo_max_speed * 0.25,
                ),
            )
        simulation = Simulation(
            body_config,
            (
                config.window.width * 0.5,
                config.window.height * 0.5,
            ),
        )
        solver = TwoBoneIKSolver(
            epsilon=config.ik.epsilon,
            continuity_weight=config.ik.continuity_weight,
        )
        static_assembly: StaticLegAssembly | None = None
        stepping_assembly: SingleLegStepAssembly | None = None
        grouped_assembly: FourPhaseGaitAssembly | None = None
        constrained_assembly: ConstrainedGaitAssembly | None = None
        static_renderer: StaticGaitRenderer | None = None
        stepping_renderer: SingleStepGaitRenderer | None = None
        grouped_renderer: FourPhaseGaitRenderer | None = None
        constrained_renderer: ConstrainedGaitRenderer | None = None

        if mode == "static":
            static_assembly = StaticLegAssembly(
                body_config,
                config.leg,
                solver,
            )
            static_renderer = StaticGaitRenderer(surface, config)
        elif mode == "single-step":
            stepping_assembly = SingleLegStepAssembly(
                body_config,
                config.leg,
                config.foot_step,
                simulation.snapshot().body,
                solver,
            )
            stepping_renderer = SingleStepGaitRenderer(surface, config)
        elif mode == "four-phase":
            grouped_assembly = FourPhaseGaitAssembly(
                body_config,
                config.leg,
                config.foot_step,
                config.four_phase_gait,
                simulation.snapshot().body,
                solver,
            )
            grouped_renderer = FourPhaseGaitRenderer(surface, config)
        else:
            constrained_assembly = ConstrainedGaitAssembly(
                body_config,
                config.leg,
                config.foot_step,
                config.four_phase_gait,
                config.landing,
                simulation.snapshot().body,
                solver,
            )
            constrained_renderer = ConstrainedGaitRenderer(surface, config)

        debug_enabled = config.debug.enabled
        running = True
        rendered_frames = 0

        while running:
            raw_delta_seconds = (
                clock.tick(config.window.target_fps) / 1000.0
            )
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_F1:
                    debug_enabled = not debug_enabled

            if not running:
                break

            simulation.set_target(pygame.mouse.get_pos())
            substeps = split_delta_time(
                raw_delta_seconds,
                config.timing,
            )
            frame_bend_anomalies = []
            frame_landing_violation = None
            for delta_seconds in substeps:
                simulation.update(delta_seconds)
                if stepping_assembly is not None:
                    stepping_assembly.update(
                        simulation.snapshot().body,
                        delta_seconds,
                    )
                elif constrained_assembly is not None:
                    substep_snapshot = constrained_assembly.update(
                        simulation.snapshot().body,
                        delta_seconds,
                    )
                    frame_bend_anomalies.extend(substep_snapshot.bend_anomalies)
                    if substep_snapshot.landing_violation is not None:
                        frame_landing_violation = substep_snapshot.landing_violation
                elif grouped_assembly is not None:
                    grouped_assembly.update(
                        simulation.snapshot().body,
                        delta_seconds,
                    )

            simulation_snapshot = simulation.snapshot()
            frame_info = FrameDebugInfo(
                frames_per_second=clock.get_fps(),
                raw_delta_seconds=raw_delta_seconds,
                substep_count=len(substeps),
            )
            if static_assembly is not None and static_renderer is not None:
                static_renderer.draw(
                    simulation_snapshot,
                    static_assembly.solve(simulation_snapshot.body),
                    frame_info,
                    debug_enabled=debug_enabled,
                )
            elif (
                stepping_assembly is not None
                and stepping_renderer is not None
            ):
                stepping_renderer.draw(
                    simulation_snapshot,
                    stepping_assembly.solve(simulation_snapshot.body),
                    frame_info,
                    debug_enabled=debug_enabled,
                )
            elif (
                constrained_assembly is not None
                and constrained_renderer is not None
            ):
                constrained_snapshot = constrained_assembly.solve(
                    simulation_snapshot.body
                )
                constrained_renderer.draw(
                    simulation_snapshot,
                    replace(
                        constrained_snapshot,
                        bend_anomalies=tuple(frame_bend_anomalies),
                        landing_violation=frame_landing_violation,
                        landing_decision=(
                            LandingDecision.DEGRADED
                            if frame_landing_violation is not None
                            else constrained_snapshot.landing_decision
                        ),
                    ),
                    frame_info,
                    debug_enabled=debug_enabled,
                )
            elif (
                grouped_assembly is not None
                and grouped_renderer is not None
            ):
                grouped_renderer.draw(
                    simulation_snapshot,
                    grouped_assembly.solve(simulation_snapshot.body),
                    frame_info,
                    debug_enabled=debug_enabled,
                )
            else:
                raise RuntimeError("gait demo mode was not initialized")
            pygame.display.flip()

            rendered_frames += 1
            if max_frames is not None and rendered_frames >= max_frames:
                running = False
    finally:
        pygame.quit()

    return 0
