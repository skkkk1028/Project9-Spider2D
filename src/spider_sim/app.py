"""Pygame application lifecycle and main loop."""

from __future__ import annotations

import pygame

from spider_sim.config import DEFAULT_CONFIG, AppConfig
from spider_sim.renderer import FrameDebugInfo, Renderer
from spider_sim.simulation import Simulation, split_delta_time


def run(
    config: AppConfig = DEFAULT_CONFIG,
    *,
    max_frames: int | None = None,
) -> int:
    """Run the Pygame application until quit or an optional frame limit."""
    if max_frames is not None and (
        isinstance(max_frames, bool)
        or not isinstance(max_frames, int)
        or max_frames <= 0
    ):
        raise ValueError("max_frames must be a positive integer or None")

    pygame.init()
    try:
        surface = pygame.display.set_mode(config.window.size)
        pygame.display.set_caption(config.window.title)

        clock = pygame.time.Clock()
        simulation = Simulation(
            config.body,
            (
                config.window.width * 0.5,
                config.window.height * 0.5,
            ),
        )
        renderer = Renderer(surface, config)
        running = True
        rendered_frames = 0
        debug_enabled = config.debug.enabled

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
            for delta_seconds in substeps:
                simulation.update(delta_seconds)

            renderer.draw(
                simulation.snapshot(),
                FrameDebugInfo(
                    frames_per_second=clock.get_fps(),
                    raw_delta_seconds=raw_delta_seconds,
                    substep_count=len(substeps),
                ),
                debug_enabled=debug_enabled,
            )
            pygame.display.flip()
            rendered_frames += 1

            if max_frames is not None and rendered_frames >= max_frames:
                running = False
    finally:
        pygame.quit()

    return 0
