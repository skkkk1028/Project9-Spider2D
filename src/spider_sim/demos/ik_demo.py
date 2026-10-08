"""Standalone mouse-driven single-leg Two-Bone IK demonstration."""

from __future__ import annotations

import pygame

from spider_sim.config import DEFAULT_CONFIG, AppConfig
from spider_sim.ik import TwoBoneIKSolver
from spider_sim.ik_renderer import IKRenderer


def run(
    config: AppConfig = DEFAULT_CONFIG,
    *,
    max_frames: int | None = None,
) -> int:
    """Run the fixed-hip single-leg IK demo."""
    if max_frames is not None and (
        isinstance(max_frames, bool)
        or not isinstance(max_frames, int)
        or max_frames <= 0
    ):
        raise ValueError("max_frames must be a positive integer or None")

    pygame.init()
    try:
        surface = pygame.display.set_mode(config.window.size)
        pygame.display.set_caption(config.ik_debug.title)
        clock = pygame.time.Clock()
        renderer = IKRenderer(surface, config)
        solver = TwoBoneIKSolver(
            epsilon=config.ik.epsilon,
            continuity_weight=config.ik.continuity_weight,
        )
        hip = pygame.Vector2(
            config.window.width * 0.5,
            config.window.height * 0.5,
        )
        bend_hint = pygame.Vector2(config.ik.default_bend_hint)
        previous_knee: tuple[float, float] | None = None
        debug_enabled = config.debug.enabled
        running = True
        rendered_frames = 0

        while running:
            clock.tick(config.window.target_fps)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_F1:
                    debug_enabled = not debug_enabled
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_b:
                    bend_hint *= -1.0
                    previous_knee = None
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                    previous_knee = None

            if not running:
                break

            previous_for_debug = previous_knee
            result = solver.solve(
                hip,
                pygame.mouse.get_pos(),
                config.ik.upper_length,
                config.ik.lower_length,
                bend_hint,
                previous_knee,
            )
            renderer.draw(
                result,
                bend_hint,
                previous_for_debug,
                frames_per_second=clock.get_fps(),
                debug_enabled=debug_enabled,
            )
            previous_knee = result.knee

            pygame.display.flip()
            rendered_frames += 1
            if max_frames is not None and rendered_frames >= max_frames:
                running = False
    finally:
        pygame.quit()

    return 0
