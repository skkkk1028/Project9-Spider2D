"""Tests for the Milestone 2 renderer contract."""

from __future__ import annotations

import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import unittest

import pygame

from spider_sim.config import AppConfig
from spider_sim.renderer import FrameDebugInfo, Renderer
from spider_sim.simulation import Simulation


class RendererTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        pygame.init()

    @classmethod
    def tearDownClass(cls) -> None:
        pygame.quit()

    def test_draw_does_not_mutate_simulation_snapshot(self) -> None:
        config = AppConfig()
        surface = pygame.Surface(config.window.size)
        simulation = Simulation(
            config.body,
            initial_position=(600.0, 400.0),
        )
        simulation.set_target((900.0, 500.0))
        simulation.update(1.0 / 120.0)
        before = simulation.snapshot()

        renderer = Renderer(surface, config)
        renderer.draw(
            before,
            FrameDebugInfo(
                frames_per_second=120.0,
                raw_delta_seconds=1.0 / 120.0,
                substep_count=1,
            ),
            debug_enabled=True,
        )

        self.assertEqual(simulation.snapshot(), before)
        self.assertNotEqual(
            surface.get_at((600, 400))[:3],
            config.window.background_color,
        )


if __name__ == "__main__":
    unittest.main()

