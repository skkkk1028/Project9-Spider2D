"""Integration tests for the standalone Milestone 3 IK demo."""

from __future__ import annotations

import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import unittest

import pygame

from spider_sim.__main__ import main
from spider_sim.config import AppConfig
from spider_sim.demos.ik_demo import run
from spider_sim.ik import TwoBoneIKSolver
from spider_sim.ik_renderer import IKRenderer


class IKDemoSmokeTests(unittest.TestCase):
    def test_demo_initializes_renders_and_stops(self) -> None:
        result = run(max_frames=3)

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_cli_routes_to_ik_demo(self) -> None:
        result = main(["--demo", "ik", "--max-frames", "2"])

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_invalid_frame_limit_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            run(max_frames=0)


class IKRendererTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        pygame.init()

    @classmethod
    def tearDownClass(cls) -> None:
        pygame.quit()

    def test_renderer_does_not_mutate_result(self) -> None:
        config = AppConfig()
        surface = pygame.Surface(config.window.size)
        solver = TwoBoneIKSolver(
            epsilon=config.ik.epsilon,
            continuity_weight=config.ik.continuity_weight,
        )
        result = solver.solve(
            (600.0, 400.0),
            (780.0, 520.0),
            config.ik.upper_length,
            config.ik.lower_length,
            config.ik.default_bend_hint,
        )
        before = result

        IKRenderer(surface, config).draw(
            result,
            config.ik.default_bend_hint,
            previous_knee_world=None,
            frames_per_second=120.0,
            debug_enabled=True,
        )

        self.assertEqual(result, before)
        self.assertNotEqual(
            surface.get_at((600, 400))[:3],
            config.window.background_color,
        )


if __name__ == "__main__":
    unittest.main()

