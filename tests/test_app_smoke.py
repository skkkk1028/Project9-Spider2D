"""Headless smoke test for the Pygame application loop."""

from __future__ import annotations

import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import unittest

import pygame

from spider_sim.app import run


class AppSmokeTests(unittest.TestCase):
    def test_application_initializes_renders_and_stops(self) -> None:
        result = run(max_frames=3)

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_invalid_frame_limit_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            run(max_frames=0)


if __name__ == "__main__":
    unittest.main()

