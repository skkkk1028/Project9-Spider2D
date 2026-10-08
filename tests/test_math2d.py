"""Tests for body-space transforms and angular helpers."""

from __future__ import annotations

import math
import unittest

import pygame

from spider_sim.math2d import (
    local_to_world,
    move_towards,
    smooth_angle,
    world_to_local,
    wrap_angle,
)


class CoordinateTransformTests(unittest.TestCase):
    def test_local_world_round_trip(self) -> None:
        origins = ((0.0, 0.0), (320.5, -48.25))
        local_points = ((0.0, 0.0), (25.0, -12.0), (-8.5, 91.0))
        headings = (0.0, math.radians(37.0), math.radians(-179.0))

        for origin in origins:
            for local_point in local_points:
                for heading in headings:
                    with self.subTest(
                        origin=origin,
                        local_point=local_point,
                        heading=heading,
                    ):
                        world = local_to_world(
                            local_point,
                            origin,
                            heading,
                        )
                        recovered = world_to_local(
                            world,
                            origin,
                            heading,
                        )
                        self.assertTrue(
                            recovered.distance_to(local_point) < 1e-9
                        )

    def test_screen_down_is_positive_heading_from_screen_right(self) -> None:
        world = local_to_world((10.0, 0.0), (0.0, 0.0), math.pi / 2.0)

        self.assertAlmostEqual(world.x, 0.0, places=9)
        self.assertAlmostEqual(world.y, 10.0, places=9)


class VectorAndAngleTests(unittest.TestCase):
    def test_move_towards_respects_maximum_delta(self) -> None:
        result = move_towards(
            pygame.Vector2(0.0, 0.0),
            pygame.Vector2(10.0, 0.0),
            3.0,
        )

        self.assertEqual(result, pygame.Vector2(3.0, 0.0))

    def test_wrap_angle_uses_expected_interval(self) -> None:
        self.assertAlmostEqual(
            wrap_angle(math.radians(181.0)),
            math.radians(-179.0),
        )
        self.assertAlmostEqual(wrap_angle(math.pi), -math.pi)

    def test_smooth_angle_crosses_wrap_boundary_by_shortest_path(self) -> None:
        current = math.radians(179.0)
        target = math.radians(-179.0)
        delta_seconds = 1.0 / 120.0
        max_angular_speed = math.radians(90.0)

        result = smooth_angle(
            current,
            target,
            response=12.0,
            max_angular_speed=max_angular_speed,
            delta_seconds=delta_seconds,
        )

        initial_error = abs(wrap_angle(target - current))
        final_error = abs(wrap_angle(target - result))
        travelled = abs(wrap_angle(result - current))
        self.assertLess(final_error, initial_error)
        self.assertLessEqual(
            travelled,
            max_angular_speed * delta_seconds + 1e-12,
        )


if __name__ == "__main__":
    unittest.main()

