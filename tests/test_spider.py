"""Tests for the Milestone 2 spider body controller."""

from __future__ import annotations

import math
import unittest

import pygame

from spider_sim.config import BodyConfig, TimingConfig
from spider_sim.math2d import wrap_angle
from spider_sim.simulation import Simulation, split_delta_time
from spider_sim.spider import SpiderBody


class SpiderBodyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = BodyConfig()
        self.delta_seconds = 1.0 / 120.0

    def test_body_moves_smoothly_without_teleporting(self) -> None:
        body = SpiderBody(self.config, (0.0, 0.0))

        body.update((1000.0, 0.0), self.delta_seconds)
        snapshot = body.snapshot()

        self.assertGreater(snapshot.position[0], 0.0)
        self.assertLess(snapshot.position[0], 1000.0)
        self.assertLessEqual(
            pygame.Vector2(snapshot.velocity).length(),
            self.config.max_acceleration * self.delta_seconds + 1e-9,
        )

    def test_speed_and_acceleration_stay_within_limits(self) -> None:
        body = SpiderBody(self.config, (0.0, 0.0))
        previous_velocity = pygame.Vector2()

        targets = (
            ((5000.0, 1000.0), 240),
            ((-5000.0, -1000.0), 240),
            ((100.0, 100.0), 120),
        )
        for target, step_count in targets:
            for _ in range(step_count):
                body.update(target, self.delta_seconds)
                velocity = pygame.Vector2(body.snapshot().velocity)
                self.assertLessEqual(
                    velocity.length(),
                    self.config.max_speed + 1e-8,
                )
                self.assertLessEqual(
                    velocity.distance_to(previous_velocity),
                    self.config.max_acceleration * self.delta_seconds + 1e-8,
                )
                previous_velocity = velocity

    def test_near_target_snap_cannot_bypass_acceleration_limit(self) -> None:
        body = SpiderBody(self.config, (0.0, 0.0))

        body.update((1000.0, 0.0), self.delta_seconds)
        body.update((1000.0, 0.0), self.delta_seconds)
        previous_velocity = pygame.Vector2(body.snapshot().velocity)
        self.assertGreater(
            previous_velocity.length(),
            self.config.max_acceleration * self.delta_seconds,
        )

        position = pygame.Vector2(body.snapshot().position)
        body.update(
            (position.x + 0.05, position.y),
            self.delta_seconds,
        )
        current_velocity = pygame.Vector2(body.snapshot().velocity)

        self.assertLessEqual(
            current_velocity.distance_to(previous_velocity),
            self.config.max_acceleration * self.delta_seconds + 1e-9,
        )

    def test_heading_rotates_toward_actual_velocity(self) -> None:
        body = SpiderBody(self.config, (0.0, 0.0))

        for _ in range(30):
            body.update((0.0, 1000.0), self.delta_seconds)

        snapshot = body.snapshot()
        self.assertGreater(snapshot.velocity[1], 0.0)
        self.assertGreater(snapshot.heading, 0.0)
        self.assertLessEqual(snapshot.heading, math.pi / 2.0)

    def test_heading_is_retained_after_body_stops(self) -> None:
        body = SpiderBody(self.config, (0.0, 0.0))

        for _ in range(120):
            body.update((0.0, 300.0), self.delta_seconds)

        stop_target = body.snapshot().position
        for _ in range(600):
            body.update(stop_target, self.delta_seconds)
            snapshot = body.snapshot()
            if (
                pygame.Vector2(snapshot.velocity).length()
                < self.config.heading_speed_threshold
                and pygame.Vector2(snapshot.position).distance_to(stop_target)
                <= self.config.stop_radius + 0.1
            ):
                break
        else:
            self.fail("body did not slow below the heading threshold")

        stopped_heading = body.snapshot().heading

        for _ in range(120):
            body.update(stop_target, self.delta_seconds)

        self.assertAlmostEqual(
            body.snapshot().heading,
            stopped_heading,
            places=12,
        )

    def test_invalid_target_is_rejected(self) -> None:
        body = SpiderBody(self.config, (0.0, 0.0))

        with self.assertRaises(ValueError):
            body.update((math.nan, 0.0), self.delta_seconds)

    def test_initial_heading_is_normalized(self) -> None:
        body = SpiderBody(
            self.config,
            (0.0, 0.0),
            initial_heading=math.radians(540.0),
        )

        self.assertAlmostEqual(body.snapshot().heading, -math.pi)


class FrameRateIndependenceTests(unittest.TestCase):
    def _run_fixed_target(
        self,
        frames_per_second: int,
    ) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
        timing = TimingConfig()
        simulation = Simulation(
            BodyConfig(),
            initial_position=(100.0, 100.0),
        )
        target = (900.0, 650.0)
        samples = []
        frame_count = frames_per_second * 2

        for frame in range(1, frame_count + 1):
            simulation.set_target(target)
            for delta_seconds in split_delta_time(
                1.0 / frames_per_second,
                timing,
            ):
                simulation.update(delta_seconds)

            if frame % (frames_per_second // 2) == 0:
                body = simulation.snapshot().body
                samples.append(
                    (body.position, body.velocity, body.heading)
                )

        return samples

    def test_fixed_target_is_nearly_frame_rate_independent(self) -> None:
        reference = self._run_fixed_target(120)

        for frames_per_second in (30, 60, 144):
            samples = self._run_fixed_target(frames_per_second)
            self.assertEqual(len(samples), len(reference))

            for actual, expected in zip(samples, reference, strict=True):
                actual_position = pygame.Vector2(actual[0])
                expected_position = pygame.Vector2(expected[0])
                actual_velocity = pygame.Vector2(actual[1])
                expected_velocity = pygame.Vector2(expected[1])

                self.assertLess(
                    actual_position.distance_to(expected_position),
                    1.5,
                )
                self.assertLess(
                    actual_velocity.distance_to(expected_velocity),
                    3.0,
                )
                self.assertLess(
                    abs(wrap_angle(actual[2] - expected[2])),
                    math.radians(0.5),
                )

    def _run_moving_target(
        self,
        frames_per_second: int,
    ) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
        timing = TimingConfig()
        simulation = Simulation(
            BodyConfig(),
            initial_position=(600.0, 400.0),
        )
        samples = []

        for frame in range(1, frames_per_second * 3 + 1):
            elapsed_at_input = (frame - 1) / frames_per_second
            simulation.set_target(
                (
                    600.0 + 300.0 * math.cos(elapsed_at_input * 1.1),
                    400.0 + 220.0 * math.sin(elapsed_at_input * 1.4),
                )
            )
            for delta_seconds in split_delta_time(
                1.0 / frames_per_second,
                timing,
            ):
                simulation.update(delta_seconds)

            if frame % (frames_per_second // 2) == 0:
                body = simulation.snapshot().body
                samples.append(
                    (body.position, body.velocity, body.heading)
                )

        return samples

    def test_scripted_moving_target_remains_close_across_frame_rates(self) -> None:
        reference = self._run_moving_target(120)

        for frames_per_second in (30, 60, 144):
            samples = self._run_moving_target(frames_per_second)
            for actual, expected in zip(samples, reference, strict=True):
                self.assertLess(
                    pygame.Vector2(actual[0]).distance_to(expected[0]),
                    8.0,
                )
                self.assertLess(
                    pygame.Vector2(actual[1]).distance_to(expected[1]),
                    15.0,
                )
                self.assertLess(
                    abs(wrap_angle(actual[2] - expected[2])),
                    math.radians(3.0),
                )


if __name__ == "__main__":
    unittest.main()
