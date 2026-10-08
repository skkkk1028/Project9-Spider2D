"""Tests for delta-time handling and the base simulation."""

from __future__ import annotations

import math
import unittest

from spider_sim.config import TimingConfig
from spider_sim.simulation import Simulation, split_delta_time


class DeltaTimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.timing = TimingConfig(
            max_frame_seconds=0.05,
            max_substep_seconds=0.01,
        )

    def test_delta_is_split_without_changing_simulated_time(self) -> None:
        substeps = split_delta_time(0.035, self.timing)

        self.assertEqual(len(substeps), 4)
        self.assertTrue(all(step <= 0.01 for step in substeps))
        self.assertTrue(math.isclose(sum(substeps), 0.035))

    def test_long_frame_is_clamped(self) -> None:
        substeps = split_delta_time(0.5, self.timing)

        self.assertTrue(math.isclose(sum(substeps), 0.05))
        self.assertEqual(len(substeps), 5)

    def test_zero_delta_produces_no_update_step(self) -> None:
        self.assertEqual(split_delta_time(0.0, self.timing), ())

    def test_substep_count_respects_configured_limit(self) -> None:
        timing = TimingConfig(
            max_frame_seconds=0.05,
            max_substep_seconds=0.01,
            max_substeps_per_frame=5,
        )

        self.assertEqual(len(split_delta_time(0.5, timing)), 5)

    def test_invalid_delta_is_rejected(self) -> None:
        for invalid_delta in (-0.1, math.inf, math.nan):
            with self.subTest(invalid_delta=invalid_delta):
                with self.assertRaises(ValueError):
                    split_delta_time(invalid_delta, self.timing)


class SimulationTests(unittest.TestCase):
    def test_updates_accumulate_elapsed_time_and_count(self) -> None:
        simulation = Simulation()

        simulation.update(0.01)
        simulation.update(0.02)
        snapshot = simulation.snapshot()

        self.assertTrue(math.isclose(snapshot.elapsed_seconds, 0.03))
        self.assertEqual(snapshot.update_count, 2)

    def test_snapshot_is_immutable(self) -> None:
        snapshot = Simulation().snapshot()

        with self.assertRaises((AttributeError, TypeError)):
            snapshot.update_count = 5  # type: ignore[misc]

    def test_target_is_injected_and_copied_into_snapshot(self) -> None:
        simulation = Simulation(initial_position=(10.0, 20.0))

        simulation.set_target((100.0, 200.0))
        simulation.update(0.01)
        snapshot = simulation.snapshot()

        self.assertEqual(snapshot.target_world, (100.0, 200.0))
        self.assertNotEqual(snapshot.body.position, (10.0, 20.0))

    def test_invalid_target_is_rejected(self) -> None:
        simulation = Simulation()

        with self.assertRaises(ValueError):
            simulation.set_target((math.inf, 0.0))


if __name__ == "__main__":
    unittest.main()
