"""Integration and rendering tests for the Milestone 4 static gait demo."""

from __future__ import annotations

import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import unittest

import pygame

from spider_sim.__main__ import main
from spider_sim.config import AppConfig, FootStepConfig
from spider_sim.demos.gait_demo import run
from spider_sim.gait_renderer import (
    ConstrainedGaitRenderer,
    FourPhaseGaitRenderer,
    SingleStepGaitRenderer,
    StaticGaitRenderer,
)
from spider_sim.landing import ConstrainedGaitAssembly
from spider_sim.leg import StaticLegAssembly
from spider_sim.renderer import FrameDebugInfo
from spider_sim.simulation import Simulation, SimulationSnapshot
from spider_sim.spider import BodySnapshot
from spider_sim.stepping import (
    FourPhaseGaitAssembly,
    SingleLegStepAssembly,
)


class GaitDemoSmokeTests(unittest.TestCase):
    def test_default_cli_runs_final_constrained_gait(self) -> None:
        result = main(["--max-frames", "2"])

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_static_demo_initializes_renders_and_stops(self) -> None:
        result = run(mode="static", max_frames=3)

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_cli_routes_to_static_gait_demo(self) -> None:
        result = main(
            [
                "--demo",
                "gait",
                "--gait-mode",
                "static",
                "--max-frames",
                "2",
            ]
        )

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_single_step_demo_initializes_renders_and_stops(self) -> None:
        result = run(mode="single-step", max_frames=3)

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_cli_routes_to_single_step_gait_demo(self) -> None:
        result = main(
            [
                "--demo",
                "gait",
                "--gait-mode",
                "single-step",
                "--max-frames",
                "2",
            ]
        )

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_four_phase_demo_initializes_renders_and_stops(self) -> None:
        result = run(mode="four-phase", max_frames=3)

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_cli_routes_to_four_phase_gait_demo(self) -> None:
        result = main(
            [
                "--demo",
                "gait",
                "--gait-mode",
                "four-phase",
                "--max-frames",
                "2",
            ]
        )

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_constrained_demo_initializes_renders_and_stops(self) -> None:
        result = run(mode="constrained", max_frames=3)

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_cli_routes_to_constrained_gait_demo(self) -> None:
        result = main(
            [
                "--demo",
                "gait",
                "--gait-mode",
                "constrained",
                "--max-frames",
                "2",
            ]
        )

        self.assertEqual(result, 0)
        self.assertFalse(pygame.get_init())

    def test_unknown_mode_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "static.*single-step"):
            run(mode="walking", max_frames=1)

    def test_invalid_frame_limit_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            run(mode="static", max_frames=0)


class StaticGaitRendererTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        pygame.init()

    @classmethod
    def tearDownClass(cls) -> None:
        pygame.quit()

    def test_draw_is_visible_and_does_not_mutate_snapshots(self) -> None:
        config = AppConfig()
        surface = pygame.Surface(config.window.size)
        simulation = Simulation(
            config.body,
            initial_position=(600.0, 400.0),
        )
        simulation.set_target((600.0, 400.0))
        simulation_before = simulation.snapshot()
        assembly_before = StaticLegAssembly(
            config.body,
            config.leg,
        ).solve(simulation_before.body)

        StaticGaitRenderer(surface, config).draw(
            simulation_before,
            assembly_before,
            FrameDebugInfo(
                frames_per_second=120.0,
                raw_delta_seconds=1.0 / 120.0,
                substep_count=1,
            ),
            debug_enabled=True,
        )

        self.assertEqual(simulation.snapshot(), simulation_before)
        self.assertEqual(
            StaticLegAssembly(config.body, config.leg).solve(
                simulation_before.body
            ),
            assembly_before,
        )
        self.assertNotEqual(
            surface.get_at((600, 400))[:3],
            config.window.background_color,
        )
        first_foot = assembly_before.legs[0].ik.foot
        self.assertNotEqual(
            surface.get_at((round(first_foot[0]), round(first_foot[1])))[:3],
            config.window.background_color,
        )


class SingleStepGaitRendererTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        pygame.init()

    @classmethod
    def tearDownClass(cls) -> None:
        pygame.quit()

    def test_draws_active_step_without_mutating_snapshot(self) -> None:
        config = AppConfig()
        surface = pygame.Surface(config.window.size)
        initial_body = BodySnapshot((600.0, 400.0), (0.0, 0.0), 0.0)
        moved_body = BodySnapshot((612.0, 400.0), (0.0, 0.0), 0.0)
        assembly = SingleLegStepAssembly(
            config.body,
            config.leg,
            FootStepConfig(
                trigger_distance=5.0,
                max_extension_ratio=1.0,
            ),
            initial_body,
        )
        assembly_snapshot = assembly.update(moved_body, 1.0 / 60.0)
        simulation_snapshot = SimulationSnapshot(
            elapsed_seconds=1.0 / 60.0,
            update_count=1,
            target_world=(900.0, 400.0),
            body=moved_body,
        )
        before = assembly_snapshot

        SingleStepGaitRenderer(surface, config).draw(
            simulation_snapshot,
            assembly_snapshot,
            FrameDebugInfo(
                frames_per_second=60.0,
                raw_delta_seconds=1.0 / 60.0,
                substep_count=2,
            ),
            debug_enabled=True,
        )

        self.assertEqual(assembly_snapshot, before)
        self.assertIsNotNone(assembly_snapshot.active_leg_id)
        self.assertNotEqual(
            surface.get_at((612, 400))[:3],
            config.window.background_color,
        )


class FourPhaseGaitRendererTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        pygame.init()

    @classmethod
    def tearDownClass(cls) -> None:
        pygame.quit()

    def test_draws_active_pair_without_mutating_snapshot(self) -> None:
        config = AppConfig()
        surface = pygame.Surface(config.window.size)
        initial_body = BodySnapshot((600.0, 400.0), (0.0, 0.0), 0.0)
        moved_body = BodySnapshot((635.0, 400.0), (0.0, 0.0), 0.0)
        assembly = FourPhaseGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            initial_body,
        )
        assembly_snapshot = assembly.update(moved_body, 1.0 / 60.0)
        simulation_snapshot = SimulationSnapshot(
            elapsed_seconds=1.0 / 60.0,
            update_count=1,
            target_world=(900.0, 400.0),
            body=moved_body,
        )
        before = assembly_snapshot

        FourPhaseGaitRenderer(surface, config).draw(
            simulation_snapshot,
            assembly_snapshot,
            FrameDebugInfo(
                frames_per_second=60.0,
                raw_delta_seconds=1.0 / 60.0,
                substep_count=2,
            ),
            debug_enabled=True,
        )

        self.assertEqual(assembly_snapshot, before)
        self.assertEqual(len(assembly_snapshot.active_leg_ids), 2)
        self.assertEqual(assembly_snapshot.support_count, 6)
        self.assertNotEqual(
            surface.get_at((635, 400))[:3],
            config.window.background_color,
        )


class ConstrainedGaitRendererTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        pygame.init()

    @classmethod
    def tearDownClass(cls) -> None:
        pygame.quit()

    def test_draws_regions_selected_candidates_and_diagnostics(
        self,
    ) -> None:
        config = AppConfig()
        surface = pygame.Surface(config.window.size)
        initial_body = BodySnapshot((600.0, 400.0), (0.0, 0.0), 0.0)
        moved_body = BodySnapshot((635.0, 400.0), (0.0, 0.0), 0.0)
        assembly = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        )
        assembly_snapshot = assembly.update(moved_body, 1.0 / 60.0)
        simulation_snapshot = SimulationSnapshot(
            elapsed_seconds=1.0 / 60.0,
            update_count=1,
            target_world=(900.0, 400.0),
            body=moved_body,
        )
        before = assembly_snapshot

        ConstrainedGaitRenderer(surface, config).draw(
            simulation_snapshot,
            assembly_snapshot,
            FrameDebugInfo(
                frames_per_second=60.0,
                raw_delta_seconds=1.0 / 60.0,
                substep_count=2,
            ),
            debug_enabled=True,
        )

        self.assertEqual(assembly_snapshot, before)
        self.assertEqual(len(assembly_snapshot.activity_regions), 8)
        selected = [
            candidate
            for candidate in assembly_snapshot.candidate_points
            if candidate.selected
        ]
        self.assertEqual(len(selected), 2)
        selected_point = selected[0].world_position
        self.assertEqual(
            surface.get_at(
                (round(selected_point[0]), round(selected_point[1]))
            )[:3],
            config.gait_debug.valid_candidate_color,
        )

        anomalous = ConstrainedGaitAssembly(
            config.body,
            config.leg,
            config.foot_step,
            config.four_phase_gait,
            config.landing,
            initial_body,
        ).update(
            BodySnapshot(
                (600.0, 400.0),
                (0.0, 0.0),
                math.pi,
            ),
            1.0 / 120.0,
        )
        self.assertTrue(anomalous.bone_intersections)
        self.assertTrue(anomalous.bend_anomalies)
        self.assertTrue(
            any(
                candidate.selected and candidate.degraded
                for candidate in anomalous.candidate_points
            )
        )
        ConstrainedGaitRenderer(surface, config).draw(
            SimulationSnapshot(
                elapsed_seconds=1.0 / 120.0,
                update_count=1,
                target_world=(600.0, 400.0),
                body=anomalous.body,
            ),
            anomalous,
            FrameDebugInfo(
                frames_per_second=120.0,
                raw_delta_seconds=1.0 / 120.0,
                substep_count=1,
            ),
            debug_enabled=True,
        )
        self.assertGreater(
            pygame.mask.from_threshold(
                surface,
                config.gait_debug.intersection_color,
                (1, 1, 1, 1),
            ).count(),
            0,
        )
        self.assertGreater(
            pygame.mask.from_threshold(
                surface,
                config.gait_debug.bend_anomaly_color,
                (1, 1, 1, 1),
            ).count(),
            0,
        )
        self.assertGreater(
            pygame.mask.from_threshold(
                surface,
                config.gait_debug.degraded_candidate_color,
                (1, 1, 1, 1),
            ).count(),
            0,
        )


if __name__ == "__main__":
    unittest.main()
