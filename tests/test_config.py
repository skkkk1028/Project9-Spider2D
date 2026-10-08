"""Tests for central configuration validation."""

from __future__ import annotations

import math
import unittest

from spider_sim.config import (
    AppConfig,
    BodyConfig,
    DebugConfig,
    FootStepConfig,
    FourPhaseGaitConfig,
    GaitDebugConfig,
    IKConfig,
    IKDebugConfig,
    LandingConstraintConfig,
    LegConfig,
    TimingConfig,
    WindowConfig,
)


class WindowConfigTests(unittest.TestCase):
    def test_default_app_config_is_valid(self) -> None:
        config = AppConfig()

        self.assertEqual(config.window.size, (1200, 800))
        self.assertGreater(config.window.target_fps, 0)
        self.assertLessEqual(
            config.timing.max_substep_seconds,
            config.timing.max_frame_seconds,
        )

    def test_invalid_window_size_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            WindowConfig(width=0)

    def test_invalid_color_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            WindowConfig(background_color=(18, 20, 999))

    def test_color_must_be_an_immutable_tuple(self) -> None:
        for invalid_color in (None, 42, [18, 20, 26]):
            with self.subTest(invalid_color=invalid_color):
                with self.assertRaises(ValueError):
                    WindowConfig(
                        background_color=invalid_color  # type: ignore[arg-type]
                    )

    def test_invalid_title_is_rejected(self) -> None:
        for invalid_title in ("", "   ", None):
            with self.subTest(invalid_title=invalid_title):
                with self.assertRaises(ValueError):
                    WindowConfig(title=invalid_title)  # type: ignore[arg-type]


class TimingConfigTests(unittest.TestCase):
    def test_substep_cannot_exceed_frame_limit(self) -> None:
        with self.assertRaises(ValueError):
            TimingConfig(
                max_frame_seconds=0.01,
                max_substep_seconds=0.02,
            )

    def test_pathological_substep_ratio_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TimingConfig(
                max_frame_seconds=0.05,
                max_substep_seconds=1e-20,
            )


class BodyConfigTests(unittest.TestCase):
    def test_invalid_movement_limit_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            BodyConfig(max_speed=0.0)

    def test_stop_radius_can_be_zero_but_not_negative(self) -> None:
        self.assertEqual(BodyConfig(stop_radius=0.0).stop_radius, 0.0)
        with self.assertRaises(ValueError):
            BodyConfig(stop_radius=-1.0)


class DebugConfigTests(unittest.TestCase):
    def test_invalid_debug_color_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            DebugConfig(target_color=(256, 0, 0))


class IKConfigTests(unittest.TestCase):
    def test_default_ik_config_is_valid(self) -> None:
        config = IKConfig()

        self.assertGreater(config.upper_length, 0.0)
        self.assertGreater(config.lower_length, 0.0)
        self.assertLess(
            config.epsilon,
            min(config.upper_length, config.lower_length),
        )

    def test_epsilon_must_be_smaller_than_bone_lengths(self) -> None:
        with self.assertRaises(ValueError):
            IKConfig(
                upper_length=10.0,
                lower_length=20.0,
                epsilon=10.0,
            )

    def test_bend_hint_must_be_non_zero_and_immutable(self) -> None:
        for invalid_hint in ((0.0, 0.0), [0.0, 1.0], (math.nan, 1.0)):
            with self.subTest(invalid_hint=invalid_hint):
                with self.assertRaises(ValueError):
                    IKConfig(
                        default_bend_hint=invalid_hint  # type: ignore[arg-type]
                    )

        tiny_hint = IKConfig(default_bend_hint=(0.0, 1e-12))
        self.assertEqual(tiny_hint.default_bend_hint, (0.0, 1e-12))


class IKDebugConfigTests(unittest.TestCase):
    def test_invalid_bone_width_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            IKDebugConfig(bone_width=0)


class LegConfigTests(unittest.TestCase):
    def test_default_rows_are_mirrored_and_run_front_to_rear(self) -> None:
        ratios = LegConfig().row_x_ratios

        self.assertEqual(len(ratios), 4)
        self.assertTrue(
            all(
                ratios[index] > ratios[index + 1]
                for index in range(len(ratios) - 1)
            )
        )
        self.assertAlmostEqual(ratios[0], -ratios[3])
        self.assertAlmostEqual(ratios[1], -ratios[2])

    def test_row_ratios_must_be_a_body_bounded_four_tuple(self) -> None:
        invalid_ratios = (
            [0.75, 0.25, -0.25, -0.75],
            (0.75, 0.25, -0.25),
            (0.75, -0.25, 0.25, -0.75),
            (0.75, 0.20, -0.25, -0.75),
            (1.20, 0.25, -0.25, -1.20),
        )
        for ratios in invalid_ratios:
            with self.subTest(ratios=ratios):
                with self.assertRaises(ValueError):
                    LegConfig(row_x_ratios=ratios)  # type: ignore[arg-type]

    def test_hip_lateral_ratio_cannot_exceed_half_width(self) -> None:
        with self.assertRaises(ValueError):
            LegConfig(hip_lateral_ratio=1.01)


class GaitDebugConfigTests(unittest.TestCase):
    def test_invalid_foot_radius_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GaitDebugConfig(foot_radius=0)

    def test_invalid_leg_color_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GaitDebugConfig(left_upper_color=(-1, 100, 100))


class FootStepConfigTests(unittest.TestCase):
    def test_phase_durations_and_thresholds_must_be_positive(self) -> None:
        for field_name in (
            "trigger_distance",
            "lift_duration",
            "swing_duration",
            "lower_duration",
            "lift_height",
        ):
            with self.subTest(field_name=field_name):
                with self.assertRaises(ValueError):
                    FootStepConfig(**{field_name: 0.0})

    def test_extension_ratio_must_be_in_unit_interval(self) -> None:
        for invalid_ratio in (0.0, -0.1, 1.01, math.inf):
            with self.subTest(invalid_ratio=invalid_ratio):
                with self.assertRaises(ValueError):
                    FootStepConfig(max_extension_ratio=invalid_ratio)


class FourPhaseGaitConfigTests(unittest.TestCase):
    def test_default_groups_cover_eight_unique_legs(self) -> None:
        groups = FourPhaseGaitConfig().groups

        self.assertEqual(len(groups), 4)
        self.assertTrue(all(len(group) == 2 for group in groups))
        self.assertEqual(
            len({leg_id for group in groups for leg_id in group}),
            8,
        )

    def test_groups_must_be_four_immutable_unique_pairs(self) -> None:
        invalid_groups = (
            [("L0", "R2"), ("R0", "L2"), ("L1", "R3"), ("R1", "L3")],
            (("L0", "R2"), ("R0", "L2"), ("L1", "R3")),
            (("L0", "R2"), ("R0", "L2"), ("L1", "R3"), ("L0", "L3")),
        )
        for groups in invalid_groups:
            with self.subTest(groups=groups):
                with self.assertRaises(ValueError):
                    FourPhaseGaitConfig(groups=groups)  # type: ignore[arg-type]


class LandingConstraintConfigTests(unittest.TestCase):
    def test_default_candidate_grid_contains_zero_offsets(self) -> None:
        config = LandingConstraintConfig()

        self.assertIn(0.0, config.candidate_longitudinal_offsets)
        self.assertIn(0.0, config.candidate_lateral_offsets)
        self.assertGreater(config.candidate_lateral_offsets[0], 0.0)
        self.assertGreater(config.min_foot_spacing, 0.0)

    def test_numeric_limits_must_be_finite_and_positive(self) -> None:
        for field_name in (
            "channel_half_width",
            "safe_midline_margin",
            "safe_outer_padding",
            "min_foot_spacing",
            "intersection_epsilon",
            "max_knee_jump",
            "deferred_retry_duration",
        ):
            with self.subTest(field_name=field_name):
                with self.assertRaises(ValueError):
                    LandingConstraintConfig(**{field_name: 0.0})

    def test_candidate_offsets_are_finite_tuples_containing_zero(
        self,
    ) -> None:
        invalid_offsets = (
            (),
            (1.0, 2.0),
            [0.0, 1.0],
            (0.0, math.inf),
        )
        for offsets in invalid_offsets:
            with self.subTest(offsets=offsets):
                with self.assertRaises(ValueError):
                    LandingConstraintConfig(
                        candidate_lateral_offsets=offsets  # type: ignore[arg-type]
                    )


if __name__ == "__main__":
    unittest.main()
