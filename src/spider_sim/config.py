"""Central configuration for the Pygame application."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

Color = tuple[int, int, int]


def _validate_color(name: str, value: Color) -> None:
    if not isinstance(value, tuple) or len(value) != 3 or any(
        isinstance(channel, bool)
        or not isinstance(channel, int)
        or not 0 <= channel <= 255
        for channel in value
    ):
        raise ValueError(f"{name} must contain three integer channels from 0 to 255")


@dataclass(frozen=True, slots=True)
class WindowConfig:
    """Window and presentation settings."""

    width: int = 1200
    height: int = 800
    title: str = "Project9 - Spider2D | Milestone 2"
    target_fps: int = 120
    background_color: Color = (18, 20, 26)

    def __post_init__(self) -> None:
        if isinstance(self.width, bool) or not isinstance(self.width, int):
            raise ValueError("window width must be an integer")
        if isinstance(self.height, bool) or not isinstance(self.height, int):
            raise ValueError("window height must be an integer")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("window dimensions must be positive")
        if isinstance(self.target_fps, bool) or not isinstance(self.target_fps, int):
            raise ValueError("target_fps must be an integer")
        if self.target_fps <= 0:
            raise ValueError("target_fps must be positive")
        if not isinstance(self.title, str) or not self.title.strip():
            raise ValueError("window title cannot be blank")
        _validate_color("background_color", self.background_color)

    @property
    def size(self) -> tuple[int, int]:
        """Return the window size in the format expected by Pygame."""
        return self.width, self.height


@dataclass(frozen=True, slots=True)
class TimingConfig:
    """Limits used by the delta-time update system."""

    max_frame_seconds: float = 0.05
    max_substep_seconds: float = 1.0 / 120.0
    max_substeps_per_frame: int = 64

    def __post_init__(self) -> None:
        values = {
            "max_frame_seconds": self.max_frame_seconds,
            "max_substep_seconds": self.max_substep_seconds,
        }
        for name, value in values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_substep_seconds > self.max_frame_seconds:
            raise ValueError(
                "max_substep_seconds cannot exceed max_frame_seconds"
            )
        if (
            isinstance(self.max_substeps_per_frame, bool)
            or not isinstance(self.max_substeps_per_frame, int)
            or self.max_substeps_per_frame <= 0
        ):
            raise ValueError("max_substeps_per_frame must be a positive integer")

        required_substeps = math.ceil(
            self.max_frame_seconds / self.max_substep_seconds
        )
        if required_substeps > self.max_substeps_per_frame:
            raise ValueError(
                "timing limits require more than max_substeps_per_frame"
            )


@dataclass(frozen=True, slots=True)
class BodyConfig:
    """Spider body dimensions and movement limits."""

    half_length: float = 34.0
    half_width: float = 22.0
    max_speed: float = 280.0
    max_acceleration: float = 900.0
    position_response: float = 4.0
    stop_radius: float = 3.0
    heading_speed_threshold: float = 8.0
    heading_response: float = 12.0
    max_turn_speed: float = math.radians(540.0)

    def __post_init__(self) -> None:
        positive_values = {
            "half_length": self.half_length,
            "half_width": self.half_width,
            "max_speed": self.max_speed,
            "max_acceleration": self.max_acceleration,
            "position_response": self.position_response,
            "heading_speed_threshold": self.heading_speed_threshold,
            "heading_response": self.heading_response,
            "max_turn_speed": self.max_turn_speed,
        }
        for name, value in positive_values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if isinstance(self.stop_radius, bool) or not isinstance(
            self.stop_radius, (int, float)
        ):
            raise ValueError("stop_radius must be a real number")
        if not math.isfinite(self.stop_radius) or self.stop_radius < 0.0:
            raise ValueError("stop_radius must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class DebugConfig:
    """Milestone 2 debug overlay settings."""

    enabled: bool = True
    font_size: int = 20
    target_radius: int = 8
    crosshair_radius: int = 13
    axis_forward_length: float = 64.0
    axis_right_length: float = 42.0
    velocity_vector_scale: float = 0.25
    hud_margin: int = 12
    body_fill_color: Color = (78, 83, 101)
    body_outline_color: Color = (210, 216, 232)
    body_front_color: Color = (255, 190, 92)
    target_color: Color = (241, 92, 110)
    target_line_color: Color = (100, 107, 128)
    forward_axis_color: Color = (94, 220, 132)
    right_axis_color: Color = (89, 170, 255)
    velocity_color: Color = (255, 215, 86)
    hud_text_color: Color = (224, 229, 240)

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError("debug enabled must be a boolean")

        integer_values = {
            "font_size": self.font_size,
            "target_radius": self.target_radius,
            "crosshair_radius": self.crosshair_radius,
            "hud_margin": self.hud_margin,
        }
        for name, value in integer_values.items():
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")

        real_values = {
            "axis_forward_length": self.axis_forward_length,
            "axis_right_length": self.axis_right_length,
            "velocity_vector_scale": self.velocity_vector_scale,
        }
        for name, value in real_values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        colors = {
            "body_fill_color": self.body_fill_color,
            "body_outline_color": self.body_outline_color,
            "body_front_color": self.body_front_color,
            "target_color": self.target_color,
            "target_line_color": self.target_line_color,
            "forward_axis_color": self.forward_axis_color,
            "right_axis_color": self.right_axis_color,
            "velocity_color": self.velocity_color,
            "hud_text_color": self.hud_text_color,
        }
        for name, color in colors.items():
            _validate_color(name, color)


@dataclass(frozen=True, slots=True)
class IKConfig:
    """Single-leg Two-Bone IK parameters for Milestone 3."""

    upper_length: float = 145.0
    lower_length: float = 125.0
    epsilon: float = 1e-6
    continuity_weight: float = 3.0
    default_bend_hint: tuple[float, float] = (0.0, 1.0)

    def __post_init__(self) -> None:
        positive_values = {
            "upper_length": self.upper_length,
            "lower_length": self.lower_length,
            "epsilon": self.epsilon,
        }
        for name, value in positive_values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.epsilon >= min(self.upper_length, self.lower_length):
            raise ValueError("epsilon must be smaller than both bone lengths")

        if isinstance(self.continuity_weight, bool) or not isinstance(
            self.continuity_weight, (int, float)
        ):
            raise ValueError("continuity_weight must be a real number")
        if (
            not math.isfinite(self.continuity_weight)
            or self.continuity_weight < 0.0
        ):
            raise ValueError("continuity_weight must be finite and non-negative")

        hint = self.default_bend_hint
        if (
            not isinstance(hint, tuple)
            or len(hint) != 2
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in hint
            )
            or math.hypot(*hint) == 0.0
        ):
            raise ValueError("default_bend_hint must be a finite non-zero tuple")


@dataclass(frozen=True, slots=True)
class IKDebugConfig:
    """Rendering parameters for the standalone single-leg IK demo."""

    title: str = "Project9 - Spider2D | Milestone 3 - Single Leg IK"
    font_size: int = 20
    bone_width: int = 8
    joint_radius: int = 9
    candidate_radius: int = 5
    target_radius: int = 8
    bend_hint_length: float = 58.0
    upper_bone_color: Color = (105, 190, 255)
    lower_bone_color: Color = (255, 190, 92)
    joint_color: Color = (230, 235, 246)
    requested_target_color: Color = (241, 92, 110)
    solved_target_color: Color = (101, 225, 143)
    reach_circle_color: Color = (69, 76, 96)
    candidate_color: Color = (180, 119, 255)
    bend_hint_color: Color = (87, 220, 215)
    hud_text_color: Color = (224, 229, 240)

    def __post_init__(self) -> None:
        if not isinstance(self.title, str) or not self.title.strip():
            raise ValueError("IK debug title cannot be blank")

        integer_values = {
            "font_size": self.font_size,
            "bone_width": self.bone_width,
            "joint_radius": self.joint_radius,
            "candidate_radius": self.candidate_radius,
            "target_radius": self.target_radius,
        }
        for name, value in integer_values.items():
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")

        if (
            isinstance(self.bend_hint_length, bool)
            or not isinstance(self.bend_hint_length, (int, float))
            or not math.isfinite(self.bend_hint_length)
            or self.bend_hint_length <= 0.0
        ):
            raise ValueError("bend_hint_length must be finite and positive")

        colors = {
            "upper_bone_color": self.upper_bone_color,
            "lower_bone_color": self.lower_bone_color,
            "joint_color": self.joint_color,
            "requested_target_color": self.requested_target_color,
            "solved_target_color": self.solved_target_color,
            "reach_circle_color": self.reach_circle_color,
            "candidate_color": self.candidate_color,
            "bend_hint_color": self.bend_hint_color,
            "hud_text_color": self.hud_text_color,
        }
        for name, color in colors.items():
            _validate_color(name, color)


@dataclass(frozen=True, slots=True)
class LegConfig:
    """Static eight-leg geometry used by the Milestone 4 assembly."""

    upper_length: float = 58.0
    lower_length: float = 62.0
    row_x_ratios: tuple[float, float, float, float] = (
        0.72,
        0.24,
        -0.24,
        -0.72,
    )
    hip_lateral_ratio: float = 0.88
    stance_lateral: float = 70.0
    longitudinal_splay: float = 0.60
    bend_longitudinal_bias: float = 1.0

    def __post_init__(self) -> None:
        positive_values = {
            "upper_length": self.upper_length,
            "lower_length": self.lower_length,
            "hip_lateral_ratio": self.hip_lateral_ratio,
            "stance_lateral": self.stance_lateral,
            "bend_longitudinal_bias": self.bend_longitudinal_bias,
        }
        for name, value in positive_values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        if isinstance(self.longitudinal_splay, bool) or not isinstance(
            self.longitudinal_splay, (int, float)
        ):
            raise ValueError("longitudinal_splay must be a real number")
        if (
            not math.isfinite(self.longitudinal_splay)
            or self.longitudinal_splay < 0.0
        ):
            raise ValueError(
                "longitudinal_splay must be finite and non-negative"
            )

        ratios = self.row_x_ratios
        if (
            not isinstance(ratios, tuple)
            or len(ratios) != 4
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in ratios
            )
        ):
            raise ValueError("row_x_ratios must be a finite four-value tuple")
        if not all(
            ratios[index] > ratios[index + 1]
            for index in range(len(ratios) - 1)
        ):
            raise ValueError("row_x_ratios must run strictly front to rear")
        if not (
            math.isclose(ratios[0], -ratios[3])
            and math.isclose(ratios[1], -ratios[2])
        ):
            raise ValueError("row_x_ratios must be front/rear symmetric")
        if any(abs(value) > 1.0 for value in ratios):
            raise ValueError("row_x_ratios must stay within the body length")
        if self.hip_lateral_ratio > 1.0:
            raise ValueError(
                "hip_lateral_ratio cannot exceed the lateral half-width"
            )


@dataclass(frozen=True, slots=True)
class FootStepConfig:
    """Milestone 5 single-active-leg stepping parameters."""

    trigger_distance: float = 28.0
    max_extension_ratio: float = 0.88
    lift_duration: float = 0.08
    swing_duration: float = 0.16
    lower_duration: float = 0.08
    lift_height: float = 18.0
    demo_max_speed: float = 10.0
    demo_max_acceleration: float = 80.0

    def __post_init__(self) -> None:
        positive_values = {
            "trigger_distance": self.trigger_distance,
            "lift_duration": self.lift_duration,
            "swing_duration": self.swing_duration,
            "lower_duration": self.lower_duration,
            "lift_height": self.lift_height,
            "demo_max_speed": self.demo_max_speed,
            "demo_max_acceleration": self.demo_max_acceleration,
        }
        for name, value in positive_values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        if isinstance(self.max_extension_ratio, bool) or not isinstance(
            self.max_extension_ratio,
            (int, float),
        ):
            raise ValueError("max_extension_ratio must be a real number")
        if (
            not math.isfinite(self.max_extension_ratio)
            or not 0.0 < self.max_extension_ratio <= 1.0
        ):
            raise ValueError(
                "max_extension_ratio must be finite and in (0, 1]"
            )


@dataclass(frozen=True, slots=True)
class FourPhaseGaitConfig:
    """Milestone 6 fixed diagonal groups and demo movement limits."""

    groups: tuple[
        tuple[str, str],
        tuple[str, str],
        tuple[str, str],
        tuple[str, str],
    ] = (
        ("L0", "R2"),
        ("R0", "L2"),
        ("L1", "R3"),
        ("R1", "L3"),
    )
    demo_max_speed: float = 18.0
    demo_max_acceleration: float = 120.0

    def __post_init__(self) -> None:
        groups = self.groups
        if (
            not isinstance(groups, tuple)
            or len(groups) != 4
            or any(
                not isinstance(group, tuple)
                or len(group) != 2
                or any(
                    not isinstance(leg_id, str) or not leg_id
                    for leg_id in group
                )
                for group in groups
            )
        ):
            raise ValueError("groups must contain four immutable leg-ID pairs")
        leg_ids = tuple(
            leg_id
            for group in groups
            for leg_id in group
        )
        if len(set(leg_ids)) != 8:
            raise ValueError("four-phase groups must contain eight unique legs")

        for name, value in {
            "demo_max_speed": self.demo_max_speed,
            "demo_max_acceleration": self.demo_max_acceleration,
        }.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True, slots=True)
class LandingConstraintConfig:
    """Milestone 7 landing-region, spacing, and continuity limits."""

    channel_half_width: float = 11.0
    safe_midline_margin: float = 24.0
    safe_outer_padding: float = 24.0
    knee_body_clearance: float = 3.0
    bone_clearance: float = 6.0
    phase_duration_scale: float = 0.625
    candidate_longitudinal_offsets: tuple[float, ...] = (
        0.0,
        -10.0,
        10.0,
    )
    candidate_lateral_offsets: tuple[float, ...] = (
        20.0,
        12.0,
        0.0,
        -12.0,
        -20.0,
    )
    min_foot_spacing: float = 18.0
    intersection_epsilon: float = 1e-6
    max_knee_jump: float = 48.0
    deferred_retry_duration: float = 0.12
    demo_max_speed: float = 20.0
    demo_max_acceleration: float = 120.0

    def __post_init__(self) -> None:
        positive_values = {
            "channel_half_width": self.channel_half_width,
            "safe_midline_margin": self.safe_midline_margin,
            "safe_outer_padding": self.safe_outer_padding,
            "knee_body_clearance": self.knee_body_clearance,
            "bone_clearance": self.bone_clearance,
            "phase_duration_scale": self.phase_duration_scale,
            "min_foot_spacing": self.min_foot_spacing,
            "intersection_epsilon": self.intersection_epsilon,
            "max_knee_jump": self.max_knee_jump,
            "deferred_retry_duration": self.deferred_retry_duration,
            "demo_max_speed": self.demo_max_speed,
            "demo_max_acceleration": self.demo_max_acceleration,
        }
        for name, value in positive_values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.phase_duration_scale > 1.0:
            raise ValueError("phase_duration_scale must not exceed 1.0")

        for name, offsets in {
            "candidate_longitudinal_offsets": (
                self.candidate_longitudinal_offsets
            ),
            "candidate_lateral_offsets": self.candidate_lateral_offsets,
        }.items():
            if (
                not isinstance(offsets, tuple)
                or not offsets
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    for value in offsets
                )
                or 0.0 not in offsets
            ):
                raise ValueError(
                    f"{name} must be a finite tuple containing zero"
                )


@dataclass(frozen=True, slots=True)
class GaitDebugConfig:
    """Rendering settings shared by Milestone 4 and 5 gait demos."""

    title: str = "Project9 - Spider2D | Milestone 4 - Static Eight Legs"
    step_title: str = (
        "Project9 - Spider2D | Milestone 5 - Locked Feet + Single Step"
    )
    four_phase_title: str = (
        "Project9 - Spider2D | Milestone 6 - Four-Phase Grouped Gait"
    )
    constrained_title: str = (
        "Project9 - Spider2D | Milestone 7 - Constrained Landings"
    )
    font_size: int = 20
    bone_width: int = 6
    joint_radius: int = 6
    foot_radius: int = 8
    label_offset: float = 14.0
    midline_half_length: float = 150.0
    left_upper_color: Color = (99, 190, 255)
    left_lower_color: Color = (125, 224, 242)
    right_upper_color: Color = (255, 154, 196)
    right_lower_color: Color = (255, 194, 116)
    hip_color: Color = (230, 235, 246)
    knee_color: Color = (215, 220, 235)
    foot_color: Color = (106, 232, 146)
    default_foot_color: Color = (98, 112, 145)
    label_color: Color = (224, 229, 240)
    midline_color: Color = (118, 126, 151)
    invalid_color: Color = (255, 76, 96)
    lift_color: Color = (255, 229, 92)
    swing_color: Color = (255, 166, 76)
    lower_color: Color = (190, 128, 255)
    landing_target_color: Color = (91, 232, 218)
    locked_foot_color: Color = (120, 135, 170)
    activity_region_color: Color = (67, 103, 145)
    valid_candidate_color: Color = (91, 232, 218)
    invalid_candidate_color: Color = (255, 92, 112)
    degraded_candidate_color: Color = (255, 166, 76)
    spacing_violation_color: Color = (255, 205, 82)
    intersection_color: Color = (255, 65, 92)
    bend_anomaly_color: Color = (204, 108, 255)
    bone_clearance_color: Color = (66, 214, 255)
    body_penetration_color: Color = (255, 112, 214)
    anatomy_violation_color: Color = (255, 224, 72)

    def __post_init__(self) -> None:
        if not isinstance(self.title, str) or not self.title.strip():
            raise ValueError("gait debug title cannot be blank")
        if not isinstance(self.step_title, str) or not self.step_title.strip():
            raise ValueError("step debug title cannot be blank")
        if (
            not isinstance(self.four_phase_title, str)
            or not self.four_phase_title.strip()
        ):
            raise ValueError("four-phase debug title cannot be blank")
        if (
            not isinstance(self.constrained_title, str)
            or not self.constrained_title.strip()
        ):
            raise ValueError("constrained debug title cannot be blank")

        integer_values = {
            "font_size": self.font_size,
            "bone_width": self.bone_width,
            "joint_radius": self.joint_radius,
            "foot_radius": self.foot_radius,
        }
        for name, value in integer_values.items():
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")

        real_values = {
            "label_offset": self.label_offset,
            "midline_half_length": self.midline_half_length,
        }
        for name, value in real_values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a real number")
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        colors = {
            "left_upper_color": self.left_upper_color,
            "left_lower_color": self.left_lower_color,
            "right_upper_color": self.right_upper_color,
            "right_lower_color": self.right_lower_color,
            "hip_color": self.hip_color,
            "knee_color": self.knee_color,
            "foot_color": self.foot_color,
            "default_foot_color": self.default_foot_color,
            "label_color": self.label_color,
            "midline_color": self.midline_color,
            "invalid_color": self.invalid_color,
            "lift_color": self.lift_color,
            "swing_color": self.swing_color,
            "lower_color": self.lower_color,
            "landing_target_color": self.landing_target_color,
            "locked_foot_color": self.locked_foot_color,
            "activity_region_color": self.activity_region_color,
            "valid_candidate_color": self.valid_candidate_color,
            "invalid_candidate_color": self.invalid_candidate_color,
            "degraded_candidate_color": self.degraded_candidate_color,
            "spacing_violation_color": self.spacing_violation_color,
            "intersection_color": self.intersection_color,
            "bend_anomaly_color": self.bend_anomaly_color,
            "bone_clearance_color": self.bone_clearance_color,
            "body_penetration_color": self.body_penetration_color,
            "anatomy_violation_color": self.anatomy_violation_color,
        }
        for name, color in colors.items():
            _validate_color(name, color)


@dataclass(frozen=True, slots=True)
class AppConfig:
    """Top-level immutable application configuration."""

    window: WindowConfig = field(default_factory=WindowConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    body: BodyConfig = field(default_factory=BodyConfig)
    debug: DebugConfig = field(default_factory=DebugConfig)
    ik: IKConfig = field(default_factory=IKConfig)
    ik_debug: IKDebugConfig = field(default_factory=IKDebugConfig)
    leg: LegConfig = field(default_factory=LegConfig)
    foot_step: FootStepConfig = field(default_factory=FootStepConfig)
    four_phase_gait: FourPhaseGaitConfig = field(
        default_factory=FourPhaseGaitConfig
    )
    landing: LandingConstraintConfig = field(
        default_factory=LandingConstraintConfig
    )
    gait_debug: GaitDebugConfig = field(default_factory=GaitDebugConfig)


DEFAULT_CONFIG = AppConfig()
