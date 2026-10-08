"""Static leg mounts and eight-leg assembly for Milestone 4."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

import pygame

from spider_sim.config import BodyConfig, LegConfig
from spider_sim.ik import IKResult, TwoBoneIKSolver
from spider_sim.math2d import heading_axes, local_to_world
from spider_sim.spider import BodySnapshot


class LegSide(IntEnum):
    """Body-local side sign; local +Y points to the spider's right."""

    LEFT = -1
    RIGHT = 1

    @property
    def prefix(self) -> str:
        return "L" if self is LegSide.LEFT else "R"


@dataclass(frozen=True, slots=True)
class LegMount:
    """Immutable body-local anatomy for one static leg."""

    leg_id: str
    side: LegSide
    row: int
    hip_local: tuple[float, float]
    default_foot_local: tuple[float, float]
    bend_hint_local: tuple[float, float]


@dataclass(frozen=True, slots=True)
class StaticLegSnapshot:
    """One solved static leg with its immutable mount definition."""

    mount: LegMount
    ik: IKResult


@dataclass(frozen=True, slots=True)
class StaticAssemblySnapshot:
    """All eight legs solved for one body pose."""

    body: BodySnapshot
    legs: tuple[StaticLegSnapshot, ...]


def build_default_mounts(
    body_config: BodyConfig,
    leg_config: LegConfig,
) -> tuple[LegMount, ...]:
    """Generate four mirrored rows on each side from centralized ratios."""
    mounts: list[LegMount] = []
    hip_lateral = body_config.half_width * leg_config.hip_lateral_ratio

    for row, row_ratio in enumerate(leg_config.row_x_ratios):
        hip_x = body_config.half_length * row_ratio
        foot_x = (
            hip_x
            + body_config.half_length
            * row_ratio
            * leg_config.longitudinal_splay
        )
        for side in (LegSide.LEFT, LegSide.RIGHT):
            side_sign = float(side)
            mounts.append(
                LegMount(
                    leg_id=f"{side.prefix}{row}",
                    side=side,
                    row=row,
                    hip_local=(
                        hip_x,
                        side_sign * hip_lateral,
                    ),
                    default_foot_local=(
                        foot_x,
                        side_sign
                        * (
                            body_config.half_width
                            + leg_config.stance_lateral
                        ),
                    ),
                    bend_hint_local=(
                        row_ratio * leg_config.bend_longitudinal_bias,
                        side_sign,
                    ),
                )
            )

    return tuple(mounts)


class StaticLegAssembly:
    """Solve eight non-locking legs directly from the current body pose."""

    def __init__(
        self,
        body_config: BodyConfig,
        leg_config: LegConfig,
        solver: TwoBoneIKSolver | None = None,
    ) -> None:
        self._leg_config = leg_config
        self._solver = solver or TwoBoneIKSolver()
        self._mounts = build_default_mounts(body_config, leg_config)
        if len(self._mounts) != 8:
            raise ValueError("static assembly must contain exactly eight legs")
        self._validate_mounts(body_config)

    @property
    def mounts(self) -> tuple[LegMount, ...]:
        """Return the immutable eight-mount definition."""
        return self._mounts

    def _validate_mounts(self, body_config: BodyConfig) -> None:
        """Reject cross-config geometry that cannot form a valid static pose."""
        leg_ids = {mount.leg_id for mount in self._mounts}
        if len(leg_ids) != len(self._mounts):
            raise ValueError("static assembly leg IDs must be unique")

        minimum_reach = abs(
            self._leg_config.upper_length
            - self._leg_config.lower_length
        )
        maximum_reach = (
            self._leg_config.upper_length
            + self._leg_config.lower_length
        )
        for mount in self._mounts:
            side_sign = float(mount.side)
            if side_sign * mount.hip_local[1] <= 0.0:
                raise ValueError(f"{mount.leg_id} hip crosses the body midline")
            if (
                side_sign * mount.default_foot_local[1]
                <= body_config.half_width
            ):
                raise ValueError(
                    f"{mount.leg_id} default foot must remain outside the body"
                )

            target_distance = math.dist(
                mount.hip_local,
                mount.default_foot_local,
            )
            if not minimum_reach <= target_distance <= maximum_reach:
                raise ValueError(
                    f"{mount.leg_id} default foot is outside the configured "
                    "two-bone reach interval"
                )

    def solve(self, body: BodySnapshot) -> StaticAssemblySnapshot:
        """Solve all legs from body-local defaults without locking any foot."""
        forward, right = heading_axes(body.heading)
        legs: list[StaticLegSnapshot] = []

        for mount in self._mounts:
            hip_world = local_to_world(
                mount.hip_local,
                body.position,
                body.heading,
            )
            foot_world = local_to_world(
                mount.default_foot_local,
                body.position,
                body.heading,
            )
            bend_local = pygame.Vector2(mount.bend_hint_local)
            bend_world = (
                forward * bend_local.x
                + right * bend_local.y
            )
            result = self._solver.solve(
                hip_world,
                foot_world,
                self._leg_config.upper_length,
                self._leg_config.lower_length,
                bend_world,
            )
            legs.append(
                StaticLegSnapshot(
                    mount=mount,
                    ik=result,
                )
            )

        return StaticAssemblySnapshot(
            body=body,
            legs=tuple(legs),
        )
