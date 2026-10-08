"""Acceptance tests for the Milestone 4 static eight-leg assembly."""

from __future__ import annotations

import math
import unittest

import pygame

from spider_sim.config import AppConfig, LegConfig
from spider_sim.ik import IKReachState
from spider_sim.leg import LegSide, StaticLegAssembly
from spider_sim.math2d import world_to_local
from spider_sim.spider import BodySnapshot


def _distance(
    point_a: tuple[float, float],
    point_b: tuple[float, float],
) -> float:
    return pygame.Vector2(point_a).distance_to(point_b)


def _local_points(
    assembly: StaticLegAssembly,
    body: BodySnapshot,
) -> dict[str, tuple[pygame.Vector2, pygame.Vector2, pygame.Vector2]]:
    snapshot = assembly.solve(body)
    return {
        leg.mount.leg_id: (
            world_to_local(leg.ik.hip, body.position, body.heading),
            world_to_local(leg.ik.knee, body.position, body.heading),
            world_to_local(leg.ik.foot, body.position, body.heading),
        )
        for leg in snapshot.legs
    }


class StaticLegAssemblyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = AppConfig()
        self.assembly = StaticLegAssembly(
            self.config.body,
            self.config.leg,
        )
        self.body = BodySnapshot(
            position=(600.0, 400.0),
            velocity=(0.0, 0.0),
            heading=0.0,
        )

    def test_has_exactly_eight_unique_two_bone_legs(self) -> None:
        snapshot = self.assembly.solve(self.body)

        self.assertEqual(
            [leg.mount.leg_id for leg in snapshot.legs],
            ["L0", "R0", "L1", "R1", "L2", "R2", "L3", "R3"],
        )
        self.assertEqual(len({leg.mount.leg_id for leg in snapshot.legs}), 8)
        self.assertEqual(
            sum(leg.mount.side is LegSide.LEFT for leg in snapshot.legs),
            4,
        )
        self.assertEqual(
            sum(leg.mount.side is LegSide.RIGHT for leg in snapshot.legs),
            4,
        )

        for leg in snapshot.legs:
            with self.subTest(leg=leg.mount.leg_id):
                self.assertAlmostEqual(
                    _distance(leg.ik.hip, leg.ik.knee),
                    self.config.leg.upper_length,
                    places=9,
                )
                self.assertAlmostEqual(
                    _distance(leg.ik.knee, leg.ik.foot),
                    self.config.leg.lower_length,
                    places=9,
                )

    def test_left_and_right_geometry_is_mirrored(self) -> None:
        snapshot = self.assembly.solve(self.body)
        by_side_and_row = {
            (leg.mount.side, leg.mount.row): leg
            for leg in snapshot.legs
        }

        for row in range(4):
            left = by_side_and_row[(LegSide.LEFT, row)]
            right = by_side_and_row[(LegSide.RIGHT, row)]
            with self.subTest(row=row):
                self.assertEqual(left.mount.hip_local[0], right.mount.hip_local[0])
                self.assertEqual(
                    left.mount.hip_local[1],
                    -right.mount.hip_local[1],
                )
                self.assertEqual(
                    left.mount.default_foot_local[0],
                    right.mount.default_foot_local[0],
                )
                self.assertEqual(
                    left.mount.default_foot_local[1],
                    -right.mount.default_foot_local[1],
                )

                left_knee = world_to_local(
                    left.ik.knee,
                    self.body.position,
                    self.body.heading,
                )
                right_knee = world_to_local(
                    right.ik.knee,
                    self.body.position,
                    self.body.heading,
                )
                self.assertAlmostEqual(left_knee.x, right_knee.x, places=9)
                self.assertAlmostEqual(left_knee.y, -right_knee.y, places=9)

    def test_rows_are_ordered_from_front_to_rear(self) -> None:
        for side in (LegSide.LEFT, LegSide.RIGHT):
            side_mounts = sorted(
                (
                    mount
                    for mount in self.assembly.mounts
                    if mount.side is side
                ),
                key=lambda mount: mount.row,
            )
            with self.subTest(side=side.name):
                self.assertEqual(
                    [mount.row for mount in side_mounts],
                    [0, 1, 2, 3],
                )
                self.assertTrue(
                    all(
                        side_mounts[index].hip_local[0]
                        > side_mounts[index + 1].hip_local[0]
                        for index in range(3)
                    )
                )
                self.assertTrue(
                    all(
                        side_mounts[index].default_foot_local[0]
                        > side_mounts[index + 1].default_foot_local[0]
                        for index in range(3)
                    )
                )

    def test_initial_targets_are_reachable_and_do_not_cross_midline(self) -> None:
        snapshot = self.assembly.solve(self.body)

        for leg in snapshot.legs:
            side_sign = float(leg.mount.side)
            points = {
                "hip": leg.ik.hip,
                "knee": leg.ik.knee,
                "foot": leg.ik.foot,
            }
            with self.subTest(leg=leg.mount.leg_id):
                self.assertIs(leg.ik.reach_state, IKReachState.REACHABLE)
                self.assertAlmostEqual(leg.ik.target_error, 0.0, places=12)
                self.assertEqual(leg.ik.foot, leg.ik.requested_target)
                hip_local = world_to_local(
                    leg.ik.hip,
                    self.body.position,
                    self.body.heading,
                )
                requested_target_local = world_to_local(
                    leg.ik.requested_target,
                    self.body.position,
                    self.body.heading,
                )
                self.assertAlmostEqual(
                    hip_local.x,
                    leg.mount.hip_local[0],
                    places=12,
                )
                self.assertAlmostEqual(
                    hip_local.y,
                    leg.mount.hip_local[1],
                    places=12,
                )
                self.assertAlmostEqual(
                    requested_target_local.x,
                    leg.mount.default_foot_local[0],
                    places=12,
                )
                self.assertAlmostEqual(
                    requested_target_local.y,
                    leg.mount.default_foot_local[1],
                    places=12,
                )
                for point_name, point in points.items():
                    local_point = world_to_local(
                        point,
                        self.body.position,
                        self.body.heading,
                    )
                    self.assertGreater(
                        side_sign * local_point.y,
                        0.0,
                        msg=f"{leg.mount.leg_id} {point_name} crossed midline",
                    )

    def test_unlocked_pose_is_rigid_in_body_local_coordinates(self) -> None:
        baseline = _local_points(self.assembly, self.body)
        poses = (
            BodySnapshot((-37.5, 912.25), (0.0, 0.0), math.radians(137.0)),
            BodySnapshot((120.0, -40.0), (0.0, 0.0), math.pi - 1e-7),
            BodySnapshot((-80.0, 70.0), (0.0, 0.0), -math.pi + 1e-7),
        )

        for body in poses:
            transformed = _local_points(self.assembly, body)
            with self.subTest(position=body.position, heading=body.heading):
                for leg_id, expected_points in baseline.items():
                    for expected, actual in zip(
                        expected_points,
                        transformed[leg_id],
                        strict=True,
                    ):
                        self.assertAlmostEqual(actual.x, expected.x, places=9)
                        self.assertAlmostEqual(actual.y, expected.y, places=9)

        moved_snapshot = self.assembly.solve(poses[0])
        baseline_snapshot = self.assembly.solve(self.body)
        self.assertNotEqual(
            moved_snapshot.legs[0].ik.foot,
            baseline_snapshot.legs[0].ik.foot,
        )

    def test_incompatible_default_pose_fails_before_solving(self) -> None:
        incompatible = LegConfig(upper_length=10.0, lower_length=10.0)

        with self.assertRaisesRegex(ValueError, "reach interval"):
            StaticLegAssembly(self.config.body, incompatible)


if __name__ == "__main__":
    unittest.main()
