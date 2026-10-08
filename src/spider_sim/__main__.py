"""Command-line entry point for ``python -m spider_sim``."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from spider_sim.app import run as run_body_demo


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the Project9 Spider2D Pygame application."
    )
    parser.add_argument(
        "--demo",
        choices=("body", "ik", "gait"),
        default="gait",
        help="Choose the body, single-leg IK, or eight-leg gait demo.",
    )
    parser.add_argument(
        "--gait-mode",
        choices=("static", "single-step", "four-phase", "constrained"),
        default="constrained",
        help=(
            "Choose M4 static, M5 single-step, M6 four-phase, "
            "or the final constrained mode."
        ),
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Exit after this many frames (intended for smoke tests).",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Parse command-line arguments and start the application."""
    args = _build_parser().parse_args(argv)
    if args.demo == "ik":
        from spider_sim.demos.ik_demo import run as run_ik_demo

        return run_ik_demo(max_frames=args.max_frames)
    if args.demo == "gait":
        from spider_sim.demos.gait_demo import run as run_gait_demo

        return run_gait_demo(
            mode=args.gait_mode,
            max_frames=args.max_frames,
        )
    return run_body_demo(max_frames=args.max_frames)


if __name__ == "__main__":
    raise SystemExit(main())
