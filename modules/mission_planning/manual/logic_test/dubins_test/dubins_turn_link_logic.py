from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float

    def as_tuple(self) -> tuple[float, float]:
        return (self.x, self.y)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_rad: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw_rad)


@dataclass(frozen=True)
class DubinsTurnLinkResult:
    start_point: Point2D
    end_point: Point2D
    link_point_1: Point2D
    link_point_2: Point2D
    exit_heading_deg: float
    entry_heading_deg: float
    heading_change_deg: float
    turn_radius_m: float
    total_length_m: float
    dubins_length_m: float
    dubins_type: str
    dubins_segment_lengths_m: tuple[float, float, float]
    path_points: tuple[Point2D, ...]
    curve_centers: tuple[Point2D, ...]
    curve_types: tuple[str, ...]

    @property
    def exit_point(self) -> Point2D:
        return self.link_point_1

    @property
    def entry_point(self) -> Point2D:
        return self.link_point_2


def _sub(a: Point2D, b: Point2D) -> tuple[float, float]:
    return (a.x - b.x, a.y - b.y)


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def _norm(v: tuple[float, float]) -> float:
    return math.hypot(v[0], v[1])


def _unit(v: tuple[float, float], name: str) -> tuple[float, float]:
    length = _norm(v)
    if length <= 1e-9:
        raise ValueError(f"{name} length is too small. Two input points must be different.")
    return (v[0] / length, v[1] / length)


def _heading_rad(v: tuple[float, float]) -> float:
    return math.atan2(v[1], v[0])


def _heading_deg(yaw_rad: float) -> float:
    return math.degrees(yaw_rad) % 360.0


def _wrap_deg(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _mod2pi(theta: float) -> float:
    return theta % (2.0 * math.pi)


def _polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), _mod2pi(math.atan2(y, x))


def _lsl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    p_sq = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))
    if p_sq < 0.0:
        return None
    tmp = math.atan2(math.cos(beta) - math.cos(alpha), d + math.sin(alpha) - math.sin(beta))
    return _mod2pi(-alpha + tmp), math.sqrt(p_sq), _mod2pi(beta - tmp)


def _rsr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    p_sq = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(beta) - math.sin(alpha))
    if p_sq < 0.0:
        return None
    tmp = math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta))
    return _mod2pi(alpha - tmp), math.sqrt(p_sq), _mod2pi(-beta + tmp)


def _lsr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    p_sq = -2.0 + d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
    return _mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp)


def _rsl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    p_sq = d * d - 2.0 + 2.0 * math.cos(alpha - beta) - 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
    return _mod2pi(alpha - tmp), p, _mod2pi(beta - tmp)


def _rlr(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    tmp = (6.0 - d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(tmp))
    t = _mod2pi(
        alpha
        - math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta))
        + p / 2.0
    )
    return t, p, _mod2pi(alpha - beta - t + p)


def _lrl(alpha: float, beta: float, d: float) -> tuple[float, float, float] | None:
    tmp = (6.0 - d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (-math.sin(alpha) + math.sin(beta))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(tmp))
    t = _mod2pi(
        -alpha
        - math.atan2(math.cos(alpha) - math.cos(beta), d + math.sin(alpha) - math.sin(beta))
        + p / 2.0
    )
    return t, p, _mod2pi(beta - alpha - t + p)


_PATH_FUNCS = {
    "LSL": _lsl,
    "RSR": _rsr,
    "LSR": _lsr,
    "RSL": _rsl,
    "RLR": _rlr,
    "LRL": _lrl,
}


def _cross(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


def dubins_candidate_paths(
    start_pose: Pose2D,
    end_pose: Pose2D,
    radius_m: float,
    allow_ccc: bool = False,
) -> list[dict[str, object]]:
    if radius_m <= 0.0:
        raise ValueError("Turn radius must be positive.")

    dx = (end_pose.x - start_pose.x) / radius_m
    dy = (end_pose.y - start_pose.y) / radius_m
    d, theta = _polar(dx, dy)
    alpha = _mod2pi(start_pose.yaw_rad - theta)
    beta = _mod2pi(end_pose.yaw_rad - theta)

    path_funcs = (
        _PATH_FUNCS
        if allow_ccc
        else {key: value for key, value in _PATH_FUNCS.items() if key in ("LSL", "RSR", "LSR", "RSL")}
    )
    candidates: list[dict[str, object]] = []
    for path_type, func in path_funcs.items():
        params = func(alpha, beta, d)
        if params is None:
            continue
        t, p, q = params
        segment_lengths = (t * radius_m, p * radius_m, q * radius_m)
        total_length = sum(segment_lengths)
        candidates.append(
            {
                "length": total_length,
                "types": path_type,
                "params": params,
                "seg_lengths": segment_lengths,
            }
        )

    if not candidates:
        raise RuntimeError("No feasible Dubins path found.")
    candidates.sort(key=lambda item: float(item["length"]))
    return candidates


def dubins_shortest_path(
    start_pose: Pose2D,
    end_pose: Pose2D,
    radius_m: float,
    allow_ccc: bool = False,
) -> dict[str, object]:
    return dubins_candidate_paths(start_pose, end_pose, radius_m, allow_ccc=allow_ccc)[0]


def _choose_outside_sweep_candidate(
    start_pose: Pose2D,
    end_pose: Pose2D,
    radius_m: float,
) -> dict[str, object]:
    candidates = dubins_candidate_paths(start_pose, end_pose, radius_m, allow_ccc=True)
    csc_candidates = [candidate for candidate in candidates if "S" in str(candidate["types"])]

    start_to_end = (end_pose.x - start_pose.x, end_pose.y - start_pose.y)
    end_to_start = (start_pose.x - end_pose.x, start_pose.y - end_pose.y)
    start_heading = (math.cos(start_pose.yaw_rad), math.sin(start_pose.yaw_rad))
    end_heading = (math.cos(end_pose.yaw_rad), math.sin(end_pose.yaw_rad))

    start_side = _cross(start_heading, start_to_end)
    end_side = _cross(end_heading, end_to_start)

    preferred_first = "R" if start_side >= 0.0 else "L"
    preferred_last = "R" if end_side >= 0.0 else "L"

    def score(candidate: dict[str, object]) -> tuple[int, int, float]:
        path_type = str(candidate["types"])
        is_csc = 0 if "S" in path_type else 1
        turn_mismatch = int(path_type[0] != preferred_first) + int(path_type[-1] != preferred_last)
        return (is_csc, turn_mismatch, float(candidate["length"]))

    pool = csc_candidates if csc_candidates else candidates
    return min(pool, key=score)


def _advance_pose(pose: Pose2D, seg_type: str, ds: float, radius_m: float) -> Pose2D:
    x, y, yaw = pose.x, pose.y, pose.yaw_rad
    if seg_type == "S":
        return Pose2D(x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw)

    dtheta = ds / radius_m
    if seg_type == "L":
        return Pose2D(
            x + radius_m * (math.sin(yaw + dtheta) - math.sin(yaw)),
            y + radius_m * (-math.cos(yaw + dtheta) + math.cos(yaw)),
            yaw + dtheta,
        )
    if seg_type == "R":
        return Pose2D(
            x + radius_m * (-math.sin(yaw - dtheta) + math.sin(yaw)),
            y + radius_m * (math.cos(yaw - dtheta) - math.cos(yaw)),
            yaw - dtheta,
        )
    raise ValueError(f"Unknown segment type: {seg_type}")


def _curve_center(pose: Pose2D, seg_type: str, radius_m: float) -> Point2D:
    if seg_type == "L":
        return Point2D(
            pose.x - radius_m * math.sin(pose.yaw_rad),
            pose.y + radius_m * math.cos(pose.yaw_rad),
        )
    if seg_type == "R":
        return Point2D(
            pose.x + radius_m * math.sin(pose.yaw_rad),
            pose.y - radius_m * math.cos(pose.yaw_rad),
        )
    raise ValueError(f"Straight segment has no curve center: {seg_type}")


def _append_unique(points: list[Point2D], extra: Iterable[Point2D]) -> None:
    for point in extra:
        if not points or _distance(points[-1], point) > 1e-9:
            points.append(point)


def _sample_segment(
    start_pose: Pose2D,
    seg_type: str,
    seg_length_m: float,
    radius_m: float,
    sample_step_m: float,
) -> tuple[Pose2D, list[Point2D]]:
    pose = start_pose
    samples: list[Point2D] = []
    traveled = 0.0
    while traveled < seg_length_m - 1e-9:
        ds = min(sample_step_m, seg_length_m - traveled)
        pose = _advance_pose(pose, seg_type, ds, radius_m)
        samples.append(Point2D(pose.x, pose.y))
        traveled += ds
    return pose, samples


def compute_turn_link(
    prev_start: Point2D,
    prev_end: Point2D,
    next_start: Point2D,
    next_end: Point2D,
    radius_m: float,
    search_limit_m: float | None = None,
    grid_step_m: float | None = None,
    sample_step_m: float = 5.0,
    allow_ccc: bool = False,
    path_policy: str = "csc_shortest",
) -> DubinsTurnLinkResult:
    del search_limit_m
    del grid_step_m

    if radius_m <= 0.0:
        raise ValueError("Turn radius must be positive.")
    if sample_step_m <= 0.0:
        raise ValueError("Sample step must be positive.")

    exit_dir = _unit(_sub(prev_end, prev_start), "prev segment")
    entry_dir = _unit(_sub(next_end, next_start), "next segment")
    start_pose = Pose2D(prev_end.x, prev_end.y, _heading_rad(exit_dir))
    end_pose = Pose2D(next_start.x, next_start.y, _heading_rad(entry_dir))

    if path_policy == "outside_sweep":
        info = _choose_outside_sweep_candidate(start_pose, end_pose, radius_m)
    elif path_policy == "all_shortest":
        info = dubins_shortest_path(start_pose, end_pose, radius_m, allow_ccc=True)
    elif path_policy == "csc_shortest":
        info = dubins_shortest_path(start_pose, end_pose, radius_m, allow_ccc=False)
    else:
        info = dubins_shortest_path(start_pose, end_pose, radius_m, allow_ccc=allow_ccc)
    path_type = str(info["types"])
    seg_lengths = tuple(float(v) for v in info["seg_lengths"])

    path_points: list[Point2D] = [Point2D(start_pose.x, start_pose.y)]
    curve_centers: list[Point2D] = []
    curve_types: list[str] = []
    boundary_points: list[Point2D] = []

    pose = start_pose
    for seg_type, seg_length in zip(path_type, seg_lengths):
        if seg_type in ("L", "R"):
            curve_centers.append(_curve_center(pose, seg_type, radius_m))
            curve_types.append(seg_type)
        pose, samples = _sample_segment(
            start_pose=pose,
            seg_type=seg_type,
            seg_length_m=seg_length,
            radius_m=radius_m,
            sample_step_m=sample_step_m,
        )
        _append_unique(path_points, samples)
        boundary_points.append(Point2D(pose.x, pose.y))

    final_error = _distance(Point2D(pose.x, pose.y), Point2D(end_pose.x, end_pose.y))
    yaw_error = abs(_wrap_deg(_heading_deg(pose.yaw_rad) - _heading_deg(end_pose.yaw_rad)))
    if final_error > 1e-4 or yaw_error > 1e-4:
        raise RuntimeError(
            f"Dubins reconstruction error is too large: pos={final_error:.6f} m, yaw={yaw_error:.6f} deg"
        )

    path_points[-1] = Point2D(end_pose.x, end_pose.y)
    link_point_1 = boundary_points[0]
    link_point_2 = boundary_points[1]

    return DubinsTurnLinkResult(
        start_point=Point2D(start_pose.x, start_pose.y),
        end_point=Point2D(end_pose.x, end_pose.y),
        link_point_1=link_point_1,
        link_point_2=link_point_2,
        exit_heading_deg=_heading_deg(start_pose.yaw_rad),
        entry_heading_deg=_heading_deg(end_pose.yaw_rad),
        heading_change_deg=_wrap_deg(_heading_deg(end_pose.yaw_rad) - _heading_deg(start_pose.yaw_rad)),
        turn_radius_m=radius_m,
        total_length_m=float(info["length"]),
        dubins_length_m=float(info["length"]),
        dubins_type=path_type,
        dubins_segment_lengths_m=seg_lengths,
        path_points=tuple(path_points),
        curve_centers=tuple(curve_centers),
        curve_types=tuple(curve_types),
    )


def format_result(result: DubinsTurnLinkResult) -> str:
    seg1, seg2, seg3 = result.dubins_segment_lengths_m
    curve_text = ", ".join(
        f"{seg_type}@({center.x:.1f}, {center.y:.1f})"
        for seg_type, center in zip(result.curve_types, result.curve_centers)
    )
    return "\n".join(
        [
            "[1] = end of first Dubins segment",
            f"      ({result.link_point_1.x:.3f}, {result.link_point_1.y:.3f}) m",
            "[2] = end of second Dubins segment",
            f"      ({result.link_point_2.x:.3f}, {result.link_point_2.y:.3f}) m",
            "",
            f"Start pose : ({result.start_point.x:.3f}, {result.start_point.y:.3f}) m",
            f"End pose   : ({result.end_point.x:.3f}, {result.end_point.y:.3f}) m",
            f"Exit heading : {result.exit_heading_deg:.3f} deg",
            f"Entry heading: {result.entry_heading_deg:.3f} deg",
            f"Heading delta: {result.heading_change_deg:.3f} deg",
            "",
            f"Turn radius   : {result.turn_radius_m:.3f} m",
            f"Dubins type   : {result.dubins_type}",
            f"Segment lengths: ({seg1:.3f}, {seg2:.3f}, {seg3:.3f}) m",
            f"Total length  : {result.total_length_m:.3f} m",
            f"Curve centers : {curve_text if curve_text else '-'}",
        ]
    )


def _parse_point(raw: str) -> Point2D:
    cleaned = raw.replace(";", ",").replace("/", ",").strip()
    if not cleaned:
        raise argparse.ArgumentTypeError("Point text is empty.")
    parts = [part for part in cleaned.replace(" ", "").split(",") if part]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Point must be in 'x,y' format.")
    try:
        return Point2D(float(parts[0]), float(parts[1]))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Point coordinates must be numeric.") from exc


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Dubins turn-link test utility for two input line segments.")
    parser.add_argument("--prev-start", type=_parse_point, required=True, help="Previous segment start: x,y")
    parser.add_argument("--prev-end", type=_parse_point, required=True, help="Previous segment end: x,y")
    parser.add_argument("--next-start", type=_parse_point, required=True, help="Next segment start: x,y")
    parser.add_argument("--next-end", type=_parse_point, required=True, help="Next segment end: x,y")
    parser.add_argument("--radius", type=float, required=True, help="Turn radius in meters")
    parser.add_argument("--sample-step", type=float, default=5.0, help="Plot sample step in meters")
    parser.add_argument("--all-dubins", action="store_true", help="Allow CCC paths (RLR/LRL) as well")
    parser.add_argument(
        "--path-policy",
        choices=("csc_shortest", "all_shortest", "outside_sweep"),
        default="csc_shortest",
        help="Path selection policy",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    result = compute_turn_link(
        prev_start=args.prev_start,
        prev_end=args.prev_end,
        next_start=args.next_start,
        next_end=args.next_end,
        radius_m=float(args.radius),
        sample_step_m=float(args.sample_step),
        allow_ccc=bool(args.all_dubins),
        path_policy=str(args.path_policy),
    )
    print(format_result(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
