"""Visual report helpers for repeated performance-evaluation runs.

The module deliberately has no dependency on Qt, the SIM server, or the
mission-status monitor.  Runtime code can therefore import it before any GUI
process starts.  Pillow is imported lazily by :func:`render_area_comparison`;
the JSON and HTML writers continue to work when image rendering is unavailable.
"""

from __future__ import annotations

import hashlib
import html
import json
import math
import os
import tempfile
from dataclasses import dataclass
from datetime import date, datetime
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence
from urllib.parse import quote


_IMAGE_SIZE = (1600, 900)
_PALETTE = (
    "#2f8f83",
    "#d59432",
    "#cf5b4c",
    "#4b80a8",
    "#6b974e",
    "#a96549",
    "#7666a9",
    "#3c9aaf",
)
_AIRCRAFT_COLORS = {
    1: "#2f8f83",
    2: "#d59432",
    3: "#cf5b4c",
    4: "#2f8f83",
    5: "#d59432",
    6: "#cf5b4c",
}


@dataclass(frozen=True)
class _Shape:
    kind: str
    paths: tuple[tuple[tuple[float, float], ...], ...]
    label: str = ""
    color: str = ""
    input_mission_id: int | None = None
    aircraft_id: int | None = None


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (datetime, date)):
        return value.isoformat()
    if isinstance(value, set):
        return sorted(value, key=str)
    if hasattr(value, "__dict__"):
        return vars(value)
    return str(value)


def _atomic_write_text(path: Path, text: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=f".{path.name}.",
            suffix=".tmp",
            dir=str(path.parent),
            delete=False,
        ) as stream:
            temporary = stream.name
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary:
            try:
                os.unlink(temporary)
            except OSError:
                pass


def write_json(path: str | Path, payload: Any) -> None:
    """Write *payload* as UTF-8 JSON using an atomic same-directory replace."""

    serialized = json.dumps(
        payload,
        ensure_ascii=False,
        indent=2,
        default=_json_default,
    )
    _atomic_write_text(Path(path), serialized + "\n")


def _as_mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _as_rows(value: Any) -> list[Any]:
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    return []


def _pick(mapping: Any, *keys: str, default: Any = None) -> Any:
    source = _as_mapping(mapping)
    for key in keys:
        if key in source:
            return source[key]
    lowered = {str(key).lower(): value for key, value in source.items()}
    for key in keys:
        if key.lower() in lowered:
            return lowered[key.lower()]
    return default


def _int_or_none(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def _number_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _coordinate(value: Any) -> tuple[float, float] | None:
    if isinstance(value, Mapping):
        lon = _number_or_none(_pick(value, "longitude", "lon", "lng", "x"))
        lat = _number_or_none(_pick(value, "latitude", "lat", "y"))
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 2:
        first = _number_or_none(value[0])
        second = _number_or_none(value[1])
        if first is None or second is None:
            return None
        # GeoJSON is [lon, lat].  Accept the common ICD [lat, lon] mistake when
        # the second value cannot be a latitude but the first one can.
        if abs(second) > 90.0 and abs(first) <= 90.0:
            lon, lat = second, first
        else:
            lon, lat = first, second
    else:
        return None
    if lon is None or lat is None or abs(lon) > 180.0 or abs(lat) > 90.0:
        return None
    return float(lon), float(lat)


def _path(values: Any) -> tuple[tuple[float, float], ...]:
    points: list[tuple[float, float]] = []
    for value in _as_rows(values):
        point = _coordinate(value)
        if point is None:
            continue
        if not points or point != points[-1]:
            points.append(point)
    return tuple(points)


def _valid_hex_color(value: Any) -> str:
    text = str(value or "").strip()
    if len(text) in (4, 7) and text.startswith("#"):
        digits = text[1:]
        if all(char in "0123456789abcdefABCDEF" for char in digits):
            if len(text) == 4:
                return "#" + "".join(char * 2 for char in digits)
            return text.lower()
    return ""


def _stable_color(label: str, input_id: int | None, aircraft_id: int | None) -> str:
    if aircraft_id is not None and aircraft_id in _AIRCRAFT_COLORS:
        return _AIRCRAFT_COLORS[aircraft_id]
    token = str(input_id) if input_id is not None else str(label or "shape")
    digest = hashlib.sha1(token.encode("utf-8", errors="replace")).digest()
    return _PALETTE[digest[0] % len(_PALETTE)]


def _context(source: Any, parent: Mapping[str, Any] | None = None) -> dict[str, Any]:
    parent = parent or {}
    item = _as_mapping(source)
    input_id = _int_or_none(
        _pick(item, "inputMissionID", "inputMissionId", default=parent.get("inputMissionID"))
    )
    aircraft_id = _int_or_none(
        _pick(item, "aircraftID", "aircraftId", "ownerAircraftID", default=parent.get("aircraftID"))
    )
    label = str(
        _pick(
            item,
            "label",
            "name",
            "regionLabel",
            "inputMissionTypeLabel",
            default=parent.get("label") or "",
        )
        or ""
    )
    color = _valid_hex_color(_pick(item, "color", default=parent.get("color")))
    return {
        "inputMissionID": input_id,
        "aircraftID": aircraft_id,
        "label": label,
        "color": color,
    }


def _shape_label(ctx: Mapping[str, Any]) -> str:
    explicit = str(ctx.get("label") or "").strip()
    tokens: list[str] = []
    input_id = _int_or_none(ctx.get("inputMissionID"))
    aircraft_id = _int_or_none(ctx.get("aircraftID"))
    if input_id is not None:
        tokens.append(f"Input {input_id}")
    if aircraft_id is not None:
        tokens.append(f"UAV{aircraft_id - 3}" if aircraft_id in (4, 5, 6) else f"AC {aircraft_id}")
    if explicit and explicit not in tokens:
        tokens.append(explicit)
    return " · ".join(tokens)


def _append_shape(
    output: list[_Shape],
    kind: str,
    paths: Iterable[Sequence[tuple[float, float]]],
    ctx: Mapping[str, Any],
) -> bool:
    normalized: list[tuple[tuple[float, float], ...]] = []
    minimum = 3 if kind == "polygon" else 2
    for values in paths:
        row = tuple(values)
        if len(row) >= minimum:
            normalized.append(row)
    if not normalized:
        return False
    input_id = _int_or_none(ctx.get("inputMissionID"))
    aircraft_id = _int_or_none(ctx.get("aircraftID"))
    label = _shape_label(ctx)
    output.append(
        _Shape(
            kind=kind,
            paths=tuple(normalized),
            label=label,
            color=_valid_hex_color(ctx.get("color")) or _stable_color(label, input_id, aircraft_id),
            input_mission_id=input_id,
            aircraft_id=aircraft_id,
        )
    )
    return True


def _extract_geojson_geometry(
    geometry: Any,
    properties: Mapping[str, Any],
    output: list[_Shape],
) -> bool:
    obj = _as_mapping(geometry)
    kind = str(obj.get("type") or "")
    coords = obj.get("coordinates")
    ctx = _context(properties)
    if kind == "Polygon":
        return _append_shape(output, "polygon", (_path(ring) for ring in _as_rows(coords)), ctx)
    if kind == "MultiPolygon":
        found = False
        for polygon in _as_rows(coords):
            found = _append_shape(output, "polygon", (_path(ring) for ring in _as_rows(polygon)), ctx) or found
        return found
    if kind == "LineString":
        return _append_shape(output, "line", [_path(coords)], ctx)
    if kind == "MultiLineString":
        return _append_shape(output, "line", (_path(line) for line in _as_rows(coords)), ctx)
    if kind == "GeometryCollection":
        found = False
        for child in _as_rows(obj.get("geometries")):
            found = _extract_geojson_geometry(child, properties, output) or found
        return found
    return False


def _extract_geojson(value: Any, output: list[_Shape], inherited: Mapping[str, Any] | None = None) -> bool:
    obj = _as_mapping(value)
    node_type = str(obj.get("type") or "")
    if node_type == "Feature":
        props = {**(inherited or {}), **_as_mapping(obj.get("properties"))}
        return _extract_geojson_geometry(obj.get("geometry"), props, output)
    if node_type == "FeatureCollection":
        found = False
        for feature in _as_rows(obj.get("features")):
            found = _extract_geojson(feature, output, inherited) or found
        return found
    if node_type in {"Polygon", "MultiPolygon", "LineString", "MultiLineString", "GeometryCollection"}:
        return _extract_geojson_geometry(obj, inherited or {}, output)
    return False


def _extract_icd_detail(
    detail: Any,
    ctx: Mapping[str, Any],
    output: list[_Shape],
    *,
    default_kind: str = "",
) -> bool:
    obj = _as_mapping(detail)
    if not obj:
        return False
    local = _context(obj, ctx)
    found = False

    area_rows = _as_rows(_pick(obj, "areaList", "areas"))
    for area in area_rows:
        area_obj = _as_mapping(area)
        ring = _path(_pick(area_obj, "coordinateList", "coordinates", "points"))
        area_ctx = _context(area_obj, local)
        found = _append_shape(output, "polygon", [ring], area_ctx) or found

    line_rows = _as_rows(_pick(obj, "lineList", "lines"))
    for line in line_rows:
        line_obj = _as_mapping(line)
        points = _path(_pick(line_obj, "coordinateList", "coordinates", "points"))
        line_ctx = _context(line_obj, local)
        found = _append_shape(output, "line", [points], line_ctx) or found

    # coordinateList is commonly a convenience duplicate of areaList/lineList.
    # Only use it when no explicit child geometry was usable.
    if not found:
        points = _path(_pick(obj, "coordinateList", "coordinates", "points"))
        if len(points) >= 2:
            kind = default_kind
            if not kind:
                mission_type = str(_pick(obj, "missionType", "shape", default="")).lower()
                kind = "polygon" if "area" in mission_type or "polygon" in mission_type else "line"
            if kind == "polygon" and len(points) < 3:
                kind = "line"
            found = _append_shape(output, kind or "line", [points], local) or found

    for key in (
        "areaSegmentList",
        "segments",
        "coverageDepthDetails",
        "coveragePassDetails",
        "coveragePassObligations",
    ):
        for child in _as_rows(obj.get(key)):
            found = _extract_icd_detail(child, local, output, default_kind=default_kind) or found
    return found


def _extract_mission_snapshot(payload: Mapping[str, Any], output: list[_Shape]) -> bool:
    missions = _as_rows(_pick(payload, "missions", "missionParts"))
    if not missions:
        return False
    found = False
    for mission in missions:
        mission_obj = _as_mapping(mission)
        ctx = _context(mission_obj)
        mission_type = str(_pick(mission_obj, "missionType", "shape", default="")).lower()
        default_kind = "polygon" if "area" in mission_type else "line" if "line" in mission_type else ""
        owner_found = False
        for owner in _as_rows(
            _pick(mission_obj, "areaOwnershipDetails", "ownershipDetails", "aircraftAssignments")
        ):
            owner_obj = _as_mapping(owner)
            owner_ctx = _context(owner_obj, ctx)
            for key in (
                "remainingDetail",
                "areaAssignmentDetail",
                "areaCoverageWorkloadDetail",
            ):
                if _extract_icd_detail(
                    owner_obj.get(key),
                    owner_ctx,
                    output,
                    default_kind="polygon",
                ):
                    owner_found = True
                    break
        if owner_found:
            found = True
        else:
            for key in (
                "remainingDetail",
                "areaAssignmentDetail",
                "areaCoverageWorkloadDetail",
                "geometry",
            ):
                if _extract_geojson(mission_obj.get(key), output, ctx):
                    found = True
                    break
                if _extract_icd_detail(
                    mission_obj.get(key),
                    ctx,
                    output,
                    default_kind=default_kind,
                ):
                    found = True
                    break
        if default_kind == "line" and not owner_found:
            source_line = _path(_pick(mission_obj, "sourceCoordinateList", "coordinateList"))
            found = _append_shape(output, "line", [source_line], ctx) or found
    return found


def _generic_scan(
    value: Any,
    output: list[_Shape],
    *,
    inherited: Mapping[str, Any] | None = None,
    depth: int = 0,
    seen: set[int] | None = None,
) -> bool:
    if depth > 12:
        return False
    if seen is None:
        seen = set()
    if isinstance(value, (Mapping, list, tuple)):
        identity = id(value)
        if identity in seen:
            return False
        seen.add(identity)
    if isinstance(value, Mapping):
        if _extract_geojson(value, output, inherited):
            return True
        ctx = _context(value, inherited)
        if _extract_icd_detail(value, ctx, output):
            return True
        found = False
        for child in value.values():
            found = _generic_scan(
                child,
                output,
                inherited=ctx,
                depth=depth + 1,
                seen=seen,
            ) or found
        return found
    if isinstance(value, (list, tuple)):
        found = False
        for child in value:
            found = _generic_scan(
                child,
                output,
                inherited=inherited,
                depth=depth + 1,
                seen=seen,
            ) or found
        return found
    return False


def _shape_signature(shape: _Shape) -> tuple[Any, ...]:
    rounded = tuple(
        tuple((round(point[0], 7), round(point[1], 7)) for point in path)
        for path in shape.paths
    )
    return shape.kind, shape.input_mission_id, shape.aircraft_id, rounded


def _extract_shapes(payload: Any) -> list[_Shape]:
    output: list[_Shape] = []
    root = _as_mapping(payload)
    _extract_geojson(root, output)
    geojson = root.get("geojson")
    if isinstance(geojson, Mapping):
        # Keep a predictable visual stacking order: initial domain first,
        # remaining work and coverage/assignment overlays last.
        preferred = (
            "inputAreas",
            "lineCorridors",
            "inputLines",
            "paths",
            "remainingAreas",
            "coverageDepth",
            "coveragePassAttribution",
            "optionAssignments",
        )
        visited: set[str] = set()
        for key in preferred:
            if key in geojson:
                _extract_geojson(geojson[key], output, {"label": key})
                visited.add(key)
        for key, value in geojson.items():
            if str(key) not in visited:
                _extract_geojson(value, output, {"label": str(key)})
    _extract_mission_snapshot(root, output)
    if not output:
        _generic_scan(payload, output)

    unique: list[_Shape] = []
    seen: set[tuple[Any, ...]] = set()
    for shape in output:
        signature = _shape_signature(shape)
        if signature in seen:
            continue
        seen.add(signature)
        unique.append(shape)
    return unique


def _font_candidates(*, bold: bool) -> tuple[str, ...]:
    windows = Path(os.environ.get("WINDIR") or "C:/Windows") / "Fonts"
    names = ("malgunbd.ttf", "NanumGothicBold.ttf") if bold else ("malgun.ttf", "NanumGothic.ttf")
    return tuple(str(windows / name) for name in names) + (
        "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf" if bold else "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )


def _load_font(image_font: Any, size: int, *, bold: bool = False) -> Any:
    for candidate in _font_candidates(bold=bold):
        try:
            if Path(candidate).is_file():
                return image_font.truetype(candidate, size=size)
        except Exception:
            continue
    return image_font.load_default()


def _rgb(value: str) -> tuple[int, int, int]:
    color = _valid_hex_color(value) or "#62736b"
    return tuple(int(color[index : index + 2], 16) for index in (1, 3, 5))


def _all_points(shapes: Iterable[_Shape]) -> list[tuple[float, float]]:
    return [point for shape in shapes for path in shape.paths for point in path]


def _shared_projection(
    before: list[_Shape],
    after: list[_Shape],
) -> tuple[float, tuple[float, float, float, float]]:
    points = _all_points([*before, *after])
    if not points:
        return 1.0, (0.0, 0.0, 1.0, 1.0)
    mean_lat = sum(point[1] for point in points) / len(points)
    cosine = max(0.05, abs(math.cos(math.radians(mean_lat))))
    xs = [point[0] * cosine for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max_x - min_x
    span_y = max_y - min_y
    if span_x < 1e-8:
        min_x -= 0.005
        max_x += 0.005
    if span_y < 1e-8:
        min_y -= 0.005
        max_y += 0.005
    pad_x = max((max_x - min_x) * 0.06, 1e-5)
    pad_y = max((max_y - min_y) * 0.06, 1e-5)
    return cosine, (min_x - pad_x, min_y - pad_y, max_x + pad_x, max_y + pad_y)


def _fit_point(
    point: tuple[float, float],
    box: tuple[int, int, int, int],
    cosine: float,
    bounds: tuple[float, float, float, float],
) -> tuple[int, int]:
    left, top, right, bottom = box
    min_x, min_y, max_x, max_y = bounds
    span_x = max(max_x - min_x, 1e-12)
    span_y = max(max_y - min_y, 1e-12)
    width, height = max(1, right - left), max(1, bottom - top)
    scale = min(width / span_x, height / span_y)
    used_w, used_h = span_x * scale, span_y * scale
    origin_x = left + (width - used_w) / 2.0
    origin_y = top + (height - used_h) / 2.0
    x = origin_x + (point[0] * cosine - min_x) * scale
    y = origin_y + (max_y - point[1]) * scale
    return int(round(x)), int(round(y))


def _draw_grid(
    draw: Any,
    box: tuple[int, int, int, int],
    bounds: tuple[float, float, float, float],
    cosine: float,
    font: Any,
) -> None:
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=10, fill="#f8faf8", outline="#cbd5cf", width=1)
    min_x, min_y, max_x, max_y = bounds
    for index in range(1, 5):
        fraction = index / 5.0
        x = int(left + (right - left) * fraction)
        y = int(top + (bottom - top) * fraction)
        draw.line((x, top, x, bottom), fill="#e5ebe7", width=1)
        draw.line((left, y, right, y), fill="#e5ebe7", width=1)
        lon = (min_x + (max_x - min_x) * fraction) / max(cosine, 1e-9)
        lat = max_y - (max_y - min_y) * fraction
        draw.text((x + 3, bottom - 17), f"{lon:.3f}", font=font, fill="#829087")
        draw.text((left + 4, y + 2), f"{lat:.3f}", font=font, fill="#829087")


def _draw_shapes(
    image: Any,
    draw: Any,
    shapes: list[_Shape],
    box: tuple[int, int, int, int],
    cosine: float,
    bounds: tuple[float, float, float, float],
    label_font: Any,
) -> None:
    ordered = sorted(shapes, key=lambda item: 0 if item.kind == "polygon" else 1)
    labels_drawn = 0
    for shape in ordered:
        color = _rgb(shape.color)
        pixel_paths = [
            [_fit_point(point, box, cosine, bounds) for point in path]
            for path in shape.paths
        ]
        pixel_paths = [path for path in pixel_paths if len(path) >= (3 if shape.kind == "polygon" else 2)]
        if not pixel_paths:
            continue
        if shape.kind == "polygon":
            overlay = image.copy()
            overlay_draw = __import__("PIL.ImageDraw", fromlist=["Draw"]).Draw(overlay, "RGBA")
            overlay_draw.polygon(pixel_paths[0], fill=(*color, 58), outline=(*color, 235), width=3)
            for hole in pixel_paths[1:]:
                overlay_draw.polygon(hole, fill=(248, 250, 248, 255), outline=(*color, 180), width=2)
            image.alpha_composite(overlay)
            draw.line(pixel_paths[0] + [pixel_paths[0][0]], fill=(*color, 255), width=3, joint="curve")
            for hole in pixel_paths[1:]:
                draw.line(hole + [hole[0]], fill=(*color, 210), width=2, joint="curve")
        else:
            for path in pixel_paths:
                draw.line(path, fill=(*color, 255), width=4, joint="curve")
                radius = 4
                for point in (path[0], path[-1]):
                    draw.ellipse(
                        (point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius),
                        fill=(*color, 255),
                        outline=(255, 255, 255, 255),
                        width=1,
                    )
        if shape.label and labels_drawn < 14:
            primary = pixel_paths[0]
            center_x = sum(point[0] for point in primary) // len(primary)
            center_y = sum(point[1] for point in primary) // len(primary)
            draw.text(
                (center_x + 5, center_y - 8),
                shape.label[:34],
                font=label_font,
                fill="#18231e",
                stroke_width=3,
                stroke_fill="#f8faf8",
            )
            labels_drawn += 1


def _side_statistics(shapes: list[_Shape]) -> tuple[str, list[tuple[str, str]]]:
    polygons = sum(1 for shape in shapes if shape.kind == "polygon")
    lines = sum(1 for shape in shapes if shape.kind == "line")
    inputs = sorted({value for shape in shapes if (value := shape.input_mission_id) is not None})
    aircraft = sorted({value for shape in shapes if (value := shape.aircraft_id) is not None})
    summary = f"영역 {polygons} · 선형 {lines} · 총 {len(shapes)}개 도형"
    legends: list[tuple[str, str]] = []
    seen: set[tuple[str, str]] = set()
    for shape in shapes:
        label = shape.label or (
            f"Input {shape.input_mission_id}" if shape.input_mission_id is not None else "기타 영역"
        )
        key = (shape.color, label)
        if key not in seen:
            seen.add(key)
            legends.append(key)
    if inputs:
        summary += f" · Input {', '.join(map(str, inputs[:8]))}"
    if aircraft:
        summary += f" · AC {', '.join(map(str, aircraft))}"
    return summary, legends[:6]


def _metadata_text(metadata: Any) -> str:
    if not isinstance(metadata, Mapping):
        return str(metadata or "").strip()
    parts: list[str] = []
    for key, value in list(metadata.items())[:8]:
        if isinstance(value, (Mapping, list, tuple, set)):
            try:
                value_text = json.dumps(value, ensure_ascii=False, default=_json_default)
            except Exception:
                value_text = str(value)
        else:
            value_text = str(value)
        parts.append(f"{key}: {value_text[:72]}")
    return "  |  ".join(parts)


def render_area_comparison(
    before: Any,
    after: Any,
    output_path: str | Path,
    title: str,
    metadata: Mapping[str, Any] | None = None,
) -> bool:
    """Render a tolerant 1600x900 before/after mission-area comparison PNG.

    ``before`` and ``after`` may be mission-area snapshot dictionaries,
    ``/api/mission`` responses, raw GeoJSON, or partially populated variants of
    those schemas.  Missing geometry is rendered as an explicit empty panel.
    ``False`` is returned only when Pillow is unavailable or the image cannot be
    written.
    """

    try:
        from PIL import Image, ImageDraw, ImageFont
    except Exception:
        return False

    output = Path(output_path)
    temporary: Path | None = None
    try:
        before_shapes = _extract_shapes(before)
        after_shapes = _extract_shapes(after)
        cosine, bounds = _shared_projection(before_shapes, after_shapes)

        image = Image.new("RGBA", _IMAGE_SIZE, "#eef3ef")
        draw = ImageDraw.Draw(image, "RGBA")
        title_font = _load_font(ImageFont, 30, bold=True)
        subtitle_font = _load_font(ImageFont, 17, bold=False)
        panel_title_font = _load_font(ImageFont, 21, bold=True)
        body_font = _load_font(ImageFont, 15, bold=False)
        small_font = _load_font(ImageFont, 12, bold=False)

        draw.rectangle((0, 0, _IMAGE_SIZE[0], 92), fill="#17251f")
        draw.text((34, 19), str(title or "재계획 영역 비교")[:90], font=title_font, fill="#ffffff")
        meta_text = _metadata_text(metadata)
        draw.text(
            (36, 61),
            meta_text[:190] if meta_text else "동일 좌표계와 축척으로 전·후 영역을 비교합니다.",
            font=subtitle_font,
            fill="#c8d6ce",
        )

        panels = (
            (before_shapes, (30, 112, 788, 868), "BEFORE", "재계획 전", "#456a91"),
            (after_shapes, (812, 112, 1570, 868), "AFTER", "재계획 후", "#b3613f"),
        )
        for shapes, panel, badge, korean, accent in panels:
            left, top, right, bottom = panel
            draw.rounded_rectangle(panel, radius=16, fill="#ffffff", outline="#c4cec8", width=2)
            draw.rounded_rectangle((left + 20, top + 17, left + 116, top + 50), radius=9, fill=accent)
            draw.text((left + 40, top + 24), badge, font=body_font, fill="#ffffff")
            draw.text((left + 132, top + 21), korean, font=panel_title_font, fill="#21332a")
            summary, legends = _side_statistics(shapes)
            draw.text((left + 22, top + 58), summary[:94], font=body_font, fill="#58675f")

            map_box = (left + 22, top + 91, right - 22, bottom - 91)
            _draw_grid(draw, map_box, bounds, cosine, small_font)
            if shapes:
                _draw_shapes(image, draw, shapes, map_box, cosine, bounds, small_font)
            else:
                empty = "표시할 영역 데이터가 없습니다."
                text_box = draw.textbbox((0, 0), empty, font=panel_title_font)
                width = text_box[2] - text_box[0]
                height = text_box[3] - text_box[1]
                center_x = (map_box[0] + map_box[2]) // 2
                center_y = (map_box[1] + map_box[3]) // 2
                draw.text(
                    (center_x - width // 2, center_y - height // 2),
                    empty,
                    font=panel_title_font,
                    fill="#87938c",
                )
            legend_y = bottom - 71
            if legends:
                cursor_x = left + 24
                for color, label in legends:
                    if cursor_x > right - 135:
                        break
                    draw.rounded_rectangle(
                        (cursor_x, legend_y, cursor_x + 15, legend_y + 15),
                        radius=3,
                        fill=_rgb(color),
                    )
                    draw.text((cursor_x + 21, legend_y - 1), label[:20], font=small_font, fill="#405047")
                    cursor_x += min(180, 35 + max(55, len(label) * 8))
            else:
                draw.text((left + 24, legend_y), "도형 0개", font=small_font, fill="#7b8981")

        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(f".{output.stem}.{os.getpid()}.tmp.png")
        image.convert("RGB").save(temporary, format="PNG", optimize=True)
        os.replace(temporary, output)
        temporary = None
        return True
    except Exception:
        return False
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except OSError:
                pass


def _escape(value: Any) -> str:
    return html.escape(str(value if value is not None else "-"), quote=True)


def _display(value: Any, *, suffix: str = "") -> str:
    if value is None or value == "":
        return "-"
    if isinstance(value, bool):
        return "예" if value else "아니오"
    if isinstance(value, float):
        if not math.isfinite(value):
            return "-"
        text = f"{value:,.3f}".rstrip("0").rstrip(".")
        return text + suffix
    if isinstance(value, int):
        return f"{value:,}{suffix}"
    return str(value) + suffix


def _first(mapping: Any, *keys: str, default: Any = None) -> Any:
    value = _pick(mapping, *keys, default=default)
    return default if value is None else value


def _nested(mapping: Any, *path: str, default: Any = None) -> Any:
    current: Any = mapping
    for key in path:
        current = _pick(current, key, default=None)
        if current is None:
            return default
    return current


def _path_href(value: Any, html_path: Path) -> str:
    if value in (None, ""):
        return ""
    text = str(value).strip()
    if text.startswith(("data:", "http://", "https://", "#")):
        return html.escape(text, quote=True)
    target = Path(text)
    base = html_path.parent.resolve()
    try:
        if target.is_absolute():
            resolved = target.resolve()
        elif (base / target).exists():
            resolved = (base / target).resolve()
        elif target.exists():
            resolved = target.resolve()
        else:
            return html.escape(quote(target.as_posix(), safe="/._-"), quote=True)
        relative = Path(os.path.relpath(resolved, start=base)).as_posix()
    except (OSError, ValueError):
        relative = target.as_posix()
    return html.escape(quote(relative, safe="/._-"), quote=True)


def _status_tone(value: Any) -> str:
    status = str(value or "").strip().lower()
    if status in {"success", "passed", "pass", "ok", "complete", "completed", "성공", "완료"}:
        return "ok"
    if status in {"running", "pending", "partial", "warning", "warn", "진행중", "경고"}:
        return "warn"
    if status in {"failed", "failure", "error", "timeout", "interrupted", "중단", "실패"}:
        return "bad"
    return "neutral"


def _rows_from(value: Any) -> list[Any]:
    if isinstance(value, Mapping):
        return [dict(item, key=key) if isinstance(item, Mapping) else {"key": key, "value": item} for key, item in value.items()]
    return _as_rows(value)


def _report_css() -> str:
    return """
    :root{--ink:#17241e;--muted:#637169;--line:#d7dfda;--paper:#fff;--bg:#eef3ef;--accent:#2f8f83;--ok:#237b5c;--warn:#a96e19;--bad:#b5433e}
    *{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:"Malgun Gothic","Apple SD Gothic Neo",Arial,sans-serif;line-height:1.5}
    .page{max-width:1480px;margin:0 auto;padding:30px}.hero{background:linear-gradient(135deg,#15241d,#294237);color:#fff;border-radius:18px;padding:28px 32px;box-shadow:0 12px 28px #12201824}
    .hero h1{margin:0 0 8px;font-size:30px}.hero p{margin:0;color:#cbd8d1}.eyebrow{font-size:12px;letter-spacing:.14em;text-transform:uppercase;color:#8dd2c1!important}
    .cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin:18px 0}.metric{background:var(--paper);border:1px solid var(--line);border-radius:14px;padding:15px 17px;min-height:92px}
    .metric span{display:block;color:var(--muted);font-size:12px}.metric strong{display:block;margin-top:7px;font-size:21px;overflow-wrap:anywhere}.tone-ok{color:var(--ok)}.tone-warn{color:var(--warn)}.tone-bad{color:var(--bad)}
    section{background:var(--paper);border:1px solid var(--line);border-radius:16px;padding:22px;margin:18px 0}h2{margin:0 0 15px;font-size:21px}h3{margin:0;font-size:18px}.muted{color:var(--muted)}
    table{width:100%;border-collapse:collapse;font-size:13px}th,td{padding:10px 11px;border-bottom:1px solid #e5ebe7;text-align:left;vertical-align:top}th{color:#536159;background:#f5f8f6}tr:last-child td{border-bottom:0}
    .badge{display:inline-flex;align-items:center;border-radius:999px;padding:4px 9px;background:#e8eeea;color:#47564e;font-size:12px;font-weight:700}.badge.ok{background:#dff2e9;color:#176044}.badge.warn{background:#fff0d7;color:#865714}.badge.bad{background:#fbe2e0;color:#92332f}
    .timeline{list-style:none;padding:0;margin:0}.timeline li{display:grid;grid-template-columns:160px 130px 1fr;gap:12px;padding:10px 0;border-bottom:1px solid #e8edea}.timeline li:last-child{border:0}.timeline time{color:var(--muted);font-variant-numeric:tabular-nums}
    .replan{border:1px solid var(--line);border-radius:15px;margin:16px 0;overflow:hidden;break-inside:avoid}.replan-head{display:flex;justify-content:space-between;gap:16px;align-items:flex-start;padding:17px 19px;background:#f5f8f6}.replan-body{padding:17px 19px}
    .meta-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:8px 18px;margin:12px 0}.meta-grid div{border-left:3px solid #c8d5ce;padding-left:9px}.meta-grid dt{font-size:11px;color:var(--muted)}.meta-grid dd{margin:2px 0 0;font-weight:700;overflow-wrap:anywhere}
    .gallery{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:12px;margin-top:14px}.gallery figure{margin:0;border:1px solid var(--line);border-radius:11px;overflow:hidden;background:#f6f8f7}.gallery img{display:block;width:100%;height:auto}.gallery figcaption{padding:8px 10px;color:var(--muted);font-size:12px}
    .empty{padding:22px;text-align:center;color:var(--muted);background:#f6f8f7;border-radius:10px}.errors{color:var(--bad)}a{color:#1f7160;text-decoration:none}a:hover{text-decoration:underline}.footer{padding:12px;text-align:center;color:var(--muted);font-size:12px}
    @media(max-width:760px){.page{padding:12px}.timeline li{grid-template-columns:1fr}.gallery{grid-template-columns:1fr}.hero{padding:22px}}
    @media print{body{background:#fff}.page{max-width:none;padding:0}.hero,section{box-shadow:none}.replan{page-break-inside:avoid}.gallery img{max-height:640px;object-fit:contain}}
    """


def _html_document(title: str, body: str) -> str:
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{_escape(title)}</title><style>{_report_css()}</style></head><body>{body}</body></html>\n"""


def _metric_card(label: str, value: Any, tone: str = "") -> str:
    tone_class = f" tone-{tone}" if tone else ""
    return f'<div class="metric"><span>{_escape(label)}</span><strong class="{tone_class.strip()}">{_escape(_display(value))}</strong></div>'


def _timeline_html(report: Mapping[str, Any]) -> str:
    rows = _rows_from(
        _first(report, "timeline", "events", "flowEvents", "autoMissionEvents", default=[])
    )
    if not rows:
        return '<div class="empty">기록된 타임라인 이벤트가 없습니다.</div>'
    items: list[str] = []
    for row in rows:
        if isinstance(row, Mapping):
            at = _first(row, "at", "time", "timestamp", "createdAt", "startedAt", default="-")
            kind = _first(row, "kind", "type", "category", "msgId", "phase", default="EVENT")
            message = _first(row, "message", "text", "description", "event", "reason", default="-")
            status = _first(row, "status", "result", default="")
        else:
            at, kind, message, status = "-", "EVENT", row, ""
        badge = f'<span class="badge {_status_tone(status)}">{_escape(kind)}</span>'
        items.append(f"<li><time>{_escape(at)}</time>{badge}<div>{_escape(message)}</div></li>")
    return '<ol class="timeline">' + "".join(items) + "</ol>"


def _attack_html(report: Mapping[str, Any]) -> str:
    attacks = _rows_from(
        _first(report, "attackEvidence", "attacks", "targetEvents", "combatEvents", default=[])
    )
    targets = _as_mapping(_first(report, "targets", "targetSummary", default={}))
    summary_bits = []
    for label, keys in (
        ("전체 표적", ("total", "count", "targetCount")),
        ("탐지", ("detected", "detectedCount")),
        ("격파", ("destroyed", "destroyedCount")),
        ("생존", ("alive", "aliveCount")),
    ):
        value = _first(targets, *keys, default=None)
        if value is not None:
            summary_bits.append(_metric_card(label, value))
    summary = '<div class="cards">' + "".join(summary_bits) + "</div>" if summary_bits else ""
    if not attacks:
        return summary + '<div class="empty">적 공격·피격 증거 이벤트가 없습니다.</div>'
    body_rows: list[str] = []
    for index, row in enumerate(attacks, start=1):
        obj = _as_mapping(row)
        body_rows.append(
            "<tr>"
            f"<td>{index}</td>"
            f"<td>{_escape(_first(obj, 'at', 'time', 'timestamp', 'simTime', default='-'))}</td>"
            f"<td>{_escape(_first(obj, 'kind', 'type', 'event', default='공격'))}</td>"
            f"<td>{_escape(_first(obj, 'source', 'attacker', 'aircraft', default='-'))}</td>"
            f"<td>{_escape(_first(obj, 'target', 'targetID', 'targetId', default='-'))}</td>"
            f"<td>{_escape(_first(obj, 'result', 'status', 'message', default='-'))}</td>"
            "</tr>"
        )
    return summary + "<table><thead><tr><th>#</th><th>시각</th><th>유형</th><th>공격원</th><th>표적</th><th>결과/근거</th></tr></thead><tbody>" + "".join(body_rows) + "</tbody></table>"


def _comparison_image_values(row: Mapping[str, Any]) -> list[tuple[str, Any]]:
    artifacts = _as_mapping(row.get("artifacts"))
    candidates = (
        ("전·후 비교", _first(row, "comparisonImage", "comparison_image", "image", "imagePath", default=_first(artifacts, "comparison", "image"))),
        ("재계획 전", _first(row, "beforeImage", "before_image", default=_first(artifacts, "before"))),
        ("재계획 후", _first(row, "afterImage", "after_image", default=_first(artifacts, "after"))),
    )
    result: list[tuple[str, Any]] = []
    seen: set[str] = set()
    for label, value in candidates:
        if value in (None, "") or str(value) in seen:
            continue
        seen.add(str(value))
        result.append((label, value))
    return result


def _comparison_cards(html_path: Path, rows: list[dict[str, Any]]) -> str:
    if not rows:
        return '<div class="empty">재계획 영역 비교 자료가 없습니다.</div>'
    cards: list[str] = []
    for index, raw in enumerate(rows, start=1):
        row = _as_mapping(raw)
        sequence = _first(row, "sequence", "index", "replanIndex", default=index)
        title = _first(row, "title", "name", default=f"재계획 #{sequence}")
        status = _first(row, "status", "result", default="-")
        reason = _first(row, "reason", "description", "trigger", default="-")
        tone = _status_tone(status)
        fields = (
            ("Trigger", _first(row, "trigger", "triggerType", default="-")),
            ("Source Plan", _first(row, "sourcePlanID", "sourcePlanIDs", "beforePlanID", default="-")),
            ("Result Plan", _first(row, "resultPlanID", "resultPlanIDs", "afterPlanID", default="-")),
            ("총 재계획", _first(row, "elapsedMs", "totalElapsedMs", default="-")),
            ("0305 수행중까지", _first(row, "queueElapsedMs", default="-")),
            ("Planner", _first(row, "planningElapsedMs", "plannerElapsedMs", default="-")),
            ("영역 판정", _first(row, "changeSummary", "areaChange", "comparison", default="-")),
        )
        meta = '<dl class="meta-grid">' + "".join(
            f"<div><dt>{_escape(label)}</dt><dd>{_escape(_display(value, suffix=' ms' if 'Elapsed' in label or label in {'총 재계획', '0305 수행중까지', 'Planner'} and isinstance(value, (int, float)) else ''))}</dd></div>"
            for label, value in fields
        ) + "</dl>"
        images = _comparison_image_values(row)
        gallery = ""
        if images:
            figures = []
            for label, value in images:
                href = _path_href(value, html_path)
                figures.append(
                    f'<figure><a href="{href}"><img src="{href}" alt="{_escape(title)} - {_escape(label)}"></a><figcaption>{_escape(label)}</figcaption></figure>'
                )
            gallery = '<div class="gallery">' + "".join(figures) + "</div>"
        else:
            gallery = '<div class="empty">생성된 비교 이미지가 없습니다.</div>'
        cards.append(
            '<article class="replan">'
            f'<div class="replan-head"><div><h3>{_escape(title)}</h3><div class="muted">{_escape(reason)}</div></div><span class="badge {tone}">{_escape(status)}</span></div>'
            f'<div class="replan-body">{meta}{gallery}</div></article>'
        )
    return "".join(cards)


def write_run_html(
    path: str | Path,
    report: dict[str, Any],
    comparison_rows: list[dict[str, Any]],
) -> None:
    """Write a self-contained Korean HTML report for one evaluation run."""

    output = Path(path)
    source = _as_mapping(report)
    summary = _as_mapping(_first(source, "summary", "overall", default={}))
    scenario = _first(source, "scenario", "scenarioName", default=_first(summary, "scenario", default="-"))
    if isinstance(scenario, Mapping):
        scenario = _first(scenario, "name", "id", "path", default="-")
    status = _first(source, "status", "result", default=_first(summary, "status", default="-"))
    run_index = _first(source, "runIndex", "index", "run", default="-")
    coverage = _first(summary, "coveragePercent", "coverage", default=_first(source, "coveragePercent", "coverage", default="-"))
    replans = _rows_from(_first(source, "replans", "replanRecords", default=[]))
    target_summary = _as_mapping(_first(source, "targets", "targetSummary", default={}))
    errors = _rows_from(_first(source, "errors", "issues", default=[]))
    warnings = _rows_from(_first(source, "warnings", "warning", default=[]))
    title = f"성능평가 Run {run_index} · {scenario}"

    cards = "".join(
        (
            _metric_card("실행 결과", status, _status_tone(status)),
            _metric_card("Scenario", scenario),
            _metric_card("총 소요시간", _first(source, "durationSeconds", "duration", "elapsedSeconds", default="-")),
            _metric_card("최종 Plan", _first(source, "missionPlanID", "finalMissionPlanID", default=_first(summary, "planID", default="-"))),
            _metric_card("전체 Coverage", coverage),
            _metric_card("재계획", len(replans) if replans else len(comparison_rows)),
            _metric_card("격파 표적", _first(target_summary, "destroyed", "destroyedCount", default="-")),
            _metric_card(
                "오류 / 경고",
                f"{len(errors)} / {len(warnings)}",
                "bad" if errors else "warn" if warnings else "ok",
            ),
        )
    )
    error_html = ""
    if errors or warnings:
        error_items = "".join(
            f"<li>{_escape(_first(item, 'message', 'error', default=item) if isinstance(item, Mapping) else item)}</li>"
            for item in errors
        )
        warning_items = "".join(
            f"<li>{_escape(_first(item, 'message', 'warning', default=item) if isinstance(item, Mapping) else item)}</li>"
            for item in warnings
        )
        error_block = f'<h3>오류</h3><ul class="errors">{error_items}</ul>' if errors else ""
        warning_block = f'<h3>경고</h3><ul>{warning_items}</ul>' if warnings else ""
        error_html = f'<section><h2>오류 및 경고</h2>{error_block}{warning_block}</section>'

    body = f"""
<div class="page">
  <header class="hero"><p class="eyebrow">KU MISSION PERFORMANCE EVALUATION</p><h1>{_escape(title)}</h1>
    <p>{_escape(_first(source, 'startedAt', 'startTime', default='-'))} → {_escape(_first(source, 'completedAt', 'endTime', default='-'))}</p></header>
  <div class="cards">{cards}</div>
  <section><h2>자동 실행 타임라인</h2>{_timeline_html(source)}</section>
  <section><h2>적 탐지·공격·격파 증거</h2>{_attack_html(source)}</section>
  <section><h2>재계획 영역 전·후 비교</h2>{_comparison_cards(output, comparison_rows)}</section>
  {error_html}
  <div class="footer">생성: {_escape(datetime.now().astimezone().isoformat(timespec='seconds'))} · 모든 수치는 실행 당시 저장된 스냅샷 기준</div>
</div>"""
    _atomic_write_text(output, _html_document(title, body))


def write_session_html(path: str | Path, session: dict[str, Any]) -> None:
    """Write a self-contained Korean summary HTML for repeated runs."""

    output = Path(path)
    source = _as_mapping(session)
    runs = _rows_from(_first(source, "runs", "results", "runResults", default=[]))
    requested = _first(source, "requestedRuns", "runCount", "requested", default=len(runs))
    successes = sum(
        1 for row in runs if _status_tone(_first(row, "status", "result", default="")) == "ok"
    )
    failures = sum(
        1 for row in runs if _status_tone(_first(row, "status", "result", default="")) == "bad"
    )
    durations = [
        number
        for row in runs
        if (number := _number_or_none(_first(row, "durationSeconds", "duration", "elapsedSeconds", default=None))) is not None
    ]
    average_duration = sum(durations) / len(durations) if durations else None
    title = str(_first(source, "title", "name", default="KU 반복 성능평가 요약"))

    table_rows: list[str] = []
    for index, raw in enumerate(runs, start=1):
        row = _as_mapping(raw)
        run_index = _first(row, "runIndex", "index", "run", default=index)
        status = _first(row, "status", "result", default="-")
        scenario = _first(row, "scenario", "scenarioName", default="-")
        if isinstance(scenario, Mapping):
            scenario = _first(scenario, "name", "id", default="-")
        report_path = _first(row, "reportHtml", "html", "reportPath", "report", default="")
        report_link = (
            f'<a href="{_path_href(report_path, output)}">보고서 열기</a>' if report_path else "-"
        )
        errors = _rows_from(_first(row, "errors", "issues", default=[]))
        table_rows.append(
            "<tr>"
            f"<td>{_escape(run_index)}</td>"
            f'<td><span class="badge {_status_tone(status)}">{_escape(status)}</span></td>'
            f"<td>{_escape(scenario)}</td>"
            f"<td>{_escape(_display(_first(row, 'durationSeconds', 'duration', default='-'), suffix=' s' if isinstance(_first(row, 'durationSeconds', 'duration', default=None), (int, float)) else ''))}</td>"
            f"<td>{_escape(_first(row, 'replanCount', default=len(_rows_from(_first(row, 'replans', default=[])))))}</td>"
            f"<td>{_escape(len(errors))}</td>"
            f"<td>{report_link}</td>"
            "</tr>"
        )
    table = (
        "<table><thead><tr><th>Run</th><th>결과</th><th>Scenario</th><th>소요시간</th><th>재계획</th><th>오류</th><th>산출물</th></tr></thead><tbody>"
        + "".join(table_rows)
        + "</tbody></table>"
        if table_rows
        else '<div class="empty">실행 결과가 없습니다.</div>'
    )
    cards = "".join(
        (
            _metric_card("요청 횟수", requested),
            _metric_card("완료 횟수", len(runs)),
            _metric_card("성공", successes, "ok"),
            _metric_card("실패", failures, "bad" if failures else "ok"),
            _metric_card("평균 소요시간", f"{average_duration:,.3f} s" if average_duration is not None else "-"),
            _metric_card("성공률", f"{successes / len(runs) * 100.0:.1f}%" if runs else "-"),
        )
    )
    body = f"""
<div class="page">
  <header class="hero"><p class="eyebrow">KU ROBUSTNESS SESSION</p><h1>{_escape(title)}</h1>
    <p>{_escape(_first(source, 'startedAt', 'startTime', default='-'))} → {_escape(_first(source, 'completedAt', 'endTime', default='-'))}</p></header>
  <div class="cards">{cards}</div>
  <section><h2>회차별 결과</h2>{table}</section>
  <div class="footer">생성: {_escape(datetime.now().astimezone().isoformat(timespec='seconds'))}</div>
</div>"""
    _atomic_write_text(output, _html_document(title, body))


__all__ = [
    "render_area_comparison",
    "write_json",
    "write_run_html",
    "write_session_html",
]
