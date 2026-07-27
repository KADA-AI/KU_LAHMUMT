from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _coord(item: Any) -> Optional[List[float]]:
    if not isinstance(item, dict):
        return None
    lat = item.get("latitude", item.get("Latitude"))
    lon = item.get("longitude", item.get("Longitude"))
    if lat is None or lon is None:
        return None
    return [float(lon), float(lat)]


def _coord_list(items: Any) -> List[List[float]]:
    coords: List[List[float]] = []
    if not isinstance(items, list):
        return coords
    for item in items:
        coord = _coord(item)
        if coord is not None:
            coords.append(coord)
    return coords


def _polyline_entries(mission: Dict[str, Any]) -> List[Dict[str, Any]]:
    modern = mission.get("PolyLines")
    if isinstance(modern, dict):
        line_list = modern.get("LineList") or []
        if isinstance(line_list, dict):
            line_list = [line_list]
        return [item for item in line_list if isinstance(item, dict)]
    polylines = mission.get("Polylines")
    if isinstance(polylines, list):
        return [item for item in polylines if isinstance(item, dict)]
    legacy = mission.get("PolyLine")
    if isinstance(legacy, dict):
        return [legacy]
    return []


def _single_json(dir_path: Path) -> Optional[Path]:
    files = sorted(dir_path.glob("*.json"))
    if len(files) == 1:
        return files[0]
    return None


def _resolve_bundle_db_root(path: Path) -> Optional[Path]:
    path = path.resolve()
    if path.is_dir() and (path / "InputMissionPlan").is_dir() and (path / "Scenario").is_dir():
        return path
    if not path.is_dir():
        return None
    matches = []
    for child in path.iterdir():
        if child.is_dir() and (child / "InputMissionPlan").is_dir() and (child / "Scenario").is_dir():
            matches.append(child)
    if len(matches) == 1:
        return matches[0]
    return None


def _resolve_inputs(input_path: Path) -> Dict[str, Optional[Path]]:
    input_path = input_path.resolve()
    bundle_db_root: Optional[Path] = None
    scenario_path: Optional[Path] = None
    imp_path: Optional[Path] = None
    mr_path: Optional[Path] = None
    tgt_path: Optional[Path] = None

    if input_path.is_dir():
        bundle_db_root = _resolve_bundle_db_root(input_path)
        if bundle_db_root is None:
            raise FileNotFoundError(
                "bundle folder not found. Pass the agency folder or Random_Scenario_* folder."
            )
        scenario_path = _single_json(bundle_db_root / "Scenario")
        imp_path = _single_json(bundle_db_root / "InputMissionPlan")
        mr_path = _single_json(bundle_db_root / "MissionReferenceInfo")
        tgt_path = _single_json(bundle_db_root / "TargetInfo")
    elif input_path.is_file():
        scenario_path = input_path
        if input_path.parent.name == "Scenario":
            bundle_db_root = input_path.parent.parent
        elif input_path.parent.parent.name == "Scenario":
            bundle_db_root = input_path.parent.parent.parent
        if bundle_db_root and bundle_db_root.is_dir():
            imp_path = _single_json(bundle_db_root / "InputMissionPlan")
            mr_path = _single_json(bundle_db_root / "MissionReferenceInfo")
            tgt_path = _single_json(bundle_db_root / "TargetInfo")
    else:
        raise FileNotFoundError(f"path not found: {input_path}")

    if scenario_path is None or not scenario_path.exists():
        raise FileNotFoundError("Scenario json not found")

    return {
        "bundle_db_root": bundle_db_root,
        "scenario_path": scenario_path,
        "imp_path": imp_path,
        "mr_path": mr_path,
        "tgt_path": tgt_path,
    }


def _pick_input_path() -> Optional[Path]:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception:
        return None

    root = tk.Tk()
    root.withdraw()
    try:
        root.attributes("-topmost", True)
    except Exception:
        pass

    try:
        file_path = filedialog.askopenfilename(
            title="Select Scenario JSON",
            filetypes=[
                ("Scenario JSON", "randomScenario_*.json"),
                ("JSON", "*.json"),
                ("All Files", "*.*"),
            ],
        )
        if file_path:
            return Path(file_path)

        dir_path = filedialog.askdirectory(
            title="Select Random_Scenario_* folder or agency bundle folder"
        )
        if dir_path:
            return Path(dir_path)
        return None
    finally:
        try:
            root.destroy()
        except Exception:
            pass


def _collect_plot_data(
    scenario_payload: Dict[str, Any],
    tgt_payload: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    preview: Dict[str, Any] = {
        "flight_areas": [],
        "prohibited_areas": [],
        "takeover": [],
        "handover": [],
        "rtb": [],
        "mission_lines": [],
        "mission_areas": [],
        "mission_points": [],
        "targets": [],
        "target_paths": [],
        "aircraft_ids": [],
        "scenario_name": scenario_payload.get("ScenarioName", ""),
    }

    init = scenario_payload.get("InitScenario") or {}
    pkg = init.get("InputMissionPackage") or {}
    ref = init.get("MissionReferencePackage") or {}

    aircraft_ids = [int(aid) for aid in pkg.get("AircraftIDs") or [] if int(aid) > 0]
    preview["aircraft_ids"] = aircraft_ids

    for area in ref.get("FlightAreaList") or []:
        coords = _coord_list(area.get("AreaLatLonList"))
        if coords:
            preview["flight_areas"].append(coords)
    for area in ref.get("ProhibitedAreaList") or []:
        coords = _coord_list(area.get("AreaLatLonList"))
        if coords:
            preview["prohibited_areas"].append(coords)
    for item in ref.get("TakeOverInfoList") or []:
        coord = _coord(item.get("CoordinateList"))
        if coord is not None:
            preview["takeover"].append({"id": int(item.get("AircraftID", 0)), "coord": coord})
    for item in ref.get("HandOverInfoList") or []:
        coord = _coord(item.get("CoordinateList"))
        if coord is not None:
            preview["handover"].append({"id": int(item.get("AircraftID", 0)), "coord": coord})
    for item in ref.get("RTBCoordinateList") or []:
        coord = _coord(item)
        if coord is not None:
            preview["rtb"].append(coord)

    for mission in pkg.get("InputMissionList") or []:
        seq = int(mission.get("SequenceNumber", 0) or 0)
        polyline_entries = _polyline_entries(mission)
        area_lists = (mission.get("Polygons") or {}).get("AreaList") or []
        if polyline_entries:
            for polyline in polyline_entries:
                line_coords = _coord_list(polyline.get("CoordinateList"))
                if line_coords:
                    preview["mission_lines"].append({"id": seq, "coords": line_coords})
            continue
        if area_lists:
            for area in area_lists:
                area_coords = _coord_list(area.get("CoordinateList"))
                if area_coords:
                    preview["mission_areas"].append({"id": seq, "coords": area_coords})
            continue
        point_coord = _coord(mission.get("Coordinate"))
        if point_coord is not None:
            preview["mission_points"].append({"id": seq, "coord": point_coord})

    if tgt_payload is not None:
        for target in tgt_payload.get("targetList") or []:
            coord = _coord(target.get("location"))
            if coord is None:
                continue
            preview["targets"].append(
                {
                    "id": int(target.get("targetID", 0) or 0),
                    "type": int(target.get("targetType", 0) or 0),
                    "mission": int(target.get("inputMissionID", 0) or 0),
                    "coord": coord,
                }
            )
            path = _coord_list(target.get("path"))
            if path:
                preview["target_paths"].append({"id": int(target.get("targetID", 0) or 0), "coords": path})
    else:
        aircraft_id_set = set(aircraft_ids)
        for unit in scenario_payload.get("UnitObjectList") or []:
            try:
                unit_id = int(unit.get("ID", 0) or 0)
            except Exception:
                continue
            if unit_id in aircraft_id_set:
                continue
            coord = _coord(unit.get("LOC"))
            if coord is None:
                continue
            preview["targets"].append({"id": unit_id, "type": None, "mission": None, "coord": coord})

    return preview


def _all_coords(plot_data: Dict[str, Any]) -> List[List[float]]:
    points: List[List[float]] = []
    for key in ("flight_areas", "prohibited_areas"):
        for coords in plot_data.get(key, []):
            points.extend(coords)
    for key in ("takeover", "handover"):
        for item in plot_data.get(key, []):
            points.append(item["coord"])
    points.extend(plot_data.get("rtb", []))
    for item in plot_data.get("mission_lines", []):
        points.extend(item["coords"])
    for item in plot_data.get("mission_areas", []):
        points.extend(item["coords"])
    for item in plot_data.get("mission_points", []):
        points.append(item["coord"])
    for item in plot_data.get("targets", []):
        points.append(item["coord"])
    for item in plot_data.get("target_paths", []):
        points.extend(item["coords"])
    return points


def _fmt_coord(coord: Optional[List[float]]) -> str:
    if not coord:
        return "-"
    return f"{coord[1]:.6f}, {coord[0]:.6f}"


def _build_info_lines(plot_data: Dict[str, Any]) -> List[str]:
    lines: List[str] = []
    lines.append(f"Scenario: {plot_data.get('scenario_name', '')}")
    lines.append("")

    lines.append("TakeOver")
    if plot_data["takeover"]:
        for item in plot_data["takeover"]:
            lines.append(f"TO{item['id']:<2}  {_fmt_coord(item['coord'])}")
    else:
        lines.append("-")
    lines.append("")

    lines.append("HandOver")
    if plot_data["handover"]:
        for item in plot_data["handover"]:
            lines.append(f"HO{item['id']:<2}  {_fmt_coord(item['coord'])}")
    else:
        lines.append("-")
    lines.append("")

    lines.append("RTB")
    if plot_data["rtb"]:
        for idx, coord in enumerate(plot_data["rtb"], start=1):
            lines.append(f"RTB{idx:<1}  {_fmt_coord(coord)}")
    else:
        lines.append("-")
    lines.append("")

    lines.append("Missions")
    mission_lines = sorted(plot_data["mission_lines"], key=lambda item: item["id"])
    mission_areas = sorted(plot_data["mission_areas"], key=lambda item: item["id"])
    mission_points = sorted(plot_data["mission_points"], key=lambda item: item["id"])
    if not (mission_lines or mission_areas or mission_points):
        lines.append("-")
    for item in mission_lines:
        start = item["coords"][0] if item["coords"] else None
        end = item["coords"][-1] if item["coords"] else None
        lines.append(f"M{item['id']:<2}  line  {_fmt_coord(start)} -> {_fmt_coord(end)}")
    for item in mission_areas:
        first = item["coords"][0] if item["coords"] else None
        lines.append(f"M{item['id']:<2}  area  {_fmt_coord(first)}")
    for item in mission_points:
        lines.append(f"M{item['id']:<2}  point {_fmt_coord(item['coord'])}")
    lines.append("")

    lines.append("Targets")
    targets = sorted(plot_data["targets"], key=lambda item: item["id"])
    if not targets:
        lines.append("-")
    for item in targets:
        mission_text = f" @M{item['mission']}" if item.get("mission") else ""
        type_text = f" type{item['type']}" if item.get("type") is not None else ""
        lines.append(f"T{item['id']:<2}{mission_text}{type_text}  {_fmt_coord(item['coord'])}")

    return lines


def _make_xy_transform(points: List[List[float]]) -> Dict[str, float]:
    if not points:
        raise ValueError("no coordinates found to plot")
    lon0 = sum(p[0] for p in points) / len(points)
    lat0 = sum(p[1] for p in points) / len(points)
    cos_lat = math.cos(math.radians(lat0))
    return {"lon0": lon0, "lat0": lat0, "cos_lat": cos_lat}


def _to_xy(coord: List[float], origin: Dict[str, float]) -> List[float]:
    x = (coord[0] - origin["lon0"]) * 111_320.0 * origin["cos_lat"]
    y = (coord[1] - origin["lat0"]) * 111_320.0
    return [x, y]


def _plot_poly(ax: Any, coords: List[List[float]], origin: Dict[str, float], **kwargs: Any) -> None:
    xy = [_to_xy(coord, origin) for coord in coords]
    if xy and xy[0] != xy[-1]:
        xy.append(xy[0])
    xs = [p[0] for p in xy]
    ys = [p[1] for p in xy]
    ax.fill(xs, ys, **kwargs)
    edge_kwargs = dict(kwargs)
    edge_kwargs.pop("label", None)
    edge_kwargs.pop("alpha", None)
    ax.plot(xs, ys, color=edge_kwargs.get("color", "#111827"), linewidth=1.6)


def _plot_preview(
    plot_data: Dict[str, Any],
    *,
    save_path: Optional[Path],
    show: bool,
    title_suffix: str,
) -> None:
    if not show:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    points = _all_coords(plot_data)
    origin = _make_xy_transform(points)

    fig = plt.figure(figsize=(15.5, 9.5))
    gs = fig.add_gridspec(1, 2, width_ratios=[3.6, 1.7])
    ax = fig.add_subplot(gs[0, 0])
    ax_info = fig.add_subplot(gs[0, 1])
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#fbfbfd")
    ax_info.set_facecolor("#f7f8fb")
    ax_info.axis("off")

    flight_area_label = True
    for coords in plot_data["flight_areas"]:
        _plot_poly(
            ax,
            coords,
            origin,
            color="#cfe8ff",
            alpha=0.35,
            label="Flight Area" if flight_area_label else None,
        )
        flight_area_label = False

    prohibited_label = True
    for coords in plot_data["prohibited_areas"]:
        _plot_poly(
            ax,
            coords,
            origin,
            color="#ffb3b3",
            alpha=0.35,
            label="Prohibited" if prohibited_label else None,
        )
        prohibited_label = False

    cmap = plt.get_cmap("tab10")
    for idx, item in enumerate(plot_data["mission_lines"], start=1):
        xy = [_to_xy(coord, origin) for coord in item["coords"]]
        xs = [p[0] for p in xy]
        ys = [p[1] for p in xy]
        color = cmap((idx - 1) % 10)
        ax.plot(xs, ys, color=color, linewidth=2.6, label="Mission Line" if idx == 1 else None)
        ax.scatter(xs[0], ys[0], color=color, s=26)
        ax.text(xs[0], ys[0], f"M{item['id']}", fontsize=9, color=color, ha="left", va="bottom")

    for idx, item in enumerate(plot_data["mission_areas"], start=1):
        coords = item["coords"]
        xy = [_to_xy(coord, origin) for coord in coords]
        if xy and xy[0] != xy[-1]:
            xy.append(xy[0])
        xs = [p[0] for p in xy]
        ys = [p[1] for p in xy]
        ax.fill(xs, ys, color="#ffdf99", alpha=0.45, label="Mission Area" if idx == 1 else None)
        ax.plot(xs, ys, color="#c77700", linewidth=1.8)
        cx = sum(p[0] for p in xy[:-1]) / max(1, len(xy) - 1)
        cy = sum(p[1] for p in xy[:-1]) / max(1, len(xy) - 1)
        ax.text(cx, cy, f"M{item['id']}", fontsize=9, color="#8a5200", ha="center", va="center")

    for idx, item in enumerate(plot_data["mission_points"], start=1):
        x, y = _to_xy(item["coord"], origin)
        ax.scatter([x], [y], marker="o", s=50, color="#111827", label="Mission Point" if idx == 1 else None)
        ax.text(x, y, f"M{item['id']}", fontsize=9, color="#111827", ha="left", va="bottom")

    if plot_data["takeover"]:
        xs, ys = zip(*[_to_xy(item["coord"], origin) for item in plot_data["takeover"]])
        ax.scatter(xs, ys, marker="^", s=80, color="#0a63ff", label="TakeOver")
        for item, x, y in zip(plot_data["takeover"], xs, ys):
            ax.text(x, y, f"TO{item['id']}", fontsize=8, color="#0a63ff", ha="left", va="bottom")

    if plot_data["handover"]:
        xs, ys = zip(*[_to_xy(item["coord"], origin) for item in plot_data["handover"]])
        ax.scatter(xs, ys, marker="v", s=80, color="#0b8f55", label="HandOver")
        for item, x, y in zip(plot_data["handover"], xs, ys):
            ax.text(x, y, f"HO{item['id']}", fontsize=8, color="#0b8f55", ha="left", va="top")

    if plot_data["rtb"]:
        xs, ys = zip(*[_to_xy(coord, origin) for coord in plot_data["rtb"]])
        ax.scatter(xs, ys, marker="s", s=60, color="#6b7280", label="RTB")
        for idx, (x, y) in enumerate(zip(xs, ys), start=1):
            ax.text(x, y, f"RTB{idx}", fontsize=8, color="#6b7280", ha="left", va="bottom")

    for idx, item in enumerate(plot_data["target_paths"], start=1):
        xy = [_to_xy(coord, origin) for coord in item["coords"]]
        xs = [p[0] for p in xy]
        ys = [p[1] for p in xy]
        ax.plot(xs, ys, color="#c1121f", linewidth=1.6, linestyle="--", alpha=0.8, label="Target Path" if idx == 1 else None)

    for idx, item in enumerate(plot_data["targets"], start=1):
        x, y = _to_xy(item["coord"], origin)
        ax.scatter([x], [y], marker="x", s=80, color="#c1121f", linewidths=2.0, label="Target" if idx == 1 else None)
        label = f"T{item['id']}"
        if item["mission"]:
            label += f"@M{item['mission']}"
        ax.text(x, y, label, fontsize=8, color="#8b0000", ha="left", va="bottom")

    anchor = plot_data["takeover"][0]["coord"] if plot_data["takeover"] else None
    first_start = None
    if plot_data["mission_lines"]:
        first_start = plot_data["mission_lines"][0]["coords"][0]
    elif plot_data["mission_points"]:
        first_start = plot_data["mission_points"][0]["coord"]
    elif plot_data["mission_areas"]:
        first_start = plot_data["mission_areas"][0]["coords"][0]
    if anchor is not None and first_start is not None:
        x1, y1 = _to_xy(anchor, origin)
        x2, y2 = _to_xy(first_start, origin)
        dy = y2 - y1
        ax.plot([x1, x2], [y1, y2], color="#111827", linewidth=1.0, linestyle=":")
        ax.text((x1 + x2) / 2.0, (y1 + y2) / 2.0, f"north {dy:.1f}m", fontsize=8, color="#111827")

    handles, labels = ax.get_legend_handles_labels()
    dedup: Dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        if label and label not in dedup:
            dedup[label] = handle
    if dedup:
        ax.legend(dedup.values(), dedup.keys(), loc="best")

    ax.set_title(f"{plot_data['scenario_name']}\n{title_suffix}", fontsize=12)
    ax.set_xlabel("East / West (m)")
    ax.set_ylabel("North / South (m)")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="datalim")

    info_lines = _build_info_lines(plot_data)
    ax_info.text(
        0.02,
        0.98,
        "\n".join(info_lines),
        va="top",
        ha="left",
        fontsize=9,
        family="monospace",
        color="#111827",
        transform=ax_info.transAxes,
    )
    plt.tight_layout()

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=180)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Visualize a Random_mission scenario json or bundle folder for quick validation."
    )
    parser.add_argument(
        "path",
        type=str,
        nargs="?",
        default="",
        help="Path to Scenario json, agency bundle folder, or Random_Scenario_* folder.",
    )
    parser.add_argument(
        "--pick",
        action="store_true",
        help="Open a file/folder picker instead of passing a path.",
    )
    parser.add_argument(
        "--save",
        type=str,
        nargs="?",
        const="__default__",
        default="",
        help="Optionally save PNG. If used without a path, save next to the Scenario json.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open the matplotlib window.",
    )
    args = parser.parse_args()

    try:
        input_path = Path(args.path) if args.path else None
        if args.pick or input_path is None:
            picked = _pick_input_path()
            if picked is None:
                raise RuntimeError("no file or folder selected")
            input_path = picked

        resolved = _resolve_inputs(input_path)
        scenario_path = resolved["scenario_path"]
        if scenario_path is None:
            raise FileNotFoundError("Scenario json not found")

        scenario_payload = _load_json(scenario_path)
        tgt_payload = _load_json(resolved["tgt_path"]) if resolved["tgt_path"] else None
        plot_data = _collect_plot_data(scenario_payload, tgt_payload=tgt_payload)

        title_bits = []
        if resolved["bundle_db_root"] is not None:
            title_bits.append(str(resolved["bundle_db_root"].name))
        title_bits.append(f"missions {len(plot_data['mission_lines']) + len(plot_data['mission_areas']) + len(plot_data['mission_points'])}")
        title_bits.append(f"targets {len(plot_data['targets'])}")

        save_path: Optional[Path] = None
        if args.save == "__default__":
            save_path = scenario_path.with_name(f"{scenario_path.stem}_preview.png")
        elif args.save:
            save_path = Path(args.save).resolve()

        show = not args.no_show
        if not show and save_path is None:
            raise ValueError("nothing to do: use default show or pass --save with --no-show")

        _plot_preview(
            plot_data,
            save_path=save_path,
            show=show,
            title_suffix=" | ".join(title_bits),
        )
        if save_path is not None:
            print(save_path)
        else:
            print(scenario_path)
        return 0
    except Exception as exc:
        print(f"visualization failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
