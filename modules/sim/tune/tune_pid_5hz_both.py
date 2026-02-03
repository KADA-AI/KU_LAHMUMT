"""
5Hz (dt=0.2s) PID auto-tuning runner for UAV + LAH.

Outputs:
- test DB: modules/sim/tune/db/*_pid_db.json
- runtime DB: modules/sim/runtime/controllers/*_pid_db.json
"""


from __future__ import annotations

import argparse
import ast
import importlib.util
import inspect
import json
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np


#  dt (    )
DT_BASE = 0.01
DEFAULT_SPEED_UAV = 90.0
DEFAULT_SPEED_LAH = 60.0


@dataclass(frozen=True)
class TunePaths:
    repo_root: Path
    sim_root: Path
    tune_root: Path

    uav_tuner_py: Path
    lah_tuner_py: Path

    uav_test_db: Path
    lah_test_db: Path

    uav_runtime_db: Path
    lah_runtime_db: Path


def _atomic_write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
    tmp.replace(path)


def _backup_if_exists(path: Path, tag: str) -> Path | None:
    if not path.exists():
        return None
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(path.suffix + f".bak_{tag}_{ts}")
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, backup)
    return backup


def _load_json_or_empty(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _merge_records_by_time_scale(existing: List[Dict[str, Any]], updates: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    def _key(r: Dict[str, Any]) -> float:
        try:
            return float(r.get("time_scale"))
        except Exception:
            return float("nan")

    merged: Dict[float, Dict[str, Any]] = {}
    # 
    for r in existing:
        k = _key(r)
        if k == k:  # NaN 
            merged[k] = r
    # 
    for r in updates:
        k = _key(r)
        if k == k:
            merged[k] = r

    out = list(merged.values())
    out.sort(key=lambda r: float(r.get("time_scale", 0.0)))
    return out


def _ensure_db_has_record(db_path: Path, record: Dict[str, Any], *, do_backup: bool = True) -> None:
    if do_backup:
        _backup_if_exists(db_path, tag="before_tune")

    data = _load_json_or_empty(db_path)
    existing_records = []
    if isinstance(data, dict) and isinstance(data.get("records"), list):
        existing_records = list(data["records"])

    merged = _merge_records_by_time_scale(existing_records, [record])
    _atomic_write_json(db_path, {"records": merged})


def _import_module_from_path(module_name: str, file_path: Path):
    spec = importlib.util.spec_from_file_location(module_name, str(file_path))
    if spec is None or spec.loader is None:
        raise ImportError(f"failed to create import spec for {file_path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod


def _extract_wp_demo_from_file(file_path: Path) -> List[Tuple[float, float, float]]:
    """
       wp_demo = [...]  AST   .
         .
    """
    default_wps = [
        (0.0, 0.0, 300.0),
        (800.0, 0.0, 300.0),
        (800.0, 800.0, 350.0),
        (0.0, 800.0, 350.0),
        (-400.0, 400.0, 400.0),
    ]
    try:
        tree = ast.parse(file_path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                for tgt in node.targets:
                    if isinstance(tgt, ast.Name) and tgt.id == "wp_demo":
                        val = ast.literal_eval(node.value)
                        if isinstance(val, list) and all(isinstance(x, (list, tuple)) and len(x) == 3 for x in val):
                            return [(float(x), float(y), float(z)) for (x, y, z) in val]
    except Exception:
        pass
    return default_wps


def _call_sweep_time_scales(
    mod,
    base_waypoints: List[Tuple[float, float, float]],
    *,
    time_scale: float,
    save_dir: Path,
    db_name: str,
    aggregate_path: Path,
    seed: int,
    n_global: int,
    keep_top: int,
    n_restarts: int,
    local_iters: int,
    dt_coarse_base: float,
    dt_fine_base: float,
    tight: bool,
):
    if not hasattr(mod, "sweep_time_scales"):
        raise AttributeError("module has no sweep_time_scales()")

    fn = getattr(mod, "sweep_time_scales")
    sig = inspect.signature(fn)
    kwargs = {}

    #   
    for k, v in {
        "n_global": int(n_global),
        "keep_top": int(keep_top),
        "n_restarts": int(n_restarts),
        "local_iters": int(local_iters),
        "seed": int(seed),
        "dt_coarse": float(dt_coarse_base),
        "dt_fine": float(dt_fine_base),
        "save_dir": str(save_dir),
        "db_name": str(db_name),
        "aggregate_path": str(aggregate_path),
        "tight": bool(tight),
    }.items():
        if k in sig.parameters:
            kwargs[k] = v

    # : sweep_time_scales(base_waypoints, [time_scale], ...)
    return fn(base_waypoints, [float(time_scale)], **kwargs)


def _fallback_tune_one_scale(
    mod,
    base_waypoints: List[Tuple[float, float, float]],
    *,
    time_scale: float,
    save_dir: Path,
    db_name: str,
    aggregate_path: Path,
    seed: int,
    n_global: int,
    keep_top: int,
    n_restarts: int,
    local_iters: int,
    dt_fine_base: float,
    dt_coarse_base: float,
    tight: bool,
) -> Tuple[Path, List[Dict[str, Any]]]:
    """
    sweep_time_scales     .
    - tune_pid_gains()   
    - records  DB 
    """
    if not hasattr(mod, "tune_pid_gains"):
        raise AttributeError("module has no tune_pid_gains() either")

    fn = getattr(mod, "tune_pid_gains")
    sig = inspect.signature(fn)

    dt_fine = float(dt_fine_base) * float(time_scale)
    dt_coarse = float(dt_coarse_base) * float(time_scale)

    save_dir.mkdir(parents=True, exist_ok=True)
    prefix = Path(db_name).stem.replace("_db", "")
    rep_path = save_dir / f"{prefix}_scale_{time_scale:.2f}.report.json"
    gains_path = save_dir / f"{prefix}_scale_{time_scale:.2f}.json"

    kwargs = {}
    for k, v in {
        "n_global": int(n_global),
        "keep_top": int(keep_top),
        "n_restarts": int(n_restarts),
        "local_iters": int(local_iters),
        "seed": int(seed),
        "dt_coarse": float(dt_coarse),
        "dt_fine": float(dt_fine),
        "save_path": str(gains_path),
        "report_path": str(rep_path),
        "tight": bool(tight),
    }.items():
        if k in sig.parameters:
            kwargs[k] = v

    gains_obj, rep = fn(base_waypoints, **kwargs)

    # gains dataclass __dict__ ,  dict  
    if hasattr(gains_obj, "__dict__"):
        gains_dict = dict(gains_obj.__dict__)
    elif isinstance(gains_obj, dict):
        gains_dict = dict(gains_obj)
    else:
        gains_dict = {"raw": str(gains_obj)}

    record = {
        "time_scale": float(time_scale),
        "dt_coarse": float(dt_coarse),
        "dt_fine": float(dt_fine),
        "best_score": float(rep.get("best_score", float("nan"))) if isinstance(rep, dict) else float("nan"),
        "best_stage": rep.get("best_stage") if isinstance(rep, dict) else None,
        "gains": gains_dict,
        "report_path": str(rep_path),
        "gains_path": str(gains_path),
        "tight": bool(tight),
        "seed": int(seed),
        "n_global": int(n_global),
        "keep_top": int(keep_top),
        "n_restarts": int(n_restarts),
        "local_iters": int(local_iters),
    }

    db_path = save_dir / db_name
    _ensure_db_has_record(db_path, record, do_backup=True)
    _ensure_db_has_record(aggregate_path, record, do_backup=True)
    return db_path, [record]


def _print_summary(name: str, *, db_path: Path, runtime_db: Path, records: List[Dict[str, Any]], time_scale: float) -> None:
    rec = None
    for r in records:
        try:
            if abs(float(r.get("time_scale", -1.0)) - float(time_scale)) < 1e-9:
                rec = r
                break
        except Exception:
            continue

    print("")
    print("=" * 78)
    print(f"[{name}] ??? ???")
    print(f"- test DB   : {db_path}")
    print(f"- runtime DB: {runtime_db}")

    if rec is None:
        print("- ???: time_scale ?????? ??? ???????? records????? ????????")
        return

    gains = rec.get("gains", {})
    best_score = rec.get("best_score", None)
    report_path = rec.get("report_path", None)

    print(f"- time_scale : {time_scale:.2f}  (dt={DT_BASE*time_scale:.3f}s, 5Hz ??? dt=0.2s??time_scale=20)")
    print(f"- best_score : {best_score}")
    print(f"- report_path: {report_path}")
    print("- gains:")
    if isinstance(gains, dict):
        for k in sorted(gains.keys()):
            print(f"    {k}: {gains[k]}")
    else:
        print(f"    {gains}")

def _pick_record_for_scale(records: List[Dict[str, Any]], time_scale: float) -> Dict[str, Any] | None:
    best = None
    best_diff = float("inf")
    for r in records:
        try:
            ts = float(r.get("time_scale"))
        except Exception:
            continue
        diff = abs(ts - float(time_scale))
        if diff < best_diff:
            best = r
            best_diff = diff
    return best


def _evaluate_sim(
    *,
    name: str,
    mod,
    gains_cls,
    gains_dict: Dict[str, Any],
    time_scale: float,
    total_time: float,
    pos_tol: float,
    speed_target: float,
    save_plot: Path | None,
    show_plot: bool,
) -> Dict[str, Any]:
    dt = DT_BASE * float(time_scale)
    wps = _extract_wp_demo_from_file(Path(mod.__file__))
    base = gains_cls().__dict__.copy()
    for k in gains_cls().__dataclass_fields__:
        if k in gains_dict:
            base[k] = gains_dict[k]
    gains_obj = gains_cls(**base)
    traj, _, errs_xy, errs_alt, info = mod.simulate_waypoints(
        wps,
        dt=dt,
        total_time=total_time,
        speed_target=speed_target,
        pos_tol=pos_tol,
        gains=gains_obj,
        return_errors=True,
        return_info=True,
    )

    errs_xy = np.asarray(errs_xy, dtype=float)
    errs_alt = np.asarray(errs_alt, dtype=float)
    summary = {
        "name": name,
        "time_scale": float(time_scale),
        "dt": float(dt),
        "total_time": float(total_time),
        "pos_tol": float(pos_tol),
        "speed_target": float(speed_target),
        "finished": bool(info.get("finished", False)),
        "steps": int(info.get("steps", 0)),
        "mean_xy": float(np.mean(errs_xy)) if errs_xy.size else None,
        "p95_xy": float(np.percentile(errs_xy, 95)) if errs_xy.size else None,
        "max_xy": float(np.max(errs_xy)) if errs_xy.size else None,
        "mean_alt": float(np.mean(errs_alt)) if errs_alt.size else None,
        "p95_alt": float(np.percentile(errs_alt, 95)) if errs_alt.size else None,
        "max_alt": float(np.max(errs_alt)) if errs_alt.size else None,
        "sat_total": int(info.get("sat_total", 0)),
        "effort": float(info.get("effort", 0.0)),
        "min_u": float(info.get("min_u", float("nan"))),
        "turn_per_km": float(info.get("turn_per_km", 0.0)),
        "aborted": bool(info.get("aborted", False)),
    }

    if show_plot or save_plot is not None:
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(9, 6))
        ax = fig.add_subplot(121, projection="3d")
        if len(traj) > 0:
            ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color="tab:blue", label="trajectory")
        wps_arr = np.array(wps, dtype=float)
        ax.scatter(wps_arr[:, 0], wps_arr[:, 1], wps_arr[:, 2], color="red", s=40, label="waypoints")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(f"{name} trajectory (dt={dt:.3f}s)")
        ax.legend()

        ax2 = fig.add_subplot(122)
        ax2.plot(errs_xy, label="XY error (m)")
        ax2.plot(errs_alt, label="Alt error (m)")
        ax2.set_xlabel("Step")
        ax2.set_ylabel("Error (m)")
        ax2.set_title("Errors over time")
        ax2.legend()
        plt.tight_layout()

        if save_plot is not None:
            save_plot.parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(save_plot, dpi=140)
            summary["plot_path"] = str(save_plot)
        if show_plot:
            plt.show()
        else:
            plt.close(fig)

    return summary


def _build_paths() -> TunePaths:
    script_path = Path(__file__).resolve()
    tune_root = script_path.parent
    sim_root = script_path.parents[1]
    repo_root = script_path.parents[3]

    return TunePaths(
        repo_root=repo_root,
        sim_root=sim_root,
        tune_root=tune_root,
        uav_tuner_py=tune_root / "uav_pid_waypoint_sim.py",
        lah_tuner_py=tune_root / "lah_pid_waypoint_sim.py",
        uav_test_db=tune_root / "db" / "uav_pid_db.json",
        lah_test_db=tune_root / "db" / "lah_pid_db.json",
        uav_runtime_db=sim_root / "runtime" / "controllers" / "uav_pid_db.json",
        lah_runtime_db=sim_root / "runtime" / "controllers" / "lah_pid_db.json",
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--target", choices=["uav", "lah", "both"], default="both", help="  ")
    p.add_argument("--time-scale", type=float, default=20.0, help="dt (0.01 ). 5Hz 20.0")
    p.add_argument("--seed", type=int, default=0, help=" ")
    p.add_argument("--tight", action="store_true", default=True, help=" dt    tight ( )")
    p.add_argument("--no-tight", dest="tight", action="store_false", help="tight  ()")
    p.add_argument("--n-global", type=int, default=1200, help="  (LHS)")
    p.add_argument("--keep-top", type=int, default=80, help="    ")
    p.add_argument("--n-restarts", type=int, default=24, help="   ")
    p.add_argument("--local-iters", type=int, default=320, help="   ")
    p.add_argument("--dt-coarse-base", type=float, default=0.01, help="coarse dt ( 0.01 -> ts=20 0.2s)")
    p.add_argument("--dt-fine-base", type=float, default=0.01, help="fine dt ( 0.01 -> ts=20 0.2s)")
    p.add_argument("--no-backup", action="store_true", help="DB (.bak_*)  ")
    p.add_argument("--no-eval", action="store_true", help="??? ?? ?????/?? ??")
    p.add_argument("--eval-total-time", type=float, default=160.0, help="??? ?? ????? ???")
    p.add_argument("--eval-pos-tol", type=float, default=30.0, help="??? ?? ?? pos_tol")
    p.add_argument("--eval-plot", action="store_true", help="??? ?? ?? ?? ??")
    p.add_argument("--eval-save-plot", type=str, default=None, help="??? ?? ?? ?? ?? ??")
    p.add_argument("--eval-json", type=str, default=None, help="??? ?? ?? ?? JSON ?? ??")
    args = p.parse_args()

    paths = _build_paths()

    time_scale = float(args.time_scale)
    dt_target = DT_BASE * time_scale

    print("[tune_pid_5hz] ")
    print(f"- repo_root: {paths.repo_root}")
    print(f"- time_scale={time_scale:.2f} -> dt={dt_target:.3f}s ( 5Hz dt=0.2s)")
    print(f"- tight={bool(args.tight)}")
    print(f"- search: n_global={args.n_global}, keep_top={args.keep_top}, n_restarts={args.n_restarts}, local_iters={args.local_iters}")
    print("")

    def run_one(name: str, tuner_path: Path, test_db: Path, runtime_db: Path) -> None:
        if not tuner_path.exists():
            raise FileNotFoundError(f"tuner file not found: {tuner_path}")

        # DB (  )
        if not args.no_backup:
            _backup_if_exists(test_db, tag=f"{name}_testdb")
            _backup_if_exists(runtime_db, tag=f"{name}_runtimedb")

        base_wps = _extract_wp_demo_from_file(tuner_path)

        mod = _import_module_from_path(f"_tuner_{name}", tuner_path)

        save_dir = test_db.parent
        db_name = test_db.name

        print(f"[{name}] sweep_time_scales([ {time_scale:.2f} ]) ")
        try:
            db_path, records = _call_sweep_time_scales(
                mod,
                base_wps,
                time_scale=time_scale,
                save_dir=save_dir,
                db_name=db_name,
                aggregate_path=runtime_db,
                seed=int(args.seed),
                n_global=int(args.n_global),
                keep_top=int(args.keep_top),
                n_restarts=int(args.n_restarts),
                local_iters=int(args.local_iters),
                dt_coarse_base=float(args.dt_coarse_base),
                dt_fine_base=float(args.dt_fine_base),
                tight=bool(args.tight),
            )
        except Exception as e:
            print(f"[{name}] sweep_time_scales : {e}")
            print(f"[{name}] fallback: tune_pid_gains  time_scale  ")
            db_path, records = _fallback_tune_one_scale(
                mod,
                base_wps,
                time_scale=time_scale,
                save_dir=save_dir,
                db_name=db_name,
                aggregate_path=runtime_db,
                seed=int(args.seed),
                n_global=int(args.n_global),
                keep_top=int(args.keep_top),
                n_restarts=int(args.n_restarts),
                local_iters=int(args.local_iters),
                dt_fine_base=float(args.dt_fine_base),
                dt_coarse_base=float(args.dt_coarse_base),
                tight=bool(args.tight),
            )

        # sweep_time_scales runtime_db         
        # records   time_scale  1  runtime DB merge
        rec = None
        for r in records:
            try:
                if abs(float(r.get("time_scale", -1.0)) - time_scale) < 1e-9:
                    rec = r
                    break
            except Exception:
                continue
        if rec is not None:
            _ensure_db_has_record(test_db, rec, do_backup=False)
            _ensure_db_has_record(runtime_db, rec, do_backup=False)

        _print_summary(name, db_path=Path(db_path), runtime_db=runtime_db, records=records, time_scale=time_scale)

        if args.no_eval:
            return

        rec = _pick_record_for_scale(records, time_scale)
        if rec is None:
            data = _load_json_or_empty(test_db)
            recs = data.get("records") if isinstance(data, dict) else None
            if isinstance(recs, list):
                rec = _pick_record_for_scale(recs, time_scale)
        if rec is None or not isinstance(rec.get("gains"), dict):
            print(f"[{name}] eval skipped (no gains record found)")
            return

        speed_target = DEFAULT_SPEED_UAV if name == "uav" else DEFAULT_SPEED_LAH
        eval_plot = Path(args.eval_save_plot) if args.eval_save_plot else None
        if eval_plot is not None and eval_plot.suffix == "":
            eval_plot = eval_plot.with_suffix(".png")
        eval_summary = _evaluate_sim(
            name=name.upper(),
            mod=mod,
            gains_cls=mod.PIDGains,
            gains_dict=rec["gains"],
            time_scale=time_scale,
            total_time=float(args.eval_total_time),
            pos_tol=float(args.eval_pos_tol),
            speed_target=float(speed_target),
            save_plot=eval_plot,
            show_plot=bool(args.eval_plot),
        )

        print("")
        print(f"[{name}] ?? ??")
        for k in [
            "finished",
            "steps",
            "mean_xy",
            "p95_xy",
            "max_xy",
            "mean_alt",
            "p95_alt",
            "max_alt",
            "sat_total",
            "effort",
            "min_u",
            "turn_per_km",
            "aborted",
        ]:
            if k in eval_summary:
                print(f"- {k}: {eval_summary[k]}")

        if args.eval_json:
            eval_path = Path(args.eval_json)
        else:
            eval_path = test_db.with_suffix("")
            eval_path = eval_path.with_name(f"{eval_path.name}_scale_{time_scale:.2f}.eval.json")
        _atomic_write_json(eval_path, eval_summary)
        print(f"- eval_json: {eval_path}")

    if args.target in ("uav", "both"):
        run_one("uav", paths.uav_tuner_py, paths.uav_test_db, paths.uav_runtime_db)

    if args.target in ("lah", "both"):
        run_one("lah", paths.lah_tuner_py, paths.lah_test_db, paths.lah_runtime_db)

    print("")
    print("[tune_pid_5hz] ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
