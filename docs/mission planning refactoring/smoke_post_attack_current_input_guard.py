from __future__ import annotations

import inspect
import os
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "mission")
    desired = (
        project_root,
        project_root / "modules",
        project_root / "modules" / "mission_planning",
        project_root / "modules" / "mission_planning" / "MissionPlanner",
    )
    for path in reversed(desired):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)


def require(source: str, needle: str, message: str) -> None:
    if needle not in source:
        raise RuntimeError(message)


def main() -> int:
    configure_import_paths()
    from modules.mission_planning.replanning.triggers.post_attack import pipeline as post_attack

    run_source = inspect.getsource(post_attack.run_post_attack_rejoin_pipeline)
    update_source = inspect.getsource(post_attack._build_post_attack_active_done_followup_update)

    require(
        run_source,
        'remaining_snapshot_unavailable = str(skip_reason or "") == "remaining_snapshot_unavailable"',
        "post-attack no longer tracks remaining snapshot unavailability",
    )
    require(
        run_source,
        "remaining_snapshot_unavailable\n                            and progress_percent is None",
        "post-attack no longer preserves current input when snapshot/progress is unavailable",
    )
    require(
        run_source,
        "include_completion_boundary_hold=bool(preserve_current_active_mission)",
        "post-attack no longer forces a boundary hold for uncertain current input",
    )
    require(
        update_source,
        "and not include_completion_boundary_hold",
        "post-attack boundary-skip optimization can still remove a forced current input hold",
    )

    print("post-attack current input guard smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
