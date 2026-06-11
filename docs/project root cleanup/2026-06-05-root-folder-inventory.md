# 2026-06-05 Root Folder Inventory

Goal:

- Keep the project root readable for day-to-day updates under `app/` and
  `modules/`.
- Avoid deleting tracked resources, logs, or reference files without an owner
  decision.
- Prevent new unclassified top-level files or folders from quietly appearing.

Current root classification:

| Path | Category | Current decision |
| --- | --- | --- |
| `app/` | core source | Keep at root. |
| `modules/` | core source | Keep at root. |
| `settings/` | project settings | Keep at root as the canonical settings folder. |
| `docs/` | refactoring, cleanup records, and reference documents | Keep during refactor. |
| `resource/` | active runtime resource | Keep for now. Code directly reads DEM/GeoTIFF, map, FOV DB, and nFusion schema resources here. |
| `Logs/` | runtime/scenario data | Defer. It has many tracked files and current scenario shapes still reference `Logs/Scenario_...`. |
| `docs/reference/` | tracked reference documents | Moved here from root `ref/` to keep the project root cleaner. |
| legacy `temp/` | runtime scratch | Removed from the root surface. `.gitignore` blocks new files; existing tracked deletions need a separate git decision. |
| `modules_bkup/` | user-managed backup copy | Preserve. The user confirmed this is a real backup and will clean it manually later. Codex must not edit, move, or delete it. |
| `modules copy/` | untracked backup copy | Same policy as `modules_bkup/` if it appears locally. |
| `.vscode/` | local workspace config | Keep unless a repo policy says otherwise. |
| `run.py`, `sim_main.py`, `log_main.py` | launchers | Keep at root. |
| `.gitignore`, `.gitattributes` | repo metadata | Keep at root. |

Observed tracking counts:

- `Logs/`: 3275 tracked files.
- `resource/`: 539 tracked files.
- legacy `ref/`: 10 tracked files before migration; current reference home is
  `docs/reference/`.
- legacy `temp/`: 14 tracked files before cleanup; current root folder is absent.
- `modules_bkup/`: 0 tracked files.
- `modules copy/`: 0 tracked files when present.

`modules_bkup/` comparison against active `modules/`:

- Excluding `__pycache__` and `*.pyc`, `modules_bkup/` has 1195 files and
  active `modules/` has 1258 files.
- Active `modules/` has 67 files that do not exist in `modules_bkup/`,
  including the new mission-planning package folders and
  `modules/common/settings_paths.py`.
- `modules_bkup/` has only 4 files not present at the same path in active
  `modules/`: the old root `mission_planning/logic_ref.md`, `map.html`, and
  `logic_memo/**` files. These have already been moved into
  `modules/mission_planning/manual/`.
- 75 common files differ by length or hash, so `modules_bkup/` is not a
  canonical source tree and should not be used for runtime imports.
- The folder is still preserved because the user identified it as a real backup.

Next safe actions:

- Do not remove, archive, move, or modify `modules_bkup/`; the user will handle
  it later.
- Keep `/modules_bkup/` and `/modules copy/` ignored so local backups are not
  accidentally staged.
- Keep `/ref/` ignored so the old root reference folder does not reappear.
- Keep `/temp/` ignored and absent from the root; it was only scratch/runtime
  state.
- Decide whether `Logs/` should become external runtime data or stay as tracked
  fixtures. Do not move it without explicit user approval.
- Decide whether `ref/` should remain at root or move under `docs/reference/`.
- Decide whether tracked `temp/` files should be removed from git history/index.
- Keep `resource/` in place until all hard-coded `resource/` contracts are
  migrated and verified.
