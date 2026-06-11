# HTML/PNG Output Artifact Manifest Progress

## Scope

This checkpoint freezes mission-planning HTML map and attack visualization PNG output contracts before moving visualization or tool modules.

## Added

- `smoke_html_png_output_artifacts.py`

## Contract Captured

- Active mission-planning visualization tab:
  - default map output remains `temp/mission_planning_map.html`.
  - `MissionVisualizationTab` keeps `map_html_path` injection.
  - map save still uses `fmap.save(str(self._map_html_path))`.
  - Qt loading still uses `QUrl.fromLocalFile`.
  - DB readers still cover `MissionPlan`, `IndividualMissionPlan`, `FlightPath`, and both `waypointList`/`lahWaypointList`.
- Corridor GUI:
  - default map output remains `temp/map.html`.
  - map save still injects `qrc:///qtwebchannel/qwebchannel.js`.
  - the click bridge remains wired through `MapBridge`, `pyqtSignal`, and `QUrl.fromLocalFile(...resolve())`.
- Manual mission visualizers:
  - temp output directory still uses `mission_visualizer_` prefix.
  - map filename remains `mission_map.html`.
  - `_write_map()` saves folium HTML, reads it as UTF-8, and loads it into `QWebEngineView` with a local base URL.
  - active, tool, and legacy duplicate visualizer copies are all covered so later duplicate cleanup can detect drift.
- Enhanced planning map renderer:
  - `build_map_html(cmpk, mrpk, split_result=None) -> str` remains an HTML-string renderer.
  - OpenStreetMap, 0203/original/split layer hooks, and split tester legend markers remain present.
- Attack assistance PNG:
  - `save_attack_visualization(...)` keeps its positional argument contract.
  - matplotlib uses the `Agg` backend.
  - directory output resolves to `attack_visualization.png`.
  - extensionless output appends `.png`.
  - output directories are created.
  - final save remains `dpi=180` with `bbox_inches="tight"`.
  - CLI `--save-png` continues to populate `visualization_png`.
  - existing subprocess call sites remain documented as output-JSON attack point helpers, not PNG renderers.

## Boundary

This smoke does not import PyQt, folium, or matplotlib. It does not launch Qt windows, render QWebEngine views, run attack terrain analysis, or write real PNG files. It checks AST signatures and source markers only.

Checked-in `map.html` copies are recorded as static generated artifacts/deletion candidates. The mission-planning root copy was moved to `manual/reference/map.html`; these files are not treated as live producer output paths.

## Why This Is Safe

No runtime code changed. The smoke reads source files and parses Python AST without executing mission planning GUI, map, or attack-assistance modules.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_html_png_output_artifacts.py"
python "docs\mission planning refactoring\smoke_html_png_output_artifacts.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
HTML/PNG output artifact manifest smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `manual/operator entrypoint inventory`.
