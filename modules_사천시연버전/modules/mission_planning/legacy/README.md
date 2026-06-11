## Legacy Notes

This folder stores mission-planning structures removed from the active runtime root.

Archive buckets:

- `wrappers/`
  - old top-level backward-compat files
  - these should delegate to active modules under `modules/mission_planning`, not hold primary logic
- `compat_packages/`
  - old package-level wrappers such as `MissionVisualizer` and `next_area_mode`
- `apps/`
  - standalone utilities not used by the active mission-planning runtime
- `ui/`
  - archived UI helpers removed from the active runtime path
- `MissionPlanner_tools/`
  - archived MissionPlanner tool scripts not used by active runtime code
- `tests/`
  - test and exploratory utilities
- `logic_test/`
  - older experimental logic and test helpers
- `docs/`
  - notes and reference documents
- `static/`
  - leftover static or generated files

Current active rule:

- general mission planning uses only the base preset `dubins_mode`
- general option variants no longer switch preset/profile-specific planning logic
- manual FOV remains a mode inside the base preset, not a separate preset

Archived concepts:

- option-label overrides such as recon-to-nadir or min-time-to-vertical
- preset-based runtime branching such as `custom` preset checks
- profile-specific type-decider branches for general mission planning

Attack planning, attack exclusion, prior mission, and replan pipelines remain outside this archive scope.
