"""Backward-compatible wrapper for next-area-mode entry point."""

from modules.mission_planning.legacy.apps.next_area_mode.main import main


if __name__ == "__main__":
    raise SystemExit(main())
