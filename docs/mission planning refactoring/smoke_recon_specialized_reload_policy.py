from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


RECON_MODULE = "modules.mission_planning.replanning.triggers.recon_specialized.pipeline"
RECON_PACKAGE = "modules.mission_planning.replanning.triggers.recon_specialized"
RECON_WRAPPER = "modules.mission_planning.pipelines.recon_specialized_pipeline"
RECON_WATCH_PATH = "modules/mission_planning/replanning/triggers/recon_specialized/pipeline.py"

RECON_EXPORTS = (
    "build_recon_specialized_runtime_payload",
    "compare_recon_option_path_signatures",
    "is_recon_specialized_option",
    "summarize_recon_area_review_guard",
    "summarize_recon_expected_path_quality",
)

GUI_DIRECT_RECON_IMPORTS = (
    "build_recon_specialized_runtime_payload",
    "is_recon_specialized_option",
)


class SmokeFailure(RuntimeError):
    pass


class Sentinel:
    def __init__(self, name: str) -> None:
        self.name = name

    def __repr__(self) -> str:
        return f"<sentinel {self.name}>"


def fail(message: str) -> None:
    raise SmokeFailure(message)


def rel_path_set(paths: tuple[Path, ...]) -> set[str]:
    return {str(path).replace("\\", "/") for path in paths}


def check_runtime_policy() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    watch_paths = rel_path_set(runtime.PLANNER_RUNTIME_WATCH_RELATIVE_PATHS)
    reload_order = tuple(runtime.PLANNER_RUNTIME_RELOAD_ORDER)
    bindings = runtime.PIPELINE_RELOAD_BINDINGS

    if RECON_WATCH_PATH not in watch_paths:
        fail(f"recon specialized pipeline is no longer watched: {RECON_WATCH_PATH}")
    if RECON_MODULE not in reload_order:
        fail(f"recon specialized pipeline is no longer reloaded: {RECON_MODULE}")
    if RECON_MODULE in bindings:
        fail("recon specialized pipeline unexpectedly became a globals rebinding target")

    signature = runtime.planner_runtime_source_signature(PROJECT_ROOT)
    signature_keys = tuple(key for key, _sig in signature)
    if RECON_WATCH_PATH not in signature_keys:
        fail(f"recon specialized pipeline is missing from source signature: {RECON_WATCH_PATH}")


def check_refresh_reloads_without_rebinding_recon_exports() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    stale_namespace = {name: Sentinel(f"stale:{name}") for name in RECON_EXPORTS}
    before = dict(stale_namespace)
    calls: list[str] = []

    original_reload = runtime.reload_planning_module
    original_invalidate_caches = importlib.invalidate_caches

    recon_module = SimpleNamespace(
        **{name: Sentinel(f"fresh:{name}") for name in RECON_EXPORTS}
    )

    def fake_reload(module_name: str) -> Any:
        calls.append(module_name)
        if module_name == RECON_MODULE:
            return recon_module
        return SimpleNamespace()

    try:
        runtime.reload_planning_module = fake_reload
        importlib.invalidate_caches = lambda: None
        runtime.refresh_live_planning_helpers(stale_namespace)
    finally:
        runtime.reload_planning_module = original_reload
        importlib.invalidate_caches = original_invalidate_caches

    if calls != list(runtime.PLANNER_RUNTIME_RELOAD_ORDER):
        fail(f"planner reload order changed while checking recon policy:\n{calls!r}")
    if RECON_MODULE not in calls:
        fail("recon specialized pipeline was not reloaded during refresh")
    for attr_name in RECON_EXPORTS:
        if stale_namespace[attr_name] is not before[attr_name]:
            fail(f"recon export was unexpectedly rebound by refresh: {attr_name}")

    absent_namespace: dict[str, Any] = {}
    try:
        runtime.reload_planning_module = fake_reload
        importlib.invalidate_caches = lambda: None
        runtime.refresh_live_planning_helpers(absent_namespace)
    finally:
        runtime.reload_planning_module = original_reload
        importlib.invalidate_caches = original_invalidate_caches

    for attr_name in RECON_EXPORTS:
        if attr_name in absent_namespace:
            fail(f"recon export was unexpectedly created in namespace: {attr_name}")


def check_wrapper_exports() -> None:
    canonical = importlib.import_module(RECON_MODULE)
    package = importlib.import_module(RECON_PACKAGE)
    wrapper = importlib.import_module(RECON_WRAPPER)

    wrapper_all = set(getattr(wrapper, "__all__", ()))
    package_all = set(getattr(package, "__all__", ()))
    for attr_name in RECON_EXPORTS:
        if not hasattr(canonical, attr_name):
            fail(f"canonical recon module missing export: {attr_name}")
        if getattr(wrapper, attr_name, None) is not getattr(canonical, attr_name):
            fail(f"recon compatibility wrapper identity changed for {attr_name}")
        if getattr(package, attr_name, None) is not getattr(canonical, attr_name):
            fail(f"recon package export identity changed for {attr_name}")
        if attr_name not in wrapper_all:
            fail(f"recon compatibility wrapper __all__ missing {attr_name}")
        if attr_name not in package_all:
            fail(f"recon package __all__ missing {attr_name}")


def check_gui_import_policy() -> None:
    gui_path = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"
    source = gui_path.read_text(encoding="utf-8-sig", errors="ignore")
    tree = ast.parse(source)

    import_sites: list[tuple[str | None, set[str]]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported = {alias.name for alias in node.names}
            if node.module in {
                "modules.mission_planning.replanning.triggers.recon_specialized.pipeline",
                "replanning.triggers.recon_specialized.pipeline",
            } or str(node.module or "").endswith(".replanning.triggers.recon_specialized.pipeline"):
                import_sites.append((node.module, imported))

    needed = set(GUI_DIRECT_RECON_IMPORTS)
    if not any(needed.issubset(imported) for _module, imported in import_sites):
        fail("mission_planning_gui no longer imports direct recon helper globals")

    refresh_function = None
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "_refresh_live_planning_helpers":
            refresh_function = node
            break
    if refresh_function is None:
        fail("mission_planning_gui missing _refresh_live_planning_helpers bridge")
    refresh_text = ast.get_source_segment(source, refresh_function) or ""
    for attr_name in GUI_DIRECT_RECON_IMPORTS:
        if attr_name in refresh_text:
            fail(f"mission_planning_gui refresh bridge unexpectedly rebinds recon helper: {attr_name}")


def main() -> int:
    try:
        check_runtime_policy()
        check_refresh_reloads_without_rebinding_recon_exports()
        check_wrapper_exports()
        check_gui_import_policy()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("recon specialized watch/reload policy smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
