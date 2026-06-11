from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


CURRENT_REMAINING_MODULE = "modules.mission_planning.replanning.triggers.remaining_hybrid.current"
AIRCRAFT_PARALLEL_MODULE = "modules.mission_planning.runtime.aircraft_parallel_0303"

CURRENT_REMAINING_BINDINGS = (
    "CurrentRemainingHybridRequest",
    "build_current_remaining_hybrid",
    "merge_current_remaining_hybrid",
    "validate_current_remaining_hybrid_request",
    "validate_current_remaining_hybrid_paths",
    "filter_generic_flightpath_missions_for_hybrid",
)

AIRCRAFT_PARALLEL_BINDINGS = (
    "build_0303_flight_plans_aircraft_parallel",
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


def make_namespace(attr_names: tuple[str, ...], prefix: str) -> dict[str, Any]:
    return {attr_name: Sentinel(f"{prefix}:{attr_name}") for attr_name in attr_names}


def all_rebinding_attrs(runtime: Any) -> tuple[str, ...]:
    attrs: list[str] = []
    attrs.extend(CURRENT_REMAINING_BINDINGS)
    attrs.extend(AIRCRAFT_PARALLEL_BINDINGS)
    for binding_names in runtime.PIPELINE_RELOAD_BINDINGS.values():
        attrs.extend(binding_names)
    seen: set[str] = set()
    unique: list[str] = []
    for attr_name in attrs:
        if attr_name in seen:
            fail(f"duplicate planner rebinding attr in fixture: {attr_name}")
        seen.add(attr_name)
        unique.append(attr_name)
    return tuple(unique)


def make_module(module_name: str, attr_names: tuple[str, ...], values: dict[str, Any]) -> Any:
    kwargs = {attr_name: Sentinel(f"fresh:{module_name}:{attr_name}") for attr_name in attr_names}
    values.update(kwargs)
    return SimpleNamespace(**kwargs)


def make_full_module_map(runtime: Any, values: dict[str, Any]) -> dict[str, Any]:
    modules = {module_name: SimpleNamespace() for module_name in runtime.PLANNER_RUNTIME_RELOAD_ORDER}
    modules[CURRENT_REMAINING_MODULE] = make_module(
        CURRENT_REMAINING_MODULE,
        CURRENT_REMAINING_BINDINGS,
        values,
    )
    modules[AIRCRAFT_PARALLEL_MODULE] = make_module(
        AIRCRAFT_PARALLEL_MODULE,
        AIRCRAFT_PARALLEL_BINDINGS,
        values,
    )
    for module_name, attr_names in runtime.PIPELINE_RELOAD_BINDINGS.items():
        modules[module_name] = make_module(module_name, tuple(attr_names), values)
    return modules


def run_with_fake_reloader(
    runtime: Any,
    modules: dict[str, Any],
    namespace: dict[str, Any],
) -> tuple[list[str], list[str], list[str]]:
    calls: list[str] = []
    invalidations: list[str] = []
    events: list[str] = []
    original_reload = runtime.reload_planning_module
    original_invalidate_caches = importlib.invalidate_caches

    def fake_reload(module_name: str) -> Any:
        calls.append(module_name)
        events.append(f"reload:{module_name}")
        return modules.get(module_name)

    def fake_invalidate_caches() -> None:
        invalidations.append("invalidate")
        events.append("invalidate")

    try:
        runtime.reload_planning_module = fake_reload
        importlib.invalidate_caches = fake_invalidate_caches
        runtime.refresh_live_planning_helpers(namespace)
    finally:
        runtime.reload_planning_module = original_reload
        importlib.invalidate_caches = original_invalidate_caches

    return calls, invalidations, events


def check_rebinds_stale_namespace() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    attr_names = all_rebinding_attrs(runtime)
    namespace = make_namespace(attr_names, "stale")
    fresh_values: dict[str, Any] = {}
    modules = make_full_module_map(runtime, fresh_values)

    calls, invalidations, events = run_with_fake_reloader(runtime, modules, namespace)

    expected_calls = list(runtime.PLANNER_RUNTIME_RELOAD_ORDER)
    if calls != expected_calls:
        fail(f"planner reload order changed during rebinding:\n{calls!r}")
    if invalidations != ["invalidate"]:
        fail(f"planner cache invalidation changed during rebinding: {invalidations!r}")
    if events[0] != "invalidate":
        fail(f"planner cache invalidation no longer happens before reload: {events!r}")
    for attr_name in attr_names:
        expected = fresh_values[attr_name]
        actual = namespace.get(attr_name)
        if actual is not expected:
            fail(f"planner rebinding did not refresh stale namespace attr: {attr_name}")


def check_missing_attrs_and_none_modules_keep_existing_namespace() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    attr_names = all_rebinding_attrs(runtime)
    namespace = make_namespace(attr_names, "existing")
    before = dict(namespace)

    modules = {module_name: SimpleNamespace() for module_name in runtime.PLANNER_RUNTIME_RELOAD_ORDER}
    missing_current_attr = "validate_current_remaining_hybrid_paths"
    modules[CURRENT_REMAINING_MODULE] = make_module(
        CURRENT_REMAINING_MODULE,
        tuple(attr for attr in CURRENT_REMAINING_BINDINGS if attr != missing_current_attr),
        {},
    )

    modules[AIRCRAFT_PARALLEL_MODULE] = SimpleNamespace()

    none_module = "modules.mission_planning.replanning.triggers.prior.pipeline"
    modules[none_module] = None

    missing_pipeline_module = "modules.mission_planning.replanning.triggers.attack.pipeline"
    missing_pipeline_attr = "run_attack_plan_pipeline"
    modules[missing_pipeline_module] = make_module(
        missing_pipeline_module,
        tuple(attr for attr in runtime.PIPELINE_RELOAD_BINDINGS[missing_pipeline_module] if attr != missing_pipeline_attr),
        {},
    )

    calls, _invalidations, _events = run_with_fake_reloader(runtime, modules, namespace)
    if calls != list(runtime.PLANNER_RUNTIME_RELOAD_ORDER):
        fail(f"planner reload order stopped or changed with None module:\n{calls!r}")

    expected_unchanged = (
        missing_current_attr,
        "build_0303_flight_plans_aircraft_parallel",
        missing_pipeline_attr,
        *runtime.PIPELINE_RELOAD_BINDINGS[none_module],
    )
    for attr_name in expected_unchanged:
        if namespace[attr_name] is not before[attr_name]:
            fail(f"planner rebinding fallback changed existing namespace attr: {attr_name}")


def check_missing_attrs_with_absent_namespace_keys_contract() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    namespace: dict[str, Any] = {}
    none_module = "modules.mission_planning.replanning.triggers.prior.pipeline"
    modules = {module_name: SimpleNamespace() for module_name in runtime.PLANNER_RUNTIME_RELOAD_ORDER}
    modules[none_module] = None

    run_with_fake_reloader(runtime, modules, namespace)

    expected_none_attrs: list[str] = []
    expected_none_attrs.extend(CURRENT_REMAINING_BINDINGS)
    expected_none_attrs.extend(AIRCRAFT_PARALLEL_BINDINGS)
    for module_name, attr_names in runtime.PIPELINE_RELOAD_BINDINGS.items():
        if module_name == none_module:
            continue
        expected_none_attrs.extend(attr_names)

    for attr_name in expected_none_attrs:
        if attr_name not in namespace:
            fail(f"planner missing attr no longer writes absent namespace key: {attr_name}")
        if namespace[attr_name] is not None:
            fail(f"planner missing attr absent-key fallback is no longer None: {attr_name}")
    for attr_name in runtime.PIPELINE_RELOAD_BINDINGS[none_module]:
        if attr_name in namespace:
            fail(f"planner None module no longer leaves absent namespace key untouched: {attr_name}")


def check_reload_planning_module_fallbacks() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    original_import_module = runtime.importlib.import_module
    original_reload = runtime.importlib.reload
    imported_module = SimpleNamespace(name="imported")
    reloaded_module = SimpleNamespace(name="reloaded")

    try:
        runtime.importlib.import_module = lambda _name: (_ for _ in ()).throw(RuntimeError("import failed"))
        result = runtime.reload_planning_module("missing.module")
        if result is not None:
            fail("reload_planning_module import-failure fallback no longer returns None")
    finally:
        runtime.importlib.import_module = original_import_module
        runtime.importlib.reload = original_reload

    try:
        runtime.importlib.import_module = lambda _name: imported_module
        runtime.importlib.reload = lambda _module: (_ for _ in ()).throw(RuntimeError("reload failed"))
        result = runtime.reload_planning_module("reload.failure")
        if result is not imported_module:
            fail("reload_planning_module reload-failure fallback no longer returns imported module")
    finally:
        runtime.importlib.import_module = original_import_module
        runtime.importlib.reload = original_reload

    try:
        runtime.importlib.import_module = lambda _name: imported_module
        runtime.importlib.reload = lambda _module: reloaded_module
        result = runtime.reload_planning_module("reload.success")
        if result is not reloaded_module:
            fail("reload_planning_module success path no longer returns reloaded module")
    finally:
        runtime.importlib.import_module = original_import_module
        runtime.importlib.reload = original_reload


def main() -> int:
    try:
        check_rebinds_stale_namespace()
        check_missing_attrs_and_none_modules_keep_existing_namespace()
        check_missing_attrs_with_absent_namespace_keys_contract()
        check_reload_planning_module_fallbacks()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("planner hot-reload globals rebinding fixture smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
