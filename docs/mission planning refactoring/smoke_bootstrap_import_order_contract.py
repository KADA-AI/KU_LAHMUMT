from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


BOOTSTRAP_MODULE = "modules.mission_planning.app.bootstrap"
PROCESS_CONSOLE_MODULE = "modules.common.process_console"
ALLOWED_EARLY_PROCESS_CONSOLE_IMPORTS = {
    "emit_process_lifecycle_event",
    "emit_process_log",
}


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def parse_source(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8-sig", errors="ignore"))


def import_module_name(node: ast.ImportFrom) -> str:
    prefix = "." * int(getattr(node, "level", 0) or 0)
    return prefix + str(node.module or "")


def top_level_call_line(tree: ast.Module, name: str) -> int:
    for node in tree.body:
        if not isinstance(node, ast.Expr):
            continue
        call = node.value
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == name:
            return int(node.lineno)
    fail(f"top-level call missing: {name}")


def first_import_line(
    tree: ast.Module,
    predicate: Callable[[str, set[str]], bool],
) -> int | None:
    lines: list[int] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            module_name = import_module_name(node)
            imported_names = {alias.name for alias in node.names}
            if predicate(module_name, imported_names):
                lines.append(int(node.lineno))
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if predicate(alias.name, {alias.name}):
                    lines.append(int(node.lineno))
    return min(lines) if lines else None


def local_module_path(module_name: str) -> Path | None:
    if module_name.startswith("."):
        return None
    if not (module_name.startswith("modules.") or module_name.startswith("app.")):
        return None
    rel = Path(*module_name.split("."))
    py_path = PROJECT_ROOT / rel.with_suffix(".py")
    if py_path.exists():
        return py_path
    init_path = PROJECT_ROOT / rel / "__init__.py"
    if init_path.exists():
        return init_path
    return None


def direct_local_imports_before(tree: ast.Module, cutoff_line: int) -> set[str]:
    modules: set[str] = set()
    for node in ast.walk(tree):
        lineno = int(getattr(node, "lineno", 10**9))
        if lineno >= cutoff_line:
            continue
        if isinstance(node, ast.ImportFrom):
            module_name = import_module_name(node)
            if module_name.startswith("modules.") or module_name.startswith("app."):
                modules.add(module_name)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                module_name = alias.name
                if module_name.startswith("modules.") or module_name.startswith("app."):
                    modules.add(module_name)
    return modules


def top_level_call_names(tree: ast.Module) -> set[str]:
    names: set[str] = set()

    def visit_statement(node: ast.AST) -> None:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            return
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.Call):
                func = child.func
                if isinstance(func, ast.Name):
                    names.add(func.id)
                elif isinstance(func, ast.Attribute):
                    names.add(func.attr)
            visit_statement(child)

    for statement in tree.body:
        visit_statement(statement)
    return names


def call_lines_before(tree: ast.Module, cutoff_line: int, names: set[str]) -> list[tuple[str, int]]:
    found: list[tuple[str, int]] = []
    for node in ast.walk(tree):
        if int(getattr(node, "lineno", 10**9)) >= cutoff_line:
            continue
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Name) and func.id in names:
            found.append((func.id, int(node.lineno)))
    return sorted(found, key=lambda row: row[1])


def assert_early_modules_have_no_role_sensitive_side_effects(module_names: set[str]) -> None:
    for module_name in sorted(module_names):
        path = local_module_path(module_name)
        if path is None:
            continue
        source = path.read_text(encoding="utf-8-sig", errors="ignore")
        if module_name != BOOTSTRAP_MODULE and "KU_ROLE" in source:
            fail(f"early import module reads/writes KU_ROLE before mission role setup: {module_name}")
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.Import, ast.ImportFrom)):
                continue
            imported_modules: list[str] = []
            if isinstance(node, ast.ImportFrom):
                imported_modules.append(import_module_name(node))
            else:
                imported_modules.extend(alias.name for alias in node.names)
            for imported_module in imported_modules:
                if (
                    "MissionPlanner" in imported_module
                    or imported_module.startswith("data_def")
                    or imported_module.startswith("AnS")
                    or ".engine.mission_generation." in imported_module
                ):
                    fail(
                        "early import module gained role-sensitive import before mission role setup: "
                        f"{module_name} imports {imported_module}"
                    )


def assert_early_modules_have_no_logging_side_effects(module_names: set[str]) -> None:
    disallowed_calls = {
        "ensure_console",
        "install_process_file_logging",
        "emit_process_log",
        "emit_process_lifecycle_event",
        "PipelineLogManager",
        "MissionPlanFileLogger",
    }
    for module_name in sorted(module_names):
        path = local_module_path(module_name)
        if path is None:
            continue
        tree = parse_source(path)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.Import, ast.ImportFrom)):
                continue
            imported_modules: list[str] = []
            if isinstance(node, ast.ImportFrom):
                imported_modules.append(import_module_name(node))
            else:
                imported_modules.extend(alias.name for alias in node.names)
            for imported_module in imported_modules:
                if (
                    ".runtime.logging." in imported_module
                    or imported_module.startswith(".runtime.logging.")
                    or ".runtime.debug_artifacts" in imported_module
                    or imported_module.startswith(".runtime.debug_artifacts")
                    or ".runtime.json_io" in imported_module
                    or imported_module.startswith(".runtime.json_io")
                ):
                    fail(
                        "early import module gained logging/runtime import before console setup: "
                        f"{module_name} imports {imported_module}"
                    )
        found_calls = top_level_call_names(tree) & disallowed_calls
        if found_calls:
            fail(
                "early import module gained top-level logging/console side-effect call before console setup: "
                f"{module_name} calls {sorted(found_calls)!r}"
            )


def check_bootstrap_module_contract() -> None:
    bootstrap_path = PROJECT_ROOT / "modules" / "mission_planning" / "app" / "bootstrap.py"
    tree = parse_source(bootstrap_path)

    for node in tree.body:
        if isinstance(node, ast.ImportFrom) and import_module_name(node) == PROCESS_CONSOLE_MODULE:
            fail("bootstrap imports process_console at module import time")
        if isinstance(node, ast.Import):
            imported = {alias.name for alias in node.names}
            if PROCESS_CONSOLE_MODULE in imported:
                fail("bootstrap imports process_console at module import time")

    runtime_import_lines: list[int] = []
    ensure_call_line: int | None = None
    install_call_line: int | None = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "configure_mission_process_console":
            for child in ast.walk(node):
                if isinstance(child, ast.ImportFrom) and import_module_name(child) == PROCESS_CONSOLE_MODULE:
                    imported = {alias.name for alias in child.names}
                    if {"ensure_console", "install_process_file_logging"}.issubset(imported):
                        runtime_import_lines.append(int(child.lineno))
                if isinstance(child, ast.Call) and isinstance(child.func, ast.Name):
                    if child.func.id == "ensure_console":
                        ensure_call_line = int(child.lineno)
                    if child.func.id == "install_process_file_logging":
                        install_call_line = int(child.lineno)
            break

    if not runtime_import_lines:
        fail("configure_mission_process_console no longer lazy-imports process_console")
    if ensure_call_line is None or install_call_line is None:
        fail("configure_mission_process_console missing console or file logging call")
    if ensure_call_line > install_call_line:
        fail("mission process console is no longer ensured before file logging install")

    bootstrap = importlib.import_module(BOOTSTRAP_MODULE)
    fake_env: dict[str, str] = {}
    role = bootstrap.configure_mission_role(fake_env)
    if role != "mission" or fake_env.get("KU_ROLE") != "mission":
        fail(f"configure_mission_role changed: role={role!r} env={fake_env!r}")

    calls: list[tuple[str, str]] = []
    fake_process_console = ModuleType(PROCESS_CONSOLE_MODULE)

    def fake_ensure_console(title: str | None = None) -> bool:
        calls.append(("ensure_console", str(title)))
        return True

    def fake_install_process_file_logging(module_name: str) -> object:
        calls.append(("install_process_file_logging", str(module_name)))
        return object()

    fake_process_console.ensure_console = fake_ensure_console  # type: ignore[attr-defined]
    fake_process_console.install_process_file_logging = fake_install_process_file_logging  # type: ignore[attr-defined]

    original_process_console = sys.modules.get(PROCESS_CONSOLE_MODULE)
    try:
        sys.modules[PROCESS_CONSOLE_MODULE] = fake_process_console
        bootstrap.configure_mission_process_console({"KU_CONSOLE_TITLE": "Custom Mission Console"})
    finally:
        if original_process_console is None:
            sys.modules.pop(PROCESS_CONSOLE_MODULE, None)
        else:
            sys.modules[PROCESS_CONSOLE_MODULE] = original_process_console

    expected = [
        ("ensure_console", "Custom Mission Console"),
        ("install_process_file_logging", "mission_planning"),
    ]
    if calls != expected:
        fail(f"mission process console setup order changed: {calls!r}")


def check_mission_gui_import_order_contract() -> None:
    gui_path = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"
    tree = parse_source(gui_path)

    role_line = top_level_call_line(tree, "configure_mission_role")
    console_line = top_level_call_line(tree, "configure_mission_process_console")
    if role_line >= console_line:
        fail("configure_mission_role no longer runs before configure_mission_process_console")

    bootstrap_import_line = first_import_line(
        tree,
        lambda module, names: module == BOOTSTRAP_MODULE
        and {"configure_mission_role", "configure_mission_process_console"}.issubset(names),
    )
    if bootstrap_import_line is None or bootstrap_import_line >= role_line:
        fail("mission bootstrap helpers are not imported before role setup")

    early_role_modules = direct_local_imports_before(tree, role_line)
    assert_early_modules_have_no_role_sensitive_side_effects(early_role_modules)

    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        module_name = import_module_name(node)
        if module_name != PROCESS_CONSOLE_MODULE or int(node.lineno) >= console_line:
            continue
        imported = {alias.name for alias in node.names}
        if not imported.issubset(ALLOWED_EARLY_PROCESS_CONSOLE_IMPORTS):
            fail(
                "mission GUI imports process_console setup functions before console setup: "
                f"{sorted(imported)!r}"
            )

    first_role_sensitive_import = first_import_line(
        tree,
        lambda module, _names: (
            "MissionPlanner" in module
            or module.startswith("data_def")
            or module.startswith("AnS")
            or ".engine.mission_generation." in module
        ),
    )
    if first_role_sensitive_import is not None and role_line >= first_role_sensitive_import:
        fail(
            "configure_mission_role no longer runs before role-sensitive planner imports "
            f"(role line {role_line}, first import line {first_role_sensitive_import})"
        )

    first_logging_or_runtime_side_effect_import = first_import_line(
        tree,
        lambda module, _names: (
            ".runtime.logging." in module
            or module.startswith(".runtime.logging.")
            or ".runtime.debug_artifacts" in module
            or module.startswith(".runtime.debug_artifacts")
            or ".runtime.json_io" in module
            or module.startswith(".runtime.json_io")
            or module.startswith("PyQt5")
            or module.endswith("mission_visualization_tab")
        ),
    )
    if (
        first_logging_or_runtime_side_effect_import is not None
        and console_line >= first_logging_or_runtime_side_effect_import
    ):
        fail(
            "configure_mission_process_console no longer runs before logging/runtime side-effect imports "
            f"(console line {console_line}, first import line {first_logging_or_runtime_side_effect_import})"
        )

    early_console_modules = direct_local_imports_before(tree, console_line)
    assert_early_modules_have_no_logging_side_effects(early_console_modules)

    early_side_effect_calls = call_lines_before(
        tree,
        console_line,
        {
            "emit_process_log",
            "emit_process_lifecycle_event",
            "PipelineLogManager",
            "MissionPlanFileLogger",
        },
    )
    if early_side_effect_calls:
        fail(f"mission GUI has process/logging side-effect calls before console setup: {early_side_effect_calls!r}")


def main() -> int:
    try:
        check_bootstrap_module_contract()
        check_mission_gui_import_order_contract()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("mission bootstrap import-order contract smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
