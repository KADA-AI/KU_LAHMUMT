from __future__ import annotations

import ast
import contextlib
import importlib
import io
import os
import sys
import tempfile
import types
from pathlib import Path
from typing import Any, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MISSION_GUI = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"
MISSION_GUI_ENV = PROJECT_ROOT / "modules" / "mission_planning" / "ui" / "mission_planning_gui_env.py"
RUN_PY = PROJECT_ROOT / "run.py"


class SmokeFailure(RuntimeError):
    pass


class FakeClr:
    def __init__(self, *, fail_message_library_stem: bool = False) -> None:
        self.fail_message_library_stem = bool(fail_message_library_stem)
        self.references: list[str] = []

    def AddReference(self, value: object) -> None:
        text = str(value)
        self.references.append(text)
        if self.fail_message_library_stem and text.endswith("MessageLibrary"):
            raise RuntimeError("stem reference failed")


@contextlib.contextmanager
def patched(obj: Any, name: str, value: Any) -> Iterator[None]:
    original = getattr(obj, name)
    setattr(obj, name, value)
    try:
        yield
    finally:
        setattr(obj, name, original)


@contextlib.contextmanager
def patched_many(patches: list[tuple[Any, str, Any]]) -> Iterator[None]:
    originals: list[tuple[Any, str, Any]] = []
    try:
        for obj, name, value in patches:
            originals.append((obj, name, getattr(obj, name)))
            setattr(obj, name, value)
        yield
    finally:
        for obj, name, original in reversed(originals):
            setattr(obj, name, original)


@contextlib.contextmanager
def fake_nfusion_imports(fake_clr: FakeClr) -> Iterator[None]:
    saved = {name: sys.modules.get(name) for name in ("dll_files", "dll_files.nFusionImports")}
    package = types.ModuleType("dll_files")
    module = types.ModuleType("dll_files.nFusionImports")
    module.clr = fake_clr
    sys.modules["dll_files"] = package
    sys.modules["dll_files.nFusionImports"] = module
    try:
        yield
    finally:
        for name, original in saved.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


def fail(message: str) -> None:
    raise SmokeFailure(message)


def ensure_project_path() -> None:
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))


def import_run_module_safely():
    ensure_project_path()
    process_console = importlib.import_module("modules.common.process_console")
    db_paths = importlib.import_module("modules.common.db_paths")
    with patched_many(
        [
            (process_console, "ensure_console", lambda *_args, **_kwargs: False),
            (process_console, "install_process_file_logging", lambda *_args, **_kwargs: None),
            (db_paths, "bootstrap_db_root", lambda: PROJECT_ROOT / "Logs"),
        ]
    ):
        return importlib.import_module("run")


def check_run_py_config_policy() -> None:
    run_module = import_run_module_safely()
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        root = tmp / "root"
        ds_dir = tmp / "decision_support"
        common_dir = tmp / "common"
        for path in (root, ds_dir, common_dir):
            path.mkdir(parents=True)

        patches = [
            (run_module, "PROJECT_ROOT", root),
            (run_module, "DS_DIR", ds_dir),
            (run_module, "COMMON_DIR", common_dir),
        ]
        with patched_many(patches):
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                missing_result = run_module._ensure_fusion_configs()
            if missing_result is not None:
                fail(f"run.py missing-config policy changed: {missing_result!r}")
            if "running without bus" not in stderr.getvalue():
                fail(f"run.py missing-config warning changed: {stderr.getvalue()!r}")

            settings_src = common_dir / "nFusionSettings.json"
            license_src = common_dir / "nFusionLicense.lic"
            settings_src.write_text('{"Middleware":{"NetworkAddress":"127.0.0.1"}}', encoding="utf-8")
            license_src.write_bytes(b"license-bytes")
            existing_result = run_module._ensure_fusion_configs()
            settings_dst = root / "settings" / "nFusionSettings.json"
            license_dst = root / "settings" / "nFusionLicense.lic"
            if existing_result != str(settings_dst):
                fail(f"run.py config return path changed: {existing_result!r}")
            if settings_dst.read_text(encoding="utf-8") != settings_src.read_text(encoding="utf-8"):
                fail("run.py did not copy nFusionSettings.json text")
            if license_dst.read_bytes() != b"license-bytes":
                fail("run.py did not copy nFusionLicense.lic bytes")


def check_mission_gui_config_policy() -> None:
    ensure_project_path()
    env_module = importlib.import_module("modules.mission_planning.ui.mission_planning_gui_env")
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        root = tmp / "root"
        common_dir = tmp / "common"
        root.mkdir(parents=True)
        common_dir.mkdir(parents=True)

        try:
            env_module._ensure_fusion_configs(root, common_dir)
        except FileNotFoundError as exc:
            if "nFusionSettings.json" not in str(exc) or "FusionSettings.json" not in str(exc):
                fail(f"mission GUI missing-config error message changed: {exc}")
        else:
            fail("mission GUI missing-config policy changed: expected FileNotFoundError")

        settings_src = common_dir / "FusionSettings.json"
        license_src = common_dir / "nFusionLicense.lic"
        settings_src.write_bytes(b'{"Middleware":{"NetworkAddress":"127.0.0.1"}}')
        license_src.write_bytes(b"mission-license")
        result = env_module._ensure_fusion_configs(root, common_dir)
        settings_dst = root / "settings" / "nFusionSettings.json"
        license_dst = root / "settings" / "nFusionLicense.lic"
        if result != str(settings_dst):
            fail(f"mission GUI config return path changed: {result!r}")
        if settings_dst.read_bytes() != settings_src.read_bytes():
            fail("mission GUI did not copy FusionSettings.json to nFusionSettings.json")
        if license_dst.read_bytes() != b"mission-license":
            fail("mission GUI did not copy nFusionLicense.lic")


def check_message_library_loader_contracts() -> None:
    ensure_project_path()
    run_module = import_run_module_safely()
    env_module = importlib.import_module("modules.mission_planning.ui.mission_planning_gui_env")
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp = Path(tmp_dir)
        common_dir = tmp / "common"
        msg_dir = common_dir / "msg_files"
        msg_dir.mkdir(parents=True)
        for name in ("MessageLibrary.dll", "K4586Model.dll", "K4586Model.Assist.dll", "MiscUtil.dll"):
            (msg_dir / name).write_bytes(b"")

        fake_clr = FakeClr(fail_message_library_stem=True)
        with fake_nfusion_imports(fake_clr), patched(run_module, "COMMON_DIR", common_dir):
            run_module._load_msglib_and_deps()
        refs = [Path(ref).name for ref in fake_clr.references]
        if refs[:2] != ["MessageLibrary", "MessageLibrary.dll"]:
            fail(f"run.py MessageLibrary fallback changed: {refs!r}")
        for expected in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
            if expected not in refs:
                fail(f"run.py optional dependency AddReference missing: {expected}")

        fake_clr = FakeClr(fail_message_library_stem=True)
        with fake_nfusion_imports(fake_clr):
            env_module._load_msglib_and_deps(common_dir)
        refs = [Path(ref).name for ref in fake_clr.references]
        if refs[:2] != ["MessageLibrary", "MessageLibrary.dll"]:
            fail(f"mission GUI MessageLibrary fallback changed: {refs!r}")
        for expected in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
            if expected not in refs:
                fail(f"mission GUI optional dependency AddReference missing: {expected}")


def call_name(node: ast.Call) -> str:
    func = node.func
    if isinstance(func, ast.Attribute):
        parts = [func.attr]
        value = func.value
        while isinstance(value, ast.Attribute):
            parts.append(value.attr)
            value = value.value
        if isinstance(value, ast.Name):
            parts.append(value.id)
        return ".".join(reversed(parts))
    if isinstance(func, ast.Name):
        return func.id
    return ""


class CallCollector(ast.NodeVisitor):
    def __init__(self) -> None:
        self.calls: list[ast.Call] = []

    def visit_Call(self, node: ast.Call) -> None:
        self.calls.append(node)
        self.generic_visit(node)


def find_class_method(tree: ast.Module, class_name: str, method_name: str) -> ast.FunctionDef:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == method_name:
                    return item
    fail(f"{class_name}.{method_name} not found")
    raise AssertionError("unreachable")


def check_mission_gui_nfusion_source_contract() -> None:
    source = MISSION_GUI.read_text(encoding="utf-8-sig", errors="ignore")
    ordered_snippets = (
        "from dll_files.nFusionImports import *",
        "_settings_path = _ensure_fusion_configs(PROJECT_ROOT, COMMON_DIR)",
        "_ = _load_msglib_and_deps(COMMON_DIR)",
        "from receive import *",
        "from Tabs.assignment_planning_tab import AssignmentPlanningTab",
    )
    positions = [source.find(snippet) for snippet in ordered_snippets]
    if any(pos < 0 for pos in positions) or positions != sorted(positions):
        fail(f"mission GUI nFusion bootstrap order changed: {positions!r}")

    tree = ast.parse(source)
    rx_setup = find_class_method(tree, "MainWindow", "_rx_setup")
    collector = CallCollector()
    collector.visit(rx_setup)
    calls = [(call_name(call), call) for call in collector.calls]
    names = [name for name, _call in calls]
    expected_order = [
        "FusionNodeIoc.Configure",
        "NodeMessenger.Initialize",
        "NodeMessenger.RegistAllConsumerFromFusionNodeIoc",
        "NodeMessenger.InitAllSubscriberFromAssembly",
        "NodeMessenger.RegistAllProviderFromFusionNodeIoc",
    ]
    indexes: list[int] = []
    for expected in expected_order:
        try:
            indexes.append(names.index(expected))
        except ValueError:
            fail(f"mission GUI _rx_setup missing call: {expected}")
    if indexes != sorted(indexes):
        fail(f"mission GUI _rx_setup call order changed: {indexes!r}")
    init_call = calls[indexes[1]][1]
    if not init_call.args or not isinstance(init_call.args[0], ast.Constant) or init_call.args[0].value != "MMR_ReceiveNode":
        fail("mission GUI NodeMessenger.Initialize channel changed")

    bus_ready_values: set[bool] = set()
    for node in ast.walk(rx_setup):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if (
                isinstance(target, ast.Attribute)
                and target.attr == "_bus_ready"
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
                and isinstance(node.value, ast.Constant)
                and isinstance(node.value.value, bool)
            ):
                bus_ready_values.add(bool(node.value.value))
    if bus_ready_values != {False, True}:
        fail(f"mission GUI _bus_ready success/failure contract changed: {bus_ready_values!r}")

    run_source = RUN_PY.read_text(encoding="utf-8-sig", errors="ignore")
    if 'NodeMessenger.Initialize("CommonChannel")' not in run_source:
        fail("run.py dashboard bus monitor channel changed")


def main() -> int:
    try:
        check_run_py_config_policy()
        check_mission_gui_config_policy()
        check_message_library_loader_contracts()
        check_mission_gui_nfusion_source_contract()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("nFusion config/MMR_ReceiveNode contract smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
