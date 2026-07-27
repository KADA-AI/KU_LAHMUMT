"""Mission-planning package.

The package root keeps old import paths alive through lazy compatibility
aliases, without keeping one wrapper file per old module name at the root.
"""
from __future__ import annotations

import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import sys
from types import ModuleType


_COMPAT_ALIASES: dict[str, tuple[str, tuple[str, ...] | None]] = {
    "attack_assignment_state": (
        "modules.mission_planning.runtime.state.attack_assignment",
        None,
    ),
    "attack_plan_pipeline": (
        "modules.mission_planning.replanning.triggers.attack.pipeline",
        None,
    ),
    "id_relationship_tab": (
        "modules.mission_planning.ui.id_relationship_tab",
        None,
    ),
    "imaging_schedule_replan_pipeline": (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        ("run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"),
    ),
    "json_io": (
        "modules.mission_planning.runtime.json_io",
        None,
    ),
    "latest_input_cache": (
        "modules.mission_planning.runtime.cache.latest_input",
        None,
    ),
    "mission_path_trim": (
        "modules.mission_planning.pipelines.mission_path_trim",
        None,
    ),
    "mission_plan_file_logger": (
        "modules.mission_planning.runtime.logging.plan_file_logger",
        None,
    ),
    "mission_planning_attack_helpers": (
        "modules.mission_planning.pipelines.mission_planning_attack_helpers",
        None,
    ),
    "mission_planning_gui_env": (
        "modules.mission_planning.ui.mission_planning_gui_env",
        None,
    ),
    "mission_planning_log_tab": (
        "modules.mission_planning.ui.mission_planning_log_tab",
        None,
    ),
    "mission_planning_pipeline_logging": (
        "modules.mission_planning.runtime.logging.pipeline_events",
        None,
    ),
    "next_collab_replan_pipeline": (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        ("run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"),
    ),
    "path_deviation_replan_pipeline": (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        ("run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"),
    ),
    "prior_mission_pipeline": (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        ("run_prior_mission_pipeline", "warm_prior_mission_pipeline"),
    ),
    "prior_mission_pipeline_impl": (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        None,
    ),
}

_COMPAT_PACKAGE_ALIASES: dict[str, str] = {
    "MissionVisualizer": "modules.mission_planning.manual.MissionVisualizer",
    "logic_test": "modules.mission_planning.manual.logic_test",
}


class _CompatAliasLoader(importlib.abc.Loader):
    def __init__(self, fullname: str, target_name: str, exports: tuple[str, ...] | None) -> None:
        self.fullname = fullname
        self.target_name = target_name
        self.exports = exports

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> ModuleType:
        return ModuleType(spec.name)

    def exec_module(self, module: ModuleType) -> None:
        target = importlib.import_module(self.target_name)
        module.__dict__.update(
            {
                "__doc__": f"Compatibility alias for {self.target_name}.",
                "__loader__": self,
                "__package__": self.fullname.rpartition(".")[0],
                "_MODULE": target,
            }
        )
        if self.exports is None:
            for name, value in vars(target).items():
                if name.startswith("__") and name.endswith("__"):
                    continue
                module.__dict__[name] = value
            if hasattr(target, "__all__"):
                module.__dict__["__all__"] = list(target.__all__)
            else:
                module.__dict__["__all__"] = [
                    name for name in vars(target) if not name.startswith("_")
                ]
            return
        for name in self.exports:
            module.__dict__[name] = getattr(target, name)
        module.__dict__["__all__"] = list(self.exports)


class _CompatTargetModuleLoader(importlib.abc.InspectLoader):
    def __init__(self, fullname: str, target_name: str) -> None:
        self.fullname = fullname
        self.target_name = target_name

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> ModuleType:
        return importlib.import_module(self.target_name)

    def exec_module(self, module: ModuleType) -> None:
        return None

    def _target_spec(self) -> importlib.machinery.ModuleSpec | None:
        return importlib.util.find_spec(self.target_name)

    def is_package(self, fullname: str) -> bool:
        target_spec = self._target_spec()
        return bool(target_spec and target_spec.submodule_search_locations is not None)

    def get_code(self, fullname: str) -> object | None:
        target_spec = self._target_spec()
        target_loader = target_spec.loader if target_spec else None
        if target_loader is not None and hasattr(target_loader, "get_code"):
            return target_loader.get_code(self.target_name)  # type: ignore[attr-defined]
        return None

    def get_source(self, fullname: str) -> str | None:
        target_spec = self._target_spec()
        target_loader = target_spec.loader if target_spec else None
        if target_loader is not None and hasattr(target_loader, "get_source"):
            return target_loader.get_source(self.target_name)  # type: ignore[attr-defined]
        return None

    def get_filename(self, fullname: str) -> str:
        target_spec = self._target_spec()
        if target_spec is not None and target_spec.origin:
            return target_spec.origin
        return self.target_name


class _CompatAliasFinder(importlib.abc.MetaPathFinder):
    package_name = __name__

    def find_spec(
        self,
        fullname: str,
        path: object | None,
        target: ModuleType | None = None,
    ) -> importlib.machinery.ModuleSpec | None:
        prefix = f"{self.package_name}."
        if not fullname.startswith(prefix):
            return None
        suffix = fullname[len(prefix):]
        package_target_name = _resolve_compat_package_alias(suffix)
        if package_target_name is not None:
            target_spec = importlib.util.find_spec(package_target_name)
            if target_spec is None:
                return None
            is_package = target_spec.submodule_search_locations is not None
            loader = _CompatTargetModuleLoader(fullname, package_target_name)
            spec = importlib.machinery.ModuleSpec(
                fullname,
                loader,
                origin=target_spec.origin or f"alias:{package_target_name}",
                is_package=is_package,
            )
            if is_package:
                spec.submodule_search_locations = list(
                    target_spec.submodule_search_locations or []
                )
            return spec
        if "." in suffix:
            return None
        alias = _COMPAT_ALIASES.get(suffix)
        if alias is None:
            return None
        target_name, exports = alias
        loader = _CompatAliasLoader(fullname, target_name, exports)
        return importlib.machinery.ModuleSpec(fullname, loader, origin=f"alias:{target_name}")


def _resolve_compat_package_alias(suffix: str) -> str | None:
    for alias_root, target_root in _COMPAT_PACKAGE_ALIASES.items():
        if suffix == alias_root:
            return target_root
        if suffix.startswith(f"{alias_root}."):
            return f"{target_root}.{suffix[len(alias_root) + 1:]}"
    return None


def _install_compat_alias_finder() -> None:
    for finder in sys.meta_path:
        if (
            isinstance(finder, _CompatAliasFinder)
            and finder.package_name == __name__
        ):
            return
    sys.meta_path.insert(0, _CompatAliasFinder())


def __getattr__(name: str) -> ModuleType:
    if name in _COMPAT_ALIASES:
        return importlib.import_module(f"{__name__}.{name}")
    if name in _COMPAT_PACKAGE_ALIASES:
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(name)


_install_compat_alias_finder()

__all__ = ["__getattr__"]
