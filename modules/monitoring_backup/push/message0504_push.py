# modules/monitoring_ver2/push/message0504_push.py
"""Lazy wrapper so monitoring reuses the common FuelWarning push implementation."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from typing import Any, Dict

_common_push = None


def _ensure_clr_loaded() -> None:
    try:
        from ..dll_files import nFusionImports  # type: ignore  # noqa: F401
        _ = nFusionImports
    except Exception:
        pass


def _get_common_push():
    global _common_push
    if _common_push is None:
        _ensure_clr_loaded()
        import importlib

        try:
            _common_push = importlib.import_module('modules.common.push.message0504_push')
        except ModuleNotFoundError as exc:
            if getattr(exc, 'name', '') and exc.name.startswith('nFusion'):
                raise ImportError('nFusion assemblies are not loaded; unable to import FuelWarning push helper.') from exc
            raise
    return _common_push


def _prepare_body(body: Any) -> Dict[str, Any]:
    if is_dataclass(body):
        body_dict = asdict(body)
    elif isinstance(body, dict):
        body_dict = dict(body)
    else:
        body_dict = dict(body or {})

    if 'sourceModuleName' in body_dict and 'source' not in body_dict:
        body_dict['source'] = body_dict.pop('sourceModuleName')

    return body_dict


def make_and_push(body: Any, node_messenger) -> bytes:
    return _get_common_push().make_and_push(_prepare_body(body), node_messenger)


def make_random_and_push(node_messenger) -> bytes:
    return _get_common_push().make_random_and_push(node_messenger)
