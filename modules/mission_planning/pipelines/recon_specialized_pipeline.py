from __future__ import annotations

import copy
from typing import Any, Dict, Optional

from modules.common.option_codes import normalize_option_code

try:
    from ..MissionPlanner.runtime_settings import (
        canonicalize_runtime_payload,
        get_runtime_manual_fov_sync_active,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        canonicalize_runtime_payload,
        get_runtime_manual_fov_sync_active,
    )


RECON_OPTION_CODE = 4
DEFAULT_RECON_AREA_SPLIT_WIDTH_M = 600.0
DEFAULT_RECON_AREA_FIXED_FOV_DEG = 15.0
DEFAULT_RECON_SWEEP_SEPARATION_SCALE = 0.50


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def is_recon_specialized_option(
    option_code: object | None = None,
    option_label: object | None = None,
) -> bool:
    code = normalize_option_code(option_code)
    if code is None:
        code = normalize_option_code(option_label)
    return int(code or 0) == RECON_OPTION_CODE


def build_recon_specialized_runtime_payload(
    base_payload: Dict[str, Any] | None,
) -> Dict[str, Any]:
    if isinstance(base_payload, dict):
        payload = copy.deepcopy(base_payload)
    else:
        payload = canonicalize_runtime_payload(None)
    values = payload.get("values")
    if not isinstance(values, dict):
        values = {}
        payload["values"] = values

    split_width_m = max(
        50.0,
        _as_float(
            values.get("recon_area_split_width_m"),
            DEFAULT_RECON_AREA_SPLIT_WIDTH_M,
        ),
    )
    fixed_fov_deg = max(
        0.1,
        _as_float(
            values.get("recon_area_fixed_fov_deg"),
            DEFAULT_RECON_AREA_FIXED_FOV_DEG,
        ),
    )
    manual_sync_active = get_runtime_manual_fov_sync_active(payload)
    if manual_sync_active:
        fixed_fov_deg = max(
            0.1,
            _as_float(
                values.get("global_manual_fov_deg"),
                fixed_fov_deg,
            ),
        )
    sweep_sep_scale = max(
        0.05,
        min(
            1.0,
            _as_float(
                values.get("recon_sweep_separation_scale"),
                DEFAULT_RECON_SWEEP_SEPARATION_SCALE,
            ),
        ),
    )
    base_sep_m = max(
        1.0,
        _as_float(
            values.get("default_sweep_separation_m"),
            1000.0,
        ),
    )
    values["enhanced_area_review_max_segment_m"] = float(split_width_m)
    values["area_custom_fov_deg"] = float(fixed_fov_deg)
    values["recon_area_fixed_fov_deg"] = float(fixed_fov_deg)
    values["default_sweep_separation_m"] = float(base_sep_m * sweep_sep_scale)
    # Recon fixes area-side values directly, but manual-global FOV must still override recon FOV.
    values["enhanced_auto_fov_from_db"] = False
    values["manual_fov_global_sync_enabled"] = bool(manual_sync_active)
    return payload
