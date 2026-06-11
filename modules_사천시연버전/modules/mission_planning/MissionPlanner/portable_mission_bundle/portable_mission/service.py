from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List
import json
import os
import uuid

from stable_baselines3 import PPO

from .env import PortableMissionEnv
from .terrain import (
    build_dem_preview,
    crop_dem_by_normalized_roi,
    describe_dem,
    list_dem_files,
    resolve_dem_path,
    save_uploaded_dem,
)

ACTION_LABELS = ["Turn Left", "Go Straight", "Turn Right"]


class MissionService:
    def __init__(self, bundle_root: Path) -> None:
        self.bundle_root = bundle_root
        self.models_dir = bundle_root / "models"
        self.inputs_dir = bundle_root / "data" / "inputs"
        self.work_dir = bundle_root / "data" / "work"
        self.model_path = self.models_dir / "latest_model.zip"
        self.config_path = self.models_dir / "model_config.json"
        self.model_config = self._load_json(self.config_path)
        self.defaults = self._build_defaults(self.model_config)
        self._model: PPO | None = None
        self._sessions: Dict[str, Dict[str, Any]] = {}

    def health(self) -> Dict[str, Any]:
        return {
            "ok": self.model_path.exists(),
            "bundle_root": str(self.bundle_root),
            "model_path": str(self.model_path),
            "model_exists": self.model_path.exists(),
            "dem_count": len(self.list_dems()),
            "session_count": len(self._sessions),
        }

    def model_info(self) -> Dict[str, Any]:
        return {
            "model_file": self.model_path.name,
            "model_exists": self.model_path.exists(),
            "config_file": self.config_path.name if self.config_path.exists() else None,
            "defaults": self.defaults,
            "action_labels": ACTION_LABELS,
        }

    def list_dems(self) -> List[Dict[str, Any]]:
        output = []
        for name in list_dem_files(self.inputs_dir):
            path = resolve_dem_path(self.inputs_dir, name)
            info = describe_dem(path)
            output.append(
                {
                    "name": info["name"],
                    "width": info["width"],
                    "height": info["height"],
                    "crs": info["crs"],
                    "is_geographic": info["is_geographic"],
                }
            )
        return output

    def dem_preview(self, name: str) -> Dict[str, Any]:
        path = resolve_dem_path(self.inputs_dir, name)
        return {
            "name": path.name,
            "dem": describe_dem(path),
            "preview": build_dem_preview(path),
        }

    def upload_dem(self, file_storage) -> Dict[str, Any]:
        saved = save_uploaded_dem(file_storage, self.inputs_dir)
        return self.dem_preview(saved.name)

    def create_session(
        self,
        *,
        dem_name: str,
        roi: Dict[str, Any],
        altitude_m: float | None = None,
        hex_step: int | None = None,
        max_steps: int | None = None,
        max_goals: int | None = None,
    ) -> Dict[str, Any]:
        source_path = resolve_dem_path(self.inputs_dir, dem_name)
        source_info = describe_dem(source_path)
        session_id = uuid.uuid4().hex[:12]
        work_dem_path = self.work_dir / f"{session_id}_{source_path.stem}_roi{source_path.suffix.lower()}"
        crop_info = crop_dem_by_normalized_roi(source_path, roi, work_dem_path)

        env = PortableMissionEnv(
            work_dem_path,
            altitude_m=altitude_m if altitude_m is not None else self.defaults["altitude_m"],
            hex_step=hex_step if hex_step is not None else self.defaults["hex_step"],
            max_steps=max_steps if max_steps is not None else self.defaults["max_steps"],
            max_goals=max_goals if max_goals is not None else self.defaults["max_goals"],
            crash_penalty=self.defaults["crash_penalty"],
            step_penalty=self.defaults["step_penalty"],
            success_reward=self.defaults["success_reward"],
            goal_reward=self.defaults["goal_reward"],
            progress_scale=self.defaults["progress_scale"],
            proximity_penalty=self.defaults["proximity_penalty"],
        )

        session = {
            "id": session_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "source_dem": source_info,
            "crop": crop_info,
            "roi": roi,
            "env": env,
            "is_geographic": source_info["is_geographic"],
            "work_dem_path": str(work_dem_path),
        }
        self._sessions[session_id] = session
        return self._session_payload(session)

    def simulate(
        self,
        *,
        session_id: str,
        start_cell: Any,
        goal_cells: Any,
        deterministic: bool = True,
    ) -> Dict[str, Any]:
        session = self._sessions.get(session_id)
        if session is None:
            raise ValueError(f"Unknown session_id: {session_id}")

        env: PortableMissionEnv = session["env"]
        obs, _ = env.configure_manual_episode(
            self._pair(start_cell),
            [self._pair(cell) for cell in (goal_cells or [])],
        )
        model = self._load_model()

        visited_cells = [env.current_cell]
        step_log = []
        total_reward = 0.0
        terminated = False
        truncated = False
        last_info: Dict[str, Any] = {}

        while True:
            action, _ = model.predict(obs, deterministic=bool(deterministic))
            action_int = int(action)
            obs, reward, terminated, truncated, last_info = env.step(action_int)
            total_reward += float(reward)
            if env.current_cell is not None:
                visited_cells.append(env.current_cell)
            step_log.append(
                {
                    "step": int(env.steps),
                    "action": action_int,
                    "action_label": ACTION_LABELS[action_int],
                    "reward": float(reward),
                    "cumulative_reward": float(total_reward),
                    "cell": self._cell_payload(env.current_cell),
                    "remaining_goals": int(last_info.get("remaining_goals", 0)),
                    "reward_components": {
                        key: float(value)
                        for key, value in dict(last_info.get("reward_components", {})).items()
                    },
                    "termination_reason": last_info.get("termination_reason"),
                }
            )
            if terminated or truncated:
                break

        world_path = [
            self._world_point(env, cell, session["is_geographic"])
            for cell in visited_cells
            if cell is not None
        ]
        dense_world_path = self._densify_world_path(world_path, session["is_geographic"])

        return {
            "session_id": session_id,
            "mission": {
                "start_cell": self._cell_payload(self._pair(start_cell)),
                "goal_cells": [self._cell_payload(self._pair(cell)) for cell in goal_cells],
                "altitude_m": float(env.altitude_m),
                "deterministic": bool(deterministic),
            },
            "summary": {
                "steps": int(env.steps),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "termination_reason": last_info.get("termination_reason"),
                "reward_total": float(total_reward),
                "goals_total": len(goal_cells),
                "goals_completed": int(last_info.get("goals_completed", 0)),
                "remaining_goals": int(last_info.get("remaining_goals", 0)),
            },
            "path": {
                "cells": [self._cell_payload(cell) for cell in visited_cells if cell is not None],
                "world_path": world_path,
                "dense_world_path": dense_world_path,
            },
            "step_log": step_log,
        }

    def _session_payload(self, session: Dict[str, Any]) -> Dict[str, Any]:
        env: PortableMissionEnv = session["env"]
        return {
            "session_id": session["id"],
            "created_at": session["created_at"],
            "source_dem": session["source_dem"],
            "work_dem_path": session["work_dem_path"],
            "roi": session["roi"],
            "crop": session["crop"],
            "config": {
                "altitude_m": float(env.altitude_m),
                "hex_step": int(env.hex_step),
                "max_steps": int(env.max_steps),
                "max_goals": int(env.max_goals),
            },
            "grid": {
                "rows": int(env.grid_shape[0]),
                "cols": int(env.grid_shape[1]),
                "occupancy": env.occupancy.astype(int).tolist(),
                "safe_count": env.safe_count,
                "blocked_count": env.blocked_count,
            },
            "coordinate_mode": "geographic" if session["is_geographic"] else "projected",
        }

    def _load_model(self) -> PPO:
        if self._model is None:
            if not self.model_path.exists():
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            device = os.environ.get("MISSION_MODEL_DEVICE", "cpu")
            self._model = PPO.load(self.model_path, device=device)
        return self._model

    @staticmethod
    def _load_json(path: Path) -> Dict[str, Any]:
        if not path.exists():
            return {}
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)

    @staticmethod
    def _build_defaults(config: Dict[str, Any]) -> Dict[str, Any]:
        env_cfg = config.get("env") if isinstance(config.get("env"), dict) else config
        return {
            "altitude_m": float(env_cfg.get("altitude_m", 700.0)),
            "hex_step": int(env_cfg.get("hex_step", 25)),
            "max_steps": int(env_cfg.get("max_steps", 3000)),
            "max_goals": int(env_cfg.get("max_goals", 20)),
            "crash_penalty": float(env_cfg.get("crash_penalty", -5.0)),
            "step_penalty": float(env_cfg.get("step_penalty", -0.1)),
            "success_reward": float(env_cfg.get("success_reward", 10.0)),
            "goal_reward": float(env_cfg.get("goal_reward", 5.0)),
            "progress_scale": float(env_cfg.get("progress_scale", 0.2)),
            "proximity_penalty": float(env_cfg.get("proximity_penalty", 0.02)),
        }

    @staticmethod
    def _pair(value: Any) -> tuple[int, int]:
        if isinstance(value, dict):
            return int(value["row"]), int(value["col"])
        row, col = value
        return int(row), int(col)

    @staticmethod
    def _cell_payload(cell: Any) -> Dict[str, int] | None:
        if cell is None:
            return None
        row, col = cell
        return {"row": int(row), "col": int(col)}

    @staticmethod
    def _world_point(
        env: PortableMissionEnv,
        cell: tuple[int, int],
        is_geographic: bool,
    ) -> Dict[str, float]:
        x, y = env.centers[cell]
        if is_geographic:
            return {
                "lon": float(x),
                "lat": float(y),
                "altitude_m": float(env.altitude_m),
            }
        return {
            "x": float(x),
            "y": float(y),
            "altitude_m": float(env.altitude_m),
        }

    @staticmethod
    def _densify_world_path(
        points: List[Dict[str, float]],
        is_geographic: bool,
        samples_per_segment: int = 6,
    ) -> List[Dict[str, float]]:
        if len(points) < 2:
            return list(points)
        key_x = "lon" if is_geographic else "x"
        key_y = "lat" if is_geographic else "y"
        dense: List[Dict[str, float]] = []
        for index in range(len(points) - 1):
            start = points[index]
            end = points[index + 1]
            for step in range(samples_per_segment):
                t = step / samples_per_segment
                dense.append(
                    {
                        key_x: float(start[key_x] + (end[key_x] - start[key_x]) * t),
                        key_y: float(start[key_y] + (end[key_y] - start[key_y]) * t),
                        "altitude_m": float(
                            start["altitude_m"] + (end["altitude_m"] - start["altitude_m"]) * t
                        ),
                    }
                )
        dense.append(points[-1])
        return dense
