from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .hex_utils import (
    ALTITUDE_BUFFER_M,
    DIRECTION_SEQUENCE,
    DIRECTION_TO_INDEX,
    aggregate_to_hex,
    compute_obstacle_mask,
    compute_ring_offsets,
    generate_hex_geometry,
    offset_to_axial,
    relative_action_to_direction,
    step_neighbor,
)
from .terrain import load_dem

FAR_RING_OFFSETS, FAR_RING_FORWARD_INDEX = compute_ring_offsets(2)
FAR_RING_SIZE = len(FAR_RING_OFFSETS[0])
NEAR_RING_SIZE = len(DIRECTION_SEQUENCE)


class PortableMissionEnv:
    def __init__(
        self,
        dem_path: str | Path,
        *,
        altitude_m: float,
        hex_step: int,
        max_steps: int,
        max_goals: int,
        crash_penalty: float = -5.0,
        step_penalty: float = -0.1,
        success_reward: float = 10.0,
        goal_reward: float = 5.0,
        progress_scale: float = 0.2,
        proximity_penalty: float = 0.02,
    ) -> None:
        self.dem_path = Path(dem_path)
        self.altitude_m = float(altitude_m)
        self.hex_step = max(1, int(hex_step))
        self.max_steps = max(1, int(max_steps))
        self.max_goals = max(1, int(max_goals))
        self.crash_penalty = float(crash_penalty)
        self.step_penalty = float(step_penalty)
        self.success_reward = float(success_reward)
        self.goal_reward = float(goal_reward)
        self.progress_scale = float(progress_scale)
        self.proximity_penalty = float(proximity_penalty)

        dem, bounds = load_dem(self.dem_path)
        threshold = self.altitude_m - ALTITUDE_BUFFER_M
        mask = compute_obstacle_mask(dem, threshold)
        self.occupancy = aggregate_to_hex(mask, self.hex_step)
        self.bounds = bounds
        self.grid_shape = self.occupancy.shape
        self.safe_cells = np.argwhere(~self.occupancy)
        if self.safe_cells.size == 0:
            raise RuntimeError("No safe cells are available inside this ROI.")

        _, self.centers, self.transform = generate_hex_geometry(bounds, self.grid_shape)
        self.current_cell: Optional[Tuple[int, int]] = None
        self.target_cell: Optional[Tuple[int, int]] = None
        self.goal_list: List[Tuple[int, int]] = []
        self.current_goal_idx = 0
        self.heading_idx: Optional[int] = None
        self.steps = 0

    @property
    def safe_count(self) -> int:
        return int(np.count_nonzero(~self.occupancy))

    @property
    def blocked_count(self) -> int:
        return int(np.count_nonzero(self.occupancy))

    def configure_manual_episode(
        self,
        start_cell: Tuple[int, int],
        goal_cells: Sequence[Tuple[int, int]],
    ) -> Tuple[np.ndarray, Dict[str, object]]:
        start = self._coerce_cell(start_cell, "start_cell")
        goals = [self._coerce_cell(cell, f"goal_cells[{idx}]") for idx, cell in enumerate(goal_cells)]
        if not goals:
            raise ValueError("At least one goal cell is required.")
        if self.occupancy[start]:
            raise ValueError("start_cell must be a safe cell.")

        seen = {start}
        for goal in goals:
            if self.occupancy[goal]:
                raise ValueError("goal_cells must contain only safe cells.")
            if goal in seen:
                raise ValueError("start_cell and goal_cells must be unique.")
            seen.add(goal)

        self.current_cell = start
        self.goal_list = list(goals)
        self.current_goal_idx = 0
        self.target_cell = self.goal_list[0]
        self.heading_idx = self._initial_heading()
        self.steps = 0
        return self._get_obs(), {
            "start_cell": self.current_cell,
            "goal_queue": self.goal_list.copy(),
            "manual_episode": True,
        }

    def step(self, action: int):
        if self.current_cell is None or self.target_cell is None or self.heading_idx is None:
            raise RuntimeError("Episode is not configured.")
        if action not in {0, 1, 2}:
            raise ValueError("Action must be 0, 1, or 2.")

        reward_components: Dict[str, float] = {"step": self.step_penalty}
        terminated = False
        truncated = False
        termination_reason: Optional[str] = None
        collision_cell: Optional[Tuple[int, int]] = None

        prev_distance = self._hex_distance(self.current_cell, self.target_cell)
        self.heading_idx, direction = relative_action_to_direction(self.heading_idx, action)
        next_row, next_col = step_neighbor(*self.current_cell, direction)

        if not (0 <= next_row < self.grid_shape[0] and 0 <= next_col < self.grid_shape[1]):
            reward_components["collision"] = self.crash_penalty
            terminated = True
            termination_reason = "out_of_bounds"
        elif self.occupancy[next_row, next_col]:
            collision_cell = (next_row, next_col)
            reward_components["collision"] = self.crash_penalty
            terminated = True
            termination_reason = "obstacle"
        else:
            self.current_cell = (next_row, next_col)
            new_distance = self._hex_distance(self.current_cell, self.target_cell)
            reward_components["progress"] = self.progress_scale * (prev_distance - new_distance)
            reward_components["proximity"] = -self.proximity_penalty * self._count_adjacent_obstacles(self.current_cell)

            if self.current_cell == self.target_cell:
                reward_components["goal"] = self.goal_reward
                self.current_goal_idx += 1
                if self.current_goal_idx >= len(self.goal_list):
                    reward_components["success"] = self.success_reward
                    terminated = True
                    termination_reason = "success"
                else:
                    self.target_cell = self.goal_list[self.current_goal_idx]

        self.steps += 1
        if self.steps >= self.max_steps and not terminated:
            truncated = True
            termination_reason = "time_limit"

        reward = float(sum(reward_components.values()))
        info = {
            "heading_direction": DIRECTION_SEQUENCE[self.heading_idx],
            "reward_components": reward_components,
            "current_goal_index": self.current_goal_idx,
            "remaining_goals": len(self.goal_list) - self.current_goal_idx,
            "termination_reason": termination_reason,
            "goals_total": len(self.goal_list),
            "goals_completed": self.current_goal_idx,
            "collision_cell": collision_cell,
        }
        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self) -> np.ndarray:
        rows, cols = self.grid_shape
        row, col = self.current_cell if self.current_cell else (-1, -1)
        target_row, target_col = self.target_cell if self.target_cell else (-1, -1)
        heading = self.heading_idx if self.heading_idx is not None else 0
        remaining = max(len(self.goal_list) - self.current_goal_idx, 0) / max(1, self.max_goals)

        def norm_row(value: int) -> float:
            return value / (rows - 1) if rows > 1 and value >= 0 else 0.0

        def norm_col(value: int) -> float:
            return value / (cols - 1) if cols > 1 and value >= 0 else 0.0

        row_norm = norm_row(row)
        col_norm = norm_col(col)
        target_row_norm = norm_row(target_row)
        target_col_norm = norm_col(target_col)
        delta_row = target_row_norm - row_norm
        delta_col = target_col_norm - col_norm

        core = np.array(
            [
                row_norm,
                col_norm,
                heading,
                target_row_norm,
                target_col_norm,
                remaining,
                delta_row,
                delta_col,
            ],
            dtype=np.float32,
        )
        return np.concatenate([core, self._near_safety_vector(), self._far_safety_vector()])

    def _initial_heading(self) -> int:
        if self.current_cell is None or self.target_cell is None:
            return DIRECTION_TO_INDEX[2]
        best_direction = 2
        best_distance = float("inf")
        for direction in DIRECTION_SEQUENCE:
            nr, nc = step_neighbor(*self.current_cell, direction)
            if not (0 <= nr < self.grid_shape[0] and 0 <= nc < self.grid_shape[1]):
                continue
            if self.occupancy[nr, nc]:
                continue
            distance = self._hex_distance((nr, nc), self.target_cell)
            if distance < best_distance:
                best_distance = distance
                best_direction = direction
        return DIRECTION_TO_INDEX[best_direction]

    def _hex_distance(self, cell: Tuple[int, int], target: Tuple[int, int]) -> float:
        (r1, c1), (r2, c2) = cell, target
        q1, s1 = offset_to_axial(r1, c1)
        q2, s2 = offset_to_axial(r2, c2)
        dq = q1 - q2
        ds = s1 - s2
        dt = -dq - ds
        return float(max(abs(dq), abs(ds), abs(dt)))

    def _count_adjacent_obstacles(self, cell: Tuple[int, int]) -> int:
        row, col = cell
        count = 0
        for direction in DIRECTION_SEQUENCE:
            nr, nc = step_neighbor(row, col, direction)
            if self._cell_safety_value(nr, nc) < 1.0:
                count += 1
        return count

    def _cell_safety_value(self, row: int, col: int) -> float:
        if not (0 <= row < self.grid_shape[0] and 0 <= col < self.grid_shape[1]):
            return -1.0
        return 0.0 if self.occupancy[row, col] else 1.0

    def _near_safety_vector(self) -> np.ndarray:
        values = np.full(NEAR_RING_SIZE, -1.0, dtype=np.float32)
        if self.current_cell is None:
            return values
        for idx, direction in enumerate(DIRECTION_SEQUENCE):
            nr, nc = step_neighbor(*self.current_cell, direction)
            values[idx] = self._cell_safety_value(nr, nc)
        heading_idx = self.heading_idx if self.heading_idx is not None else 0
        return np.roll(values, -heading_idx)

    def _far_safety_vector(self) -> np.ndarray:
        values = np.full(FAR_RING_SIZE, -1.0, dtype=np.float32)
        if self.current_cell is None:
            return values
        row, col = self.current_cell
        offsets = FAR_RING_OFFSETS[row & 1]
        for idx, (dr, dc) in enumerate(offsets):
            values[idx] = self._cell_safety_value(row + dr, col + dc)
        heading_idx = self.heading_idx if self.heading_idx is not None else 0
        heading_dir = DIRECTION_SEQUENCE[heading_idx]
        return np.roll(values, -FAR_RING_FORWARD_INDEX[heading_dir])

    def _coerce_cell(self, cell: Tuple[int, int], name: str) -> Tuple[int, int]:
        try:
            row, col = cell
            row = int(row)
            col = int(col)
        except Exception as exc:  # noqa: BLE001
            raise ValueError(f"{name} must be a pair of integers.") from exc
        if not (0 <= row < self.grid_shape[0] and 0 <= col < self.grid_shape[1]):
            raise ValueError(f"{name} is outside the grid bounds.")
        return row, col
