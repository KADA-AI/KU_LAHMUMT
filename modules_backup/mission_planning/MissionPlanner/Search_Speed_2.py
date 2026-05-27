from typing import Iterable, Sequence

import numpy as np

Vector3 = Sequence[float]


class SearchSpeedCalculator:
    """Compute sweep path speed that matches a UAV path arrival time."""

    def __init__(self, uav_path: Iterable[Vector3], sweep_path: Iterable[Vector3], uav_speed: float):
        self.uav_path = uav_path
        self.sweep_path = sweep_path
        self.uav_speed = uav_speed

    @staticmethod
    def _to_array(path: Iterable[Vector3]) -> np.ndarray:
        """Convert a path to an (N, 3) float array and validate basic shape."""
        arr = np.asarray(list(path), dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 3:
            raise ValueError("path must be an iterable of 3D points shaped (N, 3)")
        if arr.shape[0] < 2:
            raise ValueError("path must contain at least two points")
        return arr

    @classmethod
    def path_length(cls, path: Iterable[Vector3]) -> float:
        """Return total Euclidean length of a piecewise-linear 3D path."""
        arr = cls._to_array(path)
        segment_lengths = np.linalg.norm(np.diff(arr, axis=0), axis=1)
        return float(segment_lengths.sum())

    def compute_search_speed(self) -> float:
        """Return sweep speed that yields the same arrival time as the UAV path."""
        if self.uav_speed <= 0:
            raise ValueError("uav_speed must be positive")

        uav_len = self.path_length(self.uav_path)
        sweep_len = self.path_length(self.sweep_path)

        travel_time = uav_len / self.uav_speed
        if travel_time == 0:
            raise ValueError("uav_path has zero length; travel time is zero")

        return sweep_len / travel_time
