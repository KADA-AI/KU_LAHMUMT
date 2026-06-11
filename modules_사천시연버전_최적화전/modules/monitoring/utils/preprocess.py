import json
import os
from typing import Iterable

import torch

try:
    from modules.monitoring.models.dnn_models import LSTMAutoencoder
except Exception:
    LSTMAutoencoder = None


def _torch_load_weights(path, *, map_location):
    try:
        return torch.load(path, map_location=map_location, weights_only=True)
    except TypeError:
        return torch.load(path, map_location=map_location)
    except Exception:
        return torch.load(path, map_location=map_location, weights_only=False)


class MissionPreprocessor:
    def __init__(self, database_dir: str):
        self.database_dir = database_dir
        self.feature_dim = 3  # (Rel Lat, Rel Lon, Rel Alt)

    def load_json(self, path: str):
        if not os.path.exists(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None

    def get_flight_path_coords(self, data_root: str, path_id: int | str):
        """Load a FlightPath file and return absolute coordinate triples."""
        fp_path = os.path.join(data_root, "FlightPath", f"{path_id}.json")
        data = self.load_json(fp_path)
        if not data:
            return []

        wts = data.get("waypointList") or data.get("lahWaypointList") or []
        coords: list[list[float]] = []
        for wt in wts:
            c = wt.get("coordinate", {}) if isinstance(wt, dict) else {}
            lat = c.get("latitude")
            lon = c.get("longitude")
            alt = c.get("altitude")
            if lat is not None and lon is not None and alt is not None:
                coords.append([lat, lon, alt])
        return coords

    def _parse_new_format(self, data: dict):
        """Parse InputMissionPlan-like format: inputMissionList -> missionDetail -> lineList."""
        input_missions = data.get("inputMissionList", [])
        sequences: list[torch.Tensor] = []

        all_paths: list[list[list[float]]] = []
        for mission in input_missions:
            if not isinstance(mission, dict):
                continue
            detail = mission.get("missionDetail", {}) or {}
            line_list = detail.get("lineList") or []
            for line in line_list:
                if not isinstance(line, dict):
                    continue
                coords = line.get("coordinateList") or []
                if not coords:
                    continue
                path = [
                    [c["latitude"], c["longitude"], c["altitude"]]
                    for c in coords
                    if isinstance(c, dict) and "latitude" in c and "longitude" in c and "altitude" in c
                ]
                if path:
                    all_paths.append(path)

        for path in all_paths:
            base_lat, base_lon, base_alt = path[0]
            rel_coords = [
                [lat - base_lat, lon - base_lon, alt - base_alt] for lat, lon, alt in path
            ]
            sequences.append(torch.tensor(rel_coords, dtype=torch.float32))
        return sequences

    def extract_sequence(self, mp_file: str):
        """Extract normalized coordinate sequences from a MissionPlan file."""
        mp_data = self.load_json(mp_file)
        if not mp_data:
            return None

        if "inputMissionList" in mp_data:
            return self._parse_new_format(mp_data)

        mp_dir = os.path.dirname(mp_file)
        data_root = os.path.dirname(mp_dir)
        aircraft_list = mp_data.get("aircraftList", []) or []

        sequences: list[torch.Tensor] = []
        for ac in aircraft_list:
            if not isinstance(ac, dict):
                continue
            imp_id = ac.get("individualMissionPackageID")
            if imp_id is None:
                continue
            imp_path = os.path.join(data_root, "IndividualMissionPlan", f"{imp_id}.json")
            imp_data = self.load_json(imp_path)
            if not imp_data:
                continue

            full_path_coords: list[list[float]] = []
            indiv_missions = imp_data.get("individualMissionList", []) or []
            for mission in indiv_missions:
                if not isinstance(mission, dict):
                    continue
                path_id = mission.get("pathID")
                if path_id:
                    full_path_coords.extend(self.get_flight_path_coords(data_root, path_id))

            if not full_path_coords:
                continue

            base_lat, base_lon, base_alt = full_path_coords[0]
            rel_coords = [
                [lat - base_lat, lon - base_lon, alt - base_alt]
                for lat, lon, alt in full_path_coords
            ]
            sequences.append(torch.tensor(rel_coords, dtype=torch.float32))

        return sequences

    def _sample_sequence(self, seq: torch.Tensor, samples: int = 21) -> torch.Tensor:
        """Sample a sequence to a fixed number of points (no interpolation)."""
        if seq.numel() == 0:
            return torch.zeros((samples, 3), dtype=torch.float32)
        length = seq.size(0)
        if length == 1:
            return seq.repeat(samples, 1)
        indices = torch.linspace(0, length - 1, steps=samples).round().long()
        return seq[indices]

    def _fallback_embedding(self, sequences: Iterable[torch.Tensor], device: str):
        """Fallback embedding derived from mission paths (no learned vectorizer)."""
        mission_embs: list[torch.Tensor] = []
        for seq in sequences:
            seq = seq.to(torch.float32)
            sampled = self._sample_sequence(seq, samples=21)  # 21 * 3 = 63
            flat = sampled.reshape(-1)
            length_feat = torch.tensor([min(seq.size(0), 1000) / 1000.0], dtype=torch.float32)
            emb = torch.cat([flat, length_feat], dim=0)  # 64
            mission_embs.append(emb.to(device))

        if not mission_embs:
            raise ValueError("No valid paths found in mission plan for embedding.")

        while len(mission_embs) < 6:
            mission_embs.append(mission_embs[-1].clone())
        return torch.cat(mission_embs[:6], dim=0)

    def compute_embedding(self, mp_file: str, vectorizer_path: str, device: str = "cpu"):
        """Compute 384-dim mission context embedding from MissionPlan."""
        if not mp_file or not os.path.exists(mp_file):
            raise FileNotFoundError(f"MissionPlan file not found: {mp_file}")
        sequences = self.extract_sequence(mp_file)
        if not sequences:
            raise ValueError("No valid paths found in mission plan.")

        if LSTMAutoencoder is None:
            raise ImportError("LSTMAutoencoder unavailable for mission embedding.")
        if not vectorizer_path or not os.path.exists(vectorizer_path):
            raise FileNotFoundError(f"Vectorizer not found: {vectorizer_path}")

        model = LSTMAutoencoder(input_dim=3, hidden_dim=64).to(device)
        state = _torch_load_weights(vectorizer_path, map_location=device)
        model.load_state_dict(state, strict=True)
        model.eval()

        mission_embs: list[torch.Tensor] = []
        with torch.no_grad():
            for seq in sequences:
                seq = seq.unsqueeze(0).to(device)
                lengths = torch.tensor([seq.size(1)], device="cpu")
                _, emb = model(seq, lengths)
                mission_embs.append(emb.squeeze(0))

        while len(mission_embs) < 6:
            mission_embs.append(mission_embs[-1].clone())
        return torch.cat(mission_embs[:6], dim=0)
