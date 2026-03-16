
import time
import os
import json
import math
import torch
import torch.nn.functional as F
import sys
import glob
from collections import deque
from pathlib import Path

# Adjust imports to use local dummy modules if src is missing
try:
    from src.models.models import BNNMainModel, LSTMAutoencoder
    from src.data.preprocess_mission import MissionPreprocessor
except ImportError:
    # Fallback to local stubs
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from modules.monitoring.models.dnn_models import BNNMainModel, LSTMAutoencoder
    from modules.monitoring.utils.preprocess import MissionPreprocessor


def _coerce_int(value):
    try:
        return int(value)
    except Exception:
        return None


def _coerce_float(value):
    try:
        result = float(value)
    except Exception:
        return None
    if not math.isfinite(result):
        return None
    return result


def _first_present(mapping, *keys):
    if not isinstance(mapping, dict):
        return None
    for key in keys:
        if key in mapping:
            return mapping.get(key)
    return None


def _extract_coordinate(container):
    if not isinstance(container, dict):
        return None
    candidates = [
        _first_present(container, "coordinate", "Coordinate", "coord", "Coord", "currentCoordinate", "CurrentCoordinate"),
        _first_present(container, "unmannedInfo", "UnmannedInfo"),
        container,
    ]
    for candidate in candidates:
        if not isinstance(candidate, dict):
            continue
        nested = _first_present(candidate, "coordinate", "Coordinate", "coord", "Coord")
        if isinstance(nested, dict):
            candidate = nested
        lat = _coerce_float(_first_present(candidate, "latitude", "Latitude", "lat", "Lat"))
        lon = _coerce_float(_first_present(candidate, "longitude", "Longitude", "lon", "Lon", "lng", "Lng"))
        alt = _coerce_float(_first_present(candidate, "altitude", "Altitude", "alt", "Alt"))
        if lat is None or lon is None:
            continue
        if alt is None:
            alt = 0.0
        return {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": float(alt),
        }
    return None


def _extract_velocity(container):
    if not isinstance(container, dict):
        return None
    candidate = (
        _first_present(container, "velocity", "Velocity", "speedInfo", "SpeedInfo")
        or container
    )
    if not isinstance(candidate, dict):
        return None
    speed = _coerce_float(_first_present(candidate, "speed", "Speed", "groundSpeed", "GroundSpeed"))
    heading = _coerce_float(_first_present(candidate, "heading", "Heading", "course", "Course"))
    if speed is None and heading is None:
        return None
    return {
        "speed": float(speed or 0.0),
        "heading": float(heading or 0.0),
    }


def _extract_aircraft_id(container):
    if not isinstance(container, dict):
        return None
    return _coerce_int(_first_present(container, "aircraftID", "AircraftID", "aircraftId", "AircraftId"))


def _extract_is_unmanned(container, aircraft_id):
    if isinstance(container, dict):
        value = _first_present(container, "isUnmanned", "IsUnmanned")
        if value is not None:
            try:
                if isinstance(value, str):
                    lowered = value.strip().lower()
                    if lowered in {"1", "true", "yes", "y", "on"}:
                        return 1
                    if lowered in {"0", "false", "no", "n", "off"}:
                        return 0
                return 1 if int(value) != 0 else 0
            except Exception:
                pass
    try:
        return 1 if int(aircraft_id or 0) >= 4 else 0
    except Exception:
        return 0


def _extract_agent_list(raw_data):
    if not isinstance(raw_data, dict):
        return []
    if isinstance(raw_data.get("raw"), dict):
        raw_data = raw_data.get("raw") or raw_data
    raw_list = raw_data.get("agentStateList") or raw_data.get("AgentStateList")
    if not isinstance(raw_list, list):
        raw_list = raw_data.get("uavStates") or raw_data.get("UavStates") or []
    if not isinstance(raw_list, list):
        raw_list = raw_data.get("agent_states") or raw_data.get("agentStates") or []
    normalized = []
    for item in raw_list:
        if not isinstance(item, dict):
            continue
        aircraft_id = _extract_aircraft_id(item)
        coord = _extract_coordinate(item)
        vel = _extract_velocity(item)
        normalized.append(
            {
                "aircraftID": aircraft_id if aircraft_id is not None else 0,
                "coordinate": coord,
                "velocity": vel,
                "isUnmanned": _extract_is_unmanned(item, aircraft_id),
            }
        )
    normalized.sort(key=lambda x: x.get("aircraftID", 0))
    return normalized


def _resolve_base_coord(raw_data):
    agents = _extract_agent_list(raw_data)
    preferred = [a for a in agents if a.get("aircraftID") == 1]
    for agent in preferred + agents:
        coord = _extract_coordinate(agent)
        if coord is not None:
            return coord
    return None

def evaluate_risk_thresholds(mean_risk: list[float], std_risk: list[float]) -> list[int]:
    """
    Evaluate risk based on 12 model outputs (6 means, 6 stds).
    Returns a list of INDICES (0-5) for agents that exceed the risk threshold.
    Thresholds can be adjusted here.
    """
    if not mean_risk:
        return []

    risky_agents = []
    
    # Check each agent (0 to 5)
    for i in range(len(mean_risk)):
        # Example Threshold: Risk score > 0.5
        if mean_risk[i] > 0.8:
            risky_agents.append(i)
            
        # You can add uncertainty (std) checks here too
        # if std_risk[i] > 0.8:
        #    risky_agents.append(i)

    return sorted(list(set(risky_agents)))

class RealTimeInferrer:
    def __init__(self, context_path, model_path, agent_file_dummy=None, mission_file=None, input_dim=724, hidden_dim=128, output_dim=6):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.context_path = context_path
        self.model_path = model_path
        self.mission_file = mission_file
        self.mission_context = None
        self._target_vectorizer = None
        
        # Track file modification for reloading
        self.last_mission_mtime = 0
        
        # Load Model
        self._load_model(input_dim, hidden_dim, output_dim)
        
        # Load Target Vectorizer (Dummy/Optional)
        self.target_encoder = None 
        # (Skipping detailed vectorizer load for simplicity unless needed, using logic from user code if present)
        
        # Buffer State
        self.buffer = deque(maxlen=200) 
        self.base_coord = None 
        
        # Buffer params
        self.req_indices = [0, 25, 50, 75, 100]
        self.min_buffer_size = 101
        
        # Initial Mission Load
        if self.mission_file:
            self._check_and_load_mission_context()

    def _load_model(self, input_dim, hidden_dim, output_dim):
        # Ensure directory exists for stubs
        os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
        
        if not os.path.exists(self.model_path):
            print(f"Warning: Model not found at {self.model_path}. Creating output directory only.")
            # In a real scenario we'd crash, but for now we might need to mock the model itself if file missing
            # For this implementation, let's assume we initialize a fresh model if missing (for testing)
            self.model = BNNMainModel(input_dim, hidden_dim, output_dim).to(self.device)
        else:
            checkpoint = torch.load(self.model_path, map_location=self.device)
            if isinstance(checkpoint, dict) and 'config' in checkpoint:
                config = checkpoint['config']
                hidden_dim = config['hidden_dim']
                num_layers = config.get('num_layers', 2)
                input_dim = config.get('input_dim', input_dim)
                self.model = BNNMainModel(input_dim, hidden_dim, output_dim, num_layers=num_layers).to(self.device)
                state = checkpoint.get('model_state_dict', {})
                try:
                    missing, unexpected = self.model.load_state_dict(state, strict=False)
                    if missing or unexpected:
                        print(
                            "[RealTimeInferrer] Warning: checkpoint mismatch. "
                            f"missing={len(missing)}, unexpected={len(unexpected)}"
                        )
                except Exception as e:
                    print(f"[RealTimeInferrer] Warning: failed to load checkpoint, using fresh model. ({e})")
            else:
                self.model = BNNMainModel(input_dim, hidden_dim, output_dim).to(self.device)
                try:
                    missing, unexpected = self.model.load_state_dict(checkpoint, strict=False)
                    if missing or unexpected:
                        print(
                            "[RealTimeInferrer] Warning: checkpoint mismatch. "
                            f"missing={len(missing)}, unexpected={len(unexpected)}"
                        )
                except Exception as e:
                    print(f"[RealTimeInferrer] Warning: failed to load checkpoint, using fresh model. ({e})")
        
        self.model.eval()

    def _check_and_load_mission_context(self):
        """Checks if mission file has changed and reloads context if needed."""
        if not self.mission_file:
            raise RuntimeError("Mission file is not set for DL inference.")

        if self.mission_file and os.path.exists(self.mission_file):
            current_mtime = os.path.getmtime(self.mission_file)
            if current_mtime > self.last_mission_mtime:
                print(f"[RealTimeInferrer] Loading/Reloading Mission Plan: {self.mission_file}")
                
                # Check for vectorizer
                ckpt_dir = os.path.dirname(self.model_path)
                vec_path = os.path.join(ckpt_dir, "vectorizer.pth")
                
                # Use stub if file missing
                mp_processor = MissionPreprocessor(os.path.dirname(os.path.dirname(self.mission_file)))
                
                self.mission_context = mp_processor.compute_embedding(self.mission_file, vec_path, self.device)
                self.last_mission_mtime = current_mtime
                print(f"Updated Mission Context: {self.mission_context.shape}")
        else:
            raise FileNotFoundError(f"Mission file not found: {self.mission_file}")

    def extract_features(self, raw_data, base_coord):
        base = _extract_coordinate(base_coord)
        if base is None:
            raise KeyError("latitude")
        base_lat = base["latitude"]
        base_lon = base["longitude"]
        base_alt = base["altitude"]
        
        # Agent Features
        agents = _extract_agent_list(raw_data)
        row_feats = []
        
        processed_agents = 0
        for ag in agents:
            if processed_agents >= 6:
                break
            coord = _extract_coordinate(ag)
            vel = _extract_velocity(ag)
            if coord is None or vel is None:
                continue
            rel_lat = coord["latitude"] - base_lat
            rel_lon = coord["longitude"] - base_lon
            rel_alt = coord["altitude"] - base_alt
            
            agent_type = ag.get("isUnmanned", 0)
            
            row_feats.extend([rel_lat, rel_lon, rel_alt, vel["speed"], vel["heading"], agent_type])
            processed_agents += 1
            
        while processed_agents < 6:
            row_feats.extend([0, 0, 0, 0, 0, 0])
            processed_agents += 1

        # Target Embedding (from 0402 -> targetInfo.json)
        target_embedding = self._compute_target_embedding(base_coord)
        
        row_feats_tensor = torch.tensor(row_feats, dtype=torch.float32).to(self.device) 
        final_feats = torch.cat([row_feats_tensor, target_embedding], dim=0) 
        
        return final_feats.tolist()

    def _ensure_target_vectorizer(self):
        if self._target_vectorizer is not None:
            return
        ckpt_dir = os.path.dirname(self.model_path)
        vec_path = os.path.join(ckpt_dir, "target_vectorizer.pth")
        if not os.path.exists(vec_path):
            raise FileNotFoundError(f"Target vectorizer not found: {vec_path}")
        if LSTMAutoencoder is None:
            raise ImportError("LSTMAutoencoder unavailable for target embedding.")
        model = LSTMAutoencoder(input_dim=4, hidden_dim=32).to(self.device)
        state = torch.load(vec_path, map_location=self.device)
        model.load_state_dict(state, strict=True)
        model.eval()
        self._target_vectorizer = model

    def _load_target_sequence(self, base_coord):
        from modules.monitoring.logic import target_info

        info = target_info.load_target_info()
        target_map = info.get("targetList") if isinstance(info, dict) else None
        if not isinstance(target_map, dict):
            return None

        rows = []
        for entry in target_map.values():
            if not isinstance(entry, dict):
                continue
            if entry.get("isDestroyed"):
                continue
            is_used = entry.get("isUsed")
            if is_used is not None and int(is_used) != 0:
                continue
            is_ignored = entry.get("isIgnored")
            if is_ignored is not None and int(is_ignored) != 0:
                continue
            coord = entry.get("coordinate") or {}
            if not isinstance(coord, dict):
                continue
            lat = coord.get("latitude")
            lon = coord.get("longitude")
            alt = coord.get("altitude")
            if lat is None or lon is None or alt is None:
                continue
            target_type = entry.get("targetType")
            try:
                ttype = float(target_type) if target_type is not None else 0.0
            except Exception:
                ttype = 0.0
            rows.append([
                float(lat) - float(base_coord["latitude"]),
                float(lon) - float(base_coord["longitude"]),
                float(alt) - float(base_coord["altitude"]),
                ttype,
            ])

        if not rows:
            return None

        rows.sort(key=lambda r: (r[3], r[0], r[1], r[2]))
        return torch.tensor(rows, dtype=torch.float32, device=self.device)

    def _compute_target_embedding(self, base_coord):
        seq = self._load_target_sequence(base_coord)
        if seq is None:
            return torch.zeros(32, device=self.device)
        self._ensure_target_vectorizer()
        seq = seq.unsqueeze(0)  # (1, L, 4)
        lengths = torch.tensor([seq.size(1)], device="cpu")
        with torch.no_grad():
            _, emb = self._target_vectorizer(seq, lengths)
        return emb.squeeze(0)

    def infer_one_step(self, agent_data):
        """
        Process a single step of agent data (0401 dict).
        Returns: (mean_risks, std_risks) or None
        """
        # 0. Check Mission Updates
        self._check_and_load_mission_context()
        
        # 1. Base Coord
        base_coord = _extract_coordinate(self.base_coord)
        if base_coord is None:
            base_coord = _resolve_base_coord(agent_data)
            if base_coord is None:
                return None, None
            self.base_coord = base_coord
        else:
            self.base_coord = base_coord

        # 2. Extract
        feats = self.extract_features(agent_data, self.base_coord)
        self.buffer.append(feats)
        
        # 3. Infer
        if len(self.buffer) >= self.min_buffer_size:
            window_indices = [-101, -76, -51, -26, -1]
            window_data = [self.buffer[idx] for idx in window_indices]
            
            win_tensor = torch.tensor(window_data, dtype=torch.float32).unsqueeze(0).to(self.device)
            flat_win = win_tensor.view(1, -1)
            
            if self.mission_context is None:
                return None, None
            mission_in = self.mission_context.unsqueeze(0)
            final_input = torch.cat([mission_in, flat_win], dim=1)
            
            with torch.no_grad():
                mean_risk, std_risk = self.model.mc_forward(final_input, num_samples=30) # Reduced samples for speed
                
            return mean_risk[0].tolist(), std_risk[0].tolist()
            
        return None, None
