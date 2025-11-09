import re
from pathlib import Path
path = Path('modules/mission_planning/MissionPlanner/main_MP.py')
text = path.read_text(encoding='utf-8')
pattern = r"    def _write_map_html\(self\):[\s\S]*?        # \xec\a0\84\xec\9d 0201 Input Mission \(CMPK\)"
new_func = '''    def _write_map_html(self):
        bounds_coords: list[tuple[float, float]] = []

        def _push_coord(lat, lon):
            if lat is None or lon is None:
                return
            try:
                bounds_coords.append((float(lat), float(lon)))
            except (TypeError, ValueError):
                pass

        for miss in self.missions:
            info = miss.get("individualMissionInfo") or {}
            for coord in info.get("coordinateList") or []:
                _push_coord(coord.get("latitude"), coord.get("longitude"))
            for blk in info.get("lineList") or []:
                for coord in blk.get("coordinateList") or []:
                    _push_coord(coord.get("latitude"), coord.get("longitude"))
            for blk in info.get("areaList") or []:
                for coord in blk.get("coordinateList") or []:
                    _push_coord(coord.get("latitude"), coord.get("longitude"))

        if bounds_coords:
            avg_lat = sum(lat for lat, _ in bounds_coords) / len(bounds_coords)
            avg_lon = sum(lon for _, lon in bounds_coords) / len(bounds_coords)
            center = [avg_lat, avg_lon]
            zoom_start = 12
        else:
            center = [38.128774, 127.318005]
            zoom_start = 14

        fmap = folium.Map(location=center, zoom_start=zoom_start)

        _js_links = []
        hover_specs = []

        # 
        '''
match = re.search(pattern, text)
print(bool(match))
