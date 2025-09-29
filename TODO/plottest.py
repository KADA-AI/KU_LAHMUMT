# 좌표(Line + Area)를 플롯하는 예제 코드 (matplotlib만 사용, 단일 플롯)
# - 위경도를 간단한 국지 좌표(평면)로 변환해 km 단위로 표시
# - 선형(Line) 미션과 영역(Area) 폴리곤을 함께 그립니다.
# - 색상은 지정하지 않고 기본값 사용(요청 지침 준수).

import json
import math
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import font_manager as fm
from matplotlib.ft2font import FT2Font

def _set_korean_font():
    # 시스템 글꼴에서 한글 지원 폰트 자동 탐색
    cands = []
    for path in fm.findSystemFonts(fontext="ttf"):
        try:
            mp = FT2Font(path).get_charmap()
            if ord('가') in mp and ord('한') in mp:
                name = fm.FontProperties(fname=path).get_name()
                cands.append((name, path))
        except Exception:
            pass
    prefer = ["Malgun Gothic","AppleGothic","NanumGothic","Nanum Gothic",
              "Noto Sans CJK KR","Noto Sans KR","Noto Sans CJK","Noto Serif CJK KR",
              "UnDotum","UnBatang","D2Coding","KoPubWorldDotum","Spoqa Han Sans","Pretendard"]
    for p in prefer:
        for name, path in cands:
            if name.lower() == p.lower():
                mpl.rcParams["font.family"] = name
                mpl.rcParams["axes.unicode_minus"] = False
                return name
    if cands:
        mpl.rcParams["font.family"] = cands[0][0]
        mpl.rcParams["axes.unicode_minus"] = False
        return cands[0][0]
    return None




# 원본 JSON 그대로 삽입
json_text = r'''
{
  "timestamp": 812101576638,
  "inputMissionPackageID": 1,
  "inputMissionPackageType": 1,
  "mainSensor": 1,
  "availableAircraftList": [
    {
      "aircraftID": 1
    },
    {
      "aircraftID": 2
    },
    {
      "aircraftID": 3
    },
    {
      "aircraftID": 4
    },
    {
      "aircraftID": 5
    },
    {
      "aircraftID": 6
    }
  ],
  "inputMissionList": [
    {
      "inputMissionID": 1,
      "inputMissionType": 1,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": [
          {
            "width": 1000,
            "coordinateList": [
              {
                "latitude": 37.634834,
                "longitude": 128.01256,
                "altitude": 0
              },
              {
                "latitude": 37.697285,
                "longitude": 128.04446,
                "altitude": 0
              }
            ]
          }
        ],
        "areaList": null
      }
    },
    {
      "inputMissionID": 2,
      "inputMissionType": 1,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": [
          {
            "width": 1000,
            "coordinateList": [
              {
                "latitude": 37.697285,
                "longitude": 128.04446,
                "altitude": 0
              },
              {
                "latitude": 37.784523,
                "longitude": 128.06102,
                "altitude": 0
              }
            ]
          }
        ],
        "areaList": null
      }
    },
    {
      "inputMissionID": 3,
      "inputMissionType": 1,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": [
          {
            "width": 1000,
            "coordinateList": [
              {
                "latitude": 37.784523,
                "longitude": 128.06102,
                "altitude": 0
              },
              {
                "latitude": 37.866108,
                "longitude": 128.1297,
                "altitude": 0
              }
            ]
          }
        ],
        "areaList": null
      }
    },
    {
      "inputMissionID": 4,
      "inputMissionType": 2,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": null,
        "areaList": [
          {
            "isHole": false,
            "coordinateList": [
              {
                "latitude": 37.875343,
                "longitude": 128.12488,
                "altitude": 0
              },
              {
                "latitude": 37.86613,
                "longitude": 128.12488,
                "altitude": 0
              },
              {
                "latitude": 37.86604,
                "longitude": 128.14285,
                "altitude": 0
              },
              {
                "latitude": 37.87652,
                "longitude": 128.14182,
                "altitude": 0
              }
            ]
          }
        ]
      }
    },
    {
      "inputMissionID": 5,
      "inputMissionType": 1,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": [
          {
            "width": 1000,
            "coordinateList": [
              {
                "latitude": 37.87627,
                "longitude": 128.13823,
                "altitude": 0
              },
              {
                "latitude": 37.925,
                "longitude": 128.17899,
                "altitude": 0
              }
            ]
          }
        ],
        "areaList": null
      }
    },
    {
      "inputMissionID": 6,
      "inputMissionType": 2,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": null,
        "areaList": [
          {
            "isHole": false,
            "coordinateList": [
              {
                "latitude": 38.003067,
                "longitude": 128.26549,
                "altitude": 0
              },
              {
                "latitude": 37.924572,
                "longitude": 128.26755,
                "altitude": 0
              },
              {
                "latitude": 37.925114,
                "longitude": 128.15562,
                "altitude": 0
              },
              {
                "latitude": 38.002525,
                "longitude": 128.15631,
                "altitude": 0
              },
              {
                "latitude": 38.003067,
                "longitude": 128.2133,
                "altitude": 0
              }
            ]
          }
        ]
      }
    },
    {
      "inputMissionID": 7,
      "inputMissionType": 1,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": [
          {
            "width": 1000,
            "coordinateList": [
              {
                "latitude": 37.924824,
                "longitude": 128.21518,
                "altitude": 0
              },
              {
                "latitude": 37.87871,
                "longitude": 128.21971,
                "altitude": 0
              }
            ]
          }
        ],
        "areaList": null
      }
    },
    {
      "inputMissionID": 8,
      "inputMissionType": 2,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": null,
        "areaList": [
          {
            "isHole": false,
            "coordinateList": [
              {
                "latitude": 37.87866,
                "longitude": 128.21211,
                "altitude": 0
              },
              {
                "latitude": 37.869015,
                "longitude": 128.21175,
                "altitude": 0
              },
              {
                "latitude": 37.869396,
                "longitude": 128.22893,
                "altitude": 0
              },
              {
                "latitude": 37.87877,
                "longitude": 128.22807,
                "altitude": 0
              }
            ]
          }
        ]
      }
    },
    {
      "inputMissionID": 9,
      "inputMissionType": 1,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": [
          {
            "width": 1000,
            "coordinateList": [
              {
                "latitude": 37.869144,
                "longitude": 128.21759,
                "altitude": 0
              },
              {
                "latitude": 37.790607,
                "longitude": 128.17487,
                "altitude": 0
              }
            ]
          }
        ],
        "areaList": null
      }
    },
    {
      "inputMissionID": 10,
      "inputMissionType": 1,
      "isDone": false,
      "missionDetail": {
        "coordinateList": null,
        "lineList": [
          {
            "width": 1000,
            "coordinateList": [
              {
                "latitude": 37.790607,
                "longitude": 128.17487,
                "altitude": 0
              },
              {
                "latitude": 37.713314,
                "longitude": 128.10002,
                "altitude": 0
              }
            ]
          }
        ],
        "areaList": null
      }
    }
  ]
}
'''
data = json.loads(json_text)

# ──────────────────────────────────────────────────────────────
# 추출: Line 미션과 Area 폴리곤
# ──────────────────────────────────────────────────────────────
lines = []   # [{id, coords=[(lat, lon, alt), ...]}]
areas = []   # [{id, rings=[[(lat, lon, alt), ...]]}]  # (isHole 처리 확장 대비)

for m in data["inputMissionList"]:
    mid = m["inputMissionID"]
    md = m["missionDetail"] or {}
    # Line
    if md.get("lineList"):
        for item in md["lineList"]:
            coords = [(c["latitude"], c["longitude"], c.get("altitude")) for c in item["coordinateList"]]
            lines.append({"id": mid, "coords": coords})
    # Area
    if md.get("areaList"):
        rings = []
        for poly in md["areaList"]:
            coords = [(c["latitude"], c["longitude"], c.get("altitude")) for c in poly["coordinateList"]]
            rings.append(coords)
        areas.append({"id": mid, "rings": rings})

# ──────────────────────────────────────────────────────────────
# 위경도를 국지 평면(km)으로 변환 (단순 equirectangular, 기준: 전체 평균 위경도)
# ──────────────────────────────────────────────────────────────
all_lat = []
all_lon = []
for L in lines:
    for la, lo, _ in L["coords"]:
        all_lat.append(la); all_lon.append(lo)
for A in areas:
    for ring in A["rings"]:
        for la, lo, _ in ring:
            all_lat.append(la); all_lon.append(lo)

lat0 = sum(all_lat) / len(all_lat)
lon0 = sum(all_lon) / len(all_lon)
lat0_rad = math.radians(lat0)
R_km = 6371.0

def ll_to_local_km(lat, lon):
    x = R_km * math.cos(lat0_rad) * math.radians(lon - lon0)  # East
    y = R_km * math.radians(lat - lat0)                       # North
    return x, y


_set_korean_font()  # ← 플롯 그리기 전에 한 번 호출

# ──────────────────────────────────────────────────────────────
# 플롯
# ──────────────────────────────────────────────────────────────
plt.figure(figsize=(8, 8))
ax = plt.gca()

# Line missions
for L in lines:
    xs, ys = [], []
    for la, lo, _ in L["coords"]:
        x, y = ll_to_local_km(la, lo)
        xs.append(x); ys.append(y)
    ax.plot(xs, ys, marker='o', linewidth=2)  # 색상 기본값 사용
    # 라벨(중간점 부근)
    xm = sum(xs) / len(xs)
    ym = sum(ys) / len(ys)
    ax.text(xm, ym, f"Line {L['id']}", fontsize=10)

# Area polygons
for A in areas:
    for ring in A["rings"]:
        xs, ys = [], []
        for la, lo, _ in ring:
            x, y = ll_to_local_km(la, lo)
            xs.append(x); ys.append(y)
        # 폴리곤 닫기
        xs.append(xs[0]); ys.append(ys[0])
        ax.fill(xs, ys, alpha=0.2)  # 색상 지정 없이 투명도만
        ax.plot(xs, ys, linewidth=2)
        # 라벨(대략 중심)
        cx = sum(xs[:-1]) / (len(xs)-1)
        cy = sum(ys[:-1]) / (len(ys)-1)
        ax.text(cx, cy, f"Area {A['id']}", fontsize=10)

ax.set_aspect('equal', adjustable='datalim')
ax.set_xlabel("East (km) — 기준 lon {:.5f}".format(lon0))
ax.set_ylabel("North (km) — 기준 lat {:.5f}".format(lat0))
ax.set_title("Missions: Lines + Areas (local planar view, km)")
ax.grid(True)

plt.show()
