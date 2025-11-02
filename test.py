import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[0]  # 코드 위치에 맞춰 조정
MODULES_DIR = ROOT / "modules"              # 최상위 폴더에 modules가 있다면 그대로 사용

if MODULES_DIR.exists():
    sys.path.insert(0, str(MODULES_DIR))
else:
    raise RuntimeError(f"modules 경로가 없습니다: {MODULES_DIR}")

import push
print("push 패키지 경로:", push.__file__)
