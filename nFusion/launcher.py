# [launcher.py / module-level main] ─ 절대경로 호출
import sys, subprocess, os
from pathlib import Path

if __name__ == "__main__":
    py = sys.executable
    root = Path(__file__).resolve().parent
    subprocess.Popen([py, str(root / "main_KU.py")], cwd=str(root))
    subprocess.Popen([py, str(root / "main_nex1.py")], cwd=str(root))
