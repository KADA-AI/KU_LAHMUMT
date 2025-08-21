# -*- coding: utf-8 -*-
# DS 런처: 루트에서 KU/Nex1 각각 실행 (절대경로 + cwd 고정)
import sys, subprocess
from pathlib import Path

if __name__ == "__main__":
    py = sys.executable
    ds_dir = Path(__file__).resolve().parent
    root = ds_dir.parent.parent  # .../ <project root>

    subprocess.Popen([py, str(ds_dir / "main_KU.py")], cwd=str(root))
    subprocess.Popen([py, str(ds_dir / "main_Nex1.py")], cwd=str(root))
