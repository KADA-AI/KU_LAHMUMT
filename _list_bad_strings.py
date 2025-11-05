import ast
import tokenize
import io
from pathlib import Path

text = Path(r"modules/monitoring_ver2/logic/replan_actual_logic.py").read_text(encoding="utf-8")
strings = set()
for tok in tokenize.generate_tokens(io.StringIO(text).readline):
    if tok.type == tokenize.STRING:
        try:
            val = ast.literal_eval(tok.string)
        except Exception:
            continue
        if isinstance(val, str) and '?' in val:
            strings.add(val)
for s in sorted(strings):
    print(s)
