# -*- coding: utf-8 -*-
from pathlib import Path

path = Path('modules/monitoring_ver2/logic/monitoring_logic_part.py')
text = path.read_text(encoding='utf-8')
start_token = '\n    def _handle_all_input_missions_completed(self) -> None:\n'
start = text.find(start_token)
if start == -1:
    raise SystemExit('start token not found')
end_token = '\n    def _mark_individual_mission_done('
end = text.find(end_token, start)
if end == -1:
    raise SystemExit('end token not found')
new_block = '\n    def _handle_all_input_missions_completed(self) -> None:\n        if self._collab_completion_sent:\n            return\n        self._collab_completion_sent = True\n        self._monitoring_suspended = True\n        self._send_0503_notification("\uc804\uccb4 \ud611\uc5c5\uae30\uc800\uc784\ubb34 \uc644\ub8cc")\n\n'
text = text[:start] + new_block + text[end:]
path.write_text(text, encoding='utf-8')
