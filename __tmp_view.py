from pathlib import Path

text = Path('modules/monitoring_ver2/logic/monitoring_logic_part.py').read_text(encoding='utf-8')
start = text.index('_send_0503_notification("')
substr = text[start:start+40]
print([hex(ord(ch)) for ch in substr])
