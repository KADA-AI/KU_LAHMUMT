#!/usr/bin/env python3
"""Log Analyzer GUI for Scenario Logs."""

from __future__ import annotations

import json
import shutil
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
import time
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Iterable, List, Optional, Sequence

APP_TITLE = "#2 Log Analyzer"
BASE_DIR = Path(__file__).resolve().parent
MEMO_DIR = BASE_DIR / "memos"
CACHE_DIR = BASE_DIR / ".cache"
CURRENT_SCENARIO_PATH = BASE_DIR.parent / "current_scenario.json"


# NOTE: 51311과 51331은 중복 요청으로 일단 협업기저임무 계열 명칭으로 정리했습니다.
MESSAGE_NAME_MAP = {
    "0101": "시스템 운용모드",
    "0102": "모듈상태정보",
    "0103": "SW상태정보",
    "53100": "SW상태정보",
    "0201": "협업기저임무",
    "51311": "협업기저임무",
    "0203": "비행참조정보",
    "0305": "재계획수행상태정보",
    "53110": "재계획수행상태정보",
    "0301": "임무 계획",
    "53111": "임무 계획",
    "53112": "임무 계획",
    "0903": "수행임무 갱신 요청",
    "53114": "수행임무 갱신 요청",
    "0402": "전장상황인지정보",
    "52310": "전장상황인지정보",
    "51320": "유인기 상태정보",
    "51321": "무인기 상태정보",
    "51323": "고장상태 정보",
    "0401": "유무인기 상태정보",
    "0501": "임무수행 상태정보",
    "53130": "임무수행 상태정보",
    "0602": "무인기 통제명령",
    "53120": "무인기 통제명령",
    "0601": "기저행위",
    "0503": "협업기저임무완료알람",
    "53115": "협업기저임무완료알람",
    "0803": "다음협업기저임무명령",
    "51331": "다음협업기저임무명령",
    "0901": "옵션정보 생성요청",
    "0701": "옵션정보",
    "53113": "옵션정보",
    "0702": "의사결정 결과 전달",
    "0504": "연료량 부족",
    "53140": "연료량 부족",
    "0802": "강제명령",
    "51332": "강제명령",
    "0202": "선행임무정보",
    "51333": "선행임무정보",
}

TABLE_COL_WIDTHS = {
    "id": 80,
    "name": 110,
    "time": 90,
    "file": 70,
    "action": 60,
}


@dataclass
class MessageEntry:
    message_id: str
    message_name: str
    directory: Path
    files: List[Path]
    latest_update: Optional[datetime]
    category: str  # "internal" or "external"


def ensure_workspace() -> None:
    MEMO_DIR.mkdir(parents=True, exist_ok=True)
    CACHE_DIR.mkdir(parents=True, exist_ok=True)


def load_current_scenario_meta() -> dict:
    try:
        with CURRENT_SCENARIO_PATH.open("r", encoding="utf-8") as fp:
            return json.load(fp)
    except Exception:
        return {}


def get_default_base_root() -> Optional[str]:
    meta = load_current_scenario_meta()
    return meta.get("base_root")


def get_default_agency() -> str:
    meta = load_current_scenario_meta()
    return meta.get("agency", "SBC3")


def looks_like_message_dir(name: str) -> bool:
    if not name:
        return False
    if name.isdigit():
        return True
    # 영어로 된 폴더도 메시지를 담음
    return name[0].isalpha()


def resolve_message_root(path: Path) -> Path:
    """Find the directory that directly holds the message folders."""
    if not path.exists() or not path.is_dir():
        raise FileNotFoundError(f"{path} is not a directory.")

    def has_message_children(folder: Path) -> bool:
        for child in folder.iterdir():
            if not child.is_dir():
                continue
            try:
                grand_children = list(child.iterdir())
            except PermissionError:
                continue
            for grand_child in grand_children:
                if grand_child.is_file() and grand_child.suffix.lower() == ".json":
                    return True
        return False

    candidates = [path] + [child for child in path.iterdir() if child.is_dir()]
    for candidate in candidates:
        if has_message_children(candidate):
            return candidate
    raise FileNotFoundError(
        f"No message directories (0101~, 51300~, etc.) found under {path}"
    )


def load_message_entries(folder: Path) -> List[MessageEntry]:
    entries: List[MessageEntry] = []
    for child in sorted(folder.iterdir(), key=lambda p: p.name):
        if not child.is_dir():
            continue
        name = child.name
        if not looks_like_message_dir(name):
            continue

        files = sorted(
            [p for p in child.iterdir() if p.is_file() and p.suffix.lower() == ".json"],
            key=lambda p: p.name,
        )
        latest = None
        if files:
            latest = datetime.fromtimestamp(max(p.stat().st_mtime for p in files))

        if name.isdigit():
            numeric_id = int(name)
            category = "internal" if numeric_id < 51300 else "external"
        else:
            category = "internal"

        display_name = MESSAGE_NAME_MAP.get(name)
        if not display_name:
            display_name = name.replace("_", " ")

        entries.append(
            MessageEntry(
                message_id=name,
                message_name=display_name,
                directory=child,
                files=files,
                latest_update=latest,
                category=category,
            )
        )
    return entries


def copy_to_cache(message_id: str, file_path: Path) -> Path:
    """Copy the log file into the analyzer cache to avoid locking live logs."""
    dest_dir = CACHE_DIR / message_id
    dest_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    dest = dest_dir / f"{timestamp}_{file_path.name}"
    shutil.copy2(file_path, dest)
    return dest


def read_text(path: Path) -> str:
    if not path.exists():
        return ""
    try:
        return path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return path.read_text(encoding="cp949", errors="replace")


class ScrollableSection(ttk.Frame):
    def __init__(self, master: tk.Misc, title: str):
        super().__init__(master)
        self.title = ttk.Label(self, text=title, font=("Arial", 12, "bold"))
        self.title.pack(anchor="w", pady=(0, 4))

        header = ttk.Frame(self)
        header.pack(fill="x", padx=2)
        headings = [
            ("메시지 ID", "id"),
            ("메시지 명", "name"),
            ("최근 업데이트", "time"),
            ("파일명", "file"),
            ("", "action"),
        ]
        for idx, (text, key) in enumerate(headings):
            lbl = ttk.Label(header, text=text, anchor="center")
            lbl.grid(row=0, column=idx, sticky="ew", padx=2)
            header.columnconfigure(idx, weight=0, minsize=TABLE_COL_WIDTHS[key])

        self.canvas = tk.Canvas(self, height=380, borderwidth=0, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner_frame = ttk.Frame(self.canvas)
        self.inner_frame.bind("<Configure>", self._on_inner_configure)
        self.window_id = self.canvas.create_window((0, 0), window=self.inner_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.canvas.bind("<Enter>", self._bind_mousewheel)
        self.canvas.bind("<Leave>", self._unbind_mousewheel)
        self.inner_frame.bind("<Enter>", self._bind_mousewheel)
        self.inner_frame.bind("<Leave>", self._unbind_mousewheel)

        self.rows: List[MessageRow] = []

    def populate(self, entries: Sequence[MessageEntry], open_callback):
        for row in self.rows:
            row.destroy()
        self.rows.clear()
        for entry in entries:
            row = MessageRow(self.inner_frame, entry, open_callback)
            row.pack(fill="x", padx=2, pady=1)
            row.bind_mousewheel(self._on_mousewheel)
            self.rows.append(row)

    def _on_canvas_configure(self, event):
        self.canvas.itemconfigure(self.window_id, width=event.width)

    def _on_inner_configure(self, event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_mousewheel(self, event):
        if event.delta:
            self.canvas.yview_scroll(int(-event.delta / 120), "units")
        elif event.num == 4:
            self.canvas.yview_scroll(-3, "units")
        elif event.num == 5:
            self.canvas.yview_scroll(3, "units")

    def _bind_mousewheel(self, _event=None):
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.canvas.bind_all("<Button-5>", self._on_mousewheel)

    def _unbind_mousewheel(self, _event=None):
        self.canvas.unbind_all("<MouseWheel>")
        self.canvas.unbind_all("<Button-4>")
        self.canvas.unbind_all("<Button-5>")


class MessageRow(ttk.Frame):
    def __init__(self, master: tk.Misc, entry: MessageEntry, open_callback):
        super().__init__(master, relief="groove", padding=2)
        self.entry = entry
        self.open_callback = open_callback
        self.file_var = tk.StringVar()

        self.columnconfigure(0, weight=0, minsize=TABLE_COL_WIDTHS["id"])
        self.columnconfigure(1, weight=0, minsize=TABLE_COL_WIDTHS["name"])
        self.columnconfigure(2, weight=0, minsize=TABLE_COL_WIDTHS["time"])
        self.columnconfigure(3, weight=0, minsize=TABLE_COL_WIDTHS["file"])
        self.columnconfigure(4, weight=0, minsize=TABLE_COL_WIDTHS["action"])

        ttk.Label(self, text=entry.message_id, width=int(TABLE_COL_WIDTHS["id"] / 8)).grid(row=0, column=0, sticky="w", padx=2)
        ttk.Label(self, text=entry.message_name, width=int(TABLE_COL_WIDTHS["name"] / 8)).grid(row=0, column=1, sticky="w", padx=2)
        ttk.Label(
            self,
            text=entry.latest_update.strftime("%H:%M:%S")
            if entry.latest_update
            else "-",
            width=int(TABLE_COL_WIDTHS["time"] / 8),
        ).grid(row=0, column=2, sticky="w", padx=2)

        values = [p.name for p in entry.files]
        state = "readonly" if values else "disabled"
        default_value = values[0] if values else "파일 없음"
        self.file_var.set(default_value)

        self.combo = ttk.Combobox(
            self,
            textvariable=self.file_var,
            values=values,
            state=state,
            width=8,
        )
        self.combo.grid(row=0, column=3, sticky="w", padx=2)

        self.open_button = ttk.Button(
            self,
            text="열기",
            command=self.open_selected,
            state="normal" if values else "disabled",
            width=8,
        )
        self.open_button.grid(row=0, column=4, padx=(2, 4), sticky="e")

    def bind_mousewheel(self, handler):
        for widget in (self, self.combo, self.open_button):
            widget.bind("<MouseWheel>", handler)
            widget.bind("<Button-4>", handler)
            widget.bind("<Button-5>", handler)

    def open_selected(self):
        if not self.entry.files:
            return
        filename = self.file_var.get()
        file_path = next((p for p in self.entry.files if p.name == filename), None)
        if not file_path:
            messagebox.showerror("파일 선택", "선택된 파일을 찾을 수 없습니다.")
            return
        self.open_callback(self.entry, file_path)


class DetailWindow(tk.Toplevel):
    def __init__(self, parent, entry: MessageEntry, source_file: Path, cached_file: Path):
        super().__init__(parent)
        self.title(f"{entry.message_id} : {source_file.name}")
        self.geometry("700x500")
        self.transient(parent)
        self.withdraw()

        top_label = ttk.Label(
            self,
            text=f"{entry.message_id}  |  {entry.message_name}  |  {source_file.name}",
            font=("Arial", 12, "bold"),
        )
        top_label.pack(fill="x", padx=8, pady=4)

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=8, pady=4)

        paned = ttk.Panedwindow(body, orient="horizontal")
        paned.pack(fill="both", expand=True)

        memo_frame = ttk.Frame(paned)
        memo_frame.config(width=260)
        paned.add(memo_frame, weight=3)
        ttk.Label(memo_frame, text="메모").pack(anchor="w")
        self.memo_text = tk.Text(memo_frame, wrap="word")
        self.memo_text.pack(fill="both", expand=True)

        memo_btn = ttk.Button(memo_frame, text="Save", command=lambda: self.save_memo(entry))
        memo_btn.pack(pady=4)

        view_frame = ttk.Frame(paned)
        view_frame.config(width=480)
        paned.add(view_frame, weight=7)
        search_row = ttk.Frame(view_frame)
        search_row.pack(fill="x", pady=(0, 2))
        ttk.Label(search_row, text="검색").pack(side="left")
        self.search_var = tk.StringVar()
        search_entry = ttk.Entry(search_row, textvariable=self.search_var)
        search_entry.pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(search_row, text="▲", width=3, command=lambda: self.search_text(backwards=True)).pack(
            side="left", padx=2
        )
        ttk.Button(search_row, text="▼", width=3, command=lambda: self.search_text(backwards=False)).pack(
            side="left"
        )

        text_container = ttk.Frame(view_frame)
        text_container.pack(fill="both", expand=True)
        self.json_text = tk.Text(text_container, wrap="none")
        self.json_text.pack(side="left", fill="both", expand=True)
        y_scroll = ttk.Scrollbar(text_container, orient="vertical", command=self.json_text.yview)
        self.json_text.configure(yscrollcommand=y_scroll.set)
        y_scroll.pack(side="right", fill="y")

        memo_text = read_text(memo_path_for(entry.message_id))
        self.memo_text.insert("1.0", memo_text)

        json_content = read_text(cached_file)
        self.json_text.insert("1.0", json_content)
        self.json_text.configure(state="disabled")

        self.cached_file = cached_file
        # Delay layout tweaks until Tk finishes rendering to avoid zero-width panes.
        self.after_idle(lambda: self._finalize_open(paned))

    def _finalize_open(self, paned: ttk.Panedwindow):
        self.deiconify()
        self.update_idletasks()
        self._set_initial_ratio(paned, 0.3)
        self._move_near_cursor()
        self.lift()
        self.focus_force()

    def save_memo(self, entry: MessageEntry):
        memo_path = memo_path_for(entry.message_id)
        memo_path.parent.mkdir(parents=True, exist_ok=True)
        memo_path.write_text(self.memo_text.get("1.0", "end").rstrip() + "\n", encoding="utf-8")
        messagebox.showinfo("메모 저장", f"{memo_path.name} 저장 완료")

    def search_text(self, backwards: bool = False):
        needle = self.search_var.get()
        if not needle.strip():
            return
        text_widget = self.json_text
        was_disabled = str(text_widget.cget("state")) == "disabled"
        if was_disabled:
            text_widget.configure(state="normal")
        text_widget.tag_remove("search_match", "1.0", "end")
        current_index = text_widget.index("insert")
        if backwards:
            start_index = text_widget.index(f"{current_index} -1c")
            pos = text_widget.search(needle, start_index, "1.0", backwards=True, nocase=True)
            if not pos:
                pos = text_widget.search(needle, "end", "1.0", backwards=True, nocase=True)
        else:
            pos = text_widget.search(needle, f"{current_index} +1c", "end", nocase=True)
            if not pos:
                pos = text_widget.search(needle, "1.0", "end", nocase=True)
        if pos:
            end = f"{pos}+{len(needle)}c"
            text_widget.tag_add("search_match", pos, end)
            text_widget.tag_config("search_match", background="yellow")
            target_index = pos if backwards else end
            text_widget.mark_set("insert", target_index)
            text_widget.see(pos)
        if was_disabled:
            text_widget.configure(state="disabled")

    def _set_initial_ratio(self, paned, ratio: float):
        total = paned.winfo_width()
        if total <= 1:
            # Wait until Tk finishes mapping the widget; width stays at 1px while hidden.
            self.after(30, lambda: self._set_initial_ratio(paned, ratio))
            return
        paned.sashpos(0, max(1, int(total * ratio)))

    def _move_near_cursor(self):
        self.update_idletasks()
        width = max(self.winfo_width(), 1)
        height = max(self.winfo_height(), 1)
        pointer_x = self.winfo_pointerx()
        pointer_y = self.winfo_pointery()
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        x = max(min(pointer_x - width // 2, screen_w - width), 0)
        y = max(min(pointer_y - height // 2, screen_h - height), 0)
        self.geometry(f"+{x}+{y}")


def memo_path_for(message_id: str) -> Path:
    return MEMO_DIR / f"{message_id}.txt"


class LogAnalyzerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("950x700")
        self.root.minsize(950, 700)
        self.target_var = tk.StringVar()
        self.clock_var = tk.StringVar()

        top_frame = ttk.Frame(root, padding=8)
        top_frame.pack(fill="x")

        ttk.Label(top_frame, text="Target Folder").pack(anchor="w")
        entry_row = ttk.Frame(top_frame)
        entry_row.pack(fill="x", pady=(2, 6))

        self.target_entry = ttk.Entry(entry_row, textvariable=self.target_var)
        self.target_entry.pack(side="left", fill="x", expand=True)
        ttk.Button(entry_row, text="Browse", command=self.browse_folder).pack(side="left", padx=4)
        ttk.Button(entry_row, text="Load", command=self.load_logs).pack(side="left")

        clock_row = ttk.Frame(top_frame)
        clock_row.pack(fill="x", pady=(0, 8))
        ttk.Label(clock_row, text="Current Time").pack(side="left")
        ttk.Label(clock_row, textvariable=self.clock_var).pack(side="left", padx=8)

        body = ttk.Frame(root, padding=8)
        body.pack(fill="both", expand=True)

        self.internal_section = ScrollableSection(body, "내부")
        self.external_section = ScrollableSection(body, "외부")
        self.internal_section.pack(side="left", fill="both", expand=True, padx=(0, 6))
        self.external_section.pack(side="left", fill="both", expand=True)

        ensure_workspace()
        self._tick_clock()

    def browse_folder(self):
        initial_dir = get_default_base_root() or str(BASE_DIR)
        folder = filedialog.askdirectory(title="Select Scenario Log Folder", initialdir=initial_dir)
        if folder:
            self.target_var.set(folder)

    def load_logs(self):
        raw_path = self.target_var.get().strip()
        if not raw_path:
            messagebox.showwarning("경로 입력", "Target Folder 경로를 입력하거나 선택해주세요.")
            return
        try:
            base_path = self._normalize_target_path(Path(raw_path))
        except FileNotFoundError as exc:
            messagebox.showerror("경로 오류", str(exc))
            return

        if not base_path.exists():
            messagebox.showerror("경로 오류", f"{base_path} 경로를 찾을 수 없습니다.")
            return
        try:
            message_root = resolve_message_root(base_path)
        except FileNotFoundError as exc:
            messagebox.showerror("폴더 구조", str(exc))
            return

        entries = load_message_entries(message_root)
        internal = [e for e in entries if e.category == "internal"]
        external = [e for e in entries if e.category == "external"]

        self.internal_section.populate(internal, self.open_entry)
        self.external_section.populate(external, self.open_entry)

    def open_entry(self, entry: MessageEntry, file_path: Path):
        try:
            cached_path = copy_to_cache(entry.message_id, file_path)
        except Exception as exc:
            messagebox.showerror("파일 복사 실패", f"파일 복사 중 오류가 발생했습니다.\n{exc}")
            return
        DetailWindow(self.root, entry, file_path, cached_path)

    def _normalize_target_path(self, path: Path) -> Path:
        path = path.expanduser()
        if path.is_file():
            path = path.parent
        if not path.exists():
            raise FileNotFoundError(f"{path} 경로를 찾을 수 없습니다.")

        agency = get_default_agency()
        if path.name.lower() != agency.lower():
            candidate = path / agency
            if candidate.exists():
                path = candidate

        return path

    def _tick_clock(self):
        self.clock_var.set(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        self.root.after(1000, self._tick_clock)


def main():
    root = tk.Tk()
    app = LogAnalyzerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
