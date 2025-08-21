# -*- coding: utf-8 -*-
"""
파일: 30x30_GridWindow_PyQt5.py
설명: 1800x900(가변 리사이즈 가능) GUI 창에 표를 배치하고 각 셀에 숫자를 채움.
      'm' 또는 'Ctrl+M'로 병합 시 GUI 기능 텍스트를 입력받아 병합셀에 표시하고,
      터미널에 누적 목록을 모두 출력.

단축키:
 - m        : '선택된 영역' 병합 (1×1 선택이면 동작 안 함. 스팬 내부 1×1 클릭은 그 스팬 전체로 간주)
 - Ctrl+M   : '현재 가시영역(뷰포트)' 병합
"""

import os
import sys
from typing import List, Dict

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTableWidget, QTableWidgetItem,
    QAbstractItemView, QAction, QDialog, QVBoxLayout, QLabel,
    QTextEdit, QDialogButtonBox
)
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt


# ---------- 입력 다이얼로그 ----------
class FeatureDialog(QDialog):
    def __init__(self, parent=None, region_text: str = ""):
        super().__init__(parent)
        self.setWindowTitle("GUI 기능 할당")
        self.setModal(True)
        layout = QVBoxLayout(self)

        lbl = QLabel(f"병합된 영역 {region_text}에 대한 GUI 기능을 입력하세요:", self)
        self.edit = QTextEdit(self)
        self.edit.setPlaceholderText("예: 고도 지도 토글, 위협 리스트 팝업, 경로 재계산 버튼 등")

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)

        layout.addWidget(lbl)
        layout.addWidget(self.edit)
        layout.addWidget(btns)

    def get_text(self) -> str:
        return self.edit.toPlainText().strip()


class GridWindow(QMainWindow):
    def __init__(self, rows: int = 50, cols: int = 30, start: int = 1):
        super().__init__()
        self.rows = rows
        self.cols = cols
        self.start = start
        self.assignments: List[Dict] = []  # 누적 기록

        self.setWindowTitle("Grid (Resizable)")
        self.resize(1800, 900)
        self.setContentsMargins(0, 0, 0, 0)

        self._build_table()
        self.populate_table()

    def _build_table(self) -> None:
        """중앙 위젯으로 QTableWidget 구성 + 병합 단축키 연결"""
        table = QTableWidget(self.rows, self.cols, self)
        table.setContentsMargins(0, 0, 0, 0)
        table.setAlternatingRowColors(False)
        table.setShowGrid(True)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)

        # 헤더 숨김
        table.horizontalHeader().setVisible(False)
        table.verticalHeader().setVisible(False)

        # 스크롤바 제거(전체가 뷰포트에 들어오게 강제)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # 선택: 직사각형 + 셀 단위
        table.setSelectionMode(QAbstractItemView.ContiguousSelection)
        table.setSelectionBehavior(QAbstractItemView.SelectItems)

        # 가독성/공간 최적화
        table.setWordWrap(True)  # 병합셀에 여러 줄 텍스트 표시 위해 True
        table.setStyleSheet("QTableWidget::item { padding: 0px; }")
        table.setFrameShape(table.NoFrame)

        # 'm' : 선택 병합(+ 기능 텍스트 입력)
        merge_sel = QAction("MergeSelection", table)
        merge_sel.setShortcut(QKeySequence("m"))
        merge_sel.setShortcutContext(Qt.WidgetWithChildrenShortcut)
        merge_sel.triggered.connect(lambda: self._merge_current_region(use_viewport=False, prompt=True))
        table.addAction(merge_sel)

        # 'Ctrl+M' : 가시영역 병합(+ 기능 텍스트 입력)
        merge_vp = QAction("MergeViewport", table)
        merge_vp.setShortcut(QKeySequence("Ctrl+M"))
        merge_vp.setShortcutContext(Qt.WidgetWithChildrenShortcut)
        merge_vp.triggered.connect(lambda: self._merge_current_region(use_viewport=True, prompt=True))
        table.addAction(merge_vp)

        self.table = table
        self.setCentralWidget(table)
        table.setFocus()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._autosize_table()

    def showEvent(self, e):
        super().showEvent(e)
        self._autosize_table()

    def _autosize_table(self) -> None:
        """
        1) 뷰포트 크기를 rows/cols로 균등 분배
        2) 실제 사용 픽셀 기반으로 마지막 열/행에 delta 보정(빈 공간 제거)
        3) 폰트 크기를 셀 크기에 맞춰 자동 축소
        """
        tbl = getattr(self, "table", None)
        if tbl is None or tbl.rowCount() == 0 or tbl.columnCount() == 0:
            return

        vp = tbl.viewport()
        vp_w = max(1, vp.width())
        vp_h = max(1, vp.height())
        cols = tbl.columnCount()
        rows = tbl.rowCount()

        # --- 균등 분배 ---
        base_cw = max(1, vp_w // cols)
        rem_w = vp_w - base_cw * cols
        for c in range(cols):
            tbl.setColumnWidth(c, base_cw + (1 if c < rem_w else 0))

        base_rh = max(1, vp_h // rows)
        rem_h = vp_h - base_rh * rows
        for r in range(rows):
            tbl.setRowHeight(r, base_rh + (1 if r < rem_h else 0))

        # --- 미세 보정 ---
        if cols > 0:
            last_c_right = tbl.columnViewportPosition(cols - 1) + tbl.columnWidth(cols - 1)
            delta_w = vp_w - last_c_right
            if delta_w != 0:
                tbl.setColumnWidth(cols - 1, max(1, tbl.columnWidth(cols - 1) + delta_w))

        if rows > 0:
            last_r_bottom = tbl.rowViewportPosition(rows - 1) + tbl.rowHeight(rows - 1)
            delta_h = vp_h - last_r_bottom
            if delta_h != 0:
                tbl.setRowHeight(rows - 1, max(1, tbl.rowHeight(rows - 1) + delta_h))

        # --- 폰트 크기 자동 축소 ---
        min_col_w = min(tbl.columnWidth(c) for c in range(cols)) if cols else 1
        min_row_h = min(tbl.rowHeight(r) for r in range(rows)) if rows else 1
        min_cell = max(1, min(min_col_w, min_row_h))
        target_pt = max(6, int(min_cell * 0.10))  # 폰트 작게 유지
        f = tbl.font()
        if f.pointSize() != target_pt:
            f.setPointSize(target_pt)
            tbl.setFont(f)
            for r in range(rows):
                for c in range(cols):
                    it = tbl.item(r, c)
                    if it is not None:
                        it.setFont(f)

        tbl.viewport().update()

    # ---------- 병합 + 기능 입력 ----------
    def _merge_current_region(self, use_viewport: bool = False, prompt: bool = True) -> None:
        """
        병합 실행:
        - use_viewport=True  → 현재 가시영역 병합 (Ctrl+M)
        - use_viewport=False → '선택된 영역' 병합 (여러 range면 외접 사각형)
                               1×1 선택이면 동작 안 함. 단, 1×1이 기존 스팬 내부면 그 스팬 전체로 간주.
        - prompt=True        → 병합 직후 기능 텍스트 입력/저장/표시/출력
        """
        tbl = self.table

        # --- 병합 대상 사각형 결정 ---
        if use_viewport:
            vp = tbl.viewport()
            r1 = tbl.rowAt(0)
            c1 = tbl.columnAt(0)
            r2 = tbl.rowAt(vp.height() - 1)
            c2 = tbl.columnAt(vp.width() - 1)
            if r1 < 0: r1 = 0
            if c1 < 0: c1 = 0
            if r2 < 0: r2 = tbl.rowCount() - 1
            if c2 < 0: c2 = tbl.columnCount() - 1
        else:
            ranges = tbl.selectedRanges()
            if not ranges:
                return  # 선택 없음
            r1 = min(rg.topRow() for rg in ranges)
            c1 = min(rg.leftColumn() for rg in ranges)
            r2 = max(rg.bottomRow() for rg in ranges)
            c2 = max(rg.rightColumn() for rg in ranges)

            # 1×1 선택 → 스팬 내부인지 확인
            if r1 == r2 and c1 == c2:
                cell_r, cell_c = r1, c1
                found_span = False
                rows, cols = tbl.rowCount(), tbl.columnCount()
                for tr in range(rows):
                    for tc in range(cols):
                        rs = tbl.rowSpan(tr, tc)
                        cs = tbl.columnSpan(tr, tc)
                        if rs > 1 or cs > 1:
                            top, left = tr, tc
                            bot, right = tr + rs - 1, tc + cs - 1
                            if top <= cell_r <= bot and left <= cell_c <= right:
                                r1, c1, r2, c2 = top, left, bot, right
                                found_span = True
                                break
                    if found_span:
                        break
                if not found_span:
                    return  # 진짜 1×1은 무시

        # 정규화
        if r1 > r2: r1, r2 = r2, r1
        if c1 > c2: c1, c2 = c2, c1

        row_span = r2 - r1 + 1
        col_span = c2 - c1 + 1
        if row_span == 1 and col_span == 1:
            return

        # --- 겹치는 기존 스팬 해제 ---
        self._clear_spans_intersecting(r1, c1, r2, c2)

        # --- 병합 실행 ---
        tbl.setSpan(r1, c1, row_span, col_span)

        # 헤더 라인(좌상 셀)
        header = f"시작:({r1+1},{c1+1})  끝:({r2+1},{c2+1})"
        item = tbl.item(r1, c1)
        if item is None:
            item = QTableWidgetItem()
        # 기존 텍스트가 있다면 헤더를 유지하고 밑에 줄바꿈 후 누적
        curr = (item.text() or "").strip()
        if not curr:
            item.setText(header)
        elif not curr.splitlines()[0].startswith("시작:"):
            item.setText(header + "\n" + curr)
        # 정렬은 중앙 유지(요구사항에 맞게) — 필요 시 AlignLeft|AlignTop으로 변경 가능
        item.setTextAlignment(Qt.AlignCenter)
        tbl.setItem(r1, c1, item)

        # 기능 입력/저장/표시/출력
        if prompt:
            self._prompt_and_assign_feature(r1, c1, r2, c2)

        tbl.clearSelection()
        tbl.viewport().update()

    def _prompt_and_assign_feature(self, r1: int, c1: int, r2: int, c2: int) -> None:
        """다이얼로그로 기능 텍스트를 입력받아 셀에 추가하고 누적 출력"""
        region_text = f"({r1+1},{c1+1}) ~ ({r2+1},{c2+1})"
        dlg = FeatureDialog(self, region_text=region_text)
        if dlg.exec_() == QDialog.Accepted:
            text = dlg.get_text()
            if text:
                # 셀 텍스트에 아래 줄로 추가
                self._append_text_to_cell(r1, c1, r2, c2, text)
                # 누적 기록/출력
                self.assignments.append({
                    "r1": r1 + 1, "c1": c1 + 1, "r2": r2 + 1, "c2": c2 + 1, "text": text
                })
                self._print_assignments()

    def _append_text_to_cell(self, r1: int, c1: int, r2: int, c2: int, text: str) -> None:
        tbl = self.table
        item = tbl.item(r1, c1)
        if item is None:
            item = QTableWidgetItem()
        curr = (item.text() or "").rstrip()
        header = f"시작:({r1+1},{c1+1})  끝:({r2+1},{c2+1})"
        if not curr:
            new_text = f"{header}\n{text}"
        else:
            # 이미 헤더가 있으면 그대로 유지하고 내용만 추가
            if curr.splitlines()[0].startswith("시작:"):
                new_text = curr + "\n" + text
            else:
                new_text = f"{header}\n{curr}\n{text}"
        item.setText(new_text)
        item.setTextAlignment(Qt.AlignCenter)
        tbl.setItem(r1, c1, item)

    def _print_assignments(self) -> None:
        """터미널에 누적 기록 전체를 보기 좋게 출력"""
        print("\n=== GUI 기능 할당 목록 ===")
        for idx, rec in enumerate(self.assignments, 1):
            head = f"{idx:02d}. 범위: ({rec['r1']},{rec['c1']}) ~ ({rec['r2']},{rec['c2']})"
            print(head)
            lines = rec["text"].splitlines() or [""]
            for ln in lines:
                print(f"    {ln}")
        print(f"=== 총 {len(self.assignments)}건 ===")
        sys.stdout.flush()

    # ---------- 스팬 해제 ----------
    def _clear_spans_intersecting(self, r1: int, c1: int, r2: int, c2: int) -> None:
        """(r1..r2, c1..c2)와 교차하는 스팬을 모두 해제"""
        tbl = self.table
        rows, cols = tbl.rowCount(), tbl.columnCount()

        def intersects(a1, b1, a2, b2, x1, y1, x2, y2) -> bool:
            # [a1..b1]x[a2..b2] 와 [x1..x2]x[y1..y2] 교차 여부
            return not (b1 < x1 or x2 < a1 or b2 < y1 or y2 < a2)

        for r in range(rows):
            for c in range(cols):
                rs = tbl.rowSpan(r, c)
                cs = tbl.columnSpan(r, c)
                if rs > 1 or cs > 1:
                    top, left = r, c
                    bot, right = r + rs - 1, c + cs - 1
                    if intersects(r1, r2, c1, c2, top, left, bot, right):
                        tbl.setSpan(r, c, 1, 1)

        tbl.viewport().update()

    # ---------- 데이터 채움 ----------
    def populate_table(self) -> None:
        """각 셀에 1부터 순차 숫자 채우고 중앙 정렬"""
        n = self.start
        for r in range(self.rows):
            for c in range(self.cols):
                item = QTableWidgetItem(str(n))
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, item)
                n += 1
        self._autosize_table()


def main():
    # ✅ 권장: 모니터 배율을 Qt가 올바르게 인식하도록 설정
    os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    os.environ["QT_SCALE_FACTOR_ROUNDING_POLICY"] = "PassThrough"  # Qt 5.14+

    # Qt 애트리뷰트는 QApplication 생성 전에 지정
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    try:
        app.setHighDpiScaleFactorRoundingPolicy(
            Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
        )
    except AttributeError:
        pass

    # 표 크기 원하는 값으로 조정 가능
    win = GridWindow(rows=35, cols=50, start=1)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
