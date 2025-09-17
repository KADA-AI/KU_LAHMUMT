# gui/tabs/DummyTab.py: 메인 GUI의 '더미' 탭에 해당하는 UI와 기능을 정의합니다.

from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout

class DummyTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        label = QLabel("Dummy Tab (Placeholder)")
        layout.addWidget(label)
        self.setLayout(layout)

    def refresh_display(self, update_info):
        # This tab does not handle updates yet.
        pass