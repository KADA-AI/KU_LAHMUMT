from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QPen, QFont
from PyQt5.QtCore import Qt, QRectF

class CircularProgressBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.value = 0
        self.text = ""
        self.progress_color = QColor(75, 175, 255) # Default blue color
        self.setMinimumSize(100, 100) # Default size

    def setValue(self, value):
        if 0 <= value <= 100:
            self.value = value
            self.update() # Redraw the widget

    def setText(self, text):
        self.text = text
        self.update()

    def setColor(self, color: QColor):
        self.progress_color = color
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing) # For smooth edges

        rect = self.rect()
        side = min(rect.width(), rect.height())
        painter.setViewport((rect.width() - side) // 2, (rect.height() - side) // 2, side, side)
        painter.setWindow(0, 0, 100, 100)

        # Draw background circle
        painter.setPen(QPen(QColor(200, 200, 200), 8)) # Light gray pen, 8px width
        painter.drawEllipse(5, 5, 90, 90) # Draw a circle

        # Draw progress arc
        painter.setPen(QPen(self.progress_color, 8)) # Use dynamic color
        start_angle = 90 * 16 # Start from top (90 degrees clockwise from 3 o'clock)
        span_angle = int(-self.value * 360 * 16 / 100) # Fill clockwise
        painter.drawArc(5, 5, 90, 90, start_angle, span_angle)

        # Draw text (percentage or custom text)
        painter.setPen(QPen(QColor(0, 0, 0))) # Black text
        font = QFont("Arial", 15)
        painter.setFont(font)
        text_rect = QRectF(0, 0, 100, 100)
        if self.text:
            painter.drawText(text_rect, Qt.AlignCenter, self.text)
        else:
            painter.drawText(text_rect, Qt.AlignCenter, f"{self.value}%")

        painter.end()
