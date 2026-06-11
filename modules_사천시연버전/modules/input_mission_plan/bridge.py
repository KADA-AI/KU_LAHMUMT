# bridge.py
# QWebChannel로 JS에서 넘어오는 클릭 좌표(lat, lon)를 PyQt 시그널로 전달

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot

class MapBridge(QObject):
    # JS에서 보내는 지도 클릭 좌표
    pointClicked = pyqtSignal(float, float)

    @pyqtSlot(float, float)
    def sendPoint(self, lat: float, lon: float):
        """JS에서 호출되는 슬롯: 좌표를 시그널로 다시 방출"""
        self.pointClicked.emit(lat, lon)
