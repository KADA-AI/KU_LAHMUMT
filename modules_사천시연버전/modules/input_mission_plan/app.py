# app.py
# 실행:  python app.py

import sys
from PyQt5.QtWidgets import QApplication
from main_window import CMPK0201Window

def main():
    app = QApplication(sys.argv)
    win = CMPK0201Window()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
