import copy
import multiprocessing
import os
import shutil
import sys

from PyQt5 import QtCore
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QMainWindow, QMessageBox, QApplication

import config
from search_algorithm import BFS, Greedy
from ui import Ui_MainWindow


class MainWindow(QMainWindow, Ui_MainWindow):
    class MessageThread(QThread):
        signal = pyqtSignal(str)

        def __init__(self, queue):
            super(QThread, self).__init__()
            self.__message_queue = queue

        def run(self):
            while True:
                message = self.__message_queue.get()
                self.signal.emit(message)

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)

        self.pentagon_type = 0
        self.search_method = 0

        self.tangram_solver = None

        self.message_queue = multiprocessing.Queue(-1)

        self.solveBtn.clicked.connect(self.solve_puzzle)
        self.pentagon_typeCB.currentIndexChanged.connect(self.change_pentagon_type)
        self.search_methodCB.currentIndexChanged.connect(self.change_search_method)
        self.horizontalScrollBar.valueChanged.connect(self.load_image)

        self.message_thread = MainWindow.MessageThread(self.message_queue)
        self.message_thread.signal.connect(self.message_handler)
        self.message_thread.start()

    def message_handler(self, message):
        files = os.listdir(config.RESULT_PATH)
        file_count = len(files)
        QMessageBox.information(self, 'info', '生成完毕！', QMessageBox.Ok)
        self.horizontalScrollBar.setMinimum(1 if file_count > 0 else 0)
        self.horizontalScrollBar.setMaximum(file_count)
        self.horizontalScrollBar.setValue(0)
        self.image_count.setText(str(file_count))
        self.image_index.setText("1" if file_count > 0 else "0")

    def solve_puzzle(self):
        if os.path.exists(config.RESULT_PATH):
            shutil.rmtree(config.RESULT_PATH)
            os.makedirs(config.RESULT_PATH)
        if self.search_method == 0:
            pass
        if self.search_method == 1:
            self.tangram_solver = BFS(self.pentagon_type, self.message_queue)
            #           self.tangram_solver.signal.connect(self.solver_callback)
            self.tangram_solver.start()
            QMessageBox.information(self, 'info',
                                    '使用' + self.search_methodCB.currentText() + '生成' + self.pentagon_typeCB.currentText() + '，请等待~（可能需要大量时间）',
                                    QMessageBox.Ok)
        if self.search_method == 2:
            self.tangram_solver = Greedy(self.pentagon_type,self.message_queue)
            self.tangram_solver.start()
            QMessageBox.information(self, 'info',
                                    '使用' + self.search_methodCB.currentText() + '生成' + self.pentagon_typeCB.currentText() + '，请等待~（可能需要大量时间）',
                                    QMessageBox.Ok)

    def change_pentagon_type(self):
        self.pentagon_type = self.pentagon_typeCB.currentIndex()

    def change_search_method(self):
        self.search_method = self.search_methodCB.currentIndex()

    def load_image(self):
        index = self.horizontalScrollBar.value()
        self.image_index.setText(str(index))
        if index == 0:
            return
        image_file = os.path.join(config.RESULT_PATH, str(index) + ".jpg")
        image_source = QPixmap(image_file)
        image = image_source.scaled(self.imageLb.size(), QtCore.Qt.KeepAspectRatio)
        self.imageLb.setPixmap(image)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())
