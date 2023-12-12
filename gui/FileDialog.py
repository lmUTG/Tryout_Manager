from gui.IterationDialog import *
from PyQt5.QtWidgets import QWidget, QFileDialog


class FileDialog(QWidget):

    def __init__(self, option, title, extensions, set_iteration):
        super().__init__()
        self.left = 10
        self.top = 10
        self.width = 640
        self.height = 480
        self.selected = None
        self.iteration = -1
        self.set_iteration = set_iteration

        self.initUI(option, title, extensions)

    def initUI(self, option, title, extensions):
        self.setWindowTitle(title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        if option == 0:
            self.openFileNameDialog(title, extensions)
            self.show()
        elif option == 1:
            self.openFileNamesDialog(title, extensions)
            self.show()
        elif option == 2:
            self.saveFileDialog(title, extensions)
            self.show()

    def openFileNameDialog(self, title, extensions):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getOpenFileName(self, title, "", extensions, options=options)
        if file_name:
            self.selected = file_name

    def openFileNamesDialog(self, title, extensions):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        files, _ = QFileDialog.getOpenFileNames(self, title, "", extensions, options=options)
        if not len(files) == 0:
            if self.set_iteration:
                self.iteration = IterationDialog.isOkay()
                if not self.iteration == -1:
                    self.selected = files
            else:
                self.selected = files

    def saveFileDialog(self, title, extensions):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getSaveFileName(self, title, "", extensions, options=options)
        self.iteration = IterationDialog.isOkay()
        if file_name and not self.iteration == -1:
            self.selected = file_name
