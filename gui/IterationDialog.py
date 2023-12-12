from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QFormLayout, QLineEdit


class IterationDialog(QDialog):
    def __init__(self):
        super(IterationDialog, self).__init__()
        self.setWindowTitle("Iteration Number")
        self.ok = False
        layout = QFormLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(layout)

        self.iteration_number = QLineEdit()
        layout.addRow("Iteration Number:", self.iteration_number)

        self.btn_ok = QPushButton("Ok", self)
        layout.addRow(self.btn_ok)
        self.btn_ok.clicked.connect(self.button_press)
        self.btn_cancel = QPushButton("Cancel", self)
        layout.addRow(self.btn_cancel)
        self.btn_cancel.clicked.connect(self.button_press)

    def button_press(self):
        if self.sender() == self.btn_ok:
            self.ok = True
        self.close()

    @classmethod
    def isOkay(cls):
        dialog = cls()
        dialog.exec_()
        if dialog.ok:
            return dialog.iteration_number.text()
        else:
            return -1


