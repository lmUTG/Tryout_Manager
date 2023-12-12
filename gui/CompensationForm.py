from PyQt5.QtWidgets import QWidget, QVBoxLayout, QListWidget, QFormLayout, QRadioButton, QPushButton, QLineEdit, \
    QCheckBox, QMessageBox
from PyQt5.QtCore import QThread, pyqtSignal
from gui.CompensationWorker import *


def display_missing_data(error_text):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Information)
    msg.setText("Missing Data")
    msg.setInformativeText(error_text)
    msg.setWindowTitle("Error")
    msg.exec_()


class CompensationForm(QWidget):
    list_cleared_signal = pyqtSignal()

    def __init__(self):
        super(CompensationForm, self).__init__()

        self.compensation_worker = CompensationWorker()

        self.compensation_list = []

        self.target_surface = None
        self.tool_surface = None

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        self.list_view = QListWidget()

        self.mean_button = QRadioButton("Mean")
        self.mean_button.setChecked(True)
        self.max_button = QRadioButton("Max")
        self.median_button = QRadioButton("Median")

        self.compensate_button = QPushButton("Compensate")
        self.compensate_button.clicked.connect(self.start_compensation)

        self.common_form_widget = QWidget()

        self.compensation_weight = QLineEdit("1")
        self.normals_checkbox = QCheckBox("Use Normals")
        self.beta_checkbox = QCheckBox("Use Beta Function")

        self.thread = QThread()

        self.worker_setup = False

        self.init_gui()

    def init_gui(self):
        self.main_layout.addWidget(self.list_view)

        common_form_layout = QFormLayout()

        self.common_form_widget.setLayout(common_form_layout)

        common_form_layout.addRow("Compensation Weight:", self.compensation_weight)
        common_form_layout.addRow(self.normals_checkbox)
        common_form_layout.addRow(self.beta_checkbox)

        common_form_layout.addRow(self.mean_button)
        common_form_layout.addRow(self.max_button)
        common_form_layout.addRow(self.median_button)

        common_form_layout.addRow(self.compensate_button)

        self.main_layout.addWidget(self.common_form_widget)

    def setup_worker(self):
        self.compensation_worker.moveToThread(self.thread)
        self.thread.started.connect(self.compensation_worker.run)
        self.compensation_worker.finished.connect(self.thread.quit)
        # self.compensation_worker.finished.connect(self.compensation_worker.deleteLater)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(
            lambda: self.compensate_button.setEnabled(True)
        )
        self.thread.finished.connect(lambda: self.list_cleared_signal.emit())
        self.worker_setup = True

    def start_compensation(self):
        if not self.compensation_list and self.target_surface is None and self.tool_surface is None:
            display_missing_data(
                "Target surface, tool surface and source surfaces must be imported and queued for compensation.")
            return
        elif self.compensation_list and self.target_surface is not None and self.tool_surface is None:
            display_missing_data(
                "Tool surface must be imported for compensation.")
            return
        elif self.compensation_list and self.target_surface is None and self.tool_surface is not None:
            display_missing_data(
                "Target surface must be imported for compensation.")
            return
        elif not self.compensation_list and self.target_surface is not None and self.tool_surface is not None:
            display_missing_data(
                "Source surfaces must be imported and queued for compensation.")
            return
        elif not self.compensation_list and self.target_surface is None and self.tool_surface is not None:
            display_missing_data(
                "Target surface must be imported and source surfaces must be queued for compensation.")
            return
        elif not self.compensation_list and self.target_surface is not None and self.tool_surface is None:
            display_missing_data(
                "Tool surface must be imported and source surfaces must be queued for compensation.")
            return
        elif self.compensation_list and self.target_surface is None and self.tool_surface is None:
            display_missing_data(
                "Target and tool surfaces must be imported for compensation.")
            return

        self.compensation_worker.set_target_surface(self.target_surface)
        self.compensation_worker.set_tool_ctrlpts(self.tool_surface.ctrlpts)
        self.compensation_worker.use_beta(self.beta_checkbox.isChecked())
        self.compensation_worker.use_normals(self.normals_checkbox.isChecked())
        self.compensation_worker.set_compensation_weight(float(self.compensation_weight.text()))
        method = "mean"
        if self.max_button.isChecked():
            method = "max"
        elif self.median_button.isChecked():
            method = "median"

        self.compensation_worker.set_method(method)
        self.compensation_worker.set_source_ctrlpts(self.compensation_list[1])

        if not self.worker_setup:
            self.setup_worker()
        self.thread.start()

        self.compensate_button.setEnabled(False)

    def update_list(self):
        names = self.compensation_list[0]
        self.list_view.clear()
        self.list_view.addItems(names)
