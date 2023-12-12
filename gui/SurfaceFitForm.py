from PyQt5.QtWidgets import QWidget, QFormLayout, QLineEdit, QPushButton, QVBoxLayout, QListWidget, QCheckBox, \
    QMessageBox, QLabel
from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal
from gui.SurfaceFitWorker import *
import json


class SurfaceFitForm(QWidget):
    grid_queue_popped_signal = pyqtSignal()

    def __init__(self):
        super(SurfaceFitForm, self).__init__()
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        self.list_view = QListWidget()

        self.common_knotvector_u = []
        self.common_knotvector_v = []

        self.surface_fit_queue = []

        self.target_surface = None

        self.degree_u = QLineEdit()
        self.degree_v = QLineEdit()
        self.size_u = QLineEdit()
        self.size_v = QLineEdit()

        self.surface_fit_worker = SurfaceFitWorker()
        self.thread = QThread()

        self.is_target = QCheckBox("Target Surface")
        self.form = QFormLayout()

        self.surface_fit_button = QPushButton("Fit Surface")

        self.is_worker_set = False

        self.target_degree_u = 0
        self.target_degree_v = 0
        self.target_num_ctrlpts_u = 0
        self.target_num_ctrlpts_v = 0

        self.target_degree_u_label = QLabel("Target Degree u: " + str(round(self.target_degree_u)))
        self.target_degree_v_label = QLabel("Target Degree v: " + str(round(self.target_degree_v)))
        self.target_num_ctrlpts_u_label = QLabel(
            "Target Number of Control Points u: " + str(round(self.target_num_ctrlpts_u)))
        self.target_num_ctrlpts_v_label = QLabel(
            "Target Number of Control Points v: " + str(round(self.target_num_ctrlpts_v)))

        self.init_gui()

    def init_gui(self):
        self.form.addRow("Degree in u direction:", self.degree_u)
        self.form.addRow("Degree in v direction:", self.degree_v)
        self.form.addRow("Number of control points in u direction:", self.size_u)
        self.form.addRow("Number of control points in v direction:", self.size_v)

        self.main_layout.addLayout(self.form)

        self.main_layout.addWidget(self.list_view)

        self.main_layout.addWidget(self.is_target)

        self.surface_fit_button.clicked.connect(self.start_surface_generation)
        self.main_layout.addWidget(self.surface_fit_button)
        self.main_layout.addWidget(self.target_degree_u_label)
        self.main_layout.addWidget(self.target_degree_v_label)
        self.main_layout.addWidget(self.target_num_ctrlpts_u_label)
        self.main_layout.addWidget(self.target_num_ctrlpts_v_label)

    def update_list(self):
        names = self.surface_fit_queue[0]
        self.list_view.clear()
        self.list_view.addItems(names)

    @pyqtSlot()
    def pop_list(self):
        for surface_fit_list in self.surface_fit_queue:
            _ = surface_fit_list.pop(0)
        self.grid_queue_popped_signal.emit()

    def setup_worker(self):
        self.surface_fit_worker.moveToThread(self.thread)
        self.thread.started.connect(self.surface_fit_worker.run)
        self.surface_fit_worker.finished.connect(self.thread.quit)
        # self.surface_fit_worker.finished.connect(self.surface_fit_worker.deleteLater)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.surface_fit_worker.progress.connect(self.pop_list)
        self.surface_fit_worker.finished.connect(
            lambda: self.surface_fit_button.setEnabled(True)
        )

        self.is_worker_set = True

    def start_surface_generation(self):
        if not self.surface_fit_queue:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText("Empty Queue")
            msg.setInformativeText("Add grid to queue to start surface approximation.")
            msg.setWindowTitle("Error")
            msg.exec_()
            return

        paths = self.surface_fit_queue[1][:]
        names = self.surface_fit_queue[0][:]
        size_u_arr = []
        size_v_arr = []
        grid_points_arr = []
        degree_u_arr = []
        degree_v_arr = []
        ctrlpts_size_u_arr = []
        ctrlpts_size_v_arr = []

        for path in paths:
            file = open(path)
            grid_dict = json.load(file)
            size_u = grid_dict['size_u']
            size_v = grid_dict['size_v']
            grid_points = grid_dict['points']
            degree_u = int(self.degree_u.text())
            degree_v = int(self.degree_v.text())
            ctrlpts_size_u = int(self.size_u.text())
            ctrlpts_size_v = int(self.size_v.text())

            size_u_arr.append(size_u)
            size_v_arr.append(size_v)
            grid_points_arr.append(grid_points)
            degree_u_arr.append(degree_u)
            degree_v_arr.append(degree_v)
            ctrlpts_size_u_arr.append(ctrlpts_size_u)
            ctrlpts_size_v_arr.append(ctrlpts_size_v)

        self.surface_fit_worker.setup_worker(size_u_arr,
                                             size_v_arr,
                                             grid_points_arr,
                                             degree_u_arr,
                                             degree_v_arr,
                                             ctrlpts_size_u_arr,
                                             ctrlpts_size_v_arr,
                                             names,
                                             self.is_target.isChecked(),
                                             self.common_knotvector_u,
                                             self.common_knotvector_v)
        if not self.is_worker_set:
            self.setup_worker()
        self.thread.start()

        self.surface_fit_button.setEnabled(False)

    def update_target_values(self, file_path):
        file = open(file_path)
        data = json.load(file)

        self.target_degree_u_label.setText("Degree in u direction: " + str(round(data['degree_u'])))
        self.target_degree_v_label.setText("Degree in v direction: " + str(round(data['degree_v'])))
        self.target_num_ctrlpts_u_label.setText(
            "Target Number of Control Points u: " + str(round(data['size_u'])))
        self.target_num_ctrlpts_v_label.setText(
            "Target Number of Control Points v: " + str(round(data['size_v'])))
