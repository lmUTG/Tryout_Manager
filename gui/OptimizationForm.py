from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLineEdit, QFormLayout, QPushButton, QLabel, QHBoxLayout, \
    QProgressBar, QMessageBox
from PyQt5.QtCore import QThread
from gui.OptimizationWorker import OptimizationWorker
from vedo import mesh
import math


def display_target_error(error_text):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Information)
    msg.setText("Missing Target Geometry")
    msg.setInformativeText(error_text)
    msg.setWindowTitle("Error")
    msg.exec_()


class OptimizationForm(QWidget):
    def __init__(self):
        super(OptimizationForm, self).__init__()
        self.thread = QThread()
        self.optimization_worker = OptimizationWorker()

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        self.degree_u_lower_bound = QLineEdit()
        self.degree_u_upper_bound = QLineEdit()

        self.degree_v_lower_bound = QLineEdit()
        self.degree_v_upper_bound = QLineEdit()

        self.ctrlpts_size_u_lower_bound = QLineEdit()
        self.ctrlpts_size_u_upper_bound = QLineEdit()

        self.ctrlpts_size_v_lower_bound = QLineEdit()
        self.ctrlpts_size_v_upper_bound = QLineEdit()

        self.res = None

        self.target_mesh = None
        self.target_grid = None

        self.deviation = 0
        self.x = [0, 0, 0, 0]

        self.optimization_button = QPushButton("Optimize")
        self.optimization_button.clicked.connect(self.optimize)

        self.degree_u = QLabel("Degree u: " + str(round(self.x[0])))
        self.degree_v = QLabel("Degree v: " + str(round(self.x[1])))
        self.ctrlpts_size_u = QLabel("Control Points Size u: " + str(round(self.x[2])))
        self.ctrlpts_size_v = QLabel("Control Points Size v: " + str(round(self.x[3])))
        self.best_deviation_label = QLabel("Average Deviation: {}".format(self.deviation))

        self.grid_points_u = 0
        self.grid_points_v = 0

        self.grid_points_u_label = QLabel("Number of grid points u: " + str(round(self.grid_points_u)))
        self.grid_points_v_label = QLabel("Number of grid points v: " + str(round(self.grid_points_v)))

        self.form = QFormLayout()
        self.main_layout.addLayout(self.form)

        self.labels = QHBoxLayout()
        self.labels_left = QVBoxLayout()
        self.labels_right = QVBoxLayout()
        self.labels.addLayout(self.labels_left)
        self.labels.addLayout(self.labels_right)
        self.main_layout.addLayout(self.labels)

        self.labels_left.addWidget(self.degree_u)
        self.labels_left.addWidget(self.degree_v)
        self.labels_left.addWidget(self.ctrlpts_size_u)
        self.labels_left.addWidget(self.ctrlpts_size_v)
        self.labels_left.addWidget(self.best_deviation_label)

        self.labels_right.addWidget(self.grid_points_u_label)
        self.labels_right.addWidget(self.grid_points_v_label)

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)

        self.kappa = QLineEdit("2.1")
        self.num_iterations = QLineEdit("100")

        self.real_pb_value = 0

        self.worker_setup = False

        self.init_gui()

    def init_gui(self):
        self.form.addRow("Degree u Lower Bound", self.degree_u_lower_bound)
        self.form.addRow("Degree u Upper Bound", self.degree_u_upper_bound)

        self.form.addRow("Degree v Lower Bound", self.degree_v_lower_bound)
        self.form.addRow("Degree v Upper Bound", self.degree_v_upper_bound)

        self.form.addRow("Number of Control Points u Lower Bound", self.ctrlpts_size_u_lower_bound)
        self.form.addRow("Number of Control Points u Upper Bound", self.ctrlpts_size_u_upper_bound)

        self.form.addRow("Number of Control Points v Lower Bound", self.ctrlpts_size_v_lower_bound)
        self.form.addRow("Number of Control Points v Upper Bound", self.ctrlpts_size_v_upper_bound)

        self.form.addRow("Kappa", self.kappa)
        self.form.addRow("Number of Iterations", self.num_iterations)

        self.form.addRow(self.optimization_button)

        self.main_layout.addWidget(self.progress_bar)

    def setup_worker(self):
        self.optimization_worker.moveToThread(self.thread)
        self.thread.started.connect(self.optimization_worker.run)
        self.optimization_worker.progress.connect(self.update_values)
        self.optimization_worker.iteration.connect(self.increase_progress_bar)
        self.optimization_worker.finished.connect(self.thread.quit)
        # self.optimization_worker.finished.connect(self.optimization_worker.deleteLater)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(self.save_res)
        self.optimization_worker.finished.connect(
            lambda: self.optimization_button.setEnabled(True)
        )
        self.worker_setup = True

    def optimize(self):
        if self.target_grid is None or self.target_mesh is None:
            display_target_error("Target mesh and target grid have to be imported to optimize surface parameters.")
            return
        self.progress_bar.setValue(0)
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.real_pb_value = 0

        self.optimization_worker.degree_u_lower_bound = int(self.degree_u_lower_bound.text())
        self.optimization_worker.degree_u_upper_bound = int(self.degree_u_upper_bound.text())

        self.optimization_worker.degree_v_lower_bound = int(self.degree_v_lower_bound.text())
        self.optimization_worker.degree_v_upper_bound = int(self.degree_v_upper_bound.text())

        self.optimization_worker.ctrlpts_size_u_lower_bound = int(self.ctrlpts_size_u_lower_bound.text())
        self.optimization_worker.ctrlpts_size_u_upper_bound = int(self.ctrlpts_size_u_upper_bound.text())

        self.optimization_worker.ctrlpts_size_v_lower_bound = int(self.ctrlpts_size_v_lower_bound.text())
        self.optimization_worker.ctrlpts_size_v_upper_bound = int(self.ctrlpts_size_v_upper_bound.text())

        self.optimization_worker.kappa = float(self.kappa.text())
        self.optimization_worker.num_iterations = int(self.num_iterations.text())

        self.optimization_worker.target_stl = self.target_mesh

        if not self.worker_setup:
            self.setup_worker()
        self.thread.start()

        self.optimization_button.setEnabled(False)

    def increase_progress_bar(self):
        step_size = self.progress_bar.maximum() / int(self.num_iterations.text())
        self.real_pb_value += step_size
        pb_value = math.ceil(self.real_pb_value)
        if pb_value > 100:
            pb_value = 100
        self.progress_bar.setValue(pb_value)

    def save_res(self):
        self.res = self.optimization_worker.res

    def set_target_mesh(self, file_path):
        self.target_mesh = mesh.Mesh(file_path)

    def set_target_grid(self, target_grid):
        self.target_grid = target_grid
        self.optimization_worker.size_u = self.target_grid["size_u"]
        self.optimization_worker.size_v = self.target_grid["size_v"]
        self.optimization_worker.grid = self.target_grid["points"]

        self.grid_points_u = self.target_grid["size_u"]
        self.grid_points_v = self.target_grid["size_v"]
        self.grid_points_u_label.setText("Number of grid points u: " + str(round(self.grid_points_u)))
        self.grid_points_v_label.setText("Number of grid points v: " + str(round(self.grid_points_v)))

    def update_values(self):
        self.deviation = self.optimization_worker.best_deviation
        self.x = self.optimization_worker.best_x

        self.degree_u.setText("Degree u: " + str(round(self.x[0])))
        self.degree_v.setText("Degree v: " + str(round(self.x[1])))
        self.ctrlpts_size_u.setText("Control Points Size u: " + str(round(self.x[2])))
        self.ctrlpts_size_v.setText("Control Points Size v: " + str(round(self.x[3])))
        self.best_deviation_label.setText("Average Deviation: {}".format(self.deviation))
