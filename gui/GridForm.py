from creategrid_single import create_grid
import pymeshlab
from PyQt5.QtWidgets import QWidget, QFormLayout, QLineEdit, QPushButton, QTabWidget, QVBoxLayout, QListWidget, \
    QRadioButton, QLabel, QCheckBox, QMessageBox
from PyQt5.QtCore import QThread
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from gui.GridWorker import GridWorker
from mesh2grid import *
from vedo import plotter, shapes, mesh


def mesh_curve_length(mesh_curve):
    curve_vertices = mesh_curve.vertices()
    num_points, _ = curve_vertices.shape
    total_dist = 0
    for i in range(num_points - 1):
        dist = np.linalg.norm(curve_vertices[i] - curve_vertices[i + 1])
        total_dist = total_dist + dist
    return total_dist


def calculate_aspect_ratio(bordergeodesic1, bordergeodesic2, bordergeodesic3, bordergeodesic4):
    v = (bordergeodesic1, bordergeodesic2)
    u = (bordergeodesic3, bordergeodesic4)

    v_len = (mesh_curve_length(bordergeodesic1), mesh_curve_length(bordergeodesic2))
    u_len = (mesh_curve_length(bordergeodesic3), mesh_curve_length(bordergeodesic4))

    max_u = max(u_len)
    max_v = max(v_len)

    max_u_idx = u_len.index(max_u)
    max_v_idx = v_len.index(max_v)

    aspect_r = max_u / max_v

    return aspect_r, u[max_u_idx], v[max_v_idx]


def display_target_error(error_text):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Information)
    msg.setText("Missing Target Geometry")
    msg.setInformativeText(error_text)
    msg.setWindowTitle("Error")
    msg.exec_()


class GridForm(QWidget):
    grid_queue_popped_signal = pyqtSignal()

    def __init__(self):
        super(GridForm, self).__init__()
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        self.tabs = QTabWidget()
        self.main_layout.addWidget(self.tabs)
        self.list_view = QListWidget()

        self.x_start_target = QLineEdit("-78")
        self.x_end_target = QLineEdit("308")
        self.y_start_target = QLineEdit("-71")
        self.y_end_target = QLineEdit("65")
        self.z_start_target = QLineEdit("-1000")
        self.z_end_target = QLineEdit("1000")

        self.x_front_start_target = QLineEdit("195")
        self.x_front_end_target = QLineEdit("295")
        self.y_front_start_target = QLineEdit("-71")
        self.y_front_end_target = QLineEdit("65")
        self.z_front_start_target = QLineEdit("-1000")
        self.z_front_end_target = QLineEdit("1000")

        self.x_back_start_target = QLineEdit("-78")
        self.x_back_end_target = QLineEdit("10")
        self.y_back_start_target = QLineEdit("-71")
        self.y_back_end_target = QLineEdit("65")
        self.z_back_start_target = QLineEdit("-1000")
        self.z_back_end_target = QLineEdit("1000")

        self.x_start_source = QLineEdit("-78")
        self.x_end_source = QLineEdit("308")
        self.y_start_source = QLineEdit("-71")
        self.y_end_source = QLineEdit("65")
        self.z_start_source = QLineEdit("-1000")
        self.z_end_source = QLineEdit("1000")

        self.x_front_start_source = QLineEdit("200")
        self.x_front_end_source = QLineEdit("290")
        self.y_front_start_source = QLineEdit("-71")
        self.y_front_end_source = QLineEdit("65")
        self.z_front_start_source = QLineEdit("-1000")
        self.z_front_end_source = QLineEdit("1000")

        self.x_back_start_source = QLineEdit("-65")
        self.x_back_end_source = QLineEdit("0")
        self.y_back_start_source = QLineEdit("-71")
        self.y_back_end_source = QLineEdit("65")
        self.z_back_start_source = QLineEdit("-1000")
        self.z_back_end_source = QLineEdit("1000")

        self.simulated_button = QRadioButton("Geometry with 2 Layers")
        self.simulated_button.setChecked(True)
        self.measured_button = QRadioButton("Geometry with 1 Layer")

        self.align_twice = QCheckBox("Align Twice")

        self.u_sections = QLineEdit("3")
        self.v_sections = QLineEdit("5")

        self.simulated_button.clicked.connect(self.update_is_simulated)
        self.measured_button.clicked.connect(self.update_is_simulated)

        self.target_mesh = None
        self.grid_queue = []
        self.grid_queue_meshes = []

        self.is_simulated = None

        self.update_is_simulated()

        self.create_grid_button = QPushButton("Create Grid")
        self.get_aspect_ratio_button = QPushButton("Get Aspect Ratio")
        self.get_aspect_ratio_button.clicked.connect(self.get_aspect_ratio)

        self.target_tab = QWidget()
        self.source_tab = QWidget()

        self.thread = QThread()
        self.grid_worker = GridWorker()

        self.common_form_widget = QWidget()

        self.pick_points_target_button = QPushButton("Select Cut Points")
        self.pick_points_target_button.clicked.connect(self.select_cut_points)
        self.pick_points_source_button = QPushButton("Select Cut Points")
        self.pick_points_source_button.clicked.connect(self.select_cut_points)

        self.plot_cut_target_button = QPushButton("Preview Cut Geometry")
        self.plot_cut_target_button.clicked.connect(self.plot_cut_target)
        self.plot_cut_source_button = QPushButton("Preview Cut Geometry")
        self.plot_cut_source_button.clicked.connect(self.plot_cut_target)

        self.selected_point = []
        self.text = shapes.Text2D(str(self.selected_point))
        self.pltr = plotter.Plotter(title="select points")
        self.pltr.addCallback('mouse click', self.display_points_on_click)

        self.aspect_ratio = 0
        self.num_iterations = QLineEdit('5')

        self.aspect_ratio_label = QLabel("u / v = " + str(self.aspect_ratio))

        self.is_worker_set = False

        self.init_gui()

        self.main_layout.addWidget(self.aspect_ratio_label)
        self.main_layout.addWidget(self.get_aspect_ratio_button)
        self.main_layout.addWidget(self.create_grid_button)

    def init_gui(self):
        target_grid_form = QFormLayout()
        self.target_tab.setLayout(target_grid_form)
        self.tabs.addTab(self.target_tab, "Target")

        target_grid_form.addWidget(self.pick_points_target_button)
        target_grid_form.addWidget(self.plot_cut_target_button)

        target_grid_form.addRow("X Start:", self.x_start_target)
        target_grid_form.addRow("X End:", self.x_end_target)
        target_grid_form.addRow("Y Start:", self.y_start_target)
        target_grid_form.addRow("Y End:", self.y_end_target)
        target_grid_form.addRow("Z Start:", self.z_start_target)
        target_grid_form.addRow("Z End:", self.z_end_target)

        target_grid_form.addRow("X Front Start:", self.x_front_start_target)
        target_grid_form.addRow("X Front End:", self.x_front_end_target)
        target_grid_form.addRow("Y Front Start:", self.y_front_start_target)
        target_grid_form.addRow("Y Front End:", self.y_front_end_target)
        target_grid_form.addRow("Z Front Start:", self.z_front_start_target)
        target_grid_form.addRow("Z Front End:", self.z_front_end_target)

        target_grid_form.addRow("X Back Start:", self.x_back_start_target)
        target_grid_form.addRow("X Back End:", self.x_back_end_target)
        target_grid_form.addRow("Y Back Start:", self.y_back_start_target)
        target_grid_form.addRow("Y Back End:", self.y_back_end_target)
        target_grid_form.addRow("Z Back Start:", self.z_back_start_target)
        target_grid_form.addRow("Z Back End:", self.z_back_end_target)

        source_grid_form = QFormLayout()
        self.source_tab.setLayout(source_grid_form)
        self.tabs.addTab(self.source_tab, "Source")

        source_grid_form.addWidget(self.pick_points_source_button)
        source_grid_form.addWidget(self.plot_cut_source_button)

        source_grid_form.addRow("X Start:", self.x_start_source)
        source_grid_form.addRow("X End:", self.x_end_source)
        source_grid_form.addRow("Y Start:", self.y_start_source)
        source_grid_form.addRow("Y End:", self.y_end_source)
        source_grid_form.addRow("Z Start:", self.z_start_source)
        source_grid_form.addRow("Z End:", self.z_end_source)

        source_grid_form.addRow("X Front Start:", self.x_front_start_source)
        source_grid_form.addRow("X Front End:", self.x_front_end_source)
        source_grid_form.addRow("Y Front Start:", self.y_front_start_source)
        source_grid_form.addRow("Y Front End:", self.y_front_end_source)
        source_grid_form.addRow("Z Front Start:", self.z_front_start_source)
        source_grid_form.addRow("Z Front End:", self.z_front_end_source)

        source_grid_form.addRow("X Back Start:", self.x_back_start_source)
        source_grid_form.addRow("X Back End:", self.x_back_end_source)
        source_grid_form.addRow("Y Back Start:", self.y_back_start_source)
        source_grid_form.addRow("Y Back End:", self.y_back_end_source)
        source_grid_form.addRow("Z Back Start:", self.z_back_start_source)
        source_grid_form.addRow("Z Back End:", self.z_back_end_source)

        self.main_layout.addWidget(self.list_view)

        self.create_grid_button.clicked.connect(self.start_grid_generation)

        common_form_layout = QFormLayout()

        self.common_form_widget.setLayout(common_form_layout)
        common_form_layout.addRow(self.simulated_button)
        common_form_layout.addRow(self.measured_button)
        common_form_layout.addRow(self.align_twice)
        common_form_layout.addRow("Number of u Sections:", self.u_sections)
        common_form_layout.addRow("Number of v Sections:", self.v_sections)
        common_form_layout.addRow("Number of Iterations:", self.num_iterations)
        self.main_layout.addWidget(self.common_form_widget)

    def update_list(self):
        names = self.grid_queue[0]
        iterations = self.grid_queue[2]
        a = [name + " - Iteration " + iteration for name, iteration in zip(names, iterations)]
        self.list_view.clear()
        self.list_view.addItems(a)

    def update_is_simulated(self):
        if self.simulated_button.isChecked():
            self.is_simulated = "simulated"
        else:
            self.is_simulated = "measured"

    def setup_worker(self):
        self.grid_worker.moveToThread(self.thread)
        self.thread.started.connect(self.grid_worker.run)
        self.grid_worker.finished.connect(self.thread.quit)
        # self.grid_worker.finished.connect(self.grid_worker.deleteLater)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.grid_worker.progress.connect(self.pop_list)

        self.thread.finished.connect(
            lambda: self.create_grid_button.setEnabled(True)
        )
        self.thread.finished.connect(
            lambda: self.target_tab.setEnabled(True)
        )
        self.thread.finished.connect(
            lambda: self.source_tab.setEnabled(True)
        )
        self.thread.finished.connect(
            lambda: self.simulated_button.setEnabled(True)
        )
        self.thread.finished.connect(
            lambda: self.measured_button.setEnabled(True)
        )
        self.thread.finished.connect(
            lambda: self.common_form_widget.setEnabled(True)
        )
        self.is_worker_set = True

    def start_grid_generation(self):
        if self.target_mesh is None and self.grid_queue == []:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText("Missing Target and Source")
            msg.setInformativeText("Target and source STLs have to be imported and queued to create grid.")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        elif self.target_mesh is None and not self.grid_queue == []:
            display_target_error("Target STL have to be imported to create grid.")
            return
        elif self.target_mesh is not None and self.grid_queue == []:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText("Missing Source")
            msg.setInformativeText("Source STLs have to be imported and queued to create grid.")
            msg.setWindowTitle("Error")
            msg.exec_()
            return

        target_cut = [
            int(self.x_start_target.text()),
            int(self.x_end_target.text()),
            int(self.y_start_target.text()),
            int(self.y_end_target.text()),
            int(self.z_start_target.text()),
            int(self.z_end_target.text()),
        ]

        target_front_cut = [
            int(self.x_front_start_target.text()),
            int(self.x_front_end_target.text()),
            int(self.y_front_start_target.text()),
            int(self.y_front_end_target.text()),
            int(self.z_front_start_target.text()),
            int(self.z_front_end_target.text()),
        ]

        target_back_cut = [
            int(self.x_back_start_target.text()),
            int(self.x_back_end_target.text()),
            int(self.y_back_start_target.text()),
            int(self.y_back_end_target.text()),
            int(self.z_back_start_target.text()),
            int(self.z_back_end_target.text()),
        ]

        source_cut = [
            int(self.x_start_source.text()),
            int(self.x_end_source.text()),
            int(self.y_start_source.text()),
            int(self.y_end_source.text()),
            int(self.z_start_source.text()),
            int(self.z_end_source.text()),
        ]

        source_front_cut = [
            int(self.x_front_start_source.text()),
            int(self.x_front_end_source.text()),
            int(self.y_front_start_source.text()),
            int(self.y_front_end_source.text()),
            int(self.z_front_start_source.text()),
            int(self.z_front_end_source.text()),
        ]

        source_back_cut = [
            int(self.x_back_start_source.text()),
            int(self.x_back_end_source.text()),
            int(self.y_back_start_source.text()),
            int(self.y_back_end_source.text()),
            int(self.z_back_start_source.text()),
            int(self.z_back_end_source.text()),
        ]

        u_sections = int(self.u_sections.text())
        v_sections = int(self.v_sections.text())

        """
        edge1 = [int(self.x_start_target.text()) - 100, int(self.y_start_target.text()) - 100,
                 int(self.z_start_target.text())]
        edge2 = [int(self.x_end_target.text()) + 999999,
                 int(self.y_start_target.text()) - 999999,
                 int(self.z_start_target.text())]
        edge3 = [int(self.x_start_target.text()) - 999999,
                 int(self.y_end_target.text()) + 999999,
                 int(self.z_start_target.text())]
        edge4 = [int(self.x_end_target.text()) + 999999,
                 int(self.y_end_target.text()) + 999999,
                 int(self.z_start_target.text())]
        """

        edge1 = [int(self.x_start_target.text()) - 999999,
                 int(self.y_start_target.text()) - 999999,
                 int(self.z_start_target.text())]
        edge2 = [int(self.x_end_target.text()) + 999999,
                 int(self.y_start_target.text()) - 999999,
                 int(self.z_start_target.text())]
        edge3 = [int(self.x_start_target.text()) - 999999,
                 int(self.y_end_target.text()) + 999999,
                 int(self.z_start_target.text())]
        edge4 = [int(self.x_end_target.text()) + 999999,
                 int(self.y_end_target.text()) + 999999,
                 int(self.z_start_target.text())]

        self.grid_worker.setup_worker(
            target_mesh=self.target_mesh.clone(),
            grid_queue_meshes=self.grid_queue_meshes[:],
            grid_queue=self.grid_queue[:],
            target_cut=target_cut,
            target_front_cut=target_front_cut,
            target_back_cut=target_back_cut,
            source_cut=source_cut,
            source_back_cut=source_back_cut,
            source_front_cut=source_front_cut,
            is_simulated=self.is_simulated,
            u_sections=u_sections,
            v_sections=v_sections,
            edge1=edge1,
            edge2=edge2,
            edge3=edge3,
            edge4=edge4,
            num_iterations=int(self.num_iterations.text()),
            align_twice=self.align_twice.isChecked()
        )

        if not self.is_worker_set:
            self.setup_worker()
        self.thread.start()

        self.create_grid_button.setEnabled(False)
        self.target_tab.setEnabled(False)
        self.source_tab.setEnabled(False)
        self.simulated_button.setEnabled(False)
        self.measured_button.setEnabled(False)
        self.common_form_widget.setEnabled(False)

    @pyqtSlot()
    def pop_list(self):
        for grid_list in self.grid_queue:
            _ = grid_list.pop(0)
        self.grid_queue_meshes.pop(0)
        self.grid_queue_popped_signal.emit()

    def select_cut_points(self):
        if self.target_mesh is None:
            display_target_error("Target STL has to be imported.")
            return

        self.pltr = plotter.Plotter(title="select points")
        self.pltr.addCallback('mouse click', self.display_points_on_click)

        self.text = shapes.Text2D(str(self.selected_point))
        self.pltr.show(self.target_mesh, self.text, interactive=True)

    def display_points_on_click(self, evt):
        if not evt.actor:
            return
        point = evt.picked3d

        self.text = shapes.Text2D(str(self.target_mesh.closestPoint(point)))
        self.pltr.remove(self.pltr.actors[1])
        self.pltr.add(self.text)

    def get_aspect_ratio(self):
        if self.target_mesh is None:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText("Missing Target Geometry")
            msg.setInformativeText("Target STL has to be imported to calculate the aspect ratio.")
            msg.setWindowTitle("Error")
            msg.exec_()
            return

        target_cut = [
            int(self.x_start_target.text()),
            int(self.x_end_target.text()),
            int(self.y_start_target.text()),
            int(self.y_end_target.text()),
            int(self.z_start_target.text()),
            int(self.z_end_target.text()),
        ]

        tmp_mesh = self.target_mesh.clone()

        if self.is_simulated == "simulated":
            vertices = tmp_mesh.vertices()
            numVertices = len(vertices)
            halfPoint = numVertices // 2
            upperPart = vertices[halfPoint:]

            ms = pymeshlab.MeshSet()
            pc = pymeshlab.Mesh(upperPart)
            ms.add_mesh(pc)
            ms.generate_surface_reconstruction_ball_pivoting()
            currentMesh = ms.current_mesh()

            seperatedVertices = currentMesh.vertex_matrix()
            seperatedFaces = currentMesh.face_matrix()
            tmp_mesh = mesh.Mesh([seperatedVertices, seperatedFaces])

        tmp_mesh.cutWithBox(target_cut, invert=False)

        edge1 = [int(self.x_start_target.text()) - 999999,
                 int(self.y_start_target.text()) - 999999,
                 int(self.z_start_target.text())]
        edge2 = [int(self.x_end_target.text()) + 999999,
                 int(self.y_start_target.text()) - 999999,
                 int(self.z_start_target.text())]
        edge3 = [int(self.x_start_target.text()) - 999999,
                 int(self.y_end_target.text()) + 999999,
                 int(self.z_start_target.text())]
        edge4 = [int(self.x_end_target.text()) + 999999,
                 int(self.y_end_target.text()) + 999999,
                 int(self.z_start_target.text())]
        """
        edge1 = [int(self.x_start_target.text()) - 100, int(self.y_start_target.text()) - 100,
                 (int(self.z_start_target.text()) + int(self.z_start_target.text())) / 2]
        edge2 = [int(self.x_end_target.text()) + 100, int(self.y_start_target.text()) - 100,
                 (int(self.z_start_target.text()) + int(self.z_start_target.text())) / 2]
        edge3 = [int(self.x_start_target.text()) - 100, int(self.y_end_target.text()) + 100,
                 (int(self.z_start_target.text()) + int(self.z_start_target.text())) / 2]
        edge4 = [int(self.x_end_target.text()) + 100, int(self.y_end_target.text()) + 100,
                 (int(self.z_start_target.text()) + int(self.z_start_target.text())) / 2]
        """
        point1 = tmp_mesh.closestPoint(edge1, 1, returnPointId=True)
        point2 = tmp_mesh.closestPoint(edge2, 1, returnPointId=True)
        point3 = tmp_mesh.closestPoint(edge3, 1, returnPointId=True)
        point4 = tmp_mesh.closestPoint(edge4, 1, returnPointId=True)
        bordergeodesic1 = tmp_mesh.geodesic(point1, point2).color('blue')
        bordergeodesic2 = tmp_mesh.geodesic(point3, point4).color('blue')
        bordergeodesic3 = tmp_mesh.geodesic(point1, point3).color('blue')
        bordergeodesic4 = tmp_mesh.geodesic(point2, point4).color('blue')

        self.aspect_ratio, bg1, bg2 = calculate_aspect_ratio(bordergeodesic1, bordergeodesic2, bordergeodesic3,
                                                             bordergeodesic4)
        self.update_aspect_ratio_label()

        self.pltr = plotter.Plotter(title="Borders")
        self.pltr.show(bg1, bg2, tmp_mesh)

    def update_aspect_ratio_label(self):
        self.aspect_ratio_label.setText("u / v = " + str(self.aspect_ratio))

    def plot_cut_target(self):
        if self.target_mesh is None:
            display_target_error("Target STL has to be imported to plot.")
            return

        target_cut = [
            int(self.x_start_target.text()),
            int(self.x_end_target.text()),
            int(self.y_start_target.text()),
            int(self.y_end_target.text()),
            int(self.z_start_target.text()),
            int(self.z_end_target.text()),
        ]

        tmp_mesh = self.target_mesh.clone()

        if self.is_simulated == "simulated":
            vertices = tmp_mesh.vertices()
            numVertices = len(vertices)
            halfPoint = numVertices // 2
            upperPart = vertices[halfPoint:]

            ms = pymeshlab.MeshSet()
            pc = pymeshlab.Mesh(upperPart)
            ms.add_mesh(pc)
            ms.generate_surface_reconstruction_ball_pivoting()
            currentMesh = ms.current_mesh()

            seperatedVertices = currentMesh.vertex_matrix()
            seperatedFaces = currentMesh.face_matrix()
            tmp_mesh = mesh.Mesh([seperatedVertices, seperatedFaces])

        tmp_mesh.cutWithBox(target_cut, invert=False)

        self.pltr = plotter.Plotter(title="Cut Target")
        self.pltr.show(tmp_mesh)

