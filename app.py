from PyQt5.QtWidgets import QMainWindow, QAction, QApplication, QHBoxLayout, QTabWidget
from PyQt5.QtCore import pyqtSlot
from gui.FileDialog import *
from gui.Files import *
from gui.Surfaces import *
from gui.GridForm import *
from gui.SurfaceFitForm import *
from gui.CompensationTab import *
from gui.OptimizationForm import *
import sys
from pathlib import Path


class Tryout(QMainWindow):

    def __init__(self):
        super().__init__()

        with open("gui/style.stylesheet", "r") as fh:
            self.setStyleSheet(fh.read())

        Path("./gen/compensated_surfaces").mkdir(parents=True, exist_ok=True)
        Path("./gen/gridpoints").mkdir(parents=True, exist_ok=True)
        Path("./gen/mesh").mkdir(parents=True, exist_ok=True)
        Path("./gen/surfaces").mkdir(parents=True, exist_ok=True)

        self.main_widget = QTabWidget()
        self.hbox_layout_setup = QHBoxLayout()
        self.compensation_tab = CompensationTab()
        self.optimization_form = OptimizationForm()

        self.file_tracker = Files()
        self.file_tracker.grid_queue_changed_signal.connect(self.update_grid_queue)
        self.file_tracker.surface_fit_queue_changed_signal.connect(self.update_surface_fit_queue)

        self.grid_form = GridForm()
        self.grid_form.grid_queue_popped_signal.connect(self.pop_grid_queue)
        self.grid_queue_meshes = []
        self.surface_form = SurfaceFitForm()
        self.surface_form.grid_queue_popped_signal.connect(self.pop_surface_fit_queue)

        self.compensation_tab.surfaces.target_surface_changed_signal.connect(self.update_surface_menu)
        self.init_ui()

    def init_ui(self):
        self.update_surface_menu()

        self.hbox_layout_setup.addWidget(self.file_tracker)

        self.hbox_layout_setup.addWidget(self.grid_form)

        surface_column = QVBoxLayout()
        surface_column.addWidget(self.optimization_form)
        surface_column.addWidget(self.surface_form)
        self.hbox_layout_setup.addLayout(surface_column)

        setup_tab = QWidget()

        setup_tab.setLayout(self.hbox_layout_setup)
        self.main_widget.addTab(setup_tab, "Setup")
        self.main_widget.addTab(self.compensation_tab, "Compensation")
        self.setCentralWidget(self.main_widget)
        self.setGeometry(100, 100, 1400, 800)
        self.setWindowTitle('Tryout')
        self.show()

    @pyqtSlot()
    def update_surface_menu(self):
        menubar = self.menuBar()
        menubar.clear()
        fileMenu = menubar.addMenu('File')

        importStlMenu = QMenu("Import STL", self)
        importGridMenu = QMenu("Import Grid", self)
        importSurfaceMenu = QMenu("Import Surface", self)
        importTargetStlAct = QAction('Import Target STL', self)
        importSourceStlAct = QAction('Import Source STLs', self)
        importTargetGridAct = QAction('Import Target Grid', self)
        importSourceGridAct = QAction('Import Source Grids', self)
        importTargetSurfaceAct = QAction('Import Target Surface', self)
        importToolSurfaceAct = QAction('Import Tool Surface', self)
        importSimulatedSurfaceAct = QAction('Import Simulated Surfaces', self)
        import_measured_surface_act = QAction('Import Measured Surfaces', self)
        importTargetStlAct.triggered.connect(lambda: self.set_target_mesh(0, "Target STL",
                                                                          "STL Files (*.stl)"))
        importSourceStlAct.triggered.connect(lambda: self.set_source_meshes(1, "Source STL",
                                                                            "STL Files (*.stl)"))
        importTargetGridAct.triggered.connect(lambda: self.set_target_grid(0, "Target Grid",
                                                                           "JSON Files (*.json)"))
        importSourceGridAct.triggered.connect(lambda: self.set_source_grids(1, "Source Grid",
                                                                            "JSON Files (*.json)"))
        importTargetSurfaceAct.triggered.connect(lambda: self.set_target_surface(0, "Target Surface",
                                                                                 "JSON Files (*.json)"))
        importToolSurfaceAct.triggered.connect(lambda: self.set_tool_surface(0, "Target Surface",
                                                                             "Surfaces (*.json *.stp)"))
        importSimulatedSurfaceAct.triggered.connect(lambda: self.set_simulated_surfaces(1, "Simulated Surface",
                                                                                        "JSON Files (*.json)"))
        import_measured_surface_act.triggered.connect(lambda: self.set_measured_surfaces(1, "Measured Surface",
                                                                                         "JSON Files (*.json)"))
        importStlMenu.addAction(importTargetStlAct)
        importStlMenu.addAction(importSourceStlAct)
        importGridMenu.addAction(importTargetGridAct)
        importGridMenu.addAction(importSourceGridAct)
        importSurfaceMenu.addAction(importTargetSurfaceAct)
        if self.compensation_tab.surfaces.target_surface is not None:
            importSurfaceMenu.addAction(importToolSurfaceAct)
            importSurfaceMenu.addAction(importSimulatedSurfaceAct)
            importSurfaceMenu.addAction(import_measured_surface_act)

        fileMenu.addMenu(importStlMenu)
        fileMenu.addMenu(importGridMenu)
        fileMenu.addMenu(importSurfaceMenu)

    def set_target_surface(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, False)
        if fd.selected:
            self.compensation_tab.surfaces.set_target_surface(fd.selected)
            self.surface_form.update_target_values(fd.selected)
            self.surface_form.common_knotvector_u = self.compensation_tab.surfaces.target_surface.knotvector_u
            self.surface_form.common_knotvector_v = self.compensation_tab.surfaces.target_surface.knotvector_v
            self.compensation_tab.plotter.set_deviations_calculated(False)
            self.update_surface_menu()

    def set_tool_surface(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, False)
        if fd.selected:
            self.compensation_tab.surfaces.set_tool_surface(fd.selected)
            self.compensation_tab.plotter.set_deviations_calculated(False)

    def set_simulated_surfaces(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, True)
        if fd.selected:
            self.compensation_tab.surfaces.set_simulated_surfaces(fd.selected, fd.iteration)
            self.compensation_tab.plotter.set_deviations_calculated(False)

    def set_measured_surfaces(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, True)
        if fd.selected:
            self.compensation_tab.surfaces.set_measured_surfaces(fd.selected, fd.iteration)
            self.compensation_tab.plotter.set_deviations_calculated(False)

    def set_target_mesh(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, False)
        if fd.selected:
            self.file_tracker.set_target_mesh(fd.selected)
            self.grid_form.target_mesh = self.file_tracker.target_mesh
            self.compensation_tab.plotter.target_stl = self.file_tracker.target_mesh
            self.optimization_form.set_target_mesh(fd.selected)

    def set_source_meshes(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, True)
        if fd.selected:
            self.file_tracker.set_source_meshes(fd.selected, fd.iteration)

    def set_target_grid(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, False)
        if fd.selected:
            self.file_tracker.set_target_grid(fd.selected)
            self.optimization_form.set_target_grid(self.file_tracker.target_grid)

    def set_source_grids(self, option, title, extensions):
        fd = FileDialog(option, title, extensions, True)
        if fd.selected:
            self.file_tracker.set_source_grids(fd.selected, fd.iteration)

    @pyqtSlot()
    def update_grid_queue(self):
        self.grid_form.grid_queue = self.file_tracker.grid_queue
        paths = self.grid_form.grid_queue[1]
        iterations = self.grid_form.grid_queue[2]
        self.grid_queue_meshes = [
            self.file_tracker.source_meshes[i][p] if (not i == "h") else self.file_tracker.target_mesh for p, i in
            zip(paths, iterations)]
        self.grid_form.grid_queue_meshes = self.grid_queue_meshes
        self.grid_form.update_list()

    @pyqtSlot()
    def pop_grid_queue(self):
        self.file_tracker.grid_queue = self.grid_form.grid_queue
        self.grid_form.update_list()

    @pyqtSlot()
    def update_surface_fit_queue(self):
        self.surface_form.surface_fit_queue = self.file_tracker.surface_fit_queue
        self.surface_form.update_list()

    @pyqtSlot()
    def pop_surface_fit_queue(self):
        self.file_tracker.surface_fit_queue = self.surface_form.surface_fit_queue
        self.surface_form.update_list()


def main():
    app = QApplication(sys.argv)
    ex = Tryout()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
