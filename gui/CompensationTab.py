from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtCore import pyqtSlot
from gui.Surfaces import *
from gui.CompensationForm import *
from gui.Plotter import *


class CompensationTab(QWidget):
    def __init__(self):
        super(CompensationTab, self).__init__()

        with open("gui/style.stylesheet", "r") as fh:
            self.setStyleSheet(fh.read())

        self.layout = QHBoxLayout()
        self.setLayout(self.layout)
        self.surfaces = Surfaces()
        self.compensation_form = CompensationForm()
        self.plotter = Plotter()

        self.surfaces.compensation_list_changed_signal.connect(self.update_compensation_list)
        self.surfaces.target_surface_changed_signal.connect(self.update_target_surface)
        self.surfaces.tool_surface_changed_signal.connect(self.update_tool_surface)
        self.compensation_form.list_cleared_signal.connect(self.clear_list)
        self.surfaces.simulated_surfaces_changed_signal.connect(self.update_simulated_surfaces)
        self.surfaces.measured_surfaces_changed_signal.connect(self.update_measured_surfaces)

        self.init_gui()

    def init_gui(self):
        self.layout.addWidget(self.surfaces)
        self.layout.addWidget(self.compensation_form)
        self.layout.addWidget(self.plotter)

    @pyqtSlot()
    def update_compensation_list(self):
        self.compensation_form.compensation_list = self.surfaces.compensation_list
        self.compensation_form.update_list()

    @pyqtSlot()
    def update_target_surface(self):
        self.compensation_form.target_surface = self.surfaces.target_surface
        self.plotter.target_surface = self.surfaces.target_surface

    @pyqtSlot()
    def update_tool_surface(self):
        self.compensation_form.tool_surface = self.surfaces.tool_surface

    @pyqtSlot()
    def update_simulated_surfaces(self):
        self.plotter.simulated_surfaces = self.surfaces.simulated_surfaces

    @pyqtSlot()
    def update_measured_surfaces(self):
        self.plotter.measured_surfaces = self.surfaces.measured_surfaces

    @pyqtSlot()
    def clear_list(self):
        self.compensation_form.compensation_list.clear()
        self.surfaces.compensation_list.clear()
        self.compensation_form.list_view.clear()
