from vedo import plotter
from functools import partial
from geomdl import BSpline
from geomdl.visualization import VisVTK
import numpy as np
from PyQt5.QtWidgets import QWidget, QTreeView, QVBoxLayout, QAbstractItemView, QMenu
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import QFileInfo, Qt, pyqtSignal
from collections import deque
from gui.Files import create_data_element
from stp2surf import Stp2Surf
import json


class Surfaces(QWidget):
    compensation_list_changed_signal = pyqtSignal()
    target_surface_changed_signal = pyqtSignal()
    tool_surface_changed_signal = pyqtSignal()
    simulated_surfaces_changed_signal = pyqtSignal()
    measured_surfaces_changed_signal = pyqtSignal()

    def __init__(self):
        super(Surfaces, self).__init__()

        self.target_surface = None
        self.target_surface_path = ""
        self.simulated_surfaces = {}
        self.measured_surfaces = {}
        self.common_knotvector_u = None
        self.common_knotvector_v = None

        self.tool_surface = None
        self.tool_surface_path = ""

        self.compensation_list = []

        self.degree_u = None
        self.degree_v = None

        self.num_ctrlpts_u = 54
        self.num_ctrlpts_v = 84

        self.data = [
            {'unique_id': "root", 'parent_id': 0, 'name': 'Data'},
            {'unique_id': "targetsurface", 'parent_id': "root", 'name': 'Target Surface'},
            {'unique_id': "toolsurface", 'parent_id': "root", 'name': 'Tool Surface'},
            {'unique_id': "simulatedsurface", 'parent_id': "root", 'name': 'Simulated Surfaces'},
            {'unique_id': "measuredsurface", 'parent_id': "root", 'name': 'Measured Surfaces'},
        ]

        self.tree = QTreeView(self)
        self.tree.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self.open_menu)
        self.tree.doubleClicked.connect(self.plot_item)

        self.pltr = plotter.Plotter()

        tree_layout = QVBoxLayout()
        self.setLayout(tree_layout)
        tree_layout.addWidget(self.tree)

        self.model = QStandardItemModel()
        self.model.setHorizontalHeaderLabels(['Name', 'id'])
        self.tree.setModel(self.model)
        self.importData()

    def importData(self, root=None):
        data = self.data
        self.model.setRowCount(0)
        if root is None:
            root = self.model.invisibleRootItem()
        seen = {}  # List of  QStandardItem
        values = deque(data)
        while values:
            value = values.popleft()
            if value['unique_id'] == "root":
                parent = root
            else:
                pid = value['parent_id']
                if pid not in seen:
                    values.append(value)
                    continue
                parent = seen[pid]
            unique_id = value['unique_id']
            name = QStandardItem(value['name'])
            name.setEditable(False)
            item_id = QStandardItem(unique_id)
            item_id.setEditable(False)
            parent.appendRow([
                name,
                item_id
            ])
            seen[unique_id] = parent.child(parent.rowCount() - 1)

    def update_data(self):
        new_data = [
            {'unique_id': "root", 'parent_id': 0, 'name': 'Data'},
            {'unique_id': "targetsurface", 'parent_id': "root", 'name': 'Target Surface'},
            {'unique_id': "toolsurface", 'parent_id': "root", 'name': 'Tool Surface'},
            {'unique_id': "simulatedsurface", 'parent_id': "root", 'name': 'Simulated Surfaces'},
            {'unique_id': "measuredsurface", 'parent_id': "root", 'name': 'Measured Surfaces'},
        ]

        if self.target_surface is not None:
            finfo = QFileInfo(self.target_surface_path)
            element = create_data_element(unique_id=self.target_surface_path, parent_id="targetsurface",
                                          name=finfo.fileName())
            new_data.append(element)

        if self.tool_surface is not None:
            finfo = QFileInfo(self.tool_surface_path)
            element = create_data_element(unique_id=self.tool_surface_path, parent_id="toolsurface",
                                          name=finfo.fileName())
            new_data.append(element)

        seen_iteration_simulated_surface = list()
        for iteration in self.simulated_surfaces.keys():
            if iteration not in seen_iteration_simulated_surface:
                seen_iteration_simulated_surface.append(iteration)
                iteration_element = create_data_element(unique_id="simulated_surface_iteration_" + str(iteration),
                                                        parent_id="simulatedsurface",
                                                        name="Iteration " + str(iteration))
                new_data.append(iteration_element)
            for simulated_surface_path in self.simulated_surfaces[iteration].keys():
                finfo = QFileInfo(simulated_surface_path)
                element = create_data_element(unique_id=simulated_surface_path,
                                              parent_id="simulated_surface_iteration_" + str(iteration),
                                              name=finfo.fileName())
                new_data.append(element)

        seen_iteration_measured_surface = list()
        for iteration in self.measured_surfaces.keys():
            if iteration not in seen_iteration_measured_surface:
                seen_iteration_measured_surface.append(iteration)
                iteration_element = create_data_element(unique_id="measured_surface_iteration_" + str(iteration),
                                                        parent_id="measuredsurface", name="Iteration " + str(iteration))
                new_data.append(iteration_element)
            for measured_surface_path in self.measured_surfaces[iteration].keys():
                finfo = QFileInfo(measured_surface_path)
                element = create_data_element(unique_id=measured_surface_path,
                                              parent_id="measured_surface_iteration_" + str(iteration),
                                              name=finfo.fileName())
                new_data.append(element)

        self.data = new_data
        self.importData()

    def set_target_surface(self, file_path):
        self.target_surface_path = file_path
        file = open(file_path)
        data = json.load(file)

        self.degree_u = data['degree_u']
        self.degree_v = data['degree_v']

        self.num_ctrlpts_u = data['size_u']
        self.num_ctrlpts_v = data['size_v']

        surf = BSpline.Surface()
        surf.degree_u = self.degree_u
        surf.degree_v = self.degree_v
        surf.ctrlpts_size_u = self.num_ctrlpts_u
        surf.ctrlpts_size_v = self.num_ctrlpts_v

        self.common_knotvector_u = data['knotvector_u']
        self.common_knotvector_v = data['knotvector_v']

        surf.knotvector_u = self.common_knotvector_u
        surf.knotvector_v = self.common_knotvector_v
        surf.ctrlpts = data['ctrlpts']
        self.target_surface = surf
        self.update_data()
        self.target_surface_changed_signal.emit()

    def set_tool_surface(self, file_path):
        self.tool_surface_path = file_path
        if file_path[-5:] == ".json":
            file = open(file_path)
            data = json.load(file)

            surf = BSpline.Surface()
            surf.degree_u = self.degree_u
            surf.degree_v = self.degree_v
            surf.ctrlpts_size_u = self.num_ctrlpts_u
            surf.ctrlpts_size_v = self.num_ctrlpts_v

            surf.knotvector_u = self.common_knotvector_u
            surf.knotvector_v = self.common_knotvector_v
            surf.ctrlpts = data["ctrlpts"]

            self.tool_surface = surf
        elif file_path[-4:] == ".stp":
            self.tool_surface = Stp2Surf(file_path).surfaces[0]
        self.update_data()
        self.tool_surface_changed_signal.emit()

    def set_simulated_surfaces(self, file_paths, iteration):
        tmp = {}
        for path in file_paths:
            file = open(path)
            data = json.load(file)

            surf = BSpline.Surface()
            surf.degree_u = self.degree_u
            surf.degree_v = self.degree_v
            surf.ctrlpts_size_u = self.num_ctrlpts_u
            surf.ctrlpts_size_v = self.num_ctrlpts_v

            surf.knotvector_u = self.common_knotvector_u
            surf.knotvector_v = self.common_knotvector_v
            surf.ctrlpts = data['ctrlpts']

            tmp[path] = surf
        self.simulated_surfaces[iteration] = tmp
        self.update_data()
        self.simulated_surfaces_changed_signal.emit()

    def set_measured_surfaces(self, file_paths, iteration):
        tmp = {}
        for path in file_paths:
            file = open(path)
            data = json.load(file)

            surf = BSpline.Surface()
            surf.degree_u = self.degree_u
            surf.degree_v = self.degree_v
            surf.ctrlpts_size_u = self.num_ctrlpts_u
            surf.ctrlpts_size_v = self.num_ctrlpts_v

            surf.knotvector_u = self.common_knotvector_u
            surf.knotvector_v = self.common_knotvector_v
            surf.ctrlpts = data['ctrlpts']

            tmp[path] = surf
        self.measured_surfaces[iteration] = tmp
        self.update_data()
        self.measured_surfaces_changed_signal.emit()

    def set_num_ctrlpts_u(self, num):
        self.num_ctrlpts_u = num

    def set_num_ctrlpts_v(self, num):
        self.num_ctrlpts_v = num

    def set_degree_u(self, degree):
        self.degree_u = degree

    def set_degree_v(self, degree):
        self.degree_v = degree

    def add_to_compensation_list(self, surfaces):
        paths = surfaces[1]
        names = surfaces[0]
        iterations = surfaces[2]
        sim_or_measured = surfaces[3]

        fit_surfaces_ctrlpts = []
        for path, iteration, s_or_m in zip(paths, iterations, sim_or_measured):
            if s_or_m == "Simulated Surfaces":
                fit_surfaces_ctrlpts.append(self.simulated_surfaces[iteration][path].ctrlpts)
            elif s_or_m == "Measured Surfaces":
                fit_surfaces_ctrlpts.append(self.measured_surfaces[iteration][path].ctrlpts)
        self.compensation_list += [names, fit_surfaces_ctrlpts]
        self.compensation_list_changed_signal.emit()

    def open_menu(self, position):
        data = [item.data() for item in self.tree.selectedIndexes()]
        parents = [item.parent().data() for item in self.tree.selectedIndexes()]
        for parent in parents:
            if parent is None:
                return
        parent_of_parent = [item.parent().parent().data() for item in self.tree.selectedIndexes()]
        iterations = [parents[i][10:] for i in range(0, len(parents), 2)]
        names = [data[i] for i in range(0, len(data), 2)]
        ids = [data[i] for i in range(1, len(data), 2)]
        items = (names, ids, iterations, parent_of_parent)

        right_click_menu = QMenu()
        add_source_surface_action = right_click_menu.addAction(self.tr("Add To Compensation List"))
        add_source_surface_action.triggered.connect(partial(self.add_to_compensation_list, items))

        remove_items_action = right_click_menu.addAction(self.tr("Remove"))
        remove_items_action.triggered.connect(partial(self.remove_items, items))

        right_click_menu.exec_(self.sender().viewport().mapToGlobal(position))

    def remove_items(self, items):
        names = items[0]
        paths = items[1]
        parents = items[2]

        for item_name, item_path, item_parent in zip(names, paths, parents):
            if item_path == self.target_surface_path:
                self.target_surface = None
                self.target_surface_path = ""
                self.update_data()
                self.target_surface_changed_signal.emit()
            elif item_path == self.tool_surface_path:
                self.tool_surface = None
                self.tool_surface_path = ""
                self.update_data()
                self.tool_surface_changed_signal.emit()
            elif item_name[-5:] == ".json":
                it = self.simulated_surfaces[item_parent]
                if item_path in it:
                    it.pop(item_path)
                if len(it) == 0:
                    self.simulated_surfaces.pop(item_parent)

                it = self.measured_surfaces[item_parent]
                if item_path in it:
                    it.pop(item_path)
                if len(it) == 0:
                    self.measured_surfaces.pop(item_parent)
        self.update_data()

    def plot_item(self):
        name = self.tree.selectedIndexes()[0].data()
        path = self.tree.selectedIndexes()[1].data()

        if name[-5:] == ".json":
            f = open(path)
            data = json.load(f)
            if 'ctrlpts' in data:
                surf = BSpline.Surface()
                surf.degree_u = self.degree_u
                surf.degree_v = self.degree_v
                surf.ctrlpts_size_u = self.num_ctrlpts_u
                surf.ctrlpts_size_v = self.num_ctrlpts_v

                surf.knotvector_u = self.common_knotvector_u
                surf.knotvector_v = self.common_knotvector_v
                surf.ctrlpts = data['ctrlpts']
                surf.delta = 0.01
                surf.vis = VisVTK.VisSurface()
                surf.render()
