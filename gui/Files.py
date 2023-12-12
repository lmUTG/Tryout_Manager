import pymeshlab
from vedo import mesh
from functools import partial
import numpy as np
from PyQt5.QtWidgets import QWidget, QTreeView, QVBoxLayout, QAbstractItemView, QMenu
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import QFileInfo, Qt, pyqtSignal
from collections import deque
import json
from vedo import mesh, plotter, pointcloud


def create_data_element(unique_id, parent_id, name):
    element = {'unique_id': unique_id, 'parent_id': parent_id, 'name': name}
    return element


class Files(QWidget):
    grid_queue_changed_signal = pyqtSignal()
    surface_fit_queue_changed_signal = pyqtSignal()

    def __init__(self):
        super(Files, self).__init__()
        self.data = [
            {'unique_id': "root", 'parent_id': 0, 'name': 'Data'},
            {'unique_id': "targetmesh", 'parent_id': "root", 'name': 'Target Mesh'},
            {'unique_id': "sourcemesh", 'parent_id': "root", 'name': 'Source Mesh'},
            {'unique_id': "targetgrid", 'parent_id': "root", 'name': 'Target Grid'},
            {'unique_id': "sourcegrid", 'parent_id': "root", 'name': 'Source Grid'},
        ]

        self.tree = QTreeView(self)
        self.tree.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self.open_menu)
        self.tree.doubleClicked.connect(self.plot_item)

        tree_layout = QVBoxLayout()
        self.setLayout(tree_layout)
        tree_layout.addWidget(self.tree)

        self.model = QStandardItemModel()
        self.model.setHorizontalHeaderLabels(['Name', 'id'])
        self.tree.setModel(self.model)
        self.importData()

        # vedo mesh.Mesh object
        self.target_mesh = None
        self.target_mesh_path = None

        # path -> mesh
        self.source_meshes = {}

        self.target_grid = None
        self.target_grid_path = None

        # iteration -> path -> grid
        self.source_grids = {}

        # item -> unique_id
        self.item_to_id = {}

        # list of name list, path list and iteration list
        self.grid_queue = [[], [], []]

        # list of name list, path list
        self.surface_fit_queue = [[], []]

        self.pltr = plotter.Plotter()

    def update_data(self):
        newdata = [
            {'unique_id': "root", 'parent_id': 0, 'name': 'Data'},
            {'unique_id': "targetmesh", 'parent_id': "root", 'name': 'Target Mesh'},
            {'unique_id': "sourcemesh", 'parent_id': "root", 'name': 'Source Mesh'},
            {'unique_id': "targetgrid", 'parent_id': "root", 'name': 'Target Grid'},
            {'unique_id': "sourcegrid", 'parent_id': "root", 'name': 'Source Grid'},
        ]

        # add target mesh
        if self.target_mesh is not None and self.target_mesh_path is not None:
            finfo = QFileInfo(self.target_mesh_path)
            element = create_data_element(unique_id=self.target_mesh_path, parent_id="targetmesh",
                                          name=finfo.fileName())
            newdata.append(element)

        # add target grid
        if self.target_grid is not None and self.target_grid_path is not None:
            finfo = QFileInfo(self.target_grid_path)
            element = create_data_element(unique_id=self.target_grid_path, parent_id="targetgrid",
                                          name=finfo.fileName())
            newdata.append(element)

        # add source meshes
        seen_iteration_source_mesh = []
        for iteration in self.source_meshes.keys():
            if iteration not in seen_iteration_source_mesh:
                seen_iteration_source_mesh.append(iteration)
                iteration_element = create_data_element(unique_id="source_mesh_iteration_" + str(iteration),
                                                        parent_id="sourcemesh", name="Iteration " + str(iteration))
                newdata.append(iteration_element)
            for source_mesh_path in self.source_meshes[iteration].keys():
                finfo = QFileInfo(source_mesh_path)
                element = create_data_element(unique_id=source_mesh_path,
                                              parent_id="source_mesh_iteration_" + str(iteration),
                                              name=finfo.fileName())
                newdata.append(element)

        # add source grids
        seen_iteration_source_grid = []
        for iteration in self.source_grids.keys():
            if iteration not in seen_iteration_source_grid:
                seen_iteration_source_grid.append(iteration)
                iteration_element = create_data_element(unique_id="source_grid_iteration_" + str(iteration),
                                                        parent_id="sourcegrid", name="Iteration " + str(iteration))
                newdata.append(iteration_element)
            for source_grid_path in self.source_grids[iteration].keys():
                finfo = QFileInfo(source_grid_path)
                element = create_data_element(unique_id=source_grid_path,
                                              parent_id="source_grid_iteration_" + str(iteration),
                                              name=finfo.fileName())
                newdata.append(element)

        self.data = newdata
        self.importData()

    def importData(self, root=None):
        self.model.setRowCount(0)
        if root is None:
            root = self.model.invisibleRootItem()
        seen = {}  # List of  QStandardItem
        values = deque(self.data)
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

    def set_target_mesh(self, file_path):
        self.target_mesh = mesh.Mesh(file_path)
        self.target_mesh_path = file_path
        self.update_data()

    def set_source_meshes(self, file_paths, iteration):
        tmp = {}
        source_meshes_to_add = {}

        for file_path in file_paths:
            tmp[file_path] = mesh.Mesh(file_path)
        source_meshes_to_add[iteration] = tmp

        for it in source_meshes_to_add.keys():
            if it in self.source_meshes.keys():
                self.source_meshes[it] = self.source_meshes[it] | source_meshes_to_add[it]
            else:
                self.source_meshes[it] = source_meshes_to_add[it]
        self.update_data()

    def set_target_grid(self, file_path):
        f = open(file_path)
        self.target_grid = json.load(f)
        self.target_grid_path = file_path
        self.update_data()

    def set_source_grids(self, file_paths, iteration):
        tmp = {}
        source_grids_to_add = {}
        for file_path in file_paths:
            f = open(file_path)
            tmp[file_path] = json.load(f)
        source_grids_to_add[iteration] = tmp

        for it in source_grids_to_add.keys():
            if it in self.source_grids.keys():
                self.source_grids[it] = self.source_grids[it] | source_grids_to_add[it]
            else:
                self.source_grids[it] = source_grids_to_add[it]
        self.update_data()

    def open_menu(self, position):
        data = [item.data() for item in self.tree.selectedIndexes()]
        parents = [item.parent().data() for item in self.tree.selectedIndexes()]
        for parent in parents:
            if parent is None:
                return
        iterations = [parents[i][10:] for i in range(0, len(parents), 2)]
        names = [data[i] for i in range(0, len(data), 2)]
        ids = [data[i] for i in range(1, len(data), 2)]
        items = (names, ids, iterations)

        is_grid = True
        is_stl = True
        for name in names:
            if not name[-5:] == ".json":
                is_grid = False
            if not name[-4:] == ".stl":
                is_stl = False

        right_click_menu = QMenu()

        if is_stl:
            add_grid_queue_action = right_click_menu.addAction(self.tr("Add to Grid Queue"))
            add_grid_queue_action.triggered.connect(partial(self.add_to_grid_queue, items))

            split_geometry_action = right_click_menu.addAction(self.tr("Get Upper Part"))
            split_geometry_action.triggered.connect(partial(self.split_geometry_up, items))

            split_geometry_action = right_click_menu.addAction(self.tr("Get Lower Part"))
            split_geometry_action.triggered.connect(partial(self.split_geometry_down, items))
        elif is_grid:
            add_surface_fit_queue_action = right_click_menu.addAction(self.tr("Add to Surface Fit Queue"))
            add_surface_fit_queue_action.triggered.connect(partial(self.add_to_surface_fit_queue, items))

        remove_items_action = right_click_menu.addAction(self.tr("Remove"))
        remove_items_action.triggered.connect(partial(self.remove_items, items))

        right_click_menu.exec_(self.sender().viewport().mapToGlobal(position))

    def add_to_grid_queue(self, items):
        self.grid_queue[0] += items[0]
        self.grid_queue[1] += items[1]
        self.grid_queue[2] += items[2]
        self.grid_queue_changed_signal.emit()

    def add_to_surface_fit_queue(self, items):
        self.surface_fit_queue[0] += items[0]
        self.surface_fit_queue[1] += items[1]
        self.surface_fit_queue_changed_signal.emit()

    def remove_items(self, items):
        names = items[0]
        paths = items[1]
        parents = items[2]

        for item_name, item_path, item_parent in zip(names, paths, parents):
            if item_name[-5:] == ".json":
                if self.target_grid_path == item_path:
                    self.target_grid = None
                    self.target_grid_path = None
                else:
                    it = self.source_grids[item_parent]
                    if item_path in it:
                        it.pop(item_path)
                    if len(it) == 0:
                        self.source_grids.pop(item_parent)
            elif item_name[-4:] == ".stl":
                if self.target_mesh_path == item_path:
                    self.target_mesh = None
                    self.target_mesh_path = None
                else:
                    it = self.source_meshes[item_parent]
                    if item_path in it:
                        it.pop(item_path)
                    if len(it) == 0:
                        self.source_meshes.pop(item_parent)
        self.update_data()

    def plot_item(self):
        name = self.tree.selectedIndexes()[0].data()
        path = self.tree.selectedIndexes()[1].data()

        self.pltr = plotter.Plotter()

        if name[-4:] == ".stl":
            selected_mesh = mesh.Mesh(path)
            self.pltr.show(selected_mesh)
        elif name[-5:] == ".json":
            f = open(path)
            grid_dict = json.load(f)
            if 'points' in grid_dict:
                points = grid_dict['points']
                pc = pointcloud.Points(points)
                self.pltr.show(pc)

    def split_geometry_up(self, items):
        names = items[0]
        paths = items[1]
        two_sided_mesh = mesh.Mesh(paths[0])
        vertices = two_sided_mesh.vertices()
        num_vertices = len(vertices)
        half_point = num_vertices // 2

        upper_part = vertices[half_point:]

        ms = pymeshlab.MeshSet()
        pc = pymeshlab.Mesh(upper_part)
        ms.add_mesh(pc)
        ms.generate_surface_reconstruction_ball_pivoting()
        current_mesh = ms.current_mesh()

        seperated_vertices = current_mesh.vertex_matrix()
        seperated_faces = current_mesh.face_matrix()

        upper_mesh = mesh.Mesh([seperated_vertices, seperated_faces]).to_trimesh()
        upper_mesh.export("gen/mesh/" + str(names[0]) + "_upper_part.stl")

    def split_geometry_down(self, items):
        names = items[0]
        paths = items[1]
        two_sided_mesh = mesh.Mesh(paths[0])
        vertices = two_sided_mesh.vertices()
        num_vertices = len(vertices)
        half_point = num_vertices // 2

        lower_part = vertices[:half_point]

        ms = pymeshlab.MeshSet()
        pc = pymeshlab.Mesh(lower_part)
        ms.add_mesh(pc)
        ms.generate_surface_reconstruction_ball_pivoting()
        current_mesh = ms.current_mesh()

        seperated_vertices = current_mesh.vertex_matrix()
        seperated_faces = current_mesh.face_matrix()

        lower_mesh = mesh.Mesh([seperated_vertices, seperated_faces]).to_trimesh()
        lower_mesh.export("gen/mesh/" + str(names[0]) + "_lower_part.stl")

