from PyQt5.QtCore import QObject, pyqtSignal
from geomdl import fitting
from utilities import approximate_surface_with_knotvector
import json
import os
from pathlib import Path


class SurfaceFitWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal()

    def __init__(self):
        super(SurfaceFitWorker, self).__init__()
        self.size_u = None
        self.size_v = None
        self.degree_u = 3
        self.degree_v = 3
        self.ctrlpts_size_u = None
        self.ctrlpts_size_v = None
        self.grid_points = None
        self.export_name = "surface"

        self.size_u_arr = []
        self.size_v_arr = []
        self.grid_points_arr = []
        self.degree_u_arr = []
        self.degree_v_arr = []
        self.ctrlpts_size_u_arr = []
        self.ctrlpts_size_v_arr = []
        self.names_arr = []

        self.common_knotvector_u = []
        self.common_knotvector_v = []

        self.is_target = False

    def set_size_u(self, size_u):
        self.size_u = size_u

    def set_size_v(self, size_v):
        self.size_v = size_v

    def set_degree_u(self, degree_u):
        self.degree_u = degree_u

    def set_degree_v(self, degree_v):
        self.degree_v = degree_v

    def set_ctrlpts_size_u(self, size):
        self.ctrlpts_size_u = size

    def set_ctrlpts_size_v(self, size):
        self.ctrlpts_size_v = size

    def set_export_name(self, name):
        self.export_name = name

    def export_surface(self, size_u, size_v, degree_u, degree_v, knotvector_u, knotvector_v, ctrlpts, file_name):
        surface_dict = {"size_u": size_u, "size_v": size_v, "degree_u": degree_u, "degree_v": degree_v,
                        "knotvector_u": knotvector_u, "knotvector_v": knotvector_v, "ctrlpts": ctrlpts}
        if not os.path.exists('gen/surfaces'):
            Path("gen/surfaces").mkdir(parents=True, exist_ok=True)
        with open('gen/surfaces/' + file_name + '.json', 'w', encoding='utf-8') as f:
            json.dump(surface_dict, f, ensure_ascii=False, indent=4)

    def approximate_surface(self):
        surf = fitting.approximate_surface(
            self.grid_points,
            self.size_u,
            self.size_v,
            degree_u=self.degree_u,
            degree_v=self.degree_v,
            ctrlpts_size_u=self.ctrlpts_size_u,
            ctrlpts_size_v=self.ctrlpts_size_v
        )
        return surf

    def setup_worker(self,
                     size_u_arr,
                     size_v_arr,
                     grid_points_arr,
                     degree_u_arr,
                     degree_v_arr,
                     ctrlpts_size_u_arr,
                     ctrlpts_size_v_arr,
                     names_arr,
                     is_target,
                     common_knotvector_u,
                     common_knotvector_v
                     ):
        self.size_u_arr = size_u_arr
        self.size_v_arr = size_v_arr
        self.grid_points_arr = grid_points_arr
        self.degree_u_arr = degree_u_arr
        self.degree_v_arr = degree_v_arr
        self.ctrlpts_size_u_arr = ctrlpts_size_u_arr
        self.ctrlpts_size_v_arr = ctrlpts_size_v_arr
        self.names_arr = names_arr
        self.is_target = is_target
        self.common_knotvector_u = common_knotvector_u
        self.common_knotvector_v = common_knotvector_v

    def run(self):
        if self.is_target:
            surf = fitting.approximate_surface(
                self.grid_points_arr[0],
                self.size_u_arr[0],
                self.size_v_arr[0],
                degree_u=self.degree_u_arr[0],
                degree_v=self.degree_v_arr[0],
                ctrlpts_size_u=self.ctrlpts_size_u_arr[0],
                ctrlpts_size_v=self.ctrlpts_size_v_arr[0]
            )
            self.export_surface(surf.ctrlpts_size_u, surf.ctrlpts_size_v, surf.degree_u, surf.degree_v,
                                surf.knotvector_u, surf.knotvector_v, surf.ctrlpts, self.names_arr[0] + "_surface")
            self.progress.emit()

        else:
            for i in range(len(self.size_u_arr)):
                print(len(self.names_arr))
                surf = approximate_surface_with_knotvector(
                    self.grid_points_arr[i],
                    self.size_u_arr[i],
                    self.size_v_arr[i],
                    degree_u=self.degree_u_arr[i],
                    degree_v=self.degree_v_arr[i],
                    ctrlpts_size_u=self.ctrlpts_size_u_arr[i],
                    ctrlpts_size_v=self.ctrlpts_size_v_arr[i],
                    knotvector_u=self.common_knotvector_u,
                    knotvector_v=self.common_knotvector_v
                )
                self.export_surface(surf.ctrlpts_size_u, surf.ctrlpts_size_v, surf.degree_u, surf.degree_v,
                                    surf.knotvector_u, surf.knotvector_v, surf.ctrlpts, self.names_arr[i] + "_surface")
                self.progress.emit()
        self.finished.emit()
