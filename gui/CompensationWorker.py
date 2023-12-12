from PyQt5.QtCore import QObject, pyqtSignal
from utilities import *
from tools.smmothstep_ls import smoothstep_sample
import numpy as np
from geomdl.visualization import VisVTK
from vedo import pointcloud
from surf2stp import Surf2Stp


class CompensationWorker(QObject):
    finished = pyqtSignal()

    def __init__(self):
        super(CompensationWorker, self).__init__()
        self.source_ctrlpts_list = []

        self.target_surface = None

        self.tool_ctrlpts = None
        self.compensation_weight = 1
        self.use_beta_function = False
        self.compensate_with_normals = False
        self.method = "mean"
        self.ctrlpts_to_compensate = None
        self.ctrlpts_diff_norms = []

    def calculate_ctrlpts_to_compensate(self):
        self.calculate_diffs()
        control_points = np.array(self.source_ctrlpts_list)
        if self.method == "mean":
            self.ctrlpts_to_compensate = np.mean(control_points, axis=0)
        elif self.method == "median":
            self.ctrlpts_to_compensate = np.median(control_points, axis=0)
        elif self.method == "max":
            max_indices = np.argmax(self.ctrlpts_diff_norms, axis=0)
            self.ctrlpts_to_compensate = control_points[max_indices, np.arange(len(control_points[0]))]

    def calculate_diffs(self):
        self.ctrlpts_diff_norms = []
        for source_ctrlpts in self.source_ctrlpts_list:
            ctrlpts_diff_vectors = np.subtract(self.target_surface.ctrlpts, source_ctrlpts)
            ctrlpts_diff_norms = np.linalg.norm(ctrlpts_diff_vectors, axis=1)
            self.ctrlpts_diff_norms.append(ctrlpts_diff_norms)

    def set_target_surface(self, surface):
        self.target_surface = surface

    def set_tool_ctrlpts(self, surface):
        self.tool_ctrlpts = surface

    def set_compensation_weight(self, weight):
        self.compensation_weight = weight

    def use_beta(self, use):
        self.use_beta_function = use

    def use_normals(self, use):
        self.compensate_with_normals = use

    def set_source_ctrlpts(self, ctrlpts):
        self.source_ctrlpts_list = ctrlpts

    def set_method(self, method):
        if method == "mean" or method == "max" or method == "median":
            self.method = method
            return 0
        else:
            return 1

    def run(self):
        self.calculate_ctrlpts_to_compensate()

        diff_norms = np.array(self.ctrlpts_diff_norms)

        diffs = None
        if self.method == "max":
            max_indices = np.argmax(diff_norms, axis=0)
            diffs = diff_norms[max_indices, np.arange(len(diff_norms[0]))]
        elif self.method == "mean":
            diffs = np.mean(diff_norms, axis=0)
        elif self.method == "median":
            diffs = np.median(diff_norms, axis=0)
        max_deviation = max(np.abs(diffs))

        corresponding_diffs = list()
        for control_point in self.target_surface.ctrlpts:
            eval_point_index = pointcloud.Points(self.target_surface.evalpts).closestPoint(control_point,
                                                                                           returnPointId=True)
            corresponding_diffs.append(diffs[eval_point_index])

        beta_text = "no_beta"
        normals_text = "no_normals"

        if self.compensate_with_normals:
            normals_text = "normals"

        if self.use_beta_function and max_deviation > 0:
            beta = smoothstep_sample(np.abs(np.array(corresponding_diffs)) / max_deviation, order=5)
            beta_text = "beta"
        else:
            beta = np.ones(len(corresponding_diffs))

        comp_ctrlpts, _, _ = compensatecontrolpoints(self.target_surface.ctrlpts, self.tool_ctrlpts,
                                                     self.ctrlpts_to_compensate,
                                                     beta,
                                                     weight=self.compensation_weight,
                                                     normals=self.compensate_with_normals)
        comp_surf = BSpline.Surface()
        comp_surf.degree_u = self.target_surface.degree_u
        comp_surf.degree_v = self.target_surface.degree_v
        comp_surf.ctrlpts_size_v = self.target_surface.ctrlpts_size_v
        comp_surf.ctrlpts_size_u = self.target_surface.ctrlpts_size_u
        comp_surf.ctrlpts = comp_ctrlpts.tolist()
        comp_surf.knotvector_v = self.target_surface.knotvector_v
        comp_surf.knotvector_u = self.target_surface.knotvector_u
        comp_surf.delta = 0.01
        comp_surf.vis = VisVTK.VisSurface()
        comp_surf.render()

        Surf2Stp(comp_surf.ctrlpts2d, knotvector_v=comp_surf.knotvector_v, knotvector_u=comp_surf.knotvector_u,
                 degree_u=comp_surf.degree_u, degree_v=comp_surf.degree_v,
                 filename="gen/compensated_surfaces/surface_" + str(
                     self.compensation_weight) + "_" + beta_text + "_" + normals_text + ".stp")

        self.finished.emit()
