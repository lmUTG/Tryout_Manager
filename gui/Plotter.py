from PyQt5.QtWidgets import QVBoxLayout, QPushButton, QWidget, QMessageBox
import numpy as np
from geomdl import tessellate, BSpline
from vedo import mesh, plotter, pointcloud
import matplotlib.pyplot as plt


def display_missing_data(error_text):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Information)
    msg.setText("Missing Data")
    msg.setInformativeText(error_text)
    msg.setWindowTitle("Error")
    msg.exec_()


class Plotter(QWidget):
    def __init__(self):
        super(Plotter, self).__init__()

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.simulated_surfaces = {}
        self.measured_surfaces = {}
        self.target_surface = None

        self.target_stl = None

        self.figure_index = 1

        self.sim_mean_diff_iterations = []
        self.sim_mean_diff_iterations_std = None
        self.sim_max_diff_iterations = []

        self.sim_mean_eval_diff_iterations = []
        self.sim_mean_eval_diff_iterations_std = None
        self.sim_max_eval_diff_iterations = []

        self.measured_mean_diff_iterations = []
        self.measured_mean_diff_iterations_std = None
        self.measured_max_diff_iterations = []

        self.measured_mean_eval_diff_iterations = []
        self.measured_mean_eval_diff_iterations_std = None
        self.measured_max_eval_diff_iterations = []

        self.num_sim_iterations = 1
        self.num_measured_iterations = 1
        self.num_max_iterations = 1

        self.deviations_calculated = False

        self.measured_mean_ctrlpts = {}
        self.simulated_mean_ctrlpts = {}

        self.simulated_deviations = {}
        self.simulated_deviations_eval = {}
        self.measured_deviations = {}
        self.measured_deviations_eval = {}

        self.figure_index = 1

        self.init_gui()

    def init_gui(self):
        deviation_button = QPushButton("Plot Devitions")
        deviation_button.clicked.connect(self.plot_deviations)

        false_color_diagram_button = QPushButton("False Color Diagram")
        false_color_diagram_button.clicked.connect(self.plot_false_color_diagram)

        iteration_diagram_button = QPushButton("False Color Diagram all Iterations")
        iteration_diagram_button.clicked.connect(self.plot_iteration_diagrams)

        self.layout.addWidget(deviation_button)
        self.layout.addWidget(false_color_diagram_button)
        self.layout.addWidget(iteration_diagram_button)

    def calculate_deviations(self):
        self.num_sim_iterations = len(self.simulated_surfaces)
        self.num_measured_iterations = len(self.measured_surfaces)
        self.num_max_iterations = max(self.num_sim_iterations, self.num_measured_iterations)

        # calculate simulated deviations
        for iteration in self.simulated_surfaces:
            tmp_cp = []
            tmp_eval = []
            mean_control_points_current_iteration = np.zeros(
                (self.target_surface.ctrlpts_size_u * self.target_surface.ctrlpts_size_v, 3))

            for surface in self.simulated_surfaces[iteration].values():
                mean_control_points_current_iteration = mean_control_points_current_iteration + surface.ctrlpts

                surface.delta = 0.01
                diffvectors = np.subtract(self.target_surface.ctrlpts, surface.ctrlpts)
                diffnorms = np.linalg.norm(diffvectors, axis=1)
                tmp_cp.append(diffnorms)

                source_tri = tessellate.make_triangle_mesh(surface.evalpts,
                                                           surface.sample_size_u,
                                                           surface.sample_size_v)
                faces = [x.vertex_ids for x in source_tri[1]]
                vertices = [x.data for x in source_tri[0]]
                source_mesh = mesh.Mesh([vertices, faces])
                source_mesh.distanceToMesh(self.target_stl, signed=True)
                source_distance = source_mesh.getPointArray("Distance")
                tmp_eval.append(np.abs(source_distance))

            mean_control_points_current_iteration = np.divide(mean_control_points_current_iteration, len(
                self.simulated_surfaces[iteration].values()))

            self.simulated_deviations[iteration] = tmp_cp
            self.simulated_deviations_eval[iteration] = tmp_eval
            self.simulated_mean_ctrlpts[iteration] = mean_control_points_current_iteration

        self.sim_mean_diff_iterations = [np.mean(it) for it in self.simulated_deviations.values()]
        self.sim_mean_diff_iterations_std = [np.std(arr) for arr in
                                             [np.mean(it, axis=1) for it in self.simulated_deviations.values()]]
        """
        self.sim_mean_diff_iterations_std = np.std(
            np.array([np.mean(it, axis=1) for it in self.simulated_deviations.values()]),
            axis=1)
        """
        self.sim_max_diff_iterations = [np.max(it) for it in self.simulated_deviations.values()]

        self.sim_mean_eval_diff_iterations = [np.mean(it) for it in self.simulated_deviations_eval.values()]
        self.sim_mean_eval_diff_iterations_std = [np.std(arr) for arr in
                                                  [np.mean(it, axis=1) for it in
                                                   self.simulated_deviations_eval.values()]]
        """
        self.sim_mean_eval_diff_iterations_std = np.std(
            np.array([np.mean(it, axis=1) for it in self.simulated_deviations_eval.values()]),
            axis=1)
        """
        self.sim_max_eval_diff_iterations = [np.max(it) for it in self.simulated_deviations_eval.values()]

        # calculated measured deviations
        for iteration in self.measured_surfaces:
            tmp_cp = []
            tmp_eval = []
            mean_control_points_current_iteration = np.zeros(
                (self.target_surface.ctrlpts_size_u * self.target_surface.ctrlpts_size_v, 3))

            for surface in self.measured_surfaces[iteration].values():
                mean_control_points_current_iteration = mean_control_points_current_iteration + surface.ctrlpts

                surface.delta = 0.01
                diffvectors = np.subtract(self.target_surface.ctrlpts, surface.ctrlpts)
                diffnorms = np.linalg.norm(diffvectors, axis=1)
                tmp_cp.append(diffnorms)

                source_tri = tessellate.make_triangle_mesh(surface.evalpts,
                                                           surface.sample_size_u,
                                                           surface.sample_size_v)
                faces = [x.vertex_ids for x in source_tri[1]]
                vertices = [x.data for x in source_tri[0]]
                source_mesh = mesh.Mesh([vertices, faces])
                source_mesh.distanceToMesh(self.target_stl, signed=True)
                source_distance = source_mesh.getPointArray("Distance")
                tmp_eval.append(np.abs(source_distance))

            mean_control_points_current_iteration = np.divide(mean_control_points_current_iteration, len(
                self.measured_surfaces[iteration].values()))

            self.measured_deviations[iteration] = tmp_cp
            self.measured_deviations_eval[iteration] = tmp_eval
            self.measured_mean_ctrlpts[iteration] = mean_control_points_current_iteration

        self.measured_mean_diff_iterations = [np.mean(it) for it in self.measured_deviations.values()]
        self.measured_mean_diff_iterations_std = [np.std(arr) for arr in
                                                  [np.mean(it, axis=1) for it in
                                                   self.measured_deviations.values()]]
        """
        self.measured_mean_diff_iterations_std = np.std(
            np.array([np.mean(it, axis=1) for it in self.measured_deviations.values()]),
            axis=1)
        """
        self.measured_max_diff_iterations = [np.max(it) for it in self.measured_deviations.values()]

        self.measured_mean_eval_diff_iterations = [np.mean(it) for it in self.measured_deviations_eval.values()]
        self.measured_mean_eval_diff_iterations_std = [np.std(arr) for arr in
                                                       [np.mean(it, axis=1) for it in
                                                        self.measured_deviations_eval.values()]]
        """
        self.measured_mean_eval_diff_iterations_std = np.std(
            np.array([np.mean(it, axis=1) for it in self.measured_deviations_eval.values()]),
            axis=1)
        """
        self.measured_max_eval_diff_iterations = [np.max(it) for it in self.measured_deviations_eval.values()]

        self.deviations_calculated = True

    def plot_deviations(self):
        if self.target_surface is None or not self.measured_surfaces or not self.simulated_surfaces or self.target_stl is None:
            display_missing_data(
                "Target surface, target mesh, measured surfaces and simulated surfaces must be imported.")
            return
        if not self.deviations_calculated:
            self.calculate_deviations()

        plt.figure(self.figure_index, figsize=(18, 10))

        # simulated surfaces
        plt.subplot(2, 4, 1)
        plt.errorbar(range(self.num_sim_iterations), self.sim_mean_diff_iterations,
                     yerr=self.sim_mean_diff_iterations_std,
                     capsize=5,
                     fmt="o")
        plt.xticks(np.arange(self.num_sim_iterations))
        plt.xlabel("iteration")
        plt.ylabel("mean deviation")
        plt.title("simulated mean control point deviations")

        plt.subplot(2, 4, 2)
        plt.scatter(range(self.num_sim_iterations), self.sim_max_diff_iterations)
        plt.xticks(np.arange(self.num_sim_iterations))
        plt.xlabel("iteration")
        plt.ylabel("max deviation")
        plt.title("simulated max control point deviations")

        plt.subplot(2, 4, 3)
        plt.errorbar(range(self.num_sim_iterations), self.sim_mean_eval_diff_iterations,
                     yerr=self.sim_mean_eval_diff_iterations_std, capsize=5,
                     fmt="o")
        plt.xticks(np.arange(self.num_sim_iterations))
        plt.xlabel("iteration")
        plt.ylabel("mean deviation")
        plt.title("simulated mean evaluated point deviations")

        plt.subplot(2, 4, 4)
        plt.scatter(range(self.num_sim_iterations), self.sim_max_eval_diff_iterations)
        plt.xticks(np.arange(self.num_sim_iterations))
        plt.xlabel("iteration")
        plt.ylabel("max deviation")
        plt.title("simulated max evaluated point deviations")

        # measured surfaces
        plt.subplot(2, 4, 5)
        plt.errorbar(range(self.num_measured_iterations), self.measured_mean_diff_iterations,
                     yerr=self.measured_mean_diff_iterations_std,
                     capsize=5,
                     fmt="o")
        plt.xticks(np.arange(self.num_measured_iterations))
        plt.xlabel("iteration")
        plt.ylabel("mean deviation")
        plt.title("measured mean control point deviations")

        plt.subplot(2, 4, 6)
        plt.scatter(range(self.num_measured_iterations), self.measured_max_diff_iterations)
        plt.xticks(np.arange(self.num_measured_iterations))
        plt.xlabel("iteration")
        plt.ylabel("max deviation")
        plt.title("measured max control point deviations")

        plt.subplot(2, 4, 7)
        plt.errorbar(range(self.num_measured_iterations), self.measured_mean_eval_diff_iterations,
                     yerr=self.measured_mean_eval_diff_iterations_std, capsize=5,
                     fmt="o")
        plt.xticks(np.arange(self.num_measured_iterations))
        plt.xlabel("iteration")
        plt.ylabel("mean deviation")
        plt.title("measured mean evaluated point deviations")

        plt.subplot(2, 4, 8)
        plt.scatter(range(self.num_measured_iterations), self.measured_max_eval_diff_iterations)
        plt.xticks(np.arange(self.num_measured_iterations))
        plt.xlabel("iteration")
        plt.ylabel("max deviation")
        plt.title("measured max evaluated point deviations")

        plt.show()
        self.figure_index += 1

    def plot_false_color_diagram(self):
        if self.target_surface is None or not self.measured_surfaces or not self.simulated_surfaces or self.target_stl is None:
            display_missing_data(
                "Target surface, target mesh, measured surfaces and simulated surfaces must be imported.")
            return
        if not self.deviations_calculated:
            self.calculate_deviations()

        mean_sim_surface = BSpline.Surface()
        mean_sim_surface.degree_u = self.target_surface.degree_u
        mean_sim_surface.degree_v = self.target_surface.degree_v
        mean_sim_surface.ctrlpts_size_u = self.target_surface.ctrlpts_size_u
        mean_sim_surface.ctrlpts_size_v = self.target_surface.ctrlpts_size_v
        mean_sim_surface.ctrlpts = self.simulated_mean_ctrlpts[str(len(self.simulated_mean_ctrlpts) - 1)].tolist()
        mean_sim_surface.knotvector_u = self.target_surface.knotvector_u
        mean_sim_surface.knotvector_v = self.target_surface.knotvector_v
        mean_sim_surface.delta = 0.01
        mean_tri = tessellate.make_triangle_mesh(mean_sim_surface.evalpts,
                                                 mean_sim_surface.sample_size_u,
                                                 mean_sim_surface.sample_size_v)
        faces = [x.vertex_ids for x in mean_tri[1]]
        vertices = [x.data for x in mean_tri[0]]
        mean_simulated_mesh = mesh.Mesh([vertices, faces])
        mean_simulated_mesh.distanceToMesh(self.target_stl, signed=True)
        simulated_dist = mean_simulated_mesh.getPointArray("Distance")

        max_simulated_dist = max(abs(simulated_dist))
        mean_simulated_mesh.cmap("jet", simulated_dist, vmin=-max_simulated_dist, vmax=max_simulated_dist)
        mean_simulated_mesh.addScalarBar(title='Signed\nDistance')

        mean_measured_surface = BSpline.Surface()
        mean_measured_surface.degree_u = self.target_surface.degree_u
        mean_measured_surface.degree_v = self.target_surface.degree_v
        mean_measured_surface.ctrlpts_size_u = self.target_surface.ctrlpts_size_u
        mean_measured_surface.ctrlpts_size_v = self.target_surface.ctrlpts_size_v
        mean_measured_surface.ctrlpts = self.measured_mean_ctrlpts[str(len(self.measured_mean_ctrlpts) - 1)].tolist()
        mean_measured_surface.knotvector_u = self.target_surface.knotvector_u
        mean_measured_surface.knotvector_v = self.target_surface.knotvector_v
        mean_measured_surface.delta = 0.01
        mean_tri = tessellate.make_triangle_mesh(mean_measured_surface.evalpts,
                                                 mean_measured_surface.sample_size_u,
                                                 mean_measured_surface.sample_size_v)
        faces = [x.vertex_ids for x in mean_tri[1]]
        vertices = [x.data for x in mean_tri[0]]
        mean_measured_mesh = mesh.Mesh([vertices, faces])
        mean_measured_mesh.distanceToMesh(self.target_stl, signed=True)
        measured_dist = mean_measured_mesh.getPointArray("Distance")

        max_measured_dist = max(abs(measured_dist))
        mean_measured_mesh.cmap("jet", measured_dist, vmin=-max_measured_dist, vmax=max_measured_dist)
        mean_measured_mesh.addScalarBar(title='Signed\nDistance')

        pltr = plotter.Plotter(shape=[2, 2],
                               title="mean simulated surface, mean measured surface, target control points",
                               size=[1600, 900])
        pltr.addCallback('mouse click', self.display_plots_on_click)
        pltr.show(mean_simulated_mesh, at=0)
        pltr.show(mean_measured_mesh, at=1)
        pltr.show(pointcloud.Points(self.target_surface.ctrlpts).c("#3070b3"), at=2, interactive=True)

    def display_plots_on_click(self, evt):
        if not evt.actor:
            return
        point = evt.picked3d
        at = evt.at
        pc_point_index = 0
        evalpts_index = 0
        if at == 0 or at == 2:
            # simulated
            sim_cp_pc = pointcloud.Points(self.simulated_mean_ctrlpts[str(len(self.simulated_mean_ctrlpts) - 1)])
            pc_point_index = sim_cp_pc.closestPoint(point, returnPointId=True)
            last_surface = list(self.simulated_surfaces[str(len(self.simulated_surfaces) - 1)].values())[-1]
            evalpts_index = pointcloud.Points(last_surface.evalpts).closestPoint(point, returnPointId=True)
        elif at == 1:
            # measured
            measured_cp_pc = pointcloud.Points(self.measured_mean_ctrlpts[str(len(self.measured_mean_ctrlpts) - 1)])
            pc_point_index = measured_cp_pc.closestPoint(point, returnPointId=True)
            last_surface = list(self.measured_surfaces[str(len(self.measured_surfaces) - 1)].values())[-1]
            evalpts_index = pointcloud.Points(last_surface.evalpts).closestPoint(point, returnPointId=True)

        simulated_deviations = [[row[pc_point_index] for row in it] for it in self.simulated_deviations.values()]
        measured_deviations = [[row[pc_point_index] for row in it] for it in self.measured_deviations.values()]

        simulated_eval_deviations = [[row[evalpts_index] for row in it] for it in
                                     self.simulated_deviations_eval.values()]
        measured_eval_deviations = [[row[evalpts_index] for row in it] for it in self.measured_deviations_eval.values()]

        variance_simulated = list()
        sigma_simulated = list()
        for iteration in simulated_deviations:
            var = np.var(iteration)
            variance_simulated.append(var)
            sigma_simulated.append(np.sqrt(var))

        variance_measured = list()
        sigma_measured = list()
        for iteration in measured_deviations:
            var = np.var(iteration)
            variance_measured.append(var)
            sigma_measured.append(np.sqrt(var))

        variance_eval_simulated = list()
        sigma_eval_simulated = list()
        for iteration in simulated_eval_deviations:
            var = np.var(iteration)
            variance_eval_simulated.append(var)
            sigma_eval_simulated.append(np.sqrt(var))

        variance_eval_measured = list()
        sigma_eval_measured = list()
        for iteration in measured_eval_deviations:
            var = np.var(iteration)
            variance_eval_measured.append(var)
            sigma_eval_measured.append(np.sqrt(var))

        simulated_eval_mean_deviations = [np.mean([row[evalpts_index] for row in it]) for it in
                                          self.simulated_deviations_eval.values()]
        measured_eval_mean_deviations = [np.mean([row[evalpts_index] for row in it]) for it in
                                         self.measured_deviations_eval.values()]

        simulated_eval_dev_padding = np.pad(simulated_eval_mean_deviations,
                                            (0, self.num_max_iterations - len(simulated_eval_mean_deviations)))
        measured_eval_dev_padding = np.pad(measured_eval_mean_deviations,
                                           (0, self.num_max_iterations - len(measured_eval_mean_deviations)))

        sigma_simulated_eval_padding = np.pad(sigma_eval_simulated,
                                              (0, self.num_max_iterations - len(sigma_eval_simulated)))
        sigma_measured_eval_padding = np.pad(sigma_eval_measured,
                                             (0, self.num_max_iterations - len(sigma_eval_measured)))

        simulated_mean_cp = [it[pc_point_index] for it in self.simulated_mean_ctrlpts.values()]
        measured_mean_cp = [it[pc_point_index] for it in self.measured_mean_ctrlpts.values()]
        soll_cp = self.target_surface.ctrlpts[pc_point_index]

        xm = np.array([mean_cp - soll_cp for mean_cp in measured_mean_cp])
        xs = np.array([mean_cp - soll_cp for mean_cp in simulated_mean_cp])
        k = np.cross(xm[-1], xs[-1])
        k = k / np.linalg.norm(k)
        s = np.cross(k, xm[-1])

        v = xm[-1] / np.linalg.norm(xm[-1])
        s = s / np.linalg.norm(s)
        basis_matrix = np.vstack((v, s, k)).T
        basis_matrix_inv = np.linalg.inv(basis_matrix)

        measured_coordinates = np.dot(basis_matrix_inv, xm[-1])
        simulated_coordinates = np.dot(basis_matrix_inv, xs[-1])

        xm_padding = [np.dot(basis_matrix_inv, a) for a in xm]
        xs_padding = [np.dot(basis_matrix_inv, a) for a in xs]
        while len(xm_padding) < self.num_max_iterations:
            xm_padding = np.vstack((xm_padding, xm_padding[-1]))
        while len(xs_padding) < self.num_max_iterations:
            xs_padding = np.vstack((xs_padding, xs_padding[-1]))

        alpha_list = list()
        for i in range(self.num_max_iterations):
            alpha = np.arccos(np.clip(np.dot(xm_padding[i] / np.linalg.norm(xm_padding[i]),
                                             xs_padding[i] / np.linalg.norm(xs_padding[i])),
                                      -1.0, 1.0)) * 180 / np.pi
            alpha_list.append(alpha)

        circle_measured = plt.Circle(measured_coordinates, sigma_measured[-1], color='#003359', ls="--", fill=False,
                                     alpha=0.8)
        circle_simulated = plt.Circle(simulated_coordinates, sigma_simulated[-1], color='#005293', ls="--", fill=False,
                                      alpha=0.8)

        sigma_measured_padding = np.pad(sigma_measured, (0, self.num_max_iterations - len(sigma_measured)))
        sigma_simulated_padding = np.pad(sigma_simulated, (0, self.num_max_iterations - len(sigma_simulated)))
        norm_measured = np.linalg.norm(xm, axis=1)
        norm_measured_padding = np.pad(norm_measured, (0, self.num_max_iterations - len(norm_measured)))
        norm_simulated = np.linalg.norm(xs, axis=1)
        norm_simulated_padding = np.pad(norm_simulated, (0, self.num_max_iterations - len(norm_simulated)))

        plt.figure(self.figure_index, figsize=(10, 10))
        plt.subplot(2, 2, 1)
        self.figure_index += 1
        axis_limit = max(norm_measured[-1] + sigma_measured[-1],
                         norm_measured[-1] + sigma_simulated[-1]) + 0.2

        origin = np.array(([0, 0], [0, 0]))
        plt.quiver(*origin, measured_coordinates[0], measured_coordinates[1], color='#003359', angles='xy',
                   scale_units='xy', scale=1, label="measured")
        plt.quiver(*origin, simulated_coordinates[0], simulated_coordinates[1], color='#005293', angles='xy',
                   scale_units='xy', scale=1, label="simulated")
        fig = plt.gcf()
        ax = fig.gca()
        ax.add_patch(circle_measured)
        ax.add_patch(circle_simulated)
        plt.title("control point " + str(pc_point_index) + ", alpha = " + "{:.1f}".format(alpha_list[-1]) + u"\u00b0")
        plt.xlim([-axis_limit, axis_limit])
        plt.ylim([-axis_limit, axis_limit])
        plt.xlabel("v")
        plt.ylabel("s")
        plt.legend()

        plt.subplot(2, 2, 2)
        iterations = range(max(self.num_measured_iterations, self.num_sim_iterations))
        positions = np.arange(len(iterations))
        fig = plt.gcf()
        ax = fig.gca()
        width = 0.35
        rects1 = ax.bar(positions - width / 2, norm_measured_padding, width, yerr=sigma_measured_padding, capsize=10,
                        label='measured', color="#003359")
        rects2 = ax.bar(positions + width / 2, norm_simulated_padding, width, yerr=sigma_simulated_padding, capsize=10,
                        label='simulated', color="#005293")
        ax.bar_label(rects1, padding=3)
        ax.bar_label(rects2, padding=3)
        ax.set_xticks(iterations)
        ax.legend()
        plt.xlabel("iteration")
        plt.ylabel("deviation")
        plt.title("mean deviation of control point " + str(pc_point_index))

        plt.subplot(2, 2, 3)
        plt.scatter(iterations, alpha_list, c="#3070b3")
        plt.xticks(np.arange(0, len(iterations), 1))
        plt.xlabel("iteration")
        plt.ylabel("degrees (" + u"\u00b0" + ")")
        plt.title("alpha")

        plt.subplot(2, 2, 4)
        fig = plt.gcf()
        ax = fig.gca()
        width = 0.35
        rects3 = ax.bar(positions - width / 2, measured_eval_dev_padding, width, yerr=sigma_measured_eval_padding,
                        capsize=10, label='measured', color="#003359")
        rects4 = ax.bar(positions + width / 2, simulated_eval_dev_padding, width, yerr=sigma_simulated_eval_padding,
                        capsize=10, label='simulated', color="#005293")
        ax.bar_label(rects3, padding=3)
        ax.bar_label(rects4, padding=3)
        ax.set_xticks(iterations)
        ax.legend()
        plt.xlabel("iteration")
        plt.ylabel("deviation")
        plt.title("mean deviation of evaluated point " + str(evalpts_index))
        plt.show()

    def set_deviations_calculated(self, val):
        self.deviations_calculated = val

    def plot_iteration_diagrams(self):
        if self.target_surface is None or not self.measured_surfaces or not self.simulated_surfaces or self.target_stl is None:
            display_missing_data(
                "Target surface, target mesh, measured surfaces and simulated surfaces must be imported.")
            return
        if not self.deviations_calculated:
            self.calculate_deviations()

        mean_simulated_meshes = list()
        mean_simulated_distances = list()
        for ctrlpts in self.simulated_mean_ctrlpts.values():
            mean_simulated_surface = BSpline.Surface()
            mean_simulated_surface.degree_u = self.target_surface.degree_u
            mean_simulated_surface.degree_v = self.target_surface.degree_v
            mean_simulated_surface.ctrlpts_size_u = self.target_surface.ctrlpts_size_u
            mean_simulated_surface.ctrlpts_size_v = self.target_surface.ctrlpts_size_v
            mean_simulated_surface.ctrlpts = ctrlpts.tolist()
            mean_simulated_surface.knotvector_u = self.target_surface.knotvector_u
            mean_simulated_surface.knotvector_v = self.target_surface.knotvector_v
            mean_simulated_surface.delta = 0.01

            mean_simulated_tri = tessellate.make_triangle_mesh(mean_simulated_surface.evalpts,
                                                               mean_simulated_surface.sample_size_u,
                                                               mean_simulated_surface.sample_size_v)
            faces = [x.vertex_ids for x in mean_simulated_tri[1]]
            vertices = [x.data for x in mean_simulated_tri[0]]
            mean_simulated_mesh = mesh.Mesh([vertices, faces])
            mean_simulated_mesh.distanceToMesh(self.target_stl, signed=True)
            simulated_distance = mean_simulated_mesh.getPointArray("Distance")
            mean_simulated_distances.append(simulated_distance)
            mean_simulated_meshes.append(mean_simulated_mesh)

        mean_simulated_distances_flat = np.array(mean_simulated_distances).flatten()

        mean_measured_meshes = list()
        mean_measured_distances = list()
        for ctrlpts in self.measured_mean_ctrlpts.values():
            mean_measured_surface = BSpline.Surface()
            mean_measured_surface.degree_u = self.target_surface.degree_u
            mean_measured_surface.degree_v = self.target_surface.degree_v
            mean_measured_surface.ctrlpts_size_u = self.target_surface.ctrlpts_size_u
            mean_measured_surface.ctrlpts_size_v = self.target_surface.ctrlpts_size_v
            mean_measured_surface.ctrlpts = ctrlpts.tolist()
            mean_measured_surface.knotvector_u = self.target_surface.knotvector_u
            mean_measured_surface.knotvector_v = self.target_surface.knotvector_v
            mean_measured_surface.delta = 0.01

            mean_measured_tri = tessellate.make_triangle_mesh(mean_measured_surface.evalpts,
                                                              mean_measured_surface.sample_size_u,
                                                              mean_measured_surface.sample_size_v)
            faces = [x.vertex_ids for x in mean_measured_tri[1]]
            vertices = [x.data for x in mean_measured_tri[0]]
            mean_measured_mesh = mesh.Mesh([vertices, faces])
            mean_measured_mesh.distanceToMesh(self.target_stl, signed=True)
            measured_distance = mean_measured_mesh.getPointArray("Distance")
            mean_measured_distances.append(measured_distance)
            mean_measured_meshes.append(mean_measured_mesh)

        mean_measured_distances_flat = np.array(mean_measured_distances).flatten()

        max_distance = max(max(abs(mean_simulated_distances_flat)), max(abs(mean_measured_distances_flat)))

        for i in range(self.num_sim_iterations):
            mean_simulated_meshes[i].cmap("jet", mean_simulated_distances[i], vmin=-max_distance, vmax=max_distance)
            mean_simulated_meshes[i].addScalarBar(title='Signed\nDistance')

        for i in range(self.num_measured_iterations):
            mean_measured_meshes[i].cmap("jet", mean_measured_distances[i], vmin=-max_distance, vmax=max_distance)
            mean_measured_meshes[i].addScalarBar(title='Signed\nDistance')

        pltr = plotter.Plotter(shape=[2, self.num_max_iterations])
        index = 0
        for i in range(self.num_sim_iterations):
            if index == self.num_sim_iterations + self.num_measured_iterations - 1:
                pltr.show(mean_simulated_meshes[i], at=index, interactive=True)
            else:
                pltr.show(mean_simulated_meshes[i], at=index)
            index += 1

        for i in range(self.num_measured_iterations):
            if index == self.num_sim_iterations + self.num_measured_iterations - 1:
                pltr.show(mean_measured_meshes[i], at=index, interactive=True)
            else:
                pltr.show(mean_measured_meshes[i], at=index)
            index += 1
