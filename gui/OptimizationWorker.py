from PyQt5.QtCore import QObject, pyqtSignal
from geomdl import fitting, tessellate
from geomdl.visualization import VisVTK
from vedo import mesh, plotter
import numpy as np
from skopt import gp_minimize
from scipy.spatial import distance


class OptimizationWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal()
    iteration = pyqtSignal()

    def __init__(self):
        super(OptimizationWorker, self).__init__()
        self.degree_u_lower_bound = 0
        self.degree_u_upper_bound = 0

        self.degree_v_lower_bound = 0
        self.degree_v_upper_bound = 0

        self.ctrlpts_size_u_lower_bound = 0
        self.ctrlpts_size_u_upper_bound = 0

        self.ctrlpts_size_v_lower_bound = 0
        self.ctrlpts_size_v_upper_bound = 0

        self.size_u = 0
        self.size_v = 0

        self.kappa = 0

        self.res = None

        self.grid = []

        self.best_deviation = 99999999
        self.best_x = [0, 0, 0, 0]

        self.target_stl = None

        self.pltr = plotter.Plotter(title="Optimum Surface")

        self.num_iterations = 0

    def average_distance(self, x):
        opt_degree_u = int(round(x[0]))
        opt_degree_v = int(round(x[1]))
        opt_ctrlpts_size_u = int(round(x[2]))
        opt_ctrlpts_size_v = int(round(x[3]))

        surf = fitting.approximate_surface(self.grid, self.size_u, self.size_v, degree_u=opt_degree_u,
                                           degree_v=opt_degree_v,
                                           ctrlpts_size_u=opt_ctrlpts_size_u, ctrlpts_size_v=opt_ctrlpts_size_v)
        surf.sample_size_v = self.size_v
        surf.sample_size_u = self.size_u

        evalpts = np.asarray(surf.evalpts)
        grid = np.asarray(self.grid)
        dist_arr = distance.cdist(evalpts, grid)
        average_deviation = np.mean(np.diag(dist_arr))

        if average_deviation < self.best_deviation:
            self.best_deviation = average_deviation
            self.best_x = x
            self.progress.emit()

        self.iteration.emit()

        return average_deviation

    def reset_best(self):
        self.best_deviation = 99999999
        self.best_x = [0, 0, 0, 0]

    def run(self):
        print("a")
        self.reset_best()
        self.res = gp_minimize(self.average_distance,
                               [
                                   (self.degree_u_lower_bound, self.degree_u_upper_bound),
                                   (self.degree_v_lower_bound, self.degree_v_upper_bound),
                                   (self.ctrlpts_size_u_lower_bound, self.ctrlpts_size_u_upper_bound),
                                   (self.ctrlpts_size_v_lower_bound, self.ctrlpts_size_v_upper_bound)
                               ],  # the bounds on each dimension of x
                               acq_func="gp_hedge",  # the acquisition function
                               n_calls=self.num_iterations,  # the number of evaluations of f
                               n_random_starts=10,  # the number of random initialization points
                               kappa=self.kappa,
                               random_state=np.random.randint(1, 1234)  # the random seed
                               )

        opt_degree_u = int(round(self.best_x[0]))
        opt_degree_v = int(round(self.best_x[1]))
        opt_ctrlpts_size_u = int(round(self.best_x[2]))
        opt_ctrlpts_size_v = int(round(self.best_x[3]))

        surf = fitting.approximate_surface(self.grid, self.size_u, self.size_v, degree_u=opt_degree_u,
                                           degree_v=opt_degree_v,
                                           ctrlpts_size_u=opt_ctrlpts_size_u, ctrlpts_size_v=opt_ctrlpts_size_v)
        surf.delta = 0.01
        surf.vis = VisVTK.VisSurface()
        surf.render()

        # create mesh from the surface and plot the deviation
        mean_tri = tessellate.make_triangle_mesh(surf.evalpts,
                                                 surf.sample_size_u,
                                                 surf.sample_size_v)
        faces = [x.vertex_ids for x in mean_tri[1]]
        vertices = [x.data for x in mean_tri[0]]
        opt_mesh = mesh.Mesh([vertices, faces])
        opt_mesh.distanceToMesh(self.target_stl, signed=True)

        dist = opt_mesh.getPointArray("Distance")

        max_simulated_dist = max(abs(dist))
        opt_mesh.cmap("jet", dist, vmin=-max_simulated_dist, vmax=max_simulated_dist)
        opt_mesh.addScalarBar(title='Signed\nDistance')

        self.pltr = plotter.Plotter(title="Optimum Surface")
        self.pltr.show(opt_mesh)

        self.finished.emit()
