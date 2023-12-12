from creategrid_single import create_grid
from PyQt5.QtCore import QObject, pyqtSignal


class GridWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal()

    def __init__(self):
        super(GridWorker, self).__init__()
        self.target_mesh = None
        self.grid_queue_meshes = None
        self.grid_queue = None
        self.target_cut = None
        self.target_front_cut = None
        self.target_back_cut = None
        self.source_cut = None
        self.source_front_cut = None
        self.source_back_cut = None
        self.is_simulated = None
        self.u_sections = None
        self.v_sections = None
        self._isRunning = True
        self.edge1 = None
        self.edge2 = None
        self.edge3 = None
        self.edge4 = None
        self.num_iterations = None
        self.align_twice = False

    def setup_worker(self, target_mesh, grid_queue_meshes, grid_queue, target_cut, target_front_cut, target_back_cut,
                     source_cut, source_front_cut, source_back_cut, is_simulated, u_sections, v_sections, edge1, edge2,
                     edge3, edge4, num_iterations, align_twice):
        self.target_mesh = target_mesh
        self.grid_queue_meshes = grid_queue_meshes
        self.grid_queue = grid_queue
        self.target_cut = target_cut
        self.target_front_cut = target_front_cut
        self.target_back_cut = target_back_cut
        self.source_cut = source_cut
        self.source_front_cut = source_front_cut
        self.source_back_cut = source_back_cut
        self.is_simulated = is_simulated
        self.u_sections = u_sections
        self.v_sections = v_sections
        self.edge1 = edge1
        self.edge2 = edge2
        self.edge3 = edge3
        self.edge4 = edge4
        self.num_iterations = num_iterations
        self.align_twice = align_twice

    def run(self):
        target = self.target_mesh.clone()

        if target is not None:
            sources = self.grid_queue_meshes
            names = self.grid_queue[0][:]
            iterations = self.grid_queue[2][:]
            export_folder = "gen"

            for source, name, iteration in zip(sources, names, iterations):
                if iteration == "h":
                    sim = "target"
                else:
                    sim = self.is_simulated
                create_grid(
                    target_mesh=target,
                    target_cut=self.target_cut,
                    target_back_cut=self.target_back_cut,
                    target_front_cut=self.target_front_cut,
                    source_mesh=source,
                    source_name=name,
                    source_cut=self.source_cut,
                    source_back_cut=self.source_back_cut,
                    source_front_cut=self.source_front_cut,
                    simulated=sim,
                    export_folder=export_folder,
                    iteration=iteration,
                    u_sections=self.u_sections,
                    v_sections=self.v_sections,
                    edge1=self.edge1,
                    edge2=self.edge2,
                    edge3=self.edge3,
                    edge4=self.edge4,
                    num_iterations=self.num_iterations,
                    align_twice=self.align_twice
                )
                self.progress.emit()
                print(names)
                print(name)

            self.finished.emit()
