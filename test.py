from creategrid import CreateGrid
from visualizesurf import VisualizeSurf
from utilities import *
from geomdl import fitting, tessellate
from geomdl.visualization import VisVTK
from vedo import mesh, pointcloud, plotter
import matplotlib.pyplot as plt
from stp2surf import Stp2Surf
from surf2stp import Surf2Stp
from pathlib import Path
import os
from geomdl.visualization import VisMPL
from geomdl import multi
import time
import pymeshlab

opt_degree_u = 3
opt_degree_v = 2
opt_numctrlpts_u = 54
opt_numctrlpts_v = 84

figureindex = 0
targetStl = mesh.Mesh("data/targets/target_AF_bs.stl")
targetStl_cut = targetStl.clone()
targetStl_cut.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)

# exportFileName = "targetmesh.stl"


# starttime = time.time()

# CreateGrid("data/targets/target_AF_bs.stl", "data/targets/Grid", True, "data/targets/Grid/Grid")
# CreateGrid("data/targets/3DS_target_mesh.stl", "data/Gemessen", False, "gemessen")
# CreateGrid("data/targets/target_AF_bs.stl", "data/Simulation/i1", True, "grids_simulated/i1")

# time = time.time() - starttime
# print('Gridzeit : ', time/60, 'min')

size_u = 97
size_v = 161

# targetGridPoints0 = np.genfromtxt("grids_simulated/i0/gridpoints1/gridpointsfinal.csv", delimiter=",")
# targetFitSurf0 = fitting.approximate_surface(targetGridPoints0, size_u, size_v, degree_u=opt_degree_u,
#                                             degree_v=opt_degree_v, ctrlpts_size_u=opt_numctrlpts_u,
#                                             ctrlpts_size_v=opt_numctrlpts_v)

targetGridPoints1 = np.genfromtxt("grids_simulated/i1/gridpoints1/gridpointsfinal.csv", delimiter=",")
targetFitSurf1 = fitting.approximate_surface(targetGridPoints1, size_u, size_v, degree_u=opt_degree_u,
                                            degree_v=opt_degree_v, ctrlpts_size_u=opt_numctrlpts_u,
                                            ctrlpts_size_v=opt_numctrlpts_v)

#targetGridPoints2 = np.genfromtxt("grids_simulated/i2/gridpoints1/gridpointsfinal.csv", delimiter=",")
#targetFitSurf2 = fitting.approximate_surface(targetGridPoints2, size_u, size_v, degree_u=opt_degree_u,
#                                            degree_v=opt_degree_v, ctrlpts_size_u=opt_numctrlpts_u,
#                                            ctrlpts_size_v=opt_numctrlpts_v)

msurf = multi.SurfaceContainer(targetFitSurf1)
msurf.vis = VisVTK.VisSurface()
msurf.delta = 0.01
msurf.render(cpcolor=("green"))
msurf.tessellator()
