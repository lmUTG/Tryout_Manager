from stp2surf import Stp2Surf
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from vedo import mesh
from vedo import pyplot
from vedo import pointcloud
from geomdl import tessellate
from geomdl import fitting
from geomdl.visualization import VisVTK
from mesh2grid import Mesh2Grid
from utilities import approximate_surface_with_knotvector
import os
import time
from skopt import gp_minimize
from skopt.plots import plot_convergence

TARGETFILE = 'data/3DS_Matrize_BSpline_Surface.stp'
SOURCEFILE = 'data/tunnel/Tunnel_DC04_4.stl'

if (not os.path.exists('./transformationmatrix.csv')) or (not os.path.exists('data/mesh_ausgerichtet.stl')):
    # load source mesh
    trimesh.tol.merge = 1e-7  # set tolerance to merge vertices
    sourcemesh = trimesh.load(SOURCEFILE)
    sourcemesh.process(True)  # pre processing to reduce data
    sourcemesh.merge_vertices()
    trimesh.smoothing.filter_taubin(sourcemesh)  # smoothing to filter noise
    print('source mesh loaded and filtered')

# load target mesh
targetobj = Stp2Surf(TARGETFILE)
print('target mesh loaded')

# calculate transformation matrix
if not os.path.exists('./transformationmatrix.csv'):
    targetpoints = np.asarray(targetobj.getevalpoints('surface', 0, 0.05))
    transformationmatrix, _ = trimesh.registration.mesh_other(sourcemesh, targetpoints)
    np.savetxt("transformationmatrix.csv", transformationmatrix, delimiter=",")
    print('transformation matrix saved')

# align source mesh with target mesh and export the aligned mesh
if not os.path.exists('data/mesh_ausgerichtet.stl'):
    transformmatrix = np.genfromtxt('transformationmatrix.csv', delimiter=',')
    sourcemesh.apply_transform(transformmatrix)
    sourcemesh.export('data/mesh_ausgerichtet.stl')
    print('mesh aligned and exported')

alignedmesh = mesh.Mesh('data/mesh_ausgerichtet.stl')
delta = 0.01
targetpoints = targetobj.getevalpoints('surface', 0, delta)
samplesizes = targetobj.getsurfsamplesize(0, delta)
targetpointstri = tessellate.make_triangle_mesh(targetpoints, samplesizes[0], samplesizes[1])
faces = [x.vertex_ids for x in targetpointstri[1]]
vertices = [x.data for x in targetpointstri[0]]
targetmesh = mesh.Mesh([vertices, faces])
targetmesh.c('white')

alignedmesh.distanceToMesh(targetmesh, signed=True)
alignedmesh.addScalarBar(title='Signed\nDistance')

# cut aligned mesh into a rectangle
alignedmesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)


def meshcurvelength(meshcurve):
    curvevertices = meshcurve.vertices()
    numpoints, _ = curvevertices.shape
    totaldist = 0
    for i in range(numpoints - 1):
        dist = np.linalg.norm(curvevertices[i] - curvevertices[i + 1])
        totaldist = totaldist + dist
    return totaldist


if not os.path.exists('./sourcemeshgridpoints'):
    mesh2grid = Mesh2Grid(alignedmesh, 3, 5)
    aspectratio = max(meshcurvelength(mesh2grid.bordergeodesic1),
                      meshcurvelength(mesh2grid.bordergeodesic2)) / max(meshcurvelength(mesh2grid.bordergeodesic3),
                                                                        meshcurvelength(mesh2grid.bordergeodesic4))
    print("Aspect Ratio: ", aspectratio)
    mesh2grid.creategrid(5, "sourcemeshgridpoints")

#gridpoints = np.genfromtxt('./sourcemeshgridpoints/sourcemeshgridpoints_Tunnel_DC04_4_cut/gridpointsfinal.csv',
#                           delimiter=',').tolist()
points1 = np.genfromtxt('./sourcemeshgridpoints/sourcemeshgridpoints_Tunnel_DC04_1_cut/gridpointsfinal.csv',
                           delimiter=',').tolist()
points2 = np.genfromtxt('./sourcemeshgridpoints/sourcemeshgridpoints_Tunnel_DC04_2_cut/gridpointsfinal.csv',
                           delimiter=',').tolist()
points3 = np.genfromtxt('./sourcemeshgridpoints/sourcemeshgridpoints_Tunnel_DC04_3_cut/gridpointsfinal.csv',
                           delimiter=',').tolist()
points4 = np.genfromtxt('./sourcemeshgridpoints/sourcemeshgridpoints_Tunnel_DC04_4_cut/gridpointsfinal.csv',
                           delimiter=',').tolist()
pointsarray = [points1, points2, points3, points4]


def average_distance(x):
    size_u = 97
    size_v = 161

    opt_degree_u = int(round(x[0]))
    opt_degree_v = int(round(x[1]))
    opt_ctrlpts_size_u = int(round(x[2]))
    opt_ctrlpts_size_v = int(round(x[3]))

    deviations = np.empty((0, 199654))
    for gridpoints in pointsarray:
        surf = fitting.approximate_surface(gridpoints, size_u, size_v, degree_u=opt_degree_u, degree_v=opt_degree_v,
                                           ctrlpts_size_u=opt_ctrlpts_size_u, ctrlpts_size_v=opt_ctrlpts_size_v)
        surf.delta = 0.01

        fitsurfevalpts = surf.evalpts
        fitsurftri = tessellate.make_triangle_mesh(fitsurfevalpts, surf.sample_size_u, surf.sample_size_v)
        faces = [x.vertex_ids for x in fitsurftri[1]]
        vertices = [x.data for x in fitsurftri[0]]
        fitsurfmesh = mesh.Mesh([vertices, faces])
        fitsurfmesh.c("white")

        alignedmesh.distanceToMesh(fitsurfmesh, signed=True)
        deviation = abs(alignedmesh.getPointArray('Distance')).transpose()
        deviations = np.append(deviations, [deviation], axis=0)
    maxdeviations = np.max(deviations, axis=0)
    averagedeviation = sum(maxdeviations) / len(maxdeviations)
    print(opt_degree_u, opt_degree_v, opt_ctrlpts_size_u, opt_ctrlpts_size_v, " - average deviation: ", averagedeviation, time.strftime("%H:%M:%S"))
    return averagedeviation


print("start minimization")

x0 = np.array([4, 3, 50, 40])

res = gp_minimize(average_distance,
                  [(2, 4), (2, 4), (20, 60), (30, 100)],      # the bounds on each dimension of x
                  acq_func="gp_hedge",      # the acquisition function
                  n_calls=10,         # the number of evaluations of f
                  n_random_starts=5,  # the number of random initialization points
                  kappa=2.1,
                  random_state=np.random.randint(1, 1234))   # the random seed

print(res)
plot_convergence(res)
plt.show()
