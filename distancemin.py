from stp2surf import Stp2Surf
import numpy as np
import trimesh
from vedo import mesh
from vedo import pyplot
from geomdl import tessellate
from scipy import interpolate
from geomdl.visualization import VisVTK
from geomdl import utilities
from geomdl import CPGen
from geomdl import BSpline
from scipy import optimize
import os

TARGETFILE = 'data/3DS_Matrize_BSpline_Surface.stp'
SOURCEFILE = 'data/3DS_DP800_unterseite.stl'

if (not os.path.exists('./transformationmatrix.csv')) or (not os.path.exists('data/mesh_ausgerichtet.stl')):
    # load source mesh
    trimesh.tol.merge = 1e-4  # set tolerance to merge vertices
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

deviation = alignedmesh.getPointArray('Distance')
averagedeviation = sum(deviation) / len(deviation)
maxdeviation = max([max(deviation), -min(deviation)])
print('Average Deviation: ', averagedeviation)
print('Maximum Deviation: ', maxdeviation)

# pyplot.show(targetmesh, alignedmesh, at=0)

# cut aligned mesh into a rectangle
alignedmesh.cutWithBox([-78.5, 308.5, -71.5, 65.5, -1000, 1000], invert=False)
cutvertices = alignedmesh.vertices()
cutx = [x[0] for x in cutvertices]
cuty = [x[1] for x in cutvertices]
cutz = [x[2] for x in cutvertices]
sizex = max(cutx) - min(cutx)
sizey = max(cuty) - min(cuty)
print("mesh cut into a box")

surfgrid = CPGen.Grid(sizex, sizey)
sizeu = targetobj.surfaces[0].ctrlpts_size_u
sizev = targetobj.surfaces[0].ctrlpts_size_v
surfgrid.generate(sizeu - 1, sizev - 1)
points = np.asarray(surfgrid.grid) + [-79, -72, 0]
controlpoints = points.tolist()
surf = BSpline.Surface()
surf.degree_u = 3
surf.degree_v = 3
surf.ctrlpts2d = controlpoints
surf.knotvector_u = utilities.generate_knot_vector(surf.degree_u, surf.ctrlpts_size_u)
surf.knotvector_v = utilities.generate_knot_vector(surf.degree_v, surf.ctrlpts_size_v)
surf.vis = VisVTK.VisSurface(ctrlpts=False)
print('initial bspline surface created')


def distancetotarget(controlpoints):
    surf.ctrlpts = controlpoints.reshape(4000, 3).tolist()
    surf.delta = 0.5
    evalpts = surf.evalpts
    samplesizeu = surf.sample_size_u
    samplesizev = surf.sample_size_v
    tri = tessellate.make_triangle_mesh(evalpts, samplesizeu, samplesizev)
    faces = [x.vertex_ids for x in tri[1]]
    vertices = [x.data for x in tri[0]]
    updatedmesh = mesh.Mesh([vertices, faces])
    alignedmesh.distanceToMesh(updatedmesh, signed=True)
    distances = alignedmesh.getPointArray('Distance')
    distances = np.square(distances)
    averagedistance = distances.mean()
    print(averagedistance)
    return averagedistance


initvalues = np.asarray(surf.ctrlpts).reshape(12000, 1)

res = optimize.minimize(distancetotarget, initvalues, method='Powell')
np.savetxt("ctrlpts.csv", res.x, delimiter=",")

