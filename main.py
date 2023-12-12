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
q25, q75 = np.percentile(deviation,[.25,.75])
bin_width = 2*(q75 - q25)*len(deviation)**(-1/3)
bins = round((deviation.max() - deviation.min())/bin_width)
plt.hist(deviation, bins=bins)
plt.ylabel('Number of Points')
plt.xlabel('Deviation')
plt.show()

averagedeviation = sum(abs(deviation)) / len(deviation)
maxdeviation = max([max(deviation), -min(deviation)])
print('Average Deviation: ', averagedeviation)
print('Maximum Deviation: ', maxdeviation)

# pyplot.show(targetmesh, alignedmesh, at=0)

# cut aligned mesh into a rectangle
alignedmesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)
# alignedmesh.decimate(0.4)  # downsample the number of vertices to reduce calculation time of grid algotihm
print("mesh cut into a box")


def meshcurvelength(meshcurve):
    curvevertices = meshcurve.vertices()
    numpoints, _ = curvevertices.shape
    totaldist = 0
    for i in range(numpoints - 1):
        dist = np.linalg.norm(curvevertices[i]-curvevertices[i+1])
        totaldist = totaldist + dist
    return totaldist


if not os.path.exists('./sourcemeshgridpoints'):
    mesh2grid = Mesh2Grid(alignedmesh, 3, 5)
    aspectratio = max(meshcurvelength(mesh2grid.bordergeodesic1),
                      meshcurvelength(mesh2grid.bordergeodesic2)) / max(meshcurvelength(mesh2grid.bordergeodesic3),
                                                                        meshcurvelength(mesh2grid.bordergeodesic4))
    print("Aspect Ratio: ", aspectratio)
    mesh2grid.creategrid(5, "sourcemeshgridpoints")


gridpoints = np.genfromtxt('./sourcemeshgridpoints/gridpointsfinal.csv', delimiter=',').tolist()
# pointcloud.Points(gridpoints).show()

size_u = 97
size_v = 161

surf = fitting.approximate_surface(gridpoints, size_u, size_v, degree_u=3, degree_v=3, ctrlpts_size_u=60, ctrlpts_size_v=40)
surf.vis = VisVTK.VisSurface()
surf.delta = 0.01
# surf.render()

fitsurfevalpts = surf.evalpts
samplesizes = [surf.sample_size_u, surf.sample_size_v]
fitsurftri = tessellate.make_triangle_mesh(fitsurfevalpts, samplesizes[0], samplesizes[1])
faces = [x.vertex_ids for x in fitsurftri[1]]
vertices = [x.data for x in fitsurftri[0]]
fitsurfmesh = mesh.Mesh([vertices, faces])
fitsurfmesh.c("white")

alignedmesh.distanceToMesh(fitsurfmesh, signed=True)
alignedmesh.addScalarBar(title='Signed\nDistance')
deviation = alignedmesh.getPointArray('Distance')
averagedeviation = sum(abs(deviation)) / len(deviation)
maxdeviation = max([max(deviation), -min(deviation)])
print('Average Deviation: ', averagedeviation)
print('Maximum Deviation: ', maxdeviation)
# pyplot.show(alignedmesh, fitsurfmesh, at=0)
