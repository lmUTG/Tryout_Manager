import numpy as np
import trimesh
from vedo import mesh
from vedo import pyplot
from mesh2grid import Mesh2Grid
from geomdl import fitting
from geomdl.visualization import VisVTK
from geomdl import multi
from vedo import pointcloud, plotter
from utilities import approximate_surface_with_knotvector, compensatecontrolpoints
from geomdl import BSpline
from surf2stp import surf2stp
import os


def meshcurvelength(meshcurve):
    curvevertices = meshcurve.vertices()
    numpoints, _ = curvevertices.shape
    totaldist = 0
    for i in range(numpoints - 1):
        dist = np.linalg.norm(curvevertices[i]-curvevertices[i+1])
        totaldist = totaldist + dist
    return totaldist


TARGETFILE = 'data/3DS_target_mesh.stl'
targetmesh = mesh.Mesh(TARGETFILE)
targetmesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)

alignedmesh = mesh.Mesh('data/mesh_ausgerichtet.stl')
alignedmesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)

if not os.path.exists('./targetmeshgridpoints'):
    mesh2gridtarget = Mesh2Grid(targetmesh, 3, 5)
    aspectratiotarget = max(meshcurvelength(mesh2gridtarget.bordergeodesic1),
                            meshcurvelength(mesh2gridtarget.bordergeodesic2)) / max(
        meshcurvelength(mesh2gridtarget.bordergeodesic3), meshcurvelength(mesh2gridtarget.bordergeodesic4))
    print("Aspect Ratio Target: ", aspectratiotarget)
    mesh2gridtarget.creategrid(6, "targetmeshgridpoints")

if not os.path.exists('./sourcemeshgridpoints'):
    mesh2gridsource = Mesh2Grid(alignedmesh, 3, 5)
    aspectratiosource = max(meshcurvelength(mesh2gridsource.
                                            bordergeodesic1), meshcurvelength(mesh2gridsource.bordergeodesic2)) / max(
        meshcurvelength(mesh2gridsource.bordergeodesic3), meshcurvelength(mesh2gridsource.bordergeodesic4))
    print("Aspect  Source: ", aspectratiosource)
    mesh2gridsource.creategrid(6, "sourcemeshgridpoints")


targetgridpoints = np.genfromtxt('./targetmeshgridpoints/gridpoints5.csv', delimiter=',').tolist()
sourcegridpoints = np.genfromtxt('./sourcemeshgridpoints/gridpointsfinal.csv', delimiter=',').tolist()

pc1 = pointcloud.Points(targetgridpoints)
pc2 = pointcloud.Points(sourcegridpoints)

pltr = plotter.Plotter(N=2)
pltr.show(pc1, at=0)
pltr.show(pc2, at=1, interactive=True)

numctrlpts_u = 40
numctrlpts_v = 100

targetsize_u = 97
targetsize_v = 161
targetfitsurf = fitting.approximate_surface(targetgridpoints, targetsize_u, targetsize_v, degree_u=3, degree_v=3,
                                            ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
targetfitsurf.vis = VisVTK.VisSurface()
targetfitsurf.delta = 0.01

targetknotvector_u = targetfitsurf.knotvector_u
targetknotvector_v = targetfitsurf.knotvector_v

sourcesize_u = 97
sourcesize_v = 161
sourcefitsurf = approximate_surface_with_knotvector(sourcegridpoints, sourcesize_u, sourcesize_v, degree_u=3,
                                                    degree_v=3, knotvector_u=targetknotvector_u,
                                                    knotvector_v=targetknotvector_v,
                                                    ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)

sourcefitsurf.delta = 0.01
sourcefitsurf.vis = VisVTK.VisSurface()

targetctrlpts = targetfitsurf.ctrlpts
sourcectrlpts = sourcefitsurf.ctrlpts

compctrlpts, compvectors, diffvectors = compensatecontrolpoints(targetctrlpts, sourcectrlpts, 1)

newsurf = BSpline.Surface()
newsurf.degree_u = 3
newsurf.degree_v = 3
newsurf.ctrlpts_size_u = numctrlpts_u
newsurf.ctrlpts_size_v = numctrlpts_v
newsurf.ctrlpts = compctrlpts.tolist()
newsurf.knotvector_v = targetknotvector_v
newsurf.knotvector_u = targetknotvector_u
newsurf.delta = 0.01
newsurf.vis = VisVTK.VisSurface()

msurf = multi.SurfaceContainer(newsurf, targetfitsurf)
msurf.vis = VisVTK.VisSurface()
msurf.delta = 0.01
#msurf.render(cpcolor=("blue", "green"))

surf2stp(newsurf.ctrlpts2d, knotvector_v=newsurf.knotvector_v, knotvector_u=newsurf.knotvector_u,
         degree_u=3, degree_v=3)
