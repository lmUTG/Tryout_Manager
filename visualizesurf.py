import numpy as np
from vedo import pointcloud, mesh, plotter
from geomdl import fitting, tessellate
from utilities import approximate_surface_with_knotvector


def VisualizeSurf(targetfilename, sourcefilename):
    sourcegridpoints = np.genfromtxt(sourcefilename, delimiter=",")
    targetgridpoints = np.genfromtxt(targetfilename, delimiter=",")

    numctrlpts_u = 55
    numctrlpts_v = 90
    size_u = 97
    size_v = 161

    targetfitsurf = fitting.approximate_surface(targetgridpoints, size_u, size_v, degree_u=3, degree_v=3,
                                                ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
    simulatedfitsurf = approximate_surface_with_knotvector(sourcegridpoints, size_u, size_v, degree_u=3,
                                                           degree_v=3, knotvector_u=targetfitsurf.knotvector_u,
                                                           knotvector_v=targetfitsurf.knotvector_v,
                                                           ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)

    targetfitsurf.delta = 0.01
    targettrimesh = tessellate.make_triangle_mesh(targetfitsurf.evalpts, targetfitsurf.sample_size_u,
                                                  targetfitsurf.sample_size_v)
    faces = [x.vertex_ids for x in targettrimesh[1]]
    vertices = [x.data for x in targettrimesh[0]]
    targetmesh = mesh.Mesh([vertices, faces])

    simulatedfitsurf.delta = 0.01
    simulatedtrimesh = tessellate.make_triangle_mesh(simulatedfitsurf.evalpts, simulatedfitsurf.sample_size_u,
                                                     simulatedfitsurf.sample_size_v)
    faces = [x.vertex_ids for x in simulatedtrimesh[1]]
    vertices = [x.data for x in simulatedtrimesh[0]]
    simulatedmesh = mesh.Mesh([vertices, faces])

    targetstl = mesh.Mesh("data/targets/3DS_target_mesh.stl")
    targetstl.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)
    targetstl.c("white")

    targetmesh.distanceToMesh(targetstl, signed=True)
    targetmesh.addScalarBar(title='Signed\nDistance')
    simulatedmesh.distanceToMesh(targetstl, signed=True)
    simulatedmesh.addScalarBar(title='Signed\nDistance')

    meandistancetarget = np.mean(abs(targetmesh.getPointArray("Distance")))
    meandistancesource = np.mean(abs(simulatedmesh.getPointArray("Distance")))
    print("Mean distance between target stl and target b-spline:", meandistancetarget)
    print("Mean distance between target stl and simulated b-spline:", meandistancesource)

    pltr = plotter.Plotter(N=2)
    pltr.show(targetstl, targetmesh, at=0)
    pltr.show(targetstl, simulatedmesh, at=1, interactive=True)
