import numpy as np
from vedo import pointcloud, plotter, mesh
from geomdl import fitting, BSpline, tessellate
from geomdl.visualization import VisVTK
from utilities import approximate_surface_with_knotvector, compensatecontrolpoints
import scipy.stats as stats
import matplotlib.pyplot as plt
import os

figureindex = 0

measuredgridpoints = list()
for file in os.listdir("measuredgridpoints"):
    gridpoints = np.genfromtxt("measuredgridpoints/" + file + "/gridpointsfinal.csv", delimiter=",").tolist()
    measuredgridpoints.append(gridpoints)

simulatedgridpoints = list()
for file in os.listdir("simulatedgridpoints"):
    gridpoints = np.genfromtxt("simulatedgridpoints/" + file + "/gridpointsfinal.csv", delimiter=",").tolist()
    simulatedgridpoints.append(gridpoints)

targetgridpoints = np.genfromtxt('./targetmeshgridpoints/gridpoints5.csv', delimiter=',').tolist()
numctrlpts_u = 55
numctrlpts_v = 90
targetsize_u = 97
targetsize_v = 161
targetfitsurf = fitting.approximate_surface(targetgridpoints, targetsize_u, targetsize_v, degree_u=3, degree_v=3,
                                            ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
targetfitsurf.vis = VisVTK.VisSurface()
targetfitsurf.delta = 0.01
targettrimesh = tessellate.make_triangle_mesh(targetfitsurf.evalpts, targetfitsurf.sample_size_u,
                                              targetfitsurf.sample_size_v)
faces = [x.vertex_ids for x in targettrimesh[1]]
vertices = [x.data for x in targettrimesh[0]]
targetmesh = mesh.Mesh([vertices, faces])
targetmesh.c("white")

sourcesize_u = 97
sourcesize_v = 161

simulateddiffnormlist = list()
simulatedsurfaces = list()
meansimulatedctrlpts = np.zeros((numctrlpts_u * numctrlpts_v, 3))
for gridpoints in simulatedgridpoints:
    simulatedfitsurf = approximate_surface_with_knotvector(gridpoints, sourcesize_u, sourcesize_v, degree_u=3,
                                                           degree_v=3, knotvector_u=targetfitsurf.knotvector_u,
                                                           knotvector_v=targetfitsurf.knotvector_v,
                                                           ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
    simulatedsurfaces.append(simulatedfitsurf)
    _, _, diffvectors = compensatecontrolpoints(targetfitsurf.ctrlpts, simulatedfitsurf.ctrlpts, 0.5)
    diffnorms = np.linalg.norm(diffvectors, axis=1)
    simulateddiffnormlist.append(diffnorms)
    meansimulatedctrlpts = meansimulatedctrlpts + simulatedfitsurf.ctrlpts

mwsimulatedall = np.mean(simulateddiffnormlist)
variancesimulatedall = np.var(simulateddiffnormlist)
sigmasimulatedall = np.sqrt(variancesimulatedall)
xsimulatedall = np.linspace(mwsimulatedall - 3 * sigmasimulatedall, mwsimulatedall + 3 * sigmasimulatedall, 100)

meansimulatedctrlpts = np.divide(meansimulatedctrlpts, len(simulatedgridpoints))

meansimulatedsurface = BSpline.Surface()
meansimulatedsurface.degree_u = 3
meansimulatedsurface.degree_v = 3
meansimulatedsurface.ctrlpts_size_u = numctrlpts_u
meansimulatedsurface.ctrlpts_size_v = numctrlpts_v
meansimulatedsurface.ctrlpts = meansimulatedctrlpts.tolist()
meansimulatedsurface.knotvector_u = targetfitsurf.knotvector_u
meansimulatedsurface.knotvector_v = targetfitsurf.knotvector_v
meansimulatedsurface.delta = 0.01
meansimulatedtri = tessellate.make_triangle_mesh(meansimulatedsurface.evalpts, meansimulatedsurface.sample_size_u,
                                                 meansimulatedsurface.sample_size_v)
faces = [x.vertex_ids for x in meansimulatedtri[1]]
vertices = [x.data for x in meansimulatedtri[0]]
meansimulatedmesh = mesh.Mesh([vertices, faces])
meansimulatedmesh.distanceToMesh(targetmesh, signed=True)
meansimulatedmesh.addScalarBar(title='Signed\nDistance')

measureddiffnormlist = list()
measuredsurfaces = list()
meanmeasuredctrlpts = np.zeros((numctrlpts_u * numctrlpts_v, 3))
for gridpoints in measuredgridpoints:
    measuredfitsurf = approximate_surface_with_knotvector(gridpoints, sourcesize_u, sourcesize_v, degree_u=3,
                                                          degree_v=3, knotvector_u=targetfitsurf.knotvector_u,
                                                          knotvector_v=targetfitsurf.knotvector_v,
                                                          ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
    measuredsurfaces.append(measuredfitsurf)
    _, _, diffvectors = compensatecontrolpoints(targetfitsurf.ctrlpts, measuredfitsurf.ctrlpts, 0.5)
    diffnorms = np.linalg.norm(diffvectors, axis=1)
    measureddiffnormlist.append(diffnorms)
    meanmeasuredctrlpts = meanmeasuredctrlpts + measuredfitsurf.ctrlpts
mwmeasuredall = np.mean(measureddiffnormlist)
variancemeasuredall = np.var(measureddiffnormlist)
sigmameasuredall = np.sqrt(variancemeasuredall)
xmeasuredall = np.linspace(mwmeasuredall - 3 * sigmameasuredall, mwmeasuredall + 3 * sigmameasuredall, 100)

plt.figure(figureindex)
figureindex = figureindex + 1
plt.plot(xsimulatedall, stats.norm.pdf(xsimulatedall, mwsimulatedall, sigmasimulatedall), color="green",
         label="simulated")
plt.plot(xmeasuredall, stats.norm.pdf(xmeasuredall, mwmeasuredall, sigmameasuredall), color="blue", label="measured")
plt.legend()
plt.xlabel("deviation")
plt.ylabel("pdf")
plt.show()

meanmeasuredctrlpts = np.divide(meanmeasuredctrlpts, len(measuredgridpoints))

meanmeasuredsurface = BSpline.Surface()
meanmeasuredsurface.degree_u = 3
meanmeasuredsurface.degree_v = 3
meanmeasuredsurface.ctrlpts_size_u = numctrlpts_u
meanmeasuredsurface.ctrlpts_size_v = numctrlpts_v
meanmeasuredsurface.ctrlpts = meanmeasuredctrlpts.tolist()
meanmeasuredsurface.knotvector_u = targetfitsurf.knotvector_u
meanmeasuredsurface.knotvector_v = targetfitsurf.knotvector_v
meanmeasuredsurface.delta = 0.01
meanmeasuredtri = tessellate.make_triangle_mesh(meanmeasuredsurface.evalpts, meanmeasuredsurface.sample_size_u,
                                                meanmeasuredsurface.sample_size_v)
faces = [x.vertex_ids for x in meanmeasuredtri[1]]
vertices = [x.data for x in meanmeasuredtri[0]]
meanmeasuredmesh = mesh.Mesh([vertices, faces])
meanmeasuredmesh.distanceToMesh(targetmesh, signed=True)
meanmeasuredmesh.addScalarBar(title='Signed\nDistance')

meansimulatedpc = pointcloud.Points(meanmeasuredctrlpts)
meanmeasuredpc = pointcloud.Points(meanmeasuredctrlpts)
simulatedpc = pointcloud.Points(simulatedsurfaces[0].ctrlpts)
simulatedpc.c("green")
measuredpc = pointcloud.Points(measuredsurfaces[0].ctrlpts)
measuredpc.c("blue")


def func(evt):
    global figureindex
    if not evt.actor:
        return
    point = evt.picked3d
    at = evt.at
    pcpointindex = 0
    if at == 0:
        # simulated
        pcpointindex = simulatedpc.closestPoint(point, returnPointId=True)
    elif at == 1:
        # measured
        pcpointindex = measuredpc.closestPoint(point, returnPointId=True)
    elif at == 2:
        pcpointindex = simulatedpc.closestPoint(point, returnPointId=True)
    simulateddeviations = [row[pcpointindex] for row in simulateddiffnormlist]
    measureddeviations = [row[pcpointindex] for row in measureddiffnormlist]
    print("simulated deviations: ", simulateddeviations)
    print("measured deviations: ", measureddeviations)
    mwsimulated = np.mean(simulateddeviations)
    variancesimulated = np.var(simulateddeviations)
    sigmasimulated = np.sqrt(variancesimulated)
    xsimulated = np.linspace(mwsimulated - 3 * sigmasimulated, mwsimulated + 3 * sigmasimulated, 100)

    mwmeasured = np.mean(measureddeviations)
    variancemeasured = np.var(measureddeviations)
    sigmameasured = np.sqrt(variancemeasured)
    xmeasured = np.linspace(mwmeasured - 3 * sigmameasured, mwmeasured + 3 * sigmameasured, 100)

    plt.figure(figureindex)
    figureindex = figureindex + 1
    plt.plot(xsimulated, stats.norm.pdf(xsimulated, mwsimulated, sigmasimulated), color="green", label="simulated")
    plt.plot(xmeasured, stats.norm.pdf(xmeasured, mwmeasured, sigmameasured), color="blue", label="measured")
    plt.title("control point " + str(pcpointindex))
    plt.legend()
    plt.xlabel("deviation")
    plt.ylabel("pdf")
    plt.show()


pltr = plotter.Plotter(shape=[3, 1], title="simulated mean, measured mean, control points", size=[1600, 900])
pltr.addCallback('mouse click', func)
pltr.show(meansimulatedmesh, at=0)
pltr.show(meanmeasuredmesh, at=1)
pltr.show(simulatedpc, at=2, interactive=True)
