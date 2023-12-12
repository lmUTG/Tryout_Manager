import numpy as np
from vedo import pointcloud, plotter, mesh
from geomdl import fitting, BSpline, tessellate
from geomdl.visualization import VisVTK
from utilities import approximate_surface_with_knotvector, compensatecontrolpoints
import matplotlib.pyplot as plt
import os

figureindex = 0

measuredgridpoints = list()
for file in os.listdir("measuredgridpoints/iteration0"):
    gridpoints = np.genfromtxt("measuredgridpoints/iteration0/" + file + "/gridpointsfinal.csv", delimiter=",").tolist()
    measuredgridpoints.append(gridpoints)

simulatedgridpoints = list()
for file in os.listdir("simulatedgridpoints/iteration0"):
    gridpoints = np.genfromtxt("simulatedgridpoints/iteration0/" + file + "/gridpointsfinal.csv", delimiter=",").tolist()
    simulatedgridpoints.append(gridpoints)

targetgridpoints = np.genfromtxt('./targetmeshgridpoints/gridpointsfinal.csv', delimiter=',').tolist()

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
    variancesimulated = np.var(simulateddeviations)
    sigmasimulated = np.sqrt(variancesimulated)
    variancemeasured = np.var(measureddeviations)
    sigmameasured = np.sqrt(variancemeasured)

    simulatedmeancp = meansimulatedctrlpts[pcpointindex]
    measuredmeancp = meanmeasuredctrlpts[pcpointindex]
    sollcp = targetfitsurf.ctrlpts[pcpointindex]

    xm = measuredmeancp - sollcp
    xs = simulatedmeancp - sollcp
    k = np.cross(xm, xs)
    k = k / np.linalg.norm(k)
    s = np.cross(k, xm)

    v = xm / np.linalg.norm(xm)
    s = s / np.linalg.norm(s)
    basismatrix = np.vstack((v, s, k)).T
    basismatrixinv = np.linalg.inv(basismatrix)
    measuredcoordinates = np.dot(basismatrixinv, xm)[:2]
    simulatedcoordinates = np.dot(basismatrixinv, xs)[:2]

    alpha = np.arccos(np.clip(np.dot(measuredcoordinates / np.linalg.norm(measuredcoordinates),
                                     simulatedcoordinates / np.linalg.norm(simulatedcoordinates)),
                              -1.0, 1.0)) * 180 / np.pi

    circlemeasured = plt.Circle(measuredcoordinates, sigmameasured, color='#ff8000', ls="--", fill=False)
    circlesimulated = plt.Circle(simulatedcoordinates, sigmasimulated, color='#00a6ff', ls="--", fill=False)

    normmeasured = np.linalg.norm(measuredcoordinates)
    normsimulated = np.linalg.norm(simulatedcoordinates)

    plt.figure(figureindex, figsize=(16, 5))
    plt.subplot(1, 3, 1)
    figureindex = figureindex + 1
    axislimit = max((normmeasured + sigmameasured, normsimulated + sigmasimulated)) + 0.2

    origin = np.array(([0, 0], [0, 0]))
    plt.quiver(*origin, measuredcoordinates[0], measuredcoordinates[1], color='#ff8000', angles='xy', scale_units='xy',
               scale=1, label="measured")
    plt.quiver(*origin, simulatedcoordinates[0], simulatedcoordinates[1], color='#00a6ff', angles='xy',
               scale_units='xy', scale=1, label="simulated")
    fig = plt.gcf()
    ax = fig.gca()
    ax.add_patch(circlemeasured)
    ax.add_patch(circlesimulated)
    plt.title("control point " + str(pcpointindex) + ", alpha = " + "{:.1f}".format(alpha) + u"\u00b0")
    plt.xlim([-axislimit, axislimit])
    plt.ylim([-axislimit, axislimit])
    plt.xlabel("v")
    plt.ylabel("s")
    plt.legend()

    plt.subplot(1, 3, 2)
    iterations = [0]
    positions = np.arange(len(iterations))
    fig = plt.gcf()
    ax = fig.gca()
    width = 0.4
    rects1 = ax.bar(positions - width/2, normmeasured, width, yerr=sigmameasured, capsize=10, label='measured',
                    color="#ff8000")
    rects2 = ax.bar(positions + width/2, normsimulated, width, yerr=sigmasimulated, capsize=10, label='simulated',
                    color="#00a6ff")
    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)
    ax.set_xticks(iterations)
    ax.legend()
    plt.xlabel("iterations")
    plt.ylabel("deviation")

    plt.subplot(1, 3, 3)
    plt.scatter(iterations, alpha)
    plt.xlabel("iterations")
    plt.ylabel("degrees (" + u"\u00b0" + ")")
    plt.show()


pltr = plotter.Plotter(shape=[2, 2], title="simulated mean, measured mean, control points", size=[1600, 900])
pltr.addCallback('mouse click', func)
pltr.show(meansimulatedmesh, at=0)
pltr.show(meanmeasuredmesh, at=1)
pltr.show(simulatedpc, at=2, interactive=True)
