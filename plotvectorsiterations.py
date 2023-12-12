import numpy as np
from vedo import pointcloud, plotter, mesh
from geomdl import fitting, BSpline, tessellate
from geomdl.visualization import VisVTK
from utilities import approximate_surface_with_knotvector, compensatecontrolpoints
import matplotlib.pyplot as plt
import os

figureindex = 0

measuredgridpoints = list()
# example file path: measuredgridpoints/iteration0/sourcemeshgridpoints_Tunnel_DC04_1_cut/gridpointsfinal.csv
nummeasurediterations = 0
for iterationfile in os.listdir("measuredgridpoints"):
    curritergridpoints = list()
    for file in os.listdir("measuredgridpoints/" + iterationfile):
        gridpoints = np.genfromtxt("measuredgridpoints/" + iterationfile + "/" + file + "/gridpointsfinal.csv",
                                   delimiter=",").tolist()
        curritergridpoints.append(gridpoints)
    measuredgridpoints.append(curritergridpoints)
    nummeasurediterations += 1

simulatedgridpoints = list()
# example file path: simulatedgridpoints/iteration0/sourcemesh189/gridpointsfinal.csv
numsimulatediterations = 0
for iterationfile in os.listdir("simulatedgridpoints"):
    curritergridpoints = list()
    for file in os.listdir("simulatedgridpoints/" + iterationfile):
        gridpoints = np.genfromtxt("simulatedgridpoints/" + iterationfile + "/" + file + "/gridpointsfinal.csv",
                                   delimiter=",").tolist()
        curritergridpoints.append(gridpoints)
    simulatedgridpoints.append(curritergridpoints)
    numsimulatediterations += 1

targetgridpoints = np.genfromtxt('./targetmeshgridpoints/gridpointsfinal.csv', delimiter=',').tolist()

numctrlpts_u = 60
numctrlpts_v = 100

targetstl = mesh.Mesh("data/3DS_Matrize.stl")

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
meansimulatedctrlpts = list()
for iteration in simulatedgridpoints:
    curritersimulateddiffnormlist = list()
    curritersimulatedsurfaces = list()
    curritermeansimulatedctrlpts = np.zeros((numctrlpts_u * numctrlpts_v, 3))
    for gridpoints in iteration:
        simulatedfitsurf = approximate_surface_with_knotvector(gridpoints, sourcesize_u, sourcesize_v, degree_u=3,
                                                               degree_v=3, knotvector_u=targetfitsurf.knotvector_u,
                                                               knotvector_v=targetfitsurf.knotvector_v,
                                                               ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
        curritersimulatedsurfaces.append(simulatedfitsurf)
        _, _, diffvectors = compensatecontrolpoints(targetfitsurf.ctrlpts, simulatedfitsurf.ctrlpts, 0.5)
        diffnorms = np.linalg.norm(diffvectors, axis=1)
        curritersimulateddiffnormlist.append(diffnorms)
        curritermeansimulatedctrlpts = curritermeansimulatedctrlpts + simulatedfitsurf.ctrlpts
    curritermeansimulatedctrlpts = np.divide(curritermeansimulatedctrlpts, len(iteration))
    simulateddiffnormlist.append(curritersimulateddiffnormlist)
    simulatedsurfaces.append(curritersimulatedsurfaces)
    meansimulatedctrlpts.append(curritermeansimulatedctrlpts)

meansimulatedsurface = BSpline.Surface()
meansimulatedsurface.degree_u = 3
meansimulatedsurface.degree_v = 3
meansimulatedsurface.ctrlpts_size_u = numctrlpts_u
meansimulatedsurface.ctrlpts_size_v = numctrlpts_v
meansimulatedsurface.ctrlpts = meansimulatedctrlpts[-1].tolist()
meansimulatedsurface.knotvector_u = targetfitsurf.knotvector_u
meansimulatedsurface.knotvector_v = targetfitsurf.knotvector_v
meansimulatedsurface.delta = 0.01
meansimulatedtri = tessellate.make_triangle_mesh(meansimulatedsurface.evalpts, meansimulatedsurface.sample_size_u,
                                                 meansimulatedsurface.sample_size_v)
faces = [x.vertex_ids for x in meansimulatedtri[1]]
vertices = [x.data for x in meansimulatedtri[0]]
meansimulatedmesh = mesh.Mesh([vertices, faces])
meansimulatedmesh.show()
meansimulatedmesh.distanceToMesh(targetstl, signed=True)
meansimulatedmesh.addScalarBar(title='Signed\nDistance')

measureddiffnormlist = list()
measuredsurfaces = list()
meanmeasuredctrlpts = list()
for iteration in measuredgridpoints:
    curritermeasureddiffnormlist = list()
    curritermeasuredsurfaces = list()
    curritermeanmeasuredctrlpts = np.zeros((numctrlpts_u * numctrlpts_v, 3))
    for gridpoints in iteration:
        measuredfitsurf = approximate_surface_with_knotvector(gridpoints, sourcesize_u, sourcesize_v, degree_u=3,
                                                               degree_v=3, knotvector_u=targetfitsurf.knotvector_u,
                                                               knotvector_v=targetfitsurf.knotvector_v,
                                                               ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
        curritermeasuredsurfaces.append(measuredfitsurf)
        _, _, diffvectors = compensatecontrolpoints(targetfitsurf.ctrlpts, measuredfitsurf.ctrlpts, 0.5)
        diffnorms = np.linalg.norm(diffvectors, axis=1)
        curritermeasureddiffnormlist.append(diffnorms)
        curritermeanmeasuredctrlpts = curritermeanmeasuredctrlpts + measuredfitsurf.ctrlpts
    curritermeanmeasuredctrlpts = np.divide(curritermeanmeasuredctrlpts, len(iteration))
    measureddiffnormlist.append(curritermeasureddiffnormlist)
    measuredsurfaces.append(curritermeasuredsurfaces)
    meanmeasuredctrlpts.append(curritermeanmeasuredctrlpts)

meanmeasuredsurface = BSpline.Surface()
meanmeasuredsurface.degree_u = 3
meanmeasuredsurface.degree_v = 3
meanmeasuredsurface.ctrlpts_size_u = numctrlpts_u
meanmeasuredsurface.ctrlpts_size_v = numctrlpts_v
meanmeasuredsurface.ctrlpts = meanmeasuredctrlpts[-1].tolist()
meanmeasuredsurface.knotvector_u = targetfitsurf.knotvector_u
meanmeasuredsurface.knotvector_v = targetfitsurf.knotvector_v
meanmeasuredsurface.delta = 0.01
meanmeasuredtri = tessellate.make_triangle_mesh(meanmeasuredsurface.evalpts, meanmeasuredsurface.sample_size_u,
                                                meanmeasuredsurface.sample_size_v)
faces = [x.vertex_ids for x in meanmeasuredtri[1]]
vertices = [x.data for x in meanmeasuredtri[0]]
meanmeasuredmesh = mesh.Mesh([vertices, faces])
meanmeasuredmesh.distanceToMesh(targetstl, signed=True)
meanmeasuredmesh.addScalarBar(title='Signed\nDistance')

meansimulatedpc = pointcloud.Points(meansimulatedctrlpts[-1])
meanmeasuredpc = pointcloud.Points(meanmeasuredctrlpts[-1])
simulatedpc = pointcloud.Points(simulatedsurfaces[-1][0].ctrlpts)
simulatedpc.c("green")
measuredpc = pointcloud.Points(measuredsurfaces[-1][0].ctrlpts)
measuredpc.c("blue")


def func(evt):
    global figureindex
    if not evt.actor:
        return
    point = evt.picked3d
    at = evt.at
    pcpointindex = 0
    if at == 0 or at == 2:
        # simulated
        pcpointindex = simulatedpc.closestPoint(point, returnPointId=True)
    elif at == 1:
        # measured
        pcpointindex = measuredpc.closestPoint(point, returnPointId=True)

    simulateddeviations = [[row[pcpointindex] for row in it] for it in simulateddiffnormlist]
    measureddeviations = [[row[pcpointindex] for row in it] for it in measureddiffnormlist]

    variancesimulated = list()
    sigmasimulated = list()
    for iteration in simulateddeviations:
        var = np.var(iteration)
        variancesimulated.append(var)
        sigmasimulated.append(np.sqrt(var))
    variancemeasured = list()
    sigmameasured = list()
    for iteration in measureddeviations:
        var = np.var(iteration)
        variancemeasured.append(var)
        sigmameasured.append(np.sqrt(var))

    simulatedmeancp = [it[pcpointindex] for it in meansimulatedctrlpts]
    measuredmeancp = [it[pcpointindex] for it in meanmeasuredctrlpts]
    sollcp = targetfitsurf.ctrlpts[pcpointindex]

    xm = np.array([meancp - sollcp for meancp in measuredmeancp])
    xs = np.array([meancp - sollcp for meancp in simulatedmeancp])
    k = np.cross(xm, xs)
    k = k / np.linalg.norm(k, axis=1)[:, None]
    s = np.cross(k, xm)

    v = xm / np.linalg.norm(xm, axis=1)[:, None]
    s = s / np.linalg.norm(s, axis=1)[:, None]
    basismatrices = list()
    basismatricesinv = list()
    for v1, v2, v3 in zip(v, s, k):
        basismatrix = np.vstack((v1, v2, v3)).T
        basismatrixinv = np.linalg.inv(basismatrix)
        basismatrices.append(basismatrix)
        basismatricesinv.append(basismatrixinv)

    measuredcoordinates = list()
    for matrix, vector in zip(basismatricesinv, xm):
        res = np.dot(matrix, vector)
        measuredcoordinates.append(res)
    simulatedcoordinates = list()
    for matrix, vector in zip(basismatricesinv, xs):
        res = np.dot(matrix, vector)
        simulatedcoordinates.append(res)

    alphalist = list()
    for i in range(len(measuredcoordinates)):
        alpha = np.arccos(np.clip(np.dot(measuredcoordinates[i] / np.linalg.norm(measuredcoordinates[i]),
                                         simulatedcoordinates[i] / np.linalg.norm(simulatedcoordinates[i])),
                                  -1.0, 1.0)) * 180 / np.pi
        alphalist.append(alpha)

    circlemeasured = plt.Circle(measuredcoordinates[-1], sigmameasured[-1], color='#ff8000', ls="--", fill=False)
    circlesimulated = plt.Circle(simulatedcoordinates[-1], sigmasimulated[-1], color='#00a6ff', ls="--", fill=False)

    normmeasured = np.linalg.norm(measuredcoordinates, axis=1)
    normsimulated = np.linalg.norm(simulatedcoordinates, axis=1)

    plt.figure(figureindex, figsize=(16, 5))
    plt.subplot(1, 3, 1)
    figureindex = figureindex + 1
    axislimit = max(normmeasured[-1] + sigmameasured[-1], normsimulated[-1] + sigmasimulated[-1]) + 0.2

    origin = np.array(([0, 0], [0, 0]))
    plt.quiver(*origin, measuredcoordinates[-1][0], measuredcoordinates[-1][1], color='#ff8000', angles='xy',
               scale_units='xy', scale=1, label="measured")
    plt.quiver(*origin, simulatedcoordinates[-1][0], simulatedcoordinates[-1][1], color='#00a6ff', angles='xy',
               scale_units='xy', scale=1, label="simulated")
    fig = plt.gcf()
    ax = fig.gca()
    ax.add_patch(circlemeasured)
    ax.add_patch(circlesimulated)
    plt.title("control point " + str(pcpointindex) + ", alpha = " + "{:.1f}".format(alphalist[-1]) + u"\u00b0")
    plt.xlim([-axislimit, axislimit])
    plt.ylim([-axislimit, axislimit])
    plt.xlabel("v")
    plt.ylabel("s")
    plt.legend()

    plt.subplot(1, 3, 2)
    iterations = range(max(nummeasurediterations, numsimulatediterations))
    positions = np.arange(len(iterations))
    fig = plt.gcf()
    ax = fig.gca()
    width = 0.35
    rects1 = ax.bar(positions - width/2, normmeasured, width, yerr=sigmameasured, capsize=10, label='measured',
                    color="#ff8000")
    rects2 = ax.bar(positions + width/2, normsimulated, width, yerr=sigmasimulated, capsize=10, label='simulated',
                    color="#00a6ff")
    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)
    ax.set_xticks(iterations)
    ax.legend()
    plt.xlabel("iteration")
    plt.ylabel("deviation")
    plt.title("mean deviation of control point " + str(pcpointindex))

    plt.subplot(1, 3, 3)
    plt.scatter(iterations, alphalist)
    plt.xticks(np.arange(0, len(iterations), 1))
    plt.xlabel("iteration")
    plt.ylabel("degrees (" + u"\u00b0" + ")")
    plt.title("alpha")
    plt.show()


pltr = plotter.Plotter(shape=[2, 2], title="simulated mean, measured mean, control points", size=[1600, 900])
pltr.addCallback('mouse click', func)
pltr.show(meansimulatedmesh, at=0)
pltr.show(meanmeasuredmesh, at=1)
pltr.show(simulatedpc, at=2, interactive=True)
