import numpy as np
import trimesh
from vedo import mesh
from vedo import plotter
from vedo import pointcloud
from mesh2grid import Mesh2Grid
from geomdl import multi
from geomdl import fitting
from geomdl.visualization import VisVTK
from utilities import approximate_surface_with_knotvector, compensatecontrolpoints
from surf2stp import surf2stp
from geomdl import BSpline
import pymeshlab
from pathlib import Path
import matplotlib.pyplot as plt
import os

TARGETFILE = 'data/3DS_target_mesh.stl'
targetmesh = mesh.Mesh(TARGETFILE)
targettrimesh = targetmesh.to_trimesh()
targetmesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)

if not os.path.exists('./targetmeshgridpoints'):
    mesh2gridtarget = Mesh2Grid(targetmesh, 3, 5)
    mesh2gridtarget.creategrid(5, "targetmeshgridpoints")

iterationfilenames = sorted(os.listdir("data/iterations"))

if not os.path.exists("./sourcemeshgridpointsiterations"):
    Path('./sourcemeshgridpointsiterations').mkdir(parents=True, exist_ok=True)
    for iteration in iterationfilenames:
        iterationpath = "data/iterations/" + iteration
        sourcemeshnames = sorted(os.listdir(iterationpath))
        for file in sourcemeshnames:
            sourcemeshpath = iterationpath + "/" + file
            sourcemesh = mesh.Mesh(sourcemeshpath)
            vertices = sourcemesh.vertices()
            numvertices = len(vertices)
            halfpoint = numvertices // 2
            upperpart = vertices[halfpoint:]
            lowerpart = vertices[:halfpoint]

            ms = pymeshlab.MeshSet()
            pc = pymeshlab.Mesh(upperpart)
            ms.add_mesh(pc)
            ms.surface_reconstruction_ball_pivoting()
            currentmesh = ms.current_mesh()

            seperatedvertices = currentmesh.vertex_matrix()
            seperatedfaces = currentmesh.face_matrix()
            seperatedtrimesh = trimesh.Trimesh(seperatedvertices, seperatedfaces)

            transformationmatrix, _ = trimesh.registration.mesh_other(seperatedtrimesh, targettrimesh)
            seperatedtrimesh.apply_transform(transformationmatrix)
            alignedvertices = seperatedtrimesh.vertices
            alignedfaces = seperatedtrimesh.faces
            seperatedmesh = mesh.Mesh([alignedvertices, alignedfaces])
            seperatedmesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)
            print(file + " seperated")
            mesh2gridsource = Mesh2Grid(seperatedmesh, 3, 5)
            savepath = "sourcemeshgridpointsiterations/" + iteration[24:] + "/sourcemesh" + file[10:13]
            mesh2gridsource.creategrid(5, savepath)

targetgridpoints = np.genfromtxt('./targetmeshgridpoints/gridpoints5.csv', delimiter=',').tolist()
sourcegridpoints = list()
sourcegridpointsdir = "sourcemeshgridpointsiterations"
sourcegridpointsiterations = sorted(os.listdir(sourcegridpointsdir))
numiterations = len(sourcegridpointsiterations)

for iteration in sourcegridpointsiterations:
    gridpointscurrentiteration = list()
    path = sourcegridpointsdir + "/" + iteration
    sourcegridpointsfiles = sorted(os.listdir(path))
    for file in sourcegridpointsfiles:
        gridpath = path + "/" + file + "/gridpointsfinal.csv"
        gridpoints = np.genfromtxt(gridpath, delimiter=",")
        gridpoints = gridpoints.tolist()
        gridpointscurrentiteration.append(gridpoints)
    sourcegridpoints.append(gridpointscurrentiteration)


numctrlpts_u = 55
numctrlpts_v = 90

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
sourcefitsurfaces = list()
sourcefitsurfacectrlpts = list()
sourcefitsurfacectrlpts2d = list()
for iteration in sourcegridpoints:
    surfacecurrentiteration = list()
    ctrlptscurrentiteration = list()
    ctrlpts2dcurrentiteration = list()
    for gridpoints in iteration:
        sourcefitsurf = approximate_surface_with_knotvector(gridpoints, sourcesize_u, sourcesize_v, degree_u=3,
                                                            degree_v=3, knotvector_u=targetknotvector_u,
                                                            knotvector_v=targetknotvector_v,
                                                            ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
        surfacecurrentiteration.append(sourcefitsurf)
        ctrlptscurrentiteration.append(sourcefitsurf.ctrlpts)
        ctrlpts2dcurrentiteration.append(sourcefitsurf.ctrlpts2d)
    sourcefitsurfaces.append(surfacecurrentiteration)
    sourcefitsurfacectrlpts.append(ctrlptscurrentiteration)
    sourcefitsurfacectrlpts2d.append(ctrlpts2dcurrentiteration)

targetctrlpts = targetfitsurf.ctrlpts

means = list()
compctrlptsvector = list()
diffnormlist = list()

for iteration in sourcefitsurfacectrlpts:
    diffnormlistcurrentiteration = list()
    meanscurrentiteration = list()
    compctrlptsvectorcurrrentiteration = list()
    for ctrlpts in iteration:
        compctrlpts, compvectors, diffvectors = compensatecontrolpoints(targetctrlpts, ctrlpts, 0.5)
        diffnorms = np.linalg.norm(diffvectors, axis=1)
        diffnormsum = sum(diffnorms)

        diffnormlistcurrentiteration.append(diffnorms)
        meanscurrentiteration.append(diffnormsum / (numctrlpts_v * numctrlpts_u))
        compctrlptsvectorcurrrentiteration.append(compctrlpts)
    means.append(meanscurrentiteration)
    compctrlptsvector.append(compctrlptsvectorcurrrentiteration)
    diffnormlist.append(diffnormlistcurrentiteration)

diffnormlistlastiteration = diffnormlist[-1]
sourcefitctrlptspreviteration = None
sourcemaxctrlptsprev = targetctrlpts
sourcemeanctrlptsprev = targetctrlpts
sourcemedianctrlptsprev = targetctrlpts

if numiterations > 1:
    diffnormlistpreviteration = diffnormlist[-2]
    sourcefitctrlptspreviteration = np.array(sourcefitsurfacectrlpts[-2])
    maxindicesprev = np.argmax(diffnormlistpreviteration, axis=0)
    sourcemaxctrlptsprev = sourcefitctrlptspreviteration[maxindicesprev,
                                                         np.arange(len(sourcefitctrlptspreviteration[0]))]
    sourcemeanctrlptsprev = np.mean(sourcefitctrlptspreviteration, axis=0)
    sourcemedianctrlptsprev = np.median(sourcefitctrlptspreviteration, axis=0)

sourcefitctrlptslastiteration = np.array(sourcefitsurfacectrlpts[-1])
maxindices = np.argmax(diffnormlistlastiteration, axis=0)
sourcemaxctrlpts = sourcefitctrlptslastiteration[maxindices, np.arange(len(sourcefitctrlptslastiteration[0]))]
sourcemeanctrlpts = np.mean(sourcefitctrlptslastiteration, axis=0)
sourcemedianctrlpts = np.median(sourcefitctrlptslastiteration, axis=0)

compctrlptsmean, compvectorsmean, diffvectorsmean = compensatecontrolpoints(sourcemeanctrlptsprev,
                                                                            sourcemeanctrlpts, 0.5)
compctrlptsmedian, compvectorsmedian, diffvectorsmedian = compensatecontrolpoints(sourcemedianctrlptsprev,
                                                                                  sourcemedianctrlpts, 0.5)
compctrlptsmax, compvectorsmax, diffvectorsmax = compensatecontrolpoints(sourcemaxctrlptsprev,
                                                                         sourcemaxctrlpts, 0.5)

meancompsurf = BSpline.Surface()
meancompsurf.degree_u = 3
meancompsurf.degree_v = 3
meancompsurf.ctrlpts_size_u = numctrlpts_u
meancompsurf.ctrlpts_size_v = numctrlpts_v
meancompsurf.ctrlpts = compctrlptsmean.tolist()
meancompsurf.knotvector_v = targetknotvector_v
meancompsurf.knotvector_u = targetknotvector_u
meancompsurf.delta = 0.01
meancompsurf.vis = VisVTK.VisSurface()

msurf = multi.SurfaceContainer(targetfitsurf, meancompsurf)
msurf.vis = VisVTK.VisSurface()
msurf.delta = 0.01
# msurf.render(cpcolor=("blue", "green"))

mediancompsurf = BSpline.Surface()
mediancompsurf.degree_u = 3
mediancompsurf.degree_v = 3
mediancompsurf.ctrlpts_size_u = numctrlpts_u
mediancompsurf.ctrlpts_size_v = numctrlpts_v
mediancompsurf.ctrlpts = compctrlptsmedian.tolist()
mediancompsurf.knotvector_v = targetknotvector_v
mediancompsurf.knotvector_u = targetknotvector_u

maxcompsurf = BSpline.Surface()
maxcompsurf.degree_u = 3
maxcompsurf.degree_v = 3
maxcompsurf.ctrlpts_size_u = numctrlpts_u
maxcompsurf.ctrlpts_size_v = numctrlpts_v
maxcompsurf.ctrlpts = compctrlptsmax.tolist()
maxcompsurf.knotvector_v = targetknotvector_v
maxcompsurf.knotvector_u = targetknotvector_u

surf2stp(meancompsurf.ctrlpts2d, knotvector_v=meancompsurf.knotvector_v, knotvector_u=meancompsurf.knotvector_u,
         degree_u=3, degree_v=3, filename="compensatedsurface_mean_i" + str(numiterations - 1) + ".stp")
surf2stp(maxcompsurf.ctrlpts2d, knotvector_v=maxcompsurf.knotvector_v, knotvector_u=maxcompsurf.knotvector_u,
         degree_u=3, degree_v=3, filename="compensatedsurface_max.stp" + str(numiterations - 1) + ".stp")
surf2stp(mediancompsurf.ctrlpts2d, knotvector_v=mediancompsurf.knotvector_v, knotvector_u=mediancompsurf.knotvector_u,
         degree_u=3, degree_v=3, filename="compensatedsurface_median.stp" + str(numiterations - 1) + ".stp")


emodul = [[189] * 25, [193] * 25, [195] * 25, [202] * 25, [204] * 25, [208] * 25, [210] * 25, [214] * 25, [221] * 25,
          [222] * 25, [228] * 25, [231] * 25]
emodul2 = [189, 193, 195, 202, 204, 208, 210, 214, 221, 222, 228, 231]


def getarea(pointindices):
    area = list()
    for i in range(-2, 3):
        for j in range(-2, 3):
            point = [pointindices[0] - i, pointindices[1] - j]
            area.append(point)
    return area


selectedpoint1 = [12, 7]
areaindices1 = getarea(selectedpoint1)
deviations1 = list()

selectedpoint2 = [27, 20]
areaindices2 = getarea(selectedpoint2)
deviations2 = list()

selectedpoint3 = [12, 45]
areaindices3 = getarea(selectedpoint3)
deviations3 = list()

selectedpoint4 = [42, 84]
areaindices4 = getarea(selectedpoint4)
deviations4 = list()

selectedpoint5 = [35, 70]
areaindices5 = getarea(selectedpoint5)
deviations5 = list()

selectedpoint6 = [20, 82]
areaindices6 = getarea(selectedpoint6)
deviations6 = list()

selectedpoint7 = [33, 45]
areaindices7 = getarea(selectedpoint7)
deviations7 = list()

selectedpoint8 = [4, 84]
areaindices8 = getarea(selectedpoint8)
deviations8 = list()

mean1list = list()
median1list = list()
mean2list = list()
median2list = list()
mean3list = list()
median3list = list()
mean4list = list()
median4list = list()
mean5list = list()
median5list = list()
mean6list = list()
median6list = list()
mean7list = list()
median7list = list()
mean8list = list()
median8list = list()
points = list()


def calculatedeviation(point, ctrlpts):
    sourcepoint = ctrlpts[point[0]][point[1]]
    targetpoint = targetfitsurf.ctrlpts2d[point[0]][point[1]]
    deviation = np.linalg.norm(np.subtract(targetpoint, sourcepoint))
    return deviation, sourcepoint


def calculatemeanmedian(areaindices, iterationctrlpts):
    meancurrentiteration = list()
    mediancurrentiteration = list()
    deviationscurrentiteration = list()
    sourcepoints = list()
    for ctrlpts in iterationctrlpts:
        mean = 0
        areadeviations = list()
        for point in areaindices:
            deviation, sourcepoint = calculatedeviation(point, ctrlpts)
            sourcepoints.append(sourcepoint)
            mean = mean + deviation
            deviationscurrentiteration.append(deviation)
            areadeviations.append(deviation)
        mean = mean / 25
        meancurrentiteration.append(mean)
        areadeviations = sorted(areadeviations)
        median = areadeviations[12]
        mediancurrentiteration.append(median)
    return meancurrentiteration, mediancurrentiteration, deviationscurrentiteration, sourcepoints


aresourcepointssaved = False
for iteration in sourcefitsurfacectrlpts2d:
    mean1currentiteration, median1currentiteration, deviations1currentiteration, sp = calculatemeanmedian(areaindices1,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean1list.append(mean1currentiteration)
    deviations1.append(deviations1currentiteration)
    median1list.append(median1currentiteration)

    mean2currentiteration, median2currentiteration, deviations2currentiteration, sp = calculatemeanmedian(areaindices2,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean2list.append(mean2currentiteration)
    deviations2.append(deviations2currentiteration)
    median2list.append(median2currentiteration)

    mean3currentiteration, median3currentiteration, deviations3currentiteration, sp = calculatemeanmedian(areaindices3,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean3list.append(mean3currentiteration)
    deviations3.append(deviations3currentiteration)
    median3list.append(median3currentiteration)

    mean4currentiteration, median4currentiteration, deviations4currentiteration, sp = calculatemeanmedian(areaindices4,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean4list.append(mean4currentiteration)
    deviations4.append(deviations4currentiteration)
    median4list.append(median4currentiteration)

    mean5currentiteration, median5currentiteration, deviations5currentiteration, sp = calculatemeanmedian(areaindices5,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean5list.append(mean5currentiteration)
    deviations5.append(deviations5currentiteration)
    median5list.append(median5currentiteration)

    mean6currentiteration, median6currentiteration, deviations6currentiteration, sp = calculatemeanmedian(areaindices6,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean6list.append(mean6currentiteration)
    deviations6.append(deviations6currentiteration)
    median6list.append(median6currentiteration)

    mean7currentiteration, median7currentiteration, deviations7currentiteration, sp = calculatemeanmedian(areaindices7,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean7list.append(mean7currentiteration)
    deviations7.append(deviations7currentiteration)
    median7list.append(median7currentiteration)

    mean8currentiteration, median8currentiteration, deviations8currentiteration, sp = calculatemeanmedian(areaindices8,
                                                                                                          iteration)
    if not aresourcepointssaved:
        points.append(sp)
    mean8list.append(mean8currentiteration)
    deviations8.append(deviations8currentiteration)
    median8list.append(median8currentiteration)

    if not aresourcepointssaved:
        aresourcepointssaved = True

points = np.reshape(points, (np.size(points) // 3, 3))
pcld = pointcloud.Points(points)
plotter.show(pcld, targetmesh, at=0)

figindex = 1

for i in range(numiterations):
    fig = plt.figure(figindex)
    fig.suptitle("iteration " + str(i))
    plt.subplot(241)
    plt.scatter(emodul, deviations1[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean1list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median1list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    plt.subplot(242)
    plt.scatter(emodul, deviations2[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean2list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median2list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    plt.subplot(243)
    plt.scatter(emodul, deviations3[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean3list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median3list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    plt.subplot(244)
    plt.scatter(emodul, deviations4[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean4list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median4list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    plt.subplot(245)
    plt.scatter(emodul, deviations5[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean5list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median5list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    plt.subplot(246)
    plt.scatter(emodul, deviations6[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean6list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median6list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    plt.subplot(247)
    plt.scatter(emodul, deviations7[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean7list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median7list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    plt.subplot(248)
    plt.scatter(emodul, deviations8[i], s=0.1, label="deviation")
    plt.scatter(emodul2, mean8list[i], s=10, c="red", label="mean")
    plt.scatter(emodul2, median8list[i], s=10, c="green", label="median")
    plt.ylim(0, 3.5)
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.legend(loc="upper left")
    figindex = figindex + 1

plt.figure(figindex)
for i in range(numiterations):
    plt.subplot(1, numiterations, i + 1)
    plt.scatter(emodul2, means[i])
    plt.title("iteration " + str(i))
    plt.xlabel("e-modul")
    plt.ylabel("average deviation")
    plt.ylim(0, 1.5)
figindex = figindex + 1

meanofemodules = np.mean(means, axis=1)
iterations = range(numiterations)

plt.figure(figindex)
plt.scatter(iterations, meanofemodules)
plt.title("iterations and average deviation")
plt.xlabel("iteration")
plt.ylabel("average deviation")

plt.show()

targetpc = pointcloud.Points(targetctrlpts)
targetpc.c("#057805")
maxdeviation = np.max(diffnormlist)


def func(evt):
    global figindex
    if not evt.actor:
        return
    point = evt.picked3d
    pcpointindex = targetpc.closestPoint(point, returnPointId=True)
    deviationlist = list()
    for iteration in diffnormlist:
        deviation = [row[pcpointindex] for row in iteration]
        meandeviation = np.mean(deviation)
        deviationlist.append(meandeviation)
    plt.figure(figindex)
    figindex = figindex + 1
    plt.scatter(range(numiterations), deviationlist, label="deviation")
    plt.ylim(0, maxdeviation)
    plt.legend(loc="upper left")
    plt.title("control point index: " + str(pcpointindex))
    plt.xlabel("iteration")
    plt.ylabel("mean distance between control points")
    plt.show()


pltr = plotter.Plotter()
pltr.addCallback("mouse click", func)
pltr.show(targetpc).close()

