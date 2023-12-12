import numpy as np
import trimesh
from vedo import mesh
from vedo import plotter
from vedo import pointcloud
from mesh2grid import Mesh2Grid
from geomdl import fitting
from geomdl.visualization import VisVTK
from geomdl import multi
from utilities import approximate_surface_with_knotvector, compensatecontrolpoints
from geomdl import BSpline
from surf2stp import surf2stp
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

sourcemeshdir = "data/210901_E_Modul_Var_DC04"
sourcemeshnames = sorted(os.listdir(sourcemeshdir))

if not os.path.exists('./sourcemeshgridpointsmult'):
    Path('./sourcemeshgridpointsmult').mkdir(parents=True, exist_ok=True)
    for file in sourcemeshnames:
        sourcemeshpath = sourcemeshdir + "/" + file
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
        mesh2gridsource.creategrid(5, "sourcemeshgridpointsmult/sourcemesh" + file[-7:-4])


targetgridpoints = np.genfromtxt('./targetmeshgridpoints/gridpoints5.csv', delimiter=',').tolist()
sourcegridpoints = list()
sourcegridpointsdir = "sourcemeshgridpointsmult"
sourcegridpointsfiles = sorted(os.listdir(sourcegridpointsdir))

for file in sourcegridpointsfiles:
    path = sourcegridpointsdir + "/" + file + "/gridpointsfinal.csv"
    gridpoints = np.genfromtxt(path, delimiter=",").tolist()
    sourcegridpoints.append(gridpoints)

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
for gridpoints in sourcegridpoints:
    sourcefitsurf = approximate_surface_with_knotvector(gridpoints, sourcesize_u, sourcesize_v, degree_u=3,
                                                        degree_v=3, knotvector_u=targetknotvector_u,
                                                        knotvector_v=targetknotvector_v,
                                                        ctrlpts_size_u=numctrlpts_u, ctrlpts_size_v=numctrlpts_v)
    sourcefitsurfaces.append(sourcefitsurf)
    sourcefitsurfacectrlpts.append(sourcefitsurf.ctrlpts)
    sourcefitsurfacectrlpts2d.append(sourcefitsurf.ctrlpts2d)

targetctrlpts = targetfitsurf.ctrlpts

means = list()
compctrlptsvector = list()
diffnormlist = list()
for ctrlpts in sourcefitsurfacectrlpts:
    compctrlpts, compvectors, diffvectors = compensatecontrolpoints(targetctrlpts, ctrlpts, 0.5)
    diffnorms = np.linalg.norm(diffvectors, axis=1)
    diffnormlist.append(diffnorms)
    diffnormsum = sum(diffnorms)
    means.append(diffnormsum / (numctrlpts_v * numctrlpts_u))
    compctrlptsvector.append(compctrlpts)


sourcemeanctrlpts = np.mean(sourcefitsurfacectrlpts, axis=0)
compctrlptsmean, compvectorsmean, diffvectorsmean = compensatecontrolpoints(targetctrlpts, sourcemeanctrlpts, 0.5)

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

surf2stp(meancompsurf.ctrlpts2d, knotvector_v=meancompsurf.knotvector_v, knotvector_u=meancompsurf.knotvector_u,
         degree_u=3, degree_v=3, filename="compensatedsurface_i0.stp")


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


def calculatedeviation(point):
    sourcepoint = controlpoints[point[0]][point[1]]
    targetpoint = targetfitsurf.ctrlpts2d[point[0]][point[1]]
    deviation = np.linalg.norm(np.subtract(targetpoint, sourcepoint))
    return deviation, sourcepoint


for controlpoints in sourcefitsurfacectrlpts2d:
    mean1 = 0
    areadeviations1 = list()
    for point in areaindices1:
        deviation, sourcepoint = calculatedeviation(point)
        mean1 = mean1 + deviation
        deviations1.append(deviation)
        points.append(sourcepoint)
        areadeviations1.append(deviation)
    mean1 = mean1 / 25
    mean1list.append(mean1)
    areadeviations1 = sorted(areadeviations1)
    median1 = areadeviations1[12]
    median1list.append(median1)

    mean2 = 0
    areadeviations2 = list()
    for point in areaindices2:
        deviation, sourcepoint = calculatedeviation(point)
        mean2 = mean2 + deviation
        deviations2.append(deviation)
        points.append(sourcepoint)
        areadeviations2.append(deviation)
    mean2 = mean2 / 25
    mean2list.append(mean2)
    areadeviations2 = sorted(areadeviations2)
    median2 = areadeviations2[12]
    median2list.append(median2)

    mean3 = 0
    areadeviations3 = list()
    for point in areaindices3:
        deviation, sourcepoint = calculatedeviation(point)
        mean3 = mean3 + deviation
        deviations3.append(deviation)
        points.append(sourcepoint)
        areadeviations3.append(deviation)
    mean3 = mean3 / 25
    mean3list.append(mean3)
    areadeviations3 = sorted(areadeviations3)
    median3 = areadeviations3[12]
    median3list.append(median3)

    mean4 = 0
    areadeviations4 = list()
    for point in areaindices4:
        deviation, sourcepoint = calculatedeviation(point)
        mean4 = mean4 + deviation
        deviations4.append(deviation)
        points.append(sourcepoint)
        areadeviations4.append(deviation)
    mean4 = mean4 / 25
    mean4list.append(mean4)
    areadeviations4 = sorted(areadeviations4)
    median4 = areadeviations4[12]
    median4list.append(median4)

    mean5 = 0
    areadeviations5 = list()
    for point in areaindices5:
        deviation, sourcepoint = calculatedeviation(point)
        mean5 = mean5 + deviation
        deviations5.append(deviation)
        points.append(sourcepoint)
        areadeviations5.append(deviation)
    mean5 = mean5 / 25
    mean5list.append(mean5)
    areadeviations5 = sorted(areadeviations5)
    median5 = areadeviations5[12]
    median5list.append(median5)

    mean6 = 0
    areadeviations6 = list()
    for point in areaindices6:
        deviation, sourcepoint = calculatedeviation(point)
        mean6 = mean6 + deviation
        deviations6.append(deviation)
        points.append(sourcepoint)
        areadeviations6.append(deviation)
    mean6 = mean6 / 25
    mean6list.append(mean6)
    areadeviations6 = sorted(areadeviations6)
    median6 = areadeviations6[12]
    median6list.append(median6)

    mean7 = 0
    areadeviations7 = list()
    for point in areaindices7:
        deviation, sourcepoint = calculatedeviation(point)
        mean7 = mean7 + deviation
        deviations7.append(deviation)
        points.append(sourcepoint)
        areadeviations7.append(deviation)
    mean7 = mean7 / 25
    mean7list.append(mean7)
    areadeviations7 = sorted(areadeviations7)
    median7 = areadeviations7[12]
    median7list.append(median7)

    mean8 = 0
    areadeviations8 = list()
    for point in areaindices8:
        deviation, sourcepoint = calculatedeviation(point)
        mean8 = mean8 + deviation
        deviations8.append(deviation)
        points.append(sourcepoint)
        areadeviations8.append(deviation)
    mean8 = mean8 / 25
    mean8list.append(mean8)
    areadeviations8 = sorted(areadeviations8)
    median8 = areadeviations8[12]
    median8list.append(median8)

emodul = [[189] * 25, [193] * 25, [195] * 25, [202] * 25, [204] * 25, [208] * 25, [210] * 25, [214] * 25, [221] * 25, [222] * 25, [228] * 25,
          [231] * 25]

emodul2 = [189, 193, 195, 202, 204, 208, 210, 214, 221, 222, 228, 231]

pcld = pointcloud.Points(points)
plotter.show(pcld, targetmesh, at=0)

plt.figure(1)
plt.subplot(241)
plt.scatter(emodul, deviations1, s=0.1, label="deviation")
plt.scatter(emodul2, mean1list, s=10, c="red", label="mean")
plt.scatter(emodul2, median1list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")
plt.subplot(242)
plt.scatter(emodul, deviations2, s=0.1, label="deviation")
plt.scatter(emodul2, mean2list, s=10, c="red", label="mean")
plt.scatter(emodul2, median2list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")
plt.subplot(243)
plt.scatter(emodul, deviations3, s=0.1, label="deviation")
plt.scatter(emodul2, mean3list, s=10, c="red", label="mean")
plt.scatter(emodul2, median3list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")
plt.subplot(244)
plt.scatter(emodul, deviations4, s=0.1, label="deviation")
plt.scatter(emodul2, mean4list, s=10, c="red", label="mean")
plt.scatter(emodul2, median4list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")
plt.subplot(245)
plt.scatter(emodul, deviations5, s=0.1, label="deviation")
plt.scatter(emodul2, mean5list, s=10, c="red", label="mean")
plt.scatter(emodul2, median5list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")
plt.subplot(246)
plt.scatter(emodul, deviations6, s=0.1, label="deviation")
plt.scatter(emodul2, mean6list, s=10, c="red", label="mean")
plt.scatter(emodul2, median6list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")
plt.subplot(247)
plt.scatter(emodul, deviations7, s=0.1, label="deviation")
plt.scatter(emodul2, mean7list, s=10, c="red", label="mean")
plt.scatter(emodul2, median7list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")
plt.subplot(248)
plt.scatter(emodul, deviations8, s=0.1, label="deviation")
plt.scatter(emodul2, mean8list, s=10, c="red", label="mean")
plt.scatter(emodul2, median8list, s=10, c="green", label="median")
plt.ylim(0, 3.5)
plt.xlabel("e-modul")
plt.ylabel("distance between control points")
plt.legend(loc="upper left")

plt.figure(2)
plt.scatter(emodul2, means, label="mean")
plt.legend(loc="upper left")

plt.show()

targetpc = pointcloud.Points(targetctrlpts)
targetpc.c("#057805")
maxdeviation = np.max(diffnormlist)

figindex = 1


def func(evt):
    global figindex
    if not evt.actor:
        return
    point = evt.picked3d
    pcpointindex = targetpc.closestPoint(point, returnPointId=True)
    deviations = [row[pcpointindex] for row in diffnormlist]
    maxdeviationindex = deviations.index(max(deviations))
    maxdeviationkmodul = emodul2[maxdeviationindex]
    mindeviationindex = deviations.index(min(deviations))
    mindeviationkmodul = emodul2[mindeviationindex]
    print("control point index:", pcpointindex)
    print("maximum deviation:", max(deviations), " -  e-modul:", maxdeviationkmodul)
    print("minimum deviation:", min(deviations), " -  e-modul:", mindeviationkmodul)
    print("mean deviation:", np.sum(deviations) / len(deviations))
    print("********************")
    plt.figure(figindex)
    figindex = figindex + 1
    plt.scatter(emodul2, deviations, label="deviation")
    plt.ylim(0, maxdeviation)
    plt.legend(loc="upper left")
    plt.title("control point index: " + str(pcpointindex))
    plt.xlabel("e-modul")
    plt.ylabel("distance between control points")
    plt.show()


pltr = plotter.Plotter()
pltr.addCallback("mouse click", func)
pltr.show(targetpc).close()
