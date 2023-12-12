from utilities import *
from geomdl import fitting, tessellate
from geomdl.visualization import VisVTK
from vedo import mesh, pointcloud, plotter
import matplotlib.pyplot as plt
from stp2surf import Stp2Surf
from surf2stp import Surf2Stp
from pathlib import Path
import os

opt_degree_u = 4
opt_degree_v = 2
opt_numctrlpts_u = 54
opt_numctrlpts_v = 84

figureindex = 0
targetStl = mesh.Mesh("data/targets/3DS_target_mesh.stl")
targetStl_cut = targetStl.clone()
targetStl_cut.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)

targetFitSurf = BSpline.Surface()
targetFitSurf.degree_u = opt_degree_u
targetFitSurf.degree_v = opt_degree_v
targetFitSurf.ctrlpts_size_u = opt_numctrlpts_u
targetFitSurf.ctrlpts_size_v = opt_numctrlpts_v
targetFitSurf.ctrlpts = np.genfromtxt("target_fit_surf/target_crtlpts.csv", delimiter=",").tolist()
targetFitSurf.knotvector_u = np.genfromtxt("target_fit_surf/target_knotvector_u.csv", delimiter=",").tolist()
targetFitSurf.knotvector_v = np.genfromtxt("target_fit_surf/target_knotvector_v.csv", delimiter=",").tolist()
targetFitSurf.delta = 0.01

meanSimulatedCtrlPts = list()
numSimulatedIterations = len(os.listdir("simulated_fit_surf"))
for iteration in os.listdir("simulated_fit_surf"):
    currIterMeanSimulatedCtrlPts = np.zeros((opt_numctrlpts_u * opt_numctrlpts_v, 3))

    iterPath = "simulated_fit_surf/" + iteration
    ctrlpts = None
    knotvector_u = None
    knotvector_v = None
    for files in os.listdir(iterPath):
        surf = BSpline.Surface()
        surf.degree_u = opt_degree_u
        surf.degree_v = opt_degree_v
        surf.ctrlpts_size_u = opt_numctrlpts_u
        surf.ctrlpts_size_v = opt_numctrlpts_v
        surf.delta = 0.01
        for file in os.listdir(iterPath + "/" + files):
            filePath = iterPath + "/" + files + "/" + file
            if file == "simulated_crtlpts.csv":
                ctrlpts = np.genfromtxt(filePath, delimiter=",").tolist()
                surf.ctrlpts = ctrlpts
            elif file == "simulated_knotvector_u.csv":
                knotvector_u = np.genfromtxt(filePath, delimiter=",").tolist()
                surf.knotvector_u = knotvector_u
            elif file == "simulated_knotvector_v.csv":
                knotvector_v = np.genfromtxt(filePath, delimiter=",").tolist()
                surf.knotvector_v = knotvector_v
        currIterMeanSimulatedCtrlPts = currIterMeanSimulatedCtrlPts + surf.ctrlpts
    currIterMeanSimulatedCtrlPts = np.divide(currIterMeanSimulatedCtrlPts, len(os.listdir(iterPath)))
    meanSimulatedCtrlPts.append(currIterMeanSimulatedCtrlPts)

meanMeasuredCtrlPts = list()
numMeasuredIterations = len(os.listdir("measured_fit_surf"))
for iteration in os.listdir("measured_fit_surf"):
    currIterMeanMeasuredCtrlPts = np.zeros((opt_numctrlpts_u * opt_numctrlpts_v, 3))

    iterPath = "measured_fit_surf/" + iteration
    ctrlpts = None
    knotvector_u = None
    knotvector_v = None
    for files in os.listdir(iterPath):
        surf = BSpline.Surface()
        surf.degree_u = opt_degree_u
        surf.degree_v = opt_degree_v
        surf.ctrlpts_size_u = opt_numctrlpts_u
        surf.ctrlpts_size_v = opt_numctrlpts_v
        surf.delta = 0.01
        for file in os.listdir(iterPath + "/" + files):
            filePath = iterPath + "/" + files + "/" + file
            if file == "measured_crtlpts.csv":
                ctrlpts = np.genfromtxt(filePath, delimiter=",").tolist()
                surf.ctrlpts = ctrlpts
            elif file == "measured_knotvector_u.csv":
                knotvector_u = np.genfromtxt(filePath, delimiter=",").tolist()
                surf.knotvector_u = knotvector_u
            elif file == "measured_knotvector_v.csv":
                knotvector_v = np.genfromtxt(filePath, delimiter=",").tolist()
                surf.knotvector_v = knotvector_v
        currIterMeanMeasuredCtrlPts = currIterMeanMeasuredCtrlPts + surf.ctrlpts
    currIterMeanMeasuredCtrlPts = np.divide(currIterMeanMeasuredCtrlPts, len(os.listdir(iterPath)))
    meanMeasuredCtrlPts.append(currIterMeanMeasuredCtrlPts)


meanSimulatedMeshes = list()
meanSimulatedDistances = list()
for simCtrlpts in meanSimulatedCtrlPts:
    meanSimulatedSurface = BSpline.Surface()
    meanSimulatedSurface.degree_u = opt_degree_u
    meanSimulatedSurface.degree_v = opt_degree_v
    meanSimulatedSurface.ctrlpts_size_u = opt_numctrlpts_u
    meanSimulatedSurface.ctrlpts_size_v = opt_numctrlpts_v
    meanSimulatedSurface.ctrlpts = simCtrlpts.tolist()
    meanSimulatedSurface.knotvector_u = targetFitSurf.knotvector_u
    meanSimulatedSurface.knotvector_v = targetFitSurf.knotvector_v
    meanSimulatedSurface.delta = 0.01
    meanSimulatedTri = tessellate.make_triangle_mesh(meanSimulatedSurface.evalpts, meanSimulatedSurface.sample_size_u,
                                                     meanSimulatedSurface.sample_size_v)
    faces = [x.vertex_ids for x in meanSimulatedTri[1]]
    vertices = [x.data for x in meanSimulatedTri[0]]
    meanSimulatedMesh = mesh.Mesh([vertices, faces])
    meanSimulatedMesh.distanceToMesh(targetStl, signed=True)
    simulatedDistance = meanSimulatedMesh.getPointArray("Distance")
    meanSimulatedDistances.append(simulatedDistance)
    meanSimulatedMeshes.append(meanSimulatedMesh)

meanSimulatedDistancesFlat = np.array(meanSimulatedDistances).flatten()

meanMeasuredMeshes = list()
meanMeasuredDistances = list()
for measuredCtrlpts in meanSimulatedCtrlPts:
    meanMeasuredSurface = BSpline.Surface()
    meanMeasuredSurface.degree_u = opt_degree_u
    meanMeasuredSurface.degree_v = opt_degree_v
    meanMeasuredSurface.ctrlpts_size_u = opt_numctrlpts_u
    meanMeasuredSurface.ctrlpts_size_v = opt_numctrlpts_v
    meanMeasuredSurface.ctrlpts = measuredCtrlpts.tolist()
    meanMeasuredSurface.knotvector_u = targetFitSurf.knotvector_u
    meanMeasuredSurface.knotvector_v = targetFitSurf.knotvector_v
    meanMeasuredSurface.delta = 0.01
    meanMeasuredTri = tessellate.make_triangle_mesh(meanMeasuredSurface.evalpts, meanMeasuredSurface.sample_size_u,
                                                    meanMeasuredSurface.sample_size_v)
    faces = [x.vertex_ids for x in meanMeasuredTri[1]]
    vertices = [x.data for x in meanMeasuredTri[0]]
    meanMeasuredMesh = mesh.Mesh([vertices, faces])
    meanMeasuredMesh.distanceToMesh(targetStl, signed=True)
    measuredDistance = meanMeasuredMesh.getPointArray("Distance")
    meanMeasuredDistances.append(measuredDistance)
    meanMeasuredMeshes.append(meanMeasuredMesh)

meanMeasuredDistancesFlat = np.array(meanMeasuredDistances).flatten()

maxDistance = max(max(abs(meanSimulatedDistancesFlat)), max(abs(meanMeasuredDistancesFlat)))

for i in range(numSimulatedIterations):
    meanSimulatedMeshes[i].cmap("jet", meanSimulatedDistances[i], vmin=-maxDistance, vmax=maxDistance)
    meanSimulatedMeshes[i].addScalarBar(title='Signed\nDistance')

for i in range(numMeasuredIterations):
    meanMeasuredMeshes[i].cmap("jet", meanMeasuredDistances[i], vmin=-maxDistance, vmax=maxDistance)
    meanMeasuredMeshes[i].addScalarBar(title='Signed\nDistance')

maxIter = max(numSimulatedIterations, numMeasuredIterations)
pltr = plotter.Plotter(shape=[2, maxIter])
index = 0
for i in range(numSimulatedIterations):
    if index == numSimulatedIterations + numMeasuredIterations - 1:
        pltr.show(meanSimulatedMeshes[i], at=index, interactive=True)
    else:
        pltr.show(meanSimulatedMeshes[i], at=index)
    index += 1

for i in range(numMeasuredIterations):
    if index == numSimulatedIterations + numMeasuredIterations - 1:
        pltr.show(meanMeasuredMeshes[i], at=index, interactive=True)
    else:
        pltr.show(meanMeasuredMeshes[i], at=index)
    index += 1
