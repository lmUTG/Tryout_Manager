from vedo import *
from vedo.pyplot import plot
import matplotlib.pyplot as plt
import numpy as np
import os

delta = 0.01

# all stl files should be in this folder
stlFolder = "data/stl"

targetStl = mesh.Mesh("data/targets/target_AF_bs.stl")
targetStl_cut = targetStl.clone()
targetStl_cut.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)

folderNames = os.listdir(stlFolder)

pl = Grid(resx=1, resy=1, sx=100, sy=200).triangulate()
pl.rotateY(90).z(-30)
pl.c('green').alpha(0.4).wireframe(0).lw(0)
txt = Text2D(font='Calco', bg='yellow')
target_mesh = targetStl_cut.clone()
ax = Axes(target_mesh)

# input x cordinate of the section
while True:
    figureindex = 0
    cutx = input("x section in mm (type 'e' to exit): ")
    if cutx == "e":
        break
    else:
        cutx = float(cutx)
    pl.x(cutx)

    # Intersection
    target_section = target_mesh.intersectWith(pl).join(reset=True)
    sectionsIter = list()
    sectionDistancesIter = list()
    meanIterDistancesIter = list()
    stdIterDistancesIter = list()
    yCoordinatesIter = list()
    titlesIter = list()
    for folder in folderNames:
        sections = list()
        sectionDistances = list()
        meanSectionDistances = list()
        yCoordinates = list()
        titles = list()
        for file in os.listdir(stlFolder + "/" + folder):
            titles.append(file)
            filePath = stlFolder + "/" + folder + "/" + file
            currentMesh = mesh.Mesh(filePath).triangulate()
            currentMesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)
            currentMesh.distanceToMesh(target_mesh)
            distances = currentMesh.getPointArray("Distance")
            section = currentMesh.intersectWith(pl).join(reset=True)
            sectionVertices = section.vertices()
            sectionDistance = list()
            yCoordinate = list()
            for i in range(len(sectionVertices) - 1):
                pts = currentMesh.intersectWithLine(sectionVertices[i], sectionVertices[i + 1])
                for pt in pts:
                    ptid = currentMesh.closestPoint(pt, returnPointId=True)
                    sectionPt = currentMesh.closestPoint(pt)
                    sectionDistance.append(distances[ptid])
                    yCoordinate.append(sectionPt[1])
            meanSectionDistance = np.mean(sectionDistance)
            meanSectionDistances.append(meanSectionDistance)
            sections.append(section)
            sectionDistances.append(sectionDistance)
            yCoordinates.append(yCoordinate)
        meanIterDistance = np.mean(meanSectionDistances)
        stdIterDistance = np.std(meanSectionDistances)
        stdIterDistancesIter.append(stdIterDistance)
        meanIterDistancesIter.append(meanIterDistance)
        sectionsIter.append(sections)
        sectionDistancesIter.append(sectionDistances)
        yCoordinatesIter.append(yCoordinates)
        titlesIter.append(titles)

    iterIndex = int(input("Iteration: "))

    sectionsSelectedIter = sectionsIter[iterIndex]
    yCoordinatesSelectedIter = yCoordinatesIter[iterIndex]
    sectionDistancesSelectedIter = sectionDistancesIter[iterIndex]
    titlesSelectedIter = titlesIter[iterIndex]

    plt.figure(figureindex)
    plt.errorbar(range(len(meanIterDistancesIter)), meanIterDistancesIter, yerr=stdIterDistancesIter, capsize=5,
                 fmt="o")
    plt.show()

    pltr = Plotter(shape=[2, len(sectionsSelectedIter) // 2 + 1],
                   sharecam=False, interactive=False, size=[1600, 600], bg="white")

    pltr.show(target_mesh, ax, at=figureindex, interactive=False, azimuth=0, elevation=-30, roll=-80)
    figureindex += 1

    for i in range(len(sectionsSelectedIter)):
        ps = plot(yCoordinatesSelectedIter[i], sectionDistancesSelectedIter[i], '-',
                  lw=2,
                  xtitle='y',
                  ytitle='z',
                  title=titlesSelectedIter[i].replace("_", " "),
                  lc='red',
                  pad=0.0)
        if figureindex == len(sectionsSelectedIter):
            pltr.show(ps, at=figureindex, interactive=True, azimuth=0, elevation=-30, roll=-80)
        else:
            pltr.show(ps, at=figureindex, interactive=False, azimuth=0, elevation=-30, roll=-80)
        figureindex += 1
