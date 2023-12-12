import pymeshlab
from mesh2grid import Mesh2Grid
import trimesh
from vedo import mesh
import os


def CreateGrid(targetFileName, sourcePath, simulated, exportFolder):
    """
    :param str targetFileName: path of target file (ex. "data/targets/3DS_target_mesh.stl")
    :param str sourcePath: path of source files of current iteration (ex. "data/Simulation/i1")
    :param int simulated: True if data is simulated, False if data is measured
    :param str exportFolder: name of the file that grid points are exported to (ex. "grids_simulated/i1")
    """
    TARGETFILE = targetFileName
    trimesh.tol.merge = 1e-7

    sourceFiles = [sourcePath + "/" + fileName for fileName in os.listdir(sourcePath)]

    # load target mesh
    targetVedo = mesh.Mesh(TARGETFILE)
    targetVedo.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)
    targetVedo.fillHoles()
    targetVedoBack = targetVedo.clone()
    targetVedoFront = targetVedo.clone()
    targetVedoBack.cutWithBox([-78, 10, -71, 65, -1000, 1000], invert=False)
    targetVedoFront.cutWithBox([195, 295, -71, 65, -1000, 1000], invert=False)

    mergedTarget = mesh.merge(targetVedoFront, targetVedoBack)
    mergedTargetTrimesh = mergedTarget.to_trimesh()

    targetMesh = targetVedo.to_trimesh()
    targetMeshBack = targetVedoBack.to_trimesh()
    targetMeshFront = targetVedoFront.to_trimesh()
    print('target mesh loaded')

    index = 1
    for sourceFile in sourceFiles:
        exportFileName = 'mesh_ausgerichtet' + str(index) + ".stl"

        sourceMesh = mesh.Mesh(sourceFile)
        if simulated:
            vertices = sourceMesh.vertices()
            numVertices = len(vertices)
            halfPoint = numVertices // 2
            upperPart = vertices[halfPoint:]

            ms = pymeshlab.MeshSet()
            pc = pymeshlab.Mesh(upperPart)
            ms.add_mesh(pc)
            ms.surface_reconstruction_ball_pivoting()
            currentMesh = ms.current_mesh()

            seperatedVertices = currentMesh.vertex_matrix()
            seperatedFaces = currentMesh.face_matrix()
            sourceTrimesh = trimesh.Trimesh(seperatedVertices, seperatedFaces)
            transformationMatrix, _ = trimesh.registration.mesh_other(sourceTrimesh, targetMesh, samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)

            sourceVedo = mesh.Mesh([sourceTrimesh.vertices, sourceTrimesh.faces])
            sourceTrimesh = sourceVedo.to_trimesh()
            """
            sourceVedoBack = sourceVedo.cutWithBox([-78, 0, -71, 65, -1000, 1000], invert=False)
            sourceTrimeshBack = sourceVedoBack.to_trimesh()
            transformationMatrix, _ = trimesh.registration.mesh_other(sourceTrimeshBack, targetMeshBack, icp_final=500,
                                                                      samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)

            sourceVedo = mesh.Mesh([sourceTrimesh.vertices, sourceTrimesh.faces])
            sourceVedoFront = sourceVedo.cutWithBox([200, 290, -71, 65, -1000, 1000], invert=False)
            sourceTrimeshFront = sourceVedoFront.to_trimesh()
            transformationMatrix, _ = trimesh.registration.mesh_other(sourceTrimeshFront, targetMeshFront, icp_final=500,
                                                                      samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)
            """
            sourceVedoTmp = sourceVedo.clone()
            sourceVedoBack = sourceVedo.cutWithBox([-65, 0, -71, 65, -1000, 1000], invert=False)
            sourceVedoFront = sourceVedoTmp.cutWithBox([200, 290, -71, 65, -1000, 1000], invert=False)
            mergedSource = mesh.merge(sourceVedoFront, sourceVedoBack)
            mergedSourceTrimesh = mergedSource.to_trimesh()
            transformationMatrix, _ = trimesh.registration.mesh_other(mergedSourceTrimesh, mergedTargetTrimesh,
                                                                      icp_final=500,
                                                                      samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)
            sourceTrimesh.export(exportFileName)
        elif not simulated:
            sourceTrimesh = trimesh.load(sourceFile)
            sourceTrimesh.merge_vertices()
            trimesh.smoothing.filter_taubin(sourceTrimesh)
            transformationMatrix, _ = trimesh.registration.mesh_other(sourceTrimesh, targetMesh, samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)

            sourceVedo = mesh.Mesh([sourceTrimesh.vertices, sourceTrimesh.faces])
            sourceVedoBack = sourceVedo.cutWithBox([-78, 0, -71, 65, -1000, 1000], invert=False)
            sourceTrimeshBack = sourceVedoBack.to_trimesh()
            transformationMatrix, _ = trimesh.registration.mesh_other(sourceTrimeshBack, targetMeshBack, icp_final=500,
                                                                      samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)
            sourceTrimesh.export(exportFileName)

        alignedMesh = mesh.Mesh(exportFileName)
        alignedMesh.cutWithBox([-78, 308, -71, 65, -1000, 1000], invert=False)
        mesh2grid = Mesh2Grid(alignedMesh, 3, 5)
        mesh2grid.creategrid(5, exportFolder + "/gridpoints" + str(index))
        index = index + 1
