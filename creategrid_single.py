import pymeshlab
from mesh2grid import Mesh2Grid
import trimesh
from vedo import mesh


def create_grid(target_mesh, target_cut, target_front_cut, target_back_cut, source_mesh, source_name, source_cut,
                source_front_cut, source_back_cut, simulated, export_folder, iteration, u_sections, v_sections, edge1,
                edge2, edge3, edge4, num_iterations, align_twice):
    trimesh.tol.merge = 1e-7

    # load target mesh
    targetVedo = target_mesh
    targetVedo.cutWithBox(target_cut, invert=False)
    targetVedo.fillHoles()
    targetVedoBack = targetVedo.clone()
    targetVedoFront = targetVedo.clone()
    targetVedoBack.cutWithBox(target_back_cut, invert=False)
    targetVedoFront.cutWithBox(target_front_cut, invert=False)

    mergedTarget = mesh.merge(targetVedoFront, targetVedoBack)
    mergedTargetTrimesh = mergedTarget.to_trimesh()

    targetMesh = targetVedo.to_trimesh()
    targetMeshBack = targetVedoBack.to_trimesh()
    targetMeshFront = targetVedoFront.to_trimesh()
    print('target mesh loaded')

    # exportFileName = source_name + "_iter_" + str(iteration) + "_aligned" + ".stl"

    sourceMesh = source_mesh
    sourceTrimesh = None
    if simulated == "simulated":
        vertices = sourceMesh.vertices()
        numVertices = len(vertices)
        halfPoint = numVertices // 2
        upperPart = vertices[halfPoint:]

        ms = pymeshlab.MeshSet()
        pc = pymeshlab.Mesh(upperPart)
        ms.add_mesh(pc)
        ms.generate_surface_reconstruction_ball_pivoting()
        current_mesh = ms.current_mesh()

        seperatedVertices = current_mesh.vertex_matrix()
        seperatedFaces = current_mesh.face_matrix()
        sourceTrimesh = trimesh.Trimesh(seperatedVertices, seperatedFaces)
        transformationMatrix, _ = trimesh.registration.mesh_other(sourceTrimesh, targetMesh, samples=1000)
        sourceTrimesh.apply_transform(transformationMatrix)

        if align_twice:
            sourceVedo = mesh.Mesh([sourceTrimesh.vertices, sourceTrimesh.faces])
            sourceTrimesh = sourceVedo.to_trimesh()

            sourceVedoTmp = sourceVedo.clone()
            sourceVedoBack = sourceVedo.cutWithBox(source_back_cut, invert=False)
            sourceVedoFront = sourceVedoTmp.cutWithBox(source_front_cut, invert=False)
            mergedSource = mesh.merge(sourceVedoFront, sourceVedoBack)
            mergedSourceTrimesh = mergedSource.to_trimesh()
            transformationMatrix, _ = trimesh.registration.mesh_other(mergedSourceTrimesh, mergedTargetTrimesh,
                                                                      icp_final=500,
                                                                      samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)
        # sourceTrimesh.export(exportFileName)
    elif simulated == "measured":
        """
        currentMesh = sourceMesh.to_trimesh()

        seperatedVertices = currentMesh.vertex_matrix()
        seperatedFaces = currentMesh.face_matrix()
        sourceTrimesh = trimesh.Trimesh(seperatedVertices, seperatedFaces)
        """
        sourceTrimesh = sourceMesh.to_trimesh()
        transformationMatrix, _ = trimesh.registration.mesh_other(sourceTrimesh, targetMesh, samples=1000)
        sourceTrimesh.apply_transform(transformationMatrix)

        if align_twice:
            sourceVedo = mesh.Mesh([sourceTrimesh.vertices, sourceTrimesh.faces])
            sourceTrimesh = sourceVedo.to_trimesh()

            sourceVedoTmp = sourceVedo.clone()
            sourceVedoBack = sourceVedo.cutWithBox(source_back_cut, invert=False)
            sourceVedoFront = sourceVedoTmp.cutWithBox(source_front_cut, invert=False)
            mergedSource = mesh.merge(sourceVedoFront, sourceVedoBack)
            mergedSourceTrimesh = mergedSource.to_trimesh()
            transformationMatrix, _ = trimesh.registration.mesh_other(mergedSourceTrimesh, mergedTargetTrimesh,
                                                                      icp_final=500,
                                                                      samples=1000)
            sourceTrimesh.apply_transform(transformationMatrix)
        # sourceTrimesh.export(exportFileName)
    elif simulated == "target":
        sourceTrimesh = sourceMesh.to_trimesh()
    alignedMesh = mesh.Mesh([sourceTrimesh.vertices, sourceTrimesh.faces])
    # alignedMesh = mesh.Mesh(exportFileName)
    alignedMesh.cutWithBox(source_cut, invert=False)
    mesh2grid = Mesh2Grid(alignedMesh, u_sections, v_sections, edge1, edge2, edge3, edge4)
    mesh2grid.creategrid(num_iterations,
                         export_folder + "/gridpoints",
                         file_name=source_name + "_iter_" + str(iteration),
                         export_type="json")
