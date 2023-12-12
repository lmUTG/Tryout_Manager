from geomdl import BSpline, tessellate
from geomdl.visualization import VisVTK
from os import listdir
from os.path import isfile, join
import json
import numpy as np
from scipy.spatial import distance
from vedo import mesh, plotter, pointcloud
import os
from pathlib import Path

database_path = './database'
reference_path = './database/reference/Tunnelverstaerkung_DC04_i0_123Export-Surfaces_stl_iter_0_gridpointsfinal.json_surface.json'
grid_path = './220203_Tunnelverstaerkung_DC04_i0_0k8_1Export-Surfaces_stl_iter_0_gridpointsfinal.json'
surfaces = [f for f in listdir(database_path) if isfile(join(database_path, f))]
surface_paths = [database_path + '/' + surface for surface in surfaces]
surface_paths.append(reference_path)

'''''
x = [131, 38, 215, 108, -5, 276, 224]
y = [10, 42, 24, -58, 0, -11, -37]
'''''

x = [131]
y = [10]


# Messrauschen einstellen
messrauschen = 'false'
messwert = np.random.uniform(low=-0.05, high=0.05, size=(np.size(x),))

print(messwert)
print(messrauschen)


points = np.array([[i, j] for i, j in zip(x, y)])

delta = 0.005
tol = 0.007
b = 0.005

surfaces_z_coordinates_dict = {}
reference_surface_z = []

for surface_path in surface_paths:
    surface_file = open(surface_path)
#    print(surface_file)
    print(surface_path)

    data = json.load(surface_file)

    coordinate_data = np.empty((0, 3))
    coordinate_data_x_y = np.empty((0, 2))
    query_points_z = np.array([])

    new_points_present = False

    points_exist = False
    if 'point_coordinates' in data.keys() and not len(data['point_coordinates']) == 0:
        coordinate_data = np.array(data['point_coordinates'])
        coordinate_data_x_y = coordinate_data[:, :2]
        points_exist = True

    surf = BSpline.Surface()
    surf.degree_u = data['degree_u']
    surf.degree_v = data['degree_v']
    surf.ctrlpts_size_u = data['size_u']
    surf.ctrlpts_size_v = data['size_v']

    surf.knotvector_u = data['knotvector_u']
    surf.knotvector_v = data['knotvector_v']
    surf.ctrlpts = data['ctrlpts']

    surf.delta = delta

    delta_values_u = np.linspace(0, 1, surf.sample_size_u)
    delta_values_v = np.linspace(0, 1, surf.sample_size_v)

    eval_pts = np.array(surf.evalpts)
    x_and_y = eval_pts[:, :2]

    distances = distance.cdist(points, x_and_y)
    distance_indices = distances.min(axis=1)
    min_distances = [np.min(i) for i in distances]
    min_indices = [np.argmin(i) for i in distances]

    for point, idx in zip(points, min_indices):
        min_point_distance = 9999
        print(len(coordinate_data_x_y))
        if len(coordinate_data_x_y) > 0:
            point_distances = [np.linalg.norm(i - point[:2]) for i in coordinate_data_x_y]
            min_point_distance = np.min(point_distances)
        if (not points_exist) or min_point_distance > 1e-5:
            new_points_present = True
            surf.delta = delta
            surf.evaluate(start_u=0, stop_u=1, start_v=0, stop_v=1)

            u_idx = idx // surf.sample_size_v
            u = delta_values_u[u_idx]
            v = delta_values_v[idx % surf.sample_size_v]
            evalpt = surf.evaluate_single([u, v])
            evalptxy = evalpt[:2]
            err = np.linalg.norm(evalptxy - point)

            new_delta = delta
            while err > tol:
                new_delta = new_delta * 0.5
                surf.delta = new_delta

                start_u = u - b
                if start_u < 0:
                    start_u = 0
                stop_u = u + b
                if stop_u > 1:
                    stop_u = 1

                start_v = v - b
                if start_v < 0:
                    start_v = 0
                stop_v = v + b
                if stop_v > 1:
                    stop_v = 1

                surf.evaluate(start_u=start_u, stop_u=stop_u, start_v=start_v, stop_v=stop_v)

                delta_values_u_fine = np.linspace(0, 1, surf.sample_size_u)
                delta_values_v_fine = np.linspace(0, 1, surf.sample_size_v)

                eval_pts = np.array(surf.evalpts)
                x_and_y = eval_pts[:, :2]

                dist = distance.cdist([point], x_and_y)
                err = np.min(dist)
                idx = np.argmin(dist)

            res_point = eval_pts[idx]
            z_val = res_point[2]
            coordinate_data = np.append(coordinate_data, np.array([[point[0], point[1], z_val]]), axis=0)
            coordinate_data_x_y = np.append(coordinate_data_x_y, np.array([[point[0], point[1]]]), axis=0)
            print(err)
            print(res_point)
            if surface_path == reference_path:
                reference_surface_z.append(z_val)
            else:
                query_points_z = np.append(query_points_z, z_val)

        else:
            # read_point_idx, _ = np.where(coordinate_data_x_y == point[:2])
            read_point_idx = np.argmin(point_distances)
            read_point = coordinate_data[read_point_idx]
            if surface_path == reference_path:
                reference_surface_z.append(read_point[2])
            else:
                query_points_z = np.append(query_points_z, read_point[2])



    data['point_coordinates'] = coordinate_data.tolist()
    print(data['point_coordinates'])
    if not surface_path == reference_path:
        surfaces_z_coordinates_dict[surface_path] = query_points_z

    if new_points_present:
        with open(surface_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)

if messrauschen == 'true':
    print('mit messrauschen')
    mean_err_surfaces = {key: np.mean(np.abs(np.array(reference_surface_z) - value + messwert)) for key, value in
                        surfaces_z_coordinates_dict.items()}

    print(mean_err_surfaces)

else:
    print('ohne messrauschen')
    mean_err_surfaces = {key: np.mean(np.abs(np.array(reference_surface_z) - value)) for key, value in
                        surfaces_z_coordinates_dict.items()}

    print(mean_err_surfaces)

min_err_surf_path = min(mean_err_surfaces, key=mean_err_surfaces.get)

min_err_surface_file = open(min_err_surf_path)
min_err_surf_data = json.load(min_err_surface_file)

min_err_surf = BSpline.Surface()
min_err_surf.degree_u = min_err_surf_data['degree_u']
min_err_surf.degree_v = min_err_surf_data['degree_v']
min_err_surf.ctrlpts_size_u = min_err_surf_data['size_u']
min_err_surf.ctrlpts_size_v = min_err_surf_data['size_v']

min_err_surf.knotvector_u = min_err_surf_data['knotvector_u']
min_err_surf.knotvector_v = min_err_surf_data['knotvector_v']
min_err_surf.ctrlpts = min_err_surf_data['ctrlpts']

min_err_surf.delta = delta

min_err_surf_tri = tessellate.make_triangle_mesh(min_err_surf.evalpts,
                                                 min_err_surf.sample_size_u,
                                                 min_err_surf.sample_size_v)
faces = [x.vertex_ids for x in min_err_surf_tri[1]]
vertices = [x.data for x in min_err_surf_tri[0]]
min_err_mesh = mesh.Mesh([vertices, faces])

ref_surface_file = open(reference_path)
ref_surf_data = json.load(ref_surface_file)

ref_surf = BSpline.Surface()
ref_surf.degree_u = ref_surf_data['degree_u']
ref_surf.degree_v = ref_surf_data['degree_v']
ref_surf.ctrlpts_size_u = ref_surf_data['size_u']
ref_surf.ctrlpts_size_v = ref_surf_data['size_v']

ref_surf.knotvector_u = ref_surf_data['knotvector_u']
ref_surf.knotvector_v = ref_surf_data['knotvector_v']
ref_surf.ctrlpts = ref_surf_data['ctrlpts']

ref_surf.delta = delta

ref_surf_tri = tessellate.make_triangle_mesh(ref_surf.evalpts,
                                             ref_surf.sample_size_u,
                                             ref_surf.sample_size_v)
faces = [x.vertex_ids for x in ref_surf_tri[1]]
vertices = [x.data for x in ref_surf_tri[0]]
ref_mesh = mesh.Mesh([vertices, faces])

min_err_mesh.distanceToMesh(ref_mesh, signed=True)
mesh_dist = min_err_mesh.getPointArray("Distance")

max_dist = max(abs(mesh_dist))
min_err_mesh.cmap("jet", mesh_dist, vmin=-0.5, vmax=0.5)
min_err_mesh.addScalarBar(title='Signed\nDistance')

grid_surf_file = open(min_err_surf_path)
grid_surf_data = json.load(grid_surf_file)

grid_surf = BSpline.Surface()
grid_surf.degree_u = grid_surf_data['degree_u']
grid_surf.degree_v = grid_surf_data['degree_v']
grid_surf.ctrlpts_size_u = grid_surf_data['size_u']
grid_surf.ctrlpts_size_v = grid_surf_data['size_v']

grid_surf.knotvector_u = grid_surf_data['knotvector_u']
grid_surf.knotvector_v = grid_surf_data['knotvector_v']
grid_surf.ctrlpts = grid_surf_data['ctrlpts']

grid_surf.delta = delta

grid_surf_tri = tessellate.make_triangle_mesh(grid_surf.evalpts,
                                              grid_surf.sample_size_u,
                                              grid_surf.sample_size_v)
grid_faces = [x.vertex_ids for x in grid_surf_tri[1]]
grid_vertices = [x.data for x in grid_surf_tri[0]]
grid_mesh = mesh.Mesh([grid_vertices, grid_faces])

grid_points = [[i[0], i[1], j] for i, j in zip(points, reference_surface_z)]
grid_points_pc = pointcloud.Points(grid_points)

print(surface_path)
print('Maximale Abweichung: ', max(abs(mesh_dist)))
print('Mittlere Abweichung: ', np.mean(abs(mesh_dist)))
print('Standard Abweichung: ', np.std(abs(mesh_dist)))

pltr = plotter.Plotter(title="Surface with Minimum Mean Error to Reference Surface", shape=[1, 2], sharecam=False)
pltr.show(min_err_mesh, at=0)
pltr.show(grid_mesh, grid_points_pc, at=1, interactive=True)

sorted_mean_err_surfaces = {k: v for k, v in sorted(mean_err_surfaces.items(), key=lambda item: item[1])}
print(sorted_mean_err_surfaces)

print('---------------------')
print('*** INTERPOLATION ***')
max_delta_z = max(mean_err_surfaces.values())
min_delta_z = min(mean_err_surfaces.values())


def interpolate_control_points(surf_dict, z):
    # Traverse the dictionary
    prev_path = ''
    prev_value = -1
    for surf_path, surf_value in surf_dict.items():
        if surf_value >= z:
            prev_surf_file = open(prev_path)
            prev_surf_data = json.load(prev_surf_file)
            prev_ctrlpts = np.array(prev_surf_data['ctrlpts'])

            next_surf_file = open(surf_path)
            next_surf_data = json.load(next_surf_file)
            next_ctrlpts = np.array(next_surf_data['ctrlpts'])

            new_ctrlpts = prev_ctrlpts * (surf_value - z) + next_ctrlpts * (z - prev_value)
            new_ctrlpts = new_ctrlpts / (surf_value - prev_value)

            return new_ctrlpts
        prev_path = surf_path
        prev_value = surf_value
    # If all array elements are smaller
    return -1


while True:
    input_str = 'delta-z (type -1 to exit) (min: ' + str(min_delta_z) + ', max: ' + str(max_delta_z) + '): '
    input_z = float(input(input_str))
    if input_z == -1:
        break
    elif input_z < min_delta_z or input_z > max_delta_z:
        continue

    interpolated_ctrlpts = interpolate_control_points(sorted_mean_err_surfaces, input_z)

    interpolated_surface = BSpline.Surface()
    interpolated_surface.degree_u = min_err_surf_data['degree_u']
    interpolated_surface.degree_v = min_err_surf_data['degree_v']
    interpolated_surface.ctrlpts_size_u = min_err_surf_data['size_u']
    interpolated_surface.ctrlpts_size_v = min_err_surf_data['size_v']

    interpolated_surface.knotvector_u = min_err_surf_data['knotvector_u']
    interpolated_surface.knotvector_v = min_err_surf_data['knotvector_v']
    interpolated_surface.ctrlpts = interpolated_ctrlpts.tolist()

    interpolated_surface.delta = delta

    interpolated_surface.vis = VisVTK.VisSurface()
    interpolated_surface.render()

    surface_dict = {"size_u": interpolated_surface.ctrlpts_size_u,
                    "size_v": interpolated_surface.ctrlpts_size_v,
                    "degree_u": interpolated_surface.degree_u,
                    "degree_v": interpolated_surface.degree_v,
                    "knotvector_u": interpolated_surface.knotvector_u,
                    "knotvector_v": interpolated_surface.knotvector_v,
                    "ctrlpts": interpolated_surface.ctrlpts}
    if not os.path.exists('./interpolated-surfaces'):
        Path("./interpolated-surfaces").mkdir(parents=True, exist_ok=True)
    with open('./interpolated-surfaces/interpolated_surface_delta_z_' + str(input_z).replace('.', '_') + '.json', 'w',
              encoding='utf-8') as f:
        json.dump(surface_dict, f, ensure_ascii=False, indent=4)
