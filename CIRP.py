from geomdl import BSpline, tessellate
from geomdl.visualization import VisVTK
from os import listdir
from os.path import isfile, join
import json
import numpy as np
from scipy.spatial import distance
from scipy import interpolate
from vedo import mesh, plotter, pointcloud
import os
from pathlib import Path

database_path = './database'
reference_path = './database/reference/Tunnelverstaerkung_DC04_i0_6Export-Surfaces_stl_iter_0_gridpointsfinal.json_surface.json'
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

points = np.array([[i, j] for i, j in zip(x, y)])

delta = 0.005
tol = 0.007
b = 0.006


def evaluate_points(surf_paths):
    surfaces_z_coordinates_dict_func = {}
    reference_surface_z_func = []

    for surface_path_iter in surf_paths:
        print(surface_path_iter)
        surface_file = open(surface_path_iter)

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
                    print(err)
                    new_delta = new_delta * 0.5
                    surf.delta = new_delta
                    print(surf.delta)

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
                if surface_path_iter == reference_path:
                    reference_surface_z_func.append(z_val)
                else:
                    query_points_z = np.append(query_points_z, z_val)

            else:
                # read_point_idx, _ = np.where(coordinate_data_x_y == point[:2])
                read_point_idx = np.argmin(point_distances)
                read_point = coordinate_data[read_point_idx]
                if surface_path_iter == reference_path:
                    reference_surface_z_func.append(read_point[2])
                else:
                    query_points_z = np.append(query_points_z, read_point[2])
        data['point_coordinates'] = coordinate_data.tolist()
        print(data['point_coordinates'])
        if not surface_path_iter == reference_path:
            surfaces_z_coordinates_dict_func[surface_path_iter] = query_points_z

        if new_points_present:
            with open(surface_path_iter, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=4)

    return surfaces_z_coordinates_dict_func, reference_surface_z_func


surfaces_z_coordinates_dict, reference_surface_z = evaluate_points(surface_paths)

surfaces_z_deviations = {key: np.array(reference_surface_z) - value for key, value in
                         surfaces_z_coordinates_dict.items()}
mean_err_surfaces = {key: np.mean(np.abs(np.array(reference_surface_z) - value)) for key, value in
                     surfaces_z_coordinates_dict.items()}

max_num_points_up = 0
max_num_points_up_surfaces = []

max_num_points_down = 0
max_num_points_down_surfaces = []

for surface_path, deviations in surfaces_z_deviations.items():
    num_points_up = (deviations >= 0).sum()
    if num_points_up > max_num_points_up:
        max_num_points_up = num_points_up
        max_num_points_up_surfaces = [surface_path]
    elif num_points_up == max_num_points_up:
        max_num_points_up_surfaces.append(surface_path)

    num_points_down = (deviations < 0).sum()
    if num_points_down > max_num_points_down:
        max_num_points_down = num_points_down
        max_num_points_down_surfaces = [surface_path]
    elif num_points_down == max_num_points_down:
        max_num_points_down_surfaces.append(surface_path)

if len(max_num_points_up_surfaces) > 1:
    min_dev_surf_up = ''
    min_dev_up = 999999
    for surf_path in max_num_points_up_surfaces:
        mean_deviation = mean_err_surfaces[surf_path]
        if abs(mean_deviation) < min_dev_up:
            min_dev_up = abs(mean_deviation)
            min_dev_surf_up = surf_path
else:
    min_dev_surf_up = max_num_points_up_surfaces[0]

if len(max_num_points_down_surfaces) > 1:
    min_dev_surf_down = ''
    min_dev_down = 999999
    for surf_path in max_num_points_down_surfaces:
        mean_deviation = mean_err_surfaces[surf_path]
        if abs(mean_deviation) < min_dev_down:
            min_dev_down = abs(mean_deviation)
            min_dev_surf_down = surf_path
else:
    min_dev_surf_down = max_num_points_down_surfaces[0]

up_surf_file = open(min_dev_surf_up)
up_surf_data = json.load(up_surf_file)
up_ctrlpts = np.array(up_surf_data['ctrlpts'])

down_surf_file = open(min_dev_surf_down)
down_surf_data = json.load(down_surf_file)
down_ctrlpts = np.array(down_surf_data['ctrlpts'])

z = 0
z1 = np.mean(surfaces_z_deviations[min_dev_surf_up])
z0 = np.mean(surfaces_z_deviations[min_dev_surf_down])

interpolated_ctrlpts = up_ctrlpts * (z1 - z) + down_ctrlpts * (z - z0)
interpolated_ctrlpts = interpolated_ctrlpts / (z1 - z0)

interpolated_surface = BSpline.Surface()
interpolated_surface.degree_u = up_surf_data['degree_u']
interpolated_surface.degree_v = up_surf_data['degree_v']
interpolated_surface.ctrlpts_size_u = up_surf_data['size_u']
interpolated_surface.ctrlpts_size_v = up_surf_data['size_v']

interpolated_surface.knotvector_u = up_surf_data['knotvector_u']
interpolated_surface.knotvector_v = up_surf_data['knotvector_v']
interpolated_surface.ctrlpts = interpolated_ctrlpts.tolist()

interpolated_surface.delta = delta

interpolated_surface.vis = VisVTK.VisSurface()
# interpolated_surface.render()

surface_dict = {"size_u": interpolated_surface.ctrlpts_size_u,
                "size_v": interpolated_surface.ctrlpts_size_v,
                "degree_u": interpolated_surface.degree_u,
                "degree_v": interpolated_surface.degree_v,
                "knotvector_u": interpolated_surface.knotvector_u,
                "knotvector_v": interpolated_surface.knotvector_v,
                "ctrlpts": interpolated_surface.ctrlpts}
if not os.path.exists('./interpolated-surfaces'):
    Path("./interpolated-surfaces").mkdir(parents=True, exist_ok=True)
with open('./interpolated-surfaces/interpolated_surface.json', 'w', encoding='utf-8') as f:
    json.dump(surface_dict, f, ensure_ascii=False, indent=4)

interpolated_surface_z_coordinates_dict, _ = evaluate_points(
    ['./interpolated-surfaces/interpolated_surface.json'])

interpolated_surfaces_z_deviations = {key: np.array(reference_surface_z) - value for key, value in
                                      interpolated_surface_z_coordinates_dict.items()}
print(interpolated_surfaces_z_deviations)

interpolated_trimesh = tessellate.make_triangle_mesh(interpolated_surface.evalpts,
                                                     interpolated_surface.sample_size_u,
                                                     interpolated_surface.sample_size_v)
faces = [x.vertex_ids for x in interpolated_trimesh[1]]
vertices = [x.data for x in interpolated_trimesh[0]]
interpolated_mesh = mesh.Mesh([vertices, faces])

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

interpolated_mesh.distanceToMesh(ref_mesh, signed=True)
interpolated_mesh_dist = interpolated_mesh.getPointArray("Distance")

max_dist = max(abs(interpolated_mesh_dist))
interpolated_mesh.cmap("jet", interpolated_mesh_dist, vmin=-0.581, vmax=0.581)
interpolated_mesh.addScalarBar(title='Signed\nDistance')
interpolated_mesh.show()

print('Maximale Abweichung: ', max(abs(interpolated_mesh_dist)))
print('Mittlere Abweichung: ', np.mean(abs(interpolated_mesh_dist)))
print('Standard Abweichung: ', np.std(abs(interpolated_mesh_dist)))