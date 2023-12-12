from geomdl import BSpline, tessellate
from geomdl.visualization import VisVTK
from os import listdir
from os.path import isfile, join
import json
import numpy as np
from scipy.spatial import distance
from scipy import interpolate
from vedo import mesh, plotter, pointcloud, shapes
import os
from pathlib import Path

database_path = './database'
surfaces = [f for f in listdir(database_path) if isfile(join(database_path, f))]
surface_paths = [database_path + '/' + surface for surface in surfaces]
delta = 0.01

total_evalpts = []
evalpts_surfaces = []
sample_size_u = 0
sample_size_v = 0

for surface_path in surface_paths:
    surface_file = open(surface_path)

    data = json.load(surface_file)

    surf = BSpline.Surface()
    surf.degree_u = data['degree_u']
    surf.degree_v = data['degree_v']
    surf.ctrlpts_size_u = data['size_u']
    surf.ctrlpts_size_v = data['size_v']

    surf.knotvector_u = data['knotvector_u']
    surf.knotvector_v = data['knotvector_v']
    surf.ctrlpts = data['ctrlpts']

    surf.delta = delta

    evalpts = np.array(surf.evalpts)

    if len(total_evalpts) == 0:
        total_evalpts = np.zeros(np.shape(evalpts))
    total_evalpts += evalpts

    sample_size_u = surf.sample_size_u
    sample_size_v = surf.sample_size_v

    evalpts_surfaces.append(evalpts)

total_evalpts = total_evalpts / len(surface_paths)
mean_points_tri = tessellate.make_triangle_mesh(total_evalpts.tolist(), sample_size_u, sample_size_v)
faces = [x.vertex_ids for x in mean_points_tri[1]]
vertices = [x.data for x in mean_points_tri[0]]
mean_mesh = mesh.Mesh([vertices, faces])
mean_mesh.computeNormals()

diff = [evalpts - np.array(mean_mesh.vertices()) for evalpts in evalpts_surfaces]
diff_norm = np.linalg.norm(diff, axis=2)

"""
normal_diffs = []
for i in range(len(diff)):
    surf_diff = diff[i]
    surf_diff_norm = diff_norm[i]
    surface_normal_diffs = []
    for j in range(len(diff[0])):
        normal = mean_mesh.normalAt(i)
        normal_component = (np.dot(surf_diff[j], normal) / surf_diff_norm[j] ** 2) * normal
        normal_component_norm = np.linalg.norm(normal_component)
        surface_normal_diffs.append(normal_component_norm)
    normal_diffs.append(surface_normal_diffs)
"""
normal_diff_std = np.std(diff_norm, axis=0)

max_std = max(normal_diff_std)
print(np.mean(normal_diff_std))
mean_mesh.cmap("Greys", normal_diff_std, vmin=0, vmax=max(abs(normal_diff_std)))
mean_mesh.addScalarBar(title='Standard Deviation in Normal Direction')


def display_points_on_click(evt):
    if not evt.actor:
        return
    point = evt.picked3d

    text = shapes.Text2D(str(mean_mesh.closestPoint(point)))
    if len(pltr.actors) > 1:
        pltr.remove(pltr.actors[1])
    pltr.add(text)


pltr = plotter.Plotter(title="Standard Deviation")
pltr.addCallback('mouse click', display_points_on_click)
pltr.show(mean_mesh)
