from geomdl import BSpline, tessellate
from geomdl.visualization import VisVTK
from os import listdir
from os.path import isfile, join
import json
import numpy as np
from scipy.spatial import distance
from vedo import mesh, plotter, pointcloud
import os
from geomdl import multi
from pathlib import Path

database_path = './database'
min_err_surf_path = './database/nn/Tunnelverstaerkung_DC04_i0_Testdata3_Net7_10-50-100_Reg1_Export-Surfaces_stl_iter_0_gridpointsfinal.json_surface.json'
reference_path = './database/reference/Tunnelverstaerkung_DC04_i0_24Export-Surfaces_stl_iter_0_gridpointsfinal.json_surface.json'


'''''
x = [131, 38, 215, 108, -5, 276, 224]
y = [10, 42, 24, -58, 0, -11, -37]
'''''


delta = 0.005


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
min_err_mesh.cmap("jet", mesh_dist, vmin=-max_dist, vmax=max_dist)
min_err_mesh.addScalarBar(title='Signed\nDistance')

print('Maximale Abweichung: ', max(abs(mesh_dist)))
print('Mittlere Abweichung: ', np.mean(abs(mesh_dist)))
print('Standard Abweichung: ', np.std(abs(mesh_dist)))

pltr = plotter.Plotter(title="Surface with Minimum Mean Error to Reference Surface", shape=[1, 1], sharecam=False)
pltr.show(min_err_mesh, at=0)

msurf = multi.SurfaceContainer(min_err_surf, ref_surf)
msurf.vis = VisVTK.VisSurface()
msurf.delta = 0.01
msurf.render(cpcolor=("green"))
msurf.tessellator()