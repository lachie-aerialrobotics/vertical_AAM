import open3d as o3d
import numpy as np

height = 2.1
mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.2, height=height, resolution=100, split=50)
mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
mesh = mesh.translate(np.asarray([0, 0, height-0.1]))
o3d.io.write_triangle_mesh("../print_model/cylinder.stl", mesh)
o3d.visualization.draw_geometries([mesh])