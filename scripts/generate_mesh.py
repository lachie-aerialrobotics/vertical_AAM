import open3d as o3d
import numpy as np

mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.2, height=0.8, resolution=100, split=50)
mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
mesh = mesh.translate(np.asarray([0, 0, 0.3]))
o3d.io.write_triangle_mesh("../print_model/cylinder.stl", mesh)
o3d.visualization.draw_geometries([mesh])