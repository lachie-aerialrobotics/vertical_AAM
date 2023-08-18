#! /usr/bin/env python3
import rospy
import tf2_ros
import copy
import os
import numpy as np
import ros_numpy as rnp
import open3d as o3d
import xml.etree.ElementTree as ET

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class LidarSim:
    def __init__(self):
        # get params from paremter server
        self.lidar_rate = rospy.get_param('/lidar_sim/rate')
        self.lidar_frame = rospy.get_param('/lidar_sim/lidar_frame')
        self.map_frame = rospy.get_param('/lidar_sim/map_frame')
        self.world_dir = rospy.get_param('/lidar_sim/world_dir')
        # voxel_size = 0.005
        self.noise_mu = rospy.get_param('/lidar_sim/noise_mu')
        self.noise_sigma = rospy.get_param('/lidar_sim/noise_sigma')
        self.world_samples = rospy.get_param('/lidar_sim/world_samples')

        #init tf listener
        self.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(1/self.lidar_rate))
        tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        #init publisher
        self.pub_map = rospy.Publisher('/map', PointCloud2, queue_size=1)
        dir = self.world_dir
        world_mesh = world_parser(dir)
        if len(world_mesh.triangles) == 0:
            rospy.logwarn("Input mesh has no triangles! Lidar sim will not start.")
        else:
            #computing pointcloud
            self.pcd_full = world_mesh.sample_points_uniformly(self.world_samples)

            self.pcd_full = self.pcd_full.translate((-rospy.get_param('/lidar_sim/x'),-rospy.get_param('/lidar_sim/y'),-rospy.get_param('/lidar_sim/z')))

            # Get approx diameter of map
            diameter = np.linalg.norm(np.asarray(self.pcd_full.get_max_bound()) - np.asarray(self.pcd_full.get_min_bound()))
            # Define parameters used for hidden_point_removal.
            self.radius = diameter * 100

            self.camera1 = [0, 0, 0]
            self.camera2 = [0, 0, 2]

            #init map cloud
            self.pcd_map = o3d.geometry.PointCloud()
            # Get all points that are visible from given view point.

            _, pt_map = self.pcd_full.hidden_point_removal(self.camera1, self.radius)
            pcd_view = self.pcd_full.select_by_index(pt_map)

            # apply noise to new pointcloud
            pcd_view = apply_noise(pcd_view, self.noise_mu, self.noise_sigma)
                
            # add to map
            self.pcd_map += pcd_view

            _, pt_map = self.pcd_full.hidden_point_removal(self.camera2, self.radius)
            pcd_view = self.pcd_full.select_by_index(pt_map)

            # apply noise to new pointcloud
            pcd_view = apply_noise(pcd_view, self.noise_mu, self.noise_sigma)
                
            # add to map
            self.pcd_map += pcd_view

            o3d.io.write_point_cloud("/home/lachie/aam_ws/src/tricopter/scans/map.pcd", self.pcd_map, print_progress=True)

            #start timer callback
            self.timer = rospy.Timer(rospy.Duration(1/self.lidar_rate), self.timer_callback)
            rospy.spin()

    def timer_callback(self, event):
        # try:
        #     tf = self.tfBuffer.lookup_transform(self.odom_frame, self.lidar_frame, time=rospy.Time.now(), timeout=rospy.Duration(1/self.lidar_rate))
        #     self.camera = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
            
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.loginfo("Lidar sim tf dropped")

        # # Get all points that are visible from given view point.
        # _, pt_map = self.pcd_full.hidden_point_removal(self.camera, self.radius)
        # pcd_view = self.pcd_full.select_by_index(pt_map)

        # # apply noise to new pointcloud
        # pcd_view = apply_noise(pcd_view, self.noise_mu, self.noise_sigma)
        
        # # add to map
        # self.pcd_map += pcd_view
        
        # # perform voxelgrid downsampling
        # self.pcd_map = self.pcd_map.voxel_down_sample(voxel_size=self.voxel_size)

        # publish as pc2 message
        header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
        pc_msg = o3d_to_pc2(self.pcd_map, header)
        self.pub_map.publish(pc_msg)
     
        
def o3d_to_pc2(o3d_pc=o3d.geometry.PointCloud(), pc_header=Header()):
    #convert back to structured array
    pc_np = np.asarray(o3d_pc.points)
    pc_arr = np.zeros(np.shape(pc_np)[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        # ('rgb', np.float32),
        ])
    pc_arr['x'] = pc_np[:, 0]
    pc_arr['y'] = pc_np[:, 1]
    pc_arr['z'] = pc_np[:, 2]
    #msgify back to pointcloud2
    pc2 = rnp.msgify(PointCloud2, pc_arr, stamp=pc_header.stamp, frame_id=pc_header.frame_id)
    return pc2

def apply_noise(pcd, mu, sigma):
    noisy_pcd = copy.deepcopy(pcd)
    points = np.asarray(noisy_pcd.points)
    points += np.random.normal(mu, sigma, size=points.shape)
    noisy_pcd.points = o3d.utility.Vector3dVector(points)
    return noisy_pcd


def world_parser(world_file):
    rospy.loginfo("Reading .world file located at: "+world_file)

    world_sdf = ET.parse(world_file).getroot() # parse .world file

    world_mesh = o3d.geometry.TriangleMesh() #init final combined mesh

    for model_uri in world_sdf.findall('world/include'): #iterate through model uris in world file
        model_sdf_path = os.path.dirname(world_file) + '/../models/' + model_uri.find('uri').text.lstrip('model://')+"/model.sdf"
        model_sdf_path = os.path.abspath(model_sdf_path)
        rospy.loginfo("Reading .sdf        located at: " + model_sdf_path)
        model_sdf_root = ET.parse(model_sdf_path).getroot() # parse .world file
        for link in model_sdf_root.findall('model/link'):
            try:
                pose = link.find('pose').text #get pose as string "x y z r p y"
                pose = [float(i) for i in pose.split()] #convert pose to array
                rospy.loginfo("Mesh pose is: "+str(pose))
                for mesh in link.findall('visual/geometry/mesh'):
                    mesh_uri = mesh.find('uri').text
                    mesh_path = os.path.abspath(model_sdf_path + '/../../' + mesh_uri.lstrip('model://').lstrip('file://'))
                    rospy.loginfo("Converting mesh     located at " + mesh_path)
                    if mesh_path.endswith(".stl") or mesh_path.endswith(".obj"):
                        mesh_component = o3d.io.read_triangle_mesh(mesh_path)
                        mesh_component.translate(pose[0:3]) # translate according to pose
                        r = mesh_component.get_rotation_matrix_from_xyz(pose[3:6])
                        mesh_component.rotate(r, center=pose[0:3]) #rotate
                        world_mesh += mesh_component
                        rospy.loginfo('Converted!')
                    elif mesh_path.endswith(".dae"):
                        rospy.logwarn('.dae meshes not supported at this time, please convert to .obj or .stl')
                        rospy.logwarn('Skipping...')
                    else:
                        rospy.logwarn('Unknown file extension') 
                        rospy.logwarn('Skipping...')
            except:
                rospy.logwarn('Conversion error!')
    rospy.loginfo('Done!')
    # visualisation of final mesh: uncomment for debugging
    # world_mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([world_mesh])
    return world_mesh 
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('lidar_sim')
    lS = LidarSim()
    