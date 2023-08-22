#! /usr/bin/env python3
import open3d as o3d
import rospy
import numpy as np
import scipy as sp
import trimesh as tm
import tf2_ros
import copy
import os
import ros_numpy as rnp
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Twist, TwistStamped, WrenchStamped
from nav_msgs.msg import Path
from vertical_aam.srv import *
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse    

class PrintLayerGenerator:
    def __init__(self):
        # get config values from parameter server
        map_topic_name = "/aft_pgo_map"
        self.estimated_layer_height = 0.1
        self.scan_frame = 'camera_init'
        self.print_frame = 'print_origin'
        self.mesh_path = '../../print_model/cylinder.stl'

        self.print_origin_selected = get_print_origin()

        # init tf lookups
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(20.0))
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # init publishers
        self.print_scan_pub = rospy.Publisher('/print_scan', PointCloud2, queue_size=1, tcp_nodelay=True)
        self.trajectory_viz_pub = rospy.Publisher('/print_layer', Path , queue_size=1, tcp_nodelay=True)
        self.mesh_viz_pub = rospy.Publisher('/print_model', MarkerArray, queue_size=1)

        #don't start services until map is available
        rospy.wait_for_message(map_topic_name, PointCloud2) 
        rospy.loginfo('Map is ready!')

        # init subscribers
        sub_map = rospy.Subscriber(map_topic_name, PointCloud2, self.map_cb, queue_size=1)
        sub_map = rospy.Subscriber('/print_origin_current', PoseStamped, self.origin_cb, queue_size=1)

        # init services
        layer_generation_service = rospy.Service('generate_layer', generateLayer, self.generate_layer)
        print_init_service = rospy.Service('init_print', Empty, self.init_print)

        # timer callback
        rospy.Timer(rospy.Duration(0.1), self.timer_cb, reset=True)

        rospy.spin()

    def timer_cb(self, event):
        br = tf2_ros.TransformBroadcaster()
        self.print_origin_selected.header.stamp = rospy.Time.now()
        br.sendTransform(self.print_origin_selected)

    def map_cb(self, msg):
        # get current map
        self.scan = msg

    def origin_cb(self, pose_msg):
        self.print_origin = TransformStamped()
        self.print_origin.header = pose_msg.header
        self.print_origin.child_frame_id = self.print_frame
        self.print_origin.transform.translation = pose_msg.pose.position
        self.print_origin.transform.rotation = pose_msg.pose.orientation
    
    def generate_layer(self, req):
        print_origin_tf = self.print_origin_selected
        self.print_frame = print_origin_tf.child_frame_id
        self.map_frame = print_origin_tf.header.frame_id
        
        tf = self.tfBuffer.lookup_transform(self.scan_frame, self.map_frame, rospy.Time.now(), timeout=rospy.Duration(5))
        print_origin_tf = do_transform_transform(print_origin_tf, tf)
        print_origin_tf.header.frame_id = self.scan_frame
        print_origin_tf.child_frame_id = self.print_frame
        print_origin_tf.header.stamp = rospy.Time.now()

        mesh = get_mesh(self.mesh_path)
        self.mesh_viz(self.mesh_path)
        bbox = get_crop_region(mesh)
        bbox = transform_o3d(bbox, print_origin_tf)
        map = pc2_to_o3d(self.scan)

        # crop map to region on interest
        print_detected = map.crop(bbox)

        # get tf from map frame to printing plane and transform the pointcloud to have compatible coordinates with mesh
        tf = self.tfBuffer.lookup_transform(self.print_frame, self.scan_frame, rospy.Time.now(), timeout=rospy.Duration(1))
        print_detected = transform_o3d(print_detected, tf)
        # print_detected = clean_pcd(print_detected)

        self.print_scan_pub.publish(o3d_to_pc2(print_detected, Header(stamp=rospy.Time.now(), frame_id=self.print_frame)))

        traj = intersection_finder(print_detected, mesh, self.estimated_layer_height)
        traj.header.frame_id = self.print_frame

        publish_viz_trajectory(traj, self.trajectory_viz_pub)
        
        resp = generateLayerResponse()
        resp.trajectory = traj
        return resp
    
    def init_print(self, req):
        # populate response with latest collected print origin
        resp = EmptyResponse()
        # save data to file for re-use
        self.print_origin_selected = self.print_origin
        write_tf_to_yaml(self.print_origin_selected)
        return resp
    
    def mesh_viz(self, mesh_path="/"):
        markerArray = MarkerArray()
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)
        marker = Marker()
        marker.id = 0
        marker.mesh_resource = "package://vertical_aam/scripts/printing/"+mesh_path
        marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
        marker.type = marker.MESH_RESOURCE
        marker.header.frame_id = self.print_frame
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.6
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.pose.orientation.w = 1.0
        markerArray.markers.append(marker)

        rospy.loginfo('Published %d objects. ', len(markerArray.markers))
        self.mesh_viz_pub.publish(markerArray)

def transform_o3d(pc, tf=TransformStamped()):
    v, quat_wxyz = transform_to_numpy(tf, quat_order='wxyz')      
    pc.translate(v)
    pc.rotate(pc.get_rotation_matrix_from_quaternion(quat_wxyz), center=v)
    return pc

def pc2_to_o3d(pc2=PointCloud2()):
    pc_np = rnp.numpify(pc2)
    pc_arr = np.zeros([len(pc_np), 3])
    pc_arr[:,0] = pc_np['x']
    pc_arr[:,1] = pc_np['y']
    pc_arr[:,2] = pc_np['z']
    pc_arr = pc_arr[~np.isinf(pc_arr).any(axis=1)]
    pc_arr = pc_arr[~np.isnan(pc_arr).any(axis=1)]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pc_arr)
    return pc

def o3d_to_pc2(o3d_pc=o3d.geometry.PointCloud(), header=Header()):
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
    # pc_header = Header(stamp=rospy.Time.now(), frame_id='camera_init')
    pc2 = rnp.msgify(PointCloud2, pc_arr, stamp=header.stamp, frame_id=header.frame_id)
    return pc2

def transform_to_numpy(transform=TransformStamped(), quat_order='xyzw'):
    t_vec = np.asarray([transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z])
    if quat_order == 'xyzw':
        t_quat = np.asarray([transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w])
    elif quat_order == 'wxyz':
        t_quat = np.asarray([transform.transform.rotation.w,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z])
    return t_vec, t_quat

def get_crop_region(mesh):
    mesh_scaled = copy.deepcopy(mesh)
    mesh_scaled = mesh_scaled.scale(1.5, center=np.asarray([0,0,0]))
    bbox = mesh_scaled.get_axis_aligned_bounding_box()
    bbox = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(bbox)
    return bbox

def get_print_height(pcd, n_points=20):
    points = np.asarray(pcd.points)
    highest_points = np.argpartition(points[:,2], -n_points)[-n_points:]
    print_height = np.median(points[highest_points][:,2])
    # ensure result is not below zero so we cant fly into the printing plane
    if print_height < 0:
        print_height = 0
    return print_height

def continuous_path_from_segments(layer_segments):
    n = np.shape(layer_segments)[0]
    indicies = []
    path = np.zeros([n*2,3])
    j = 0
    segment = layer_segments[0]
    point = segment[0]
    for i in range(n):
        path[2*i,:] = point
        j = int(not(j))
        point = segment[j]
        path[2*i+1,:] = point
        for k in range(n):
            for l in range(2):
                if np.allclose(point, layer_segments[k,l]) and not np.any(np.equal(indicies,k)):
                    segment = layer_segments[k]
                    point = segment[l]
                    indicies.append(k) 
    return path[1::2] #remove duplicate nodes - probably could have been done in the loop

def mesh_plane_intersection(mesh, plane_origin=[0,0,0], plane_normal=[0,0,1]):
    #convert model to trimesh
    tri_mesh = tm.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles), vertex_normals=np.asarray(mesh.vertex_normals))
    # get layer segments from mesh/plane intersection
    layer_segments = tm.intersections.mesh_plane(mesh=tri_mesh, plane_origin=plane_origin, plane_normal=plane_normal)
    # assemble segments into a contnuous path, todo: atm this only supports closed contours
    path = continuous_path_from_segments(layer_segments)
    return path

def get_offsets(path, pcd):
    heights = []
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    n = np.shape(path)[0]
    for i in range(n):
        [k, idx, _] = pcd_tree.search_knn_vector_3d(o3d.utility.Vector3dVector(path)[i], 1)
        heights.append(path[i,2] - np.asarray(pcd.points)[idx,2][0])
    return heights

def material_model(layer_height, distance=0.1, flow_rate=0.1, expansion_ratio=20):
    speed = expansion_ratio / distance / flow_rate / layer_height
    return speed

def get_velocities(path, heights):
    use_material_model = rospy.get_param('/print_slicer/use_material_model')
    speed_default = rospy.get_param('/print_slicer/print_speed')
    n = np.shape(path)[0]
    velocities = np.zeros((n,3))
    for i in range(n):
        if use_material_model:
            speed = material_model(heights[i])
        else:
            speed = speed_default
        if i == 0:
            velocities[i] = np.zeros(3)
        else:
            edge = path[i] - path[i-1]
            edge_norm = np.linalg.norm(edge)
            if edge_norm > 0:
                velocities[i] = speed * edge / edge_norm
            else:
                velocities[i] = 0
    return velocities

def traj_from_arrays(positions, velocities):
    trajectory = MultiDOFJointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    n = np.shape(positions)[0]
    
    for i in range(n):
        trans = Transform()
        trans.translation.x = positions[i,0]
        trans.translation.y = positions[i,1]
        trans.translation.z = positions[i,2]
        q = quaternion_from_matrix()
        trans.rotation.x = q[0]
        trans.rotation.y = q[1]
        trans.rotation.z = q[2]
        trans.rotation.w = q[3]

        vel = Twist()
        vel.linear.x = velocities[i,0]
        vel.linear.y = velocities[i,1]
        vel.linear.z = velocities[i,2]
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        accel = Twist()
        accel.linear.x = 0
        accel.linear.y = 0
        accel.linear.z = 0
        accel.angular.x = 0
        accel.angular.y = 0
        accel.angular.z = 0

        if i == 0:
            t = 0
        else:
            t += np.linalg.norm(positions[i,:] - positions[i-1,:]) / np.linalg.norm(velocities[i,:])

        trajectory_point = MultiDOFJointTrajectoryPoint()
        trajectory_point.transforms.append(trans)
        trajectory_point.velocities.append(vel)
        trajectory_point.accelerations.append(accel)   
        trajectory_point.time_from_start = rospy.Duration(t)

        trajectory.points.append(trajectory_point)

    return trajectory

def publish_viz_trajectory(trajectory, publisher):
    # function to convert a MultiDOFJointTrajectory message to a Path message for visualisation in rviz
    if not (isinstance(trajectory, MultiDOFJointTrajectory) and isinstance(publisher, rospy.Publisher)):
        rospy.logerr("Incorrect input types")
    else:
        viz_path = Path()
        viz_path.header.frame_id = trajectory.header.frame_id
        viz_path.header.stamp = rospy.Time.now()
        for i in range(len(trajectory.points)):
            pose = PoseStamped()
            pose.header.frame_id = trajectory.header.frame_id
            pose.header.stamp = viz_path.header.stamp
            pose.pose.position.x = trajectory.points[i].transforms[0].translation.x
            pose.pose.position.y = trajectory.points[i].transforms[0].translation.y
            pose.pose.position.z = trajectory.points[i].transforms[0].translation.z
            pose.pose.orientation.x = trajectory.points[i].transforms[0].rotation.x
            pose.pose.orientation.y = trajectory.points[i].transforms[0].rotation.y
            pose.pose.orientation.z = trajectory.points[i].transforms[0].rotation.z
            pose.pose.orientation.w = trajectory.points[i].transforms[0].rotation.w
            viz_path.poses.append(pose)
        publisher.publish(viz_path)
    
def intersection_finder(pcd, mesh, layer_height):
    print_height = get_print_height(pcd)
    # print(print_height)
    new_layer_height = layer_height + print_height
    path = mesh_plane_intersection(mesh, np.asarray([0,0,new_layer_height]), np.asarray([0,0,1]))
    # print('path')
    # print(path)
    offsets = get_offsets(path, pcd)
    # print('offsets')
    # print(offsets)
    velocities = get_velocities(path, offsets)
    # print('velocities')
    # print(velocities)
    trajectory = traj_from_arrays(path, velocities)
    # print('trajectory')
    # print(trajectory)
    return trajectory

def quaternion_from_matrix(matrix=np.eye(3)):
    r = sp.spatial.transform.Rotation.from_matrix(matrix)
    q = r.as_quat()
    return q

def get_print_origin():
    try:
        print_origin_tf = TransformStamped()
        print_origin_tf.header.stamp = rospy.Time.now()
        print_origin_tf.header.frame_id = rospy.get_param("print_transform/frame_id")
        print_origin_tf.child_frame_id = rospy.get_param("print_transform/child_frame_id")
        print_origin_tf.transform.translation.x = rospy.get_param("print_transform/translation/x")
        print_origin_tf.transform.translation.y = rospy.get_param("print_transform/translation/y")
        print_origin_tf.transform.translation.z = rospy.get_param("print_transform/translation/z")
        print_origin_tf.transform.rotation.x = rospy.get_param("print_transform/orientation/x")
        print_origin_tf.transform.rotation.y = rospy.get_param("print_transform/orientation/y")
        print_origin_tf.transform.rotation.z = rospy.get_param("print_transform/orientation/z")
        print_origin_tf.transform.rotation.w = rospy.get_param("print_transform/orientation/w")
    except:
        rospy.loginfo("please set printing plane")
    return print_origin_tf

def transform_o3d(pc, tf=TransformStamped()):
    v, quat_wxyz = transform_to_numpy(tf, quat_order='wxyz')      
    pc.translate(v)
    pc.rotate(pc.get_rotation_matrix_from_quaternion(quat_wxyz), center=v)
    return pc

def get_crop_region(mesh):
    mesh_scaled = copy.deepcopy(mesh)
    mesh_scaled = mesh_scaled.scale(1.5, center=np.asarray([0,0,0]))
    bbox = mesh_scaled.get_axis_aligned_bounding_box()
    bbox = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(bbox)
    return bbox

def get_mesh(mesh_path="/"):
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    return mesh

def do_transform_transform(transform1, transform2):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(transform1, Transform):
        transform1 = TransformStamped(transform=transform1)
    if isinstance(transform2, Transform):
        transform2 = TransformStamped(transform=transform2)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(transform1, TransformStamped) and isinstance(transform2, TransformStamped):
        pose = PoseStamped()
        pose.pose.position = transform1.transform.translation
        pose.pose.orientation = transform1.transform.rotation
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform2)
        transform1_transformed = TransformStamped()
        transform1_transformed.transform.translation = pose_transformed.pose.position
        transform1_transformed.transform.rotation = pose_transformed.pose.orientation
        return transform1_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_transform()")
        rospy.logerr("Must be Tranform() or TransformStamped()")

def write_tf_to_yaml(tf=TransformStamped(), dir="../../cfg/print_config/"):
    # change working directory to this script - for saving .pcd files
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    filename = "print_origin.yaml"
    try:
        f = open(dir+filename, 'w')
        print(dir+filename)
        f.write('#DO_NOT_MODIFY: This file was autogenerated! \n\n')
        f.write('print_transform:\n')
        f.write('  frame_id: '+str(tf.header.frame_id)+'\n')
        f.write('  child_frame_id: '+str(tf.child_frame_id)+'\n')
        f.write('  translation:\n')
        f.write('    x: '+str(tf.transform.translation.x)+'\n')
        f.write('    y: '+str(tf.transform.translation.y)+'\n')
        f.write('    z: '+str(tf.transform.translation.z)+'\n')
        f.write('  orientation:\n')
        f.write('    x: '+str(tf.transform.rotation.x)+'\n')
        f.write('    y: '+str(tf.transform.rotation.y)+'\n')
        f.write('    z: '+str(tf.transform.rotation.z)+'\n')
        f.write('    w: '+str(tf.transform.rotation.w)+'\n')
        rospy.loginfo("Print origin saved succesfully!")
    except:
        rospy.logwarn("Writing print origin to file failed!")

if __name__ == "__main__":
    rospy.init_node('print_layer_generator_service_client')
    pLG = PrintLayerGenerator()