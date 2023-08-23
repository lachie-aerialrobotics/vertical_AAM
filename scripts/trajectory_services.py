#! /usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped, TransformStamped, Transform, Twist, WrenchStamped, Vector3, Vector3Stamped, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse
from vertical_aam.srv import *

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

def handle_fetch_poses(req):
    # service fetches waypoints from the ros parameter server for a given print layer. The format
    # should be as follows:

    # prefix:
    #   n_layers: N
    #   layer0:
    #       point0: [x, y, z, roll, pitch, yaw]
    #       ...     [x, y, z, roll, pitch, yaw]
    #       pointN: [x, y, z, roll, pitch, yaw]
    #   ...
    #   layerN:
    #       point0: [x, y, z, roll, pitch, yaw]
    #       ...

    # rospy.loginfo("Fetching poses from .yaml file")
    param_list = rospy.get_param_names()
    
    poses = PoseArray()
    poses.header.frame_id = req.frame_id
    poses.header.stamp = rospy.Time.now()

    if int(rospy.get_param("/"+str(req.prefix)+"/n_layers")) < req.layer_number:
        rospy.logwarn("Requested layer is greater than layers specified in print")

    points = list(filter(lambda k: str(req.prefix) in k, param_list))
    
    points = list(filter(lambda k: ("/layer" + str(req.layer_number)+"/") in k, points))

    if not points:
        rospy.logwarn("No waypoints found - please check .yaml file") 
    else:
        for i in range(len(points)):
            try:
                point_ref = list(filter(lambda k: ("/point" + str(i)) in k, points))  
                point = rospy.get_param(str(point_ref[0]))
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.position.z = -point[2] #z axis is reversed to match manipulator sign convention
                q = quaternion_from_euler(point[3], point[4], point[5])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                poses.poses.append(pose)
            except:
                rospy.logwarn("Error extracting waypoints - please check .yaml file")
                break

    resp = fetchPosesResponse()
    resp.poses = poses
    # rospy.loginfo("Poses returned")
    return resp

def handle_drone_trajectory(req):
    # service generates a drone trajectory offset from a supplied tooltip trajectory. x/y/z and yaw commands
    # are preserved and pitch/roll angles are ignored
    rospy.loginfo("Offset drone trajectory requested")

    drone_frame = req.drone_body_frame_id
    tip_frame = req.tooltip_frame_id

    # get tf from drone frame to tooltip frame
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    tf = tfBuffer.lookup_transform(tip_frame, drone_frame, time=rospy.Time.now(), timeout=rospy.Duration(5))
    tf_rotation = TransformStamped()
    tf_rotation.header = tf.header
    tf_rotation.child_frame_id = tf.child_frame_id
    tf_rotation.transform.rotation = tf.transform.rotation
    tf_rotation.transform.translation = Vector3(0,0,0)

    tf_inv = tfBuffer.lookup_transform(drone_frame, tip_frame, time=rospy.Time.now(), timeout=rospy.Duration(5))
    tf_inv_rotation = TransformStamped()
    tf_inv_rotation.header = tf_inv.header
    tf_inv_rotation.child_frame_id = tf_inv.child_frame_id
    tf_inv_rotation.transform.rotation = tf_inv.transform.rotation
    tf_inv_rotation.transform.translation = Vector3(0,0,0)

    resp = droneTrajectoryResponse()
    resp.drone_trajectory = MultiDOFJointTrajectory()
    resp.drone_trajectory.header.frame_id = req.toolpath_trajectory.header.frame_id
    resp.drone_trajectory.header.stamp = rospy.Time.now()

    for i in range(len(req.toolpath_trajectory.points)): 
        traj = do_transform_transform(tf_rotation, req.toolpath_trajectory.points[i].transforms[0])
        (roll, pitch, yaw) = euler_from_quaternion([traj.rotation.x,
                                                    traj.rotation.y,
                                                    traj.rotation.z,
                                                    traj.rotation.w])
        q = quaternion_from_euler(yaw,0,0,'szyx')
        traj.rotation = Quaternion(q[0],q[1],q[2],q[3])
        traj = do_transform_transform(tf_inv_rotation, traj)
        drone_transform = do_transform_transform(tf, traj)
        
        drone_trajectory_point = MultiDOFJointTrajectoryPoint()
        drone_trajectory_point.time_from_start = req.toolpath_trajectory.points[i].time_from_start
        drone_trajectory_point.transforms.append(drone_transform)
        drone_trajectory_point.velocities.append(req.toolpath_trajectory.points[i].velocities[0])
        drone_trajectory_point.accelerations.append(req.toolpath_trajectory.points[i].accelerations[0])
        resp.drone_trajectory.points.append(drone_trajectory_point)
    rospy.loginfo("Trajectory ready")
    return resp

def handle_transform_trajectory(req):
    # service takes a set of desired toolpath poses and transforms 
    # and transforms to the drone's frame of reference.

    frame_id_init = req.poses.header.frame_id
    frame_id_new = req.transformed_frame_id
    poses = req.poses
    
    rospy.loginfo("Trajectory transform requested from frame_id=" + frame_id_init + " to frame_id=" + frame_id_new + ".")
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    tf = tfBuffer.lookup_transform(frame_id_new, frame_id_init, time=rospy.Time.now(), timeout=rospy.Duration(5))

    resp = transformTrajectoryResponse()

    resp.poses_transformed = PoseArray()
    resp.poses_transformed.header.frame_id = frame_id_new
    resp.poses_transformed.header.stamp = rospy.Time.now()

    for i in range(len(poses.poses)):
        pose = Pose()
        pose = poses.poses[i]
        pose_transformed = do_transform_pose(pose, tf)
        resp.poses_transformed.poses.append(pose_transformed)

    rospy.loginfo("Trajectory ready")
    return resp

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
        transform1_transformed = Transform()
        transform1_transformed.translation = pose_transformed.pose.position
        transform1_transformed.rotation = pose_transformed.pose.orientation
        return transform1_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_transform()")
        rospy.logerr("Must be Tranform() or TransformStamped()")


def do_transform_pose(pose, transform):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(pose, Pose):
        pose = PoseStamped(pose=pose)
    if isinstance(transform, Transform):
        transform = TransformStamped(transform=transform)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(pose, PoseStamped) and isinstance(transform, TransformStamped):
        posestamped_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
        pose_transformed = Pose()
        pose_transformed = posestamped_transformed.pose
        return pose_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_pose()")
        rospy.logerr("Must be Pose() or PoseStamped()")
    

def do_transform_twist(twist, transform):
    #if either input is not ...Stamped() version of message, do conversion:
    if isinstance(twist, Twist):
        twist = TwistStamped(twist=twist)
    if isinstance(transform, Transform):
        transform = TransformStamped(transform=transform)
    #check that type is now correct, if not output an error readable by humans:
    if isinstance(twist, TwistStamped) and isinstance(transform, TransformStamped):
        wrench = WrenchStamped()
        wrench.wrench.force = twist.twist.linear
        wrench.wrench.torque = twist.twist.angular
        wrench_transformed = tf2_geometry_msgs.do_transform_wrench(
            wrench, transform)
        twist_transformed = Twist()
        twist_transformed.linear = wrench_transformed.wrench.force
        twist_transformed.angular = wrench_transformed.wrench.torque
        return twist_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_twist()")
        rospy.logerr("Must be Twist()/TwistStamped() and Transform()/TransformStamped()")

def trajectory_server():
    rospy.init_node('trajectory_services')
    # services for trajectory generation
    TOPPRA_service = rospy.Service(
        'get_TOPPRA_trajectory', TOPPRATrajectory, handle_TOPPRA_trajectory)
    transform_trajectory_service = rospy.Service(
        'get_transformed_trajectory', transformTrajectory, handle_transform_trajectory)
    drone_trajectory_service = rospy.Service(
        'get_drone_trajectory', droneTrajectory, handle_drone_trajectory)
    fetch_poses_service = rospy.Service(
        'fetch_poses', fetchPoses, handle_fetch_poses)
    rospy.spin()

if __name__ == "__main__":
    trajectory_server()