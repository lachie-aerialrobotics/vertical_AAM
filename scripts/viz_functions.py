#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory
from tricopter.srv import *

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

def publish_viz_poses(pose_array, publisher):
    # function to convert a PoseArray message to a Path message for visualisation in rviz
    if not (isinstance(pose_array, PoseArray) and isinstance(publisher, rospy.Publisher)):
        rospy.logerr("Incorrect input types")
    else:
        viz_path = Path()
        viz_path.header.frame_id = pose_array.header.frame_id
        viz_path.header.stamp = rospy.Time.now()
        for i in range(len(pose_array.poses)):
            pose = PoseStamped()
            pose.header.frame_id = viz_path.header.frame_id
            pose.header.stamp = viz_path.header.stamp            
            pose.pose.position.x = pose_array.poses[i].position.x
            pose.pose.position.y = pose_array.poses[i].position.y
            pose.pose.position.z = pose_array.poses[i].position.z
            pose.pose.orientation.x = pose_array.poses[i].orientation.x
            pose.pose.orientation.y = pose_array.poses[i].orientation.y
            pose.pose.orientation.z = pose_array.poses[i].orientation.z
            pose.pose.orientation.w = pose_array.poses[i].orientation.w
            viz_path.poses.append(pose)
        publisher.publish(viz_path)

def publish_viz_print(publisher):
    # function to visualise all layers in a print
    rospy.wait_for_service('fetch_poses')
    get_poses = rospy.ServiceProxy('fetch_poses', fetchPoses)
    request = fetchPosesRequest()
    request.prefix = "waypoints"
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "printing_plane"
    for i in range(rospy.get_param("/"+str(request.prefix)+"/n_layers")):
        request.layer_number = i
        response = get_poses(request)
        for j in range(len(response.poses.poses)):
            pose_array.poses.append(response.poses.poses[j])
    publish_viz_poses(pose_array, publisher)