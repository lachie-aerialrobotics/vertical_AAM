#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, PoseArray, PoseStamped, TwistStamped, Transform, Vector3
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tricopter.srv import *
from viz_functions import *

class trajectoryHandler:
    def __init__(self, frequency, max_vel, max_acc, max_yawrate, max_yawrate_dot):
        self.frequency = frequency
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_yawrate = max_yawrate
        self.max_yawrate_dot = max_yawrate_dot
        
        self.world_frame_id = "map"
        self.drone_frame_id = "base_link"
        self.tooltip_frame_id = "tooltip_init"
        self.print_frame_id = "printing_plane"
        self.waypoint_prefix = "waypoints"

        self._point_count = 0    
        self._drone_trajectory = MultiDOFJointTrajectory()
        self._drone_trajectory.header.frame_id = self.world_frame_id
        self._tooltip_trajectory = MultiDOFJointTrajectory()
        self._transition_trajectory = MultiDOFJointTrajectory()
        self.loiter_point = PoseStamped()
        self.loiter_point.header.frame_id = self.world_frame_id

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.world_frame_id
        self.drone_pose = PoseStamped(header=header)
        self.drone_velocity = TwistStamped(header=header)
        self.drone_acceleration = TwistStamped(header=header)

        self._n_layers = rospy.get_param(str(self.waypoint_prefix)+'/n_layers')

        # publishers for visualisation only
        self._pub_toolpath_viz = rospy.Publisher('/viz/toolpath', Path, queue_size=1)
        self._pub_dronepath_viz = rospy.Publisher('/viz/dronepath', Path, queue_size=1)
        self._pub_dronetransitionpath_viz = rospy.Publisher('/viz/dronetransitionpath', Path, queue_size=1)
        # self._pub_tooltransitionpath_viz = rospy.Publisher('/viz/toltransitionpath', Path, queue_size=1)
        self._pub_print_viz = rospy.Publisher('/viz/print', Path, queue_size=1) 
        self._pub_loiter_viz = rospy.Publisher('/viz/loiter_point', PoseStamped, queue_size=1) 

        # publish print visualisation periodically
        rospy.Timer(rospy.Duration(5.0), self._viz_timer_cb, reset=True)

    def follow_print_trajectory(self):
        header = self._drone_trajectory.header
        
        tooltip_pose = PoseStamped(header=header)
        tooltip_velocity = TwistStamped(header=header)
        tooltip_acceleration = TwistStamped(header=header)

        if self._point_count < len(self._drone_trajectory.points):   
            self.drone_pose, self.drone_velocity, self.drone_acceleration = self._read_trajectory(self._drone_trajectory, self._point_count)
            tooltip_pose, tooltip_velocity, tooltip_acceleration = self._read_trajectory(self._tooltip_trajectory, self._point_count)
            self._point_count += 1
            complete = False
        else: 
            complete = True
            self._point_count = 0
            
        return self.drone_pose, self.drone_velocity, self.drone_acceleration, tooltip_pose, tooltip_velocity, tooltip_acceleration, complete

    def follow_transition_trajectory(self):
        if self._point_count < len(self._transition_trajectory.points):   
            self.drone_pose, self.drone_velocity, self.drone_acceleration = self._read_trajectory(self._transition_trajectory, self._point_count)
            self._point_count += 1
            complete = False
        else: 
            complete = True
            self._point_count = 0
        return self.drone_pose, self.drone_velocity, self.drone_acceleration, complete

    def get_print_start_pose(self):
        header = self._drone_trajectory.header
        pose = PoseStamped(header=header)
        
        pose.pose.position = self._drone_trajectory.points[0].transforms[0].translation
        pose.pose.orientation = self._drone_trajectory.points[0].transforms[0].rotation
        return pose

    def generate_transition(self, start_pose, end_pose):
        self._point_count = 0 #ensure point count is reset in case last trajectory was interrupted
        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = start_pose.header.frame_id
        if start_pose.pose != end_pose.pose:
            if start_pose.header.frame_id != end_pose.header.frame_id:
                rospy.logerr("Cannot interpolate between poses in different reference frames.")
            else:
                poses.poses.append(start_pose.pose)
                poses.poses.append(end_pose.pose)        
                self._transition_trajectory = self._TOPPRA_interpolation(poses)
                # tip_trajectory = self._offset_tip_trajectory(self._transition_trajectory)
                publish_viz_trajectory(self._transition_trajectory, self._pub_dronetransitionpath_viz)
                # publish_viz_trajectory(tip_trajectory, self._pub_tooltransitionpath_viz)
                # return trajectory

    def generate_print_layer(self, layer_number):
        self._point_count = 0 #ensure point count is reset in case last trajectory was interrupted
        print_waypoints = self._fetch_waypoints_from_yaml(layer_number)
        print_waypoints_transformed = self._transform_trajectory(print_waypoints)
        self._tooltip_trajectory = self._TOPPRA_interpolation(print_waypoints_transformed)
        self._drone_trajectory = self._offset_drone_trajectory(self._tooltip_trajectory)
        publish_viz_trajectory(self._drone_trajectory, self._pub_dronepath_viz)
        publish_viz_trajectory(self._tooltip_trajectory, self._pub_toolpath_viz)
        # return drone_trajectory, tooltip_trajectory

    def get_loiter_point(self, layer, offset=[0, 0, 0]):
        def transformPose(pose):
            fake_array = PoseArray()
            fake_array.header.frame_id = self.print_frame_id
            fake_array.header.stamp = rospy.Time.now()
            fake_array.poses.append(pose)
            transformed_fake_array = self._transform_trajectory(fake_array)
            transformed_point = PoseStamped()
            transformed_point.header = transformed_fake_array.header
            transformed_point.pose = transformed_fake_array.poses[0]
            return transformed_point

        top_waypoints = self._fetch_waypoints_from_yaml(layer)
   
        loiter_point = Pose()
        loiter_point.position.x = top_waypoints.poses[0].position.x + offset[0]
        loiter_point.position.y = top_waypoints.poses[0].position.y + offset[1]
        loiter_point.position.z = top_waypoints.poses[0].position.z - offset[2]
        loiter_point.orientation.x = 0.0
        loiter_point.orientation.y = 0.0
        loiter_point.orientation.z = 0.0
        loiter_point.orientation.w = 1.0
        
        transformed_loiter_point = transformPose(loiter_point)

        fake_transform = Transform(translation=transformed_loiter_point.pose.position, rotation=transformed_loiter_point.pose.orientation)
        fake_points = MultiDOFJointTrajectoryPoint()
        fake_points.transforms.append(fake_transform)
        fake_points.time_from_start = rospy.Duration(0)
        fake_points.velocities.append(Twist())
        fake_points.accelerations.append(Twist())
        fake_trajectory = MultiDOFJointTrajectory(header=transformed_loiter_point.header)
        fake_trajectory.points.append(fake_points)
        fake_offset_trajectory = self._offset_drone_trajectory(fake_trajectory)

        self.loiter_point = PoseStamped()
        self.loiter_point.header.frame_id = self.world_frame_id
        self.loiter_point.header.stamp = rospy.Time.now()
        self.loiter_point.pose.position.x = fake_offset_trajectory.points[0].transforms[0].translation.x
        self.loiter_point.pose.position.y = fake_offset_trajectory.points[0].transforms[0].translation.y
        self.loiter_point.pose.position.z = fake_offset_trajectory.points[0].transforms[0].translation.z
        self.loiter_point.pose.orientation = fake_offset_trajectory.points[0].transforms[0].rotation

        return self.loiter_point

    def _read_trajectory(self, trajectory, point_num):
        pose = PoseStamped()
        velocity = TwistStamped()
        acceleration = TwistStamped() 
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = trajectory.header.frame_id
        velocity.header.stamp = rospy.Time.now()
        velocity.header.frame_id = trajectory.header.frame_id
        acceleration.header.stamp = rospy.Time.now()
        acceleration.header.frame_id = trajectory.header.frame_id
        pose.pose.position = trajectory.points[point_num].transforms[0].translation
        pose.pose.orientation = trajectory.points[point_num].transforms[0].rotation
        velocity.twist = trajectory.points[point_num].velocities[0]
        acceleration.twist = trajectory.points[point_num].accelerations[0]
        return pose, velocity, acceleration

    def _fetch_waypoints_from_yaml(self, layer_number):
        # get poses from file
        # rospy.wait_for_service('fetch_poses')
        get_poses = rospy.ServiceProxy('fetch_poses', fetchPoses)
        request = fetchPosesRequest()
        request.prefix = self.waypoint_prefix
        request.frame_id = self.print_frame_id
        request.layer_number = layer_number
        response = get_poses(request)
        return response.poses

    def _transform_trajectory(self, poses):
        #transform to world coordinates system
        # rospy.wait_for_service('get_transformed_trajectory')
        transform_poses = rospy.ServiceProxy('get_transformed_trajectory', transformTrajectory)
        request = transformTrajectoryRequest()
        request.poses = poses
        request.transformed_frame_id = self.world_frame_id
        response = transform_poses(request)
        return response.poses_transformed

    def _TOPPRA_interpolation(self, poses):
        #interpolate with TOPPRA
        # rospy.wait_for_service('get_TOPPRA_trajectory')
        get_traj = rospy.ServiceProxy('get_TOPPRA_trajectory', TOPPRATrajectory)
        request = TOPPRATrajectoryRequest()
        request.frequency = self.frequency
        request.max_vel = self.max_vel
        request.max_acc = self.max_acc
        request.max_yawrate = self.max_yawrate
        request.max_yawrate_dot = self.max_yawrate_dot
        request.poses = poses
        response = get_traj(request)
        return response.trajectory

    def _offset_drone_trajectory(self, trajectory):
        #get offset drone trajectory
        # rospy.wait_for_service('get_drone_trajectory')
        get_traj = rospy.ServiceProxy('get_drone_trajectory', droneTrajectory)
        request = droneTrajectoryRequest()
        request.drone_body_frame_id = self.drone_frame_id
        request.tooltip_frame_id = self.tooltip_frame_id
        request.toolpath_trajectory = trajectory
        response = get_traj(request)
        return response.drone_trajectory

    # def _offset_tip_trajectory(self, trajectory):
    #     #get offset drone trajectory
    #     rospy.wait_for_service('get_drone_trajectory')
    #     get_traj = rospy.ServiceProxy('get_drone_trajectory', droneTrajectory)
    #     request = droneTrajectoryRequest()
    #     request.drone_body_frame_id = self.tooltip_frame_id
    #     request.tooltip_frame_id = self.drone_frame_id
    #     request.toolpath_trajectory = trajectory
    #     response = get_traj(request)
    #     return response.drone_trajectory

    def _viz_timer_cb(self, event):
        publish_viz_print(self._pub_print_viz)
        self._pub_loiter_viz.publish(self.loiter_point)