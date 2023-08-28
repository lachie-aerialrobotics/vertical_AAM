#! /usr/bin/env python3
import rospy
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import tf2_geometry_msgs
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Transform, TransformStamped, WrenchStamped, Twist, PoseArray, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, unit_vector, quaternion_multiply, quaternion_conjugate

class trajectory:
    def __init__(self, traj_init=MultiDOFJointTrajectory(), frame_id='map'):
        self.trajectory = traj_init
        self.trajectory.header.stamp = rospy.Time.now()
        self.trajectory.header.frame_id = frame_id
        self.start_time = rospy.Time.now()
        self.i = 0
        self.complete = False
        self.started = False

    def follow(self):
        if self.started == False:
            self.start_time = rospy.Time.now()
            self.started = True
        pose = PoseStamped()
        pose.header.frame_id = self.trajectory.header.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = self.trajectory.points[self.i].transforms[0].translation
        pose.pose.orientation = self.trajectory.points[self.i].transforms[0].rotation
        twist = TwistStamped()
        twist.header.frame_id = self.trajectory.header.frame_id
        twist.header.stamp = rospy.Time.now()
        twist.twist = self.trajectory.points[self.i].velocities[0]
        while ((self.trajectory.points[self.i].time_from_start.to_sec() + self.start_time.to_sec()) < rospy.Time.now().to_sec()) and (self.i < len(self.trajectory.points)-1):
            self.i += 1
        if self.i == len(self.trajectory.points)-1:
            self.complete = True
        return self.complete, pose, twist
    
    def append_traj(self, trajectory_app=MultiDOFJointTrajectory()):
        if len(self.trajectory.points) != 0:
            last_start_time = self.trajectory.points[-1].time_from_start
            for i in range(len(trajectory_app.points)-1):
                trajectory_app.points[i].time_from_start += last_start_time
                self.trajectory.points.append(trajectory_app.points[i])
        else:
            self.trajectory.points = trajectory_app.points

    def transition(self, start_pose=PoseStamped(), end_pose=PoseStamped(), rate=30):
        vel = rospy.get_param('/print_planner/transition_vel')
        acc = rospy.get_param('/print_planner/transition_max_accel')
        yawrate = rospy.get_param('/print_planner/max_yawrate')

        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = start_pose.header.frame_id
        poses.poses.append(start_pose.pose)
        poses.poses.append(end_pose.pose)        
        transition_trajectory = self._TOPPRA_interpolation(poses, vel, acc, yawrate, 1000, rate)
        self.append_traj(transition_trajectory)
        return transition_trajectory

    def pause(self, pose=PoseStamped(), time=0, rate=10):
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id = pose.header.frame_id
        trajectory.header.stamp = pose.header.stamp
        for i in range(int(time*rate)):
            point = MultiDOFJointTrajectoryPoint()
            point.transforms.append(Transform(pose.pose.position, pose.pose.orientation))
            point.velocities.append(Twist(Vector3(0,0,0),Vector3(0,0,0)))
            point.time_from_start = rospy.Duration(float(i / rate))
            trajectory.points.append(point)
        self.append_traj(trajectory)
        return trajectory
    
    def transform_trajectory(self, traj=MultiDOFJointTrajectory(), tf2_transform=TransformStamped()):
        for i in range(len(traj.points)-1):
            traj.points[i].transforms[0] = self.do_transform_transform(traj.points[i].transforms[0], tf2_transform).transform
            traj.points[i].velocities[0] = self.do_transform_twist(traj.points[i].velocities[0], tf2_transform).twist
        traj.header.frame_id = tf2_transform.header.frame_id
        return traj
    
    def rotate_trajectory(self, traj=MultiDOFJointTrajectory(), tf2_transform=TransformStamped()):
        #todo: fix this hardcoded fudge:
        q1 = np.asarray([0,0,1,0])
        
        for i in range(len(traj.points)-1): 
            q2 = np.asarray([traj.points[i].transforms[0].rotation.x,
                traj.points[i].transforms[0].rotation.y,
                traj.points[i].transforms[0].rotation.z,
                traj.points[i].transforms[0].rotation.w])
            
            q3 = quaternion_multiply(q1, q2)
            traj.points[i].transforms[0].rotation.x = q3[0]
            traj.points[i].transforms[0].rotation.y = q3[1]
            traj.points[i].transforms[0].rotation.z = q3[2]
            traj.points[i].transforms[0].rotation.w = q3[3]
        return traj
    
    def offset_trajectory(self, toolpath_trajectory, tf_tip2drone):
        # service generates a drone trajectory offset from a supplied tooltip trajectory. x/y/z and yaw commands
        # are preserved and pitch/roll angles are ignored
        rospy.loginfo("Offset drone trajectory requested")

        tf = tf_tip2drone
        tf_rotation = TransformStamped()
        tf_rotation.header = tf.header
        tf_rotation.child_frame_id = tf.child_frame_id
        tf_rotation.transform.rotation = tf.transform.rotation
        tf_rotation.transform.translation = Vector3(0,0,0)

        q = np.asarray([tf_rotation.transform.rotation.x,
                       tf_rotation.transform.rotation.y,
                       tf_rotation.transform.rotation.z,
                       tf_rotation.transform.rotation.w])
        
        q_inv = quaternion_inverse(q)

        tf_inv_rotation = TransformStamped()
        tf_inv_rotation.header = tf.header
        tf_inv_rotation.transform.rotation.x = q_inv[0]
        tf_inv_rotation.transform.rotation.y = q_inv[1]
        tf_inv_rotation.transform.rotation.z = q_inv[2]
        tf_inv_rotation.transform.rotation.w = q_inv[3]
        tf_inv_rotation.transform.translation = Vector3(0,0,0)

        offset_trajectory = MultiDOFJointTrajectory()
        offset_trajectory.header.frame_id = toolpath_trajectory.header.frame_id
        offset_trajectory.header.stamp = rospy.Time.now()

        for i in range(len(toolpath_trajectory.points)): 
            traj = self.do_transform_transform(tf_rotation, toolpath_trajectory.points[i].transforms[0])
            (roll, pitch, yaw) = euler_from_quaternion([traj.transform.rotation.x,
                                                        traj.transform.rotation.y,
                                                        traj.transform.rotation.z,
                                                        traj.transform.rotation.w])
            q = quaternion_from_euler(yaw,0,0,'szyx')
            traj.transform.rotation = Quaternion(q[0],q[1],q[2],q[3])
            traj = self.do_transform_transform(tf_inv_rotation, traj)
            offset_transform = self.do_transform_transform(tf, traj)

            vel = toolpath_trajectory.points[i].velocities[0]
            vel = self.do_transform_twist(vel, tf)
            
            offset_trajectory_point = MultiDOFJointTrajectoryPoint()
            offset_trajectory_point.time_from_start = toolpath_trajectory.points[i].time_from_start
            offset_trajectory_point.transforms.append(offset_transform.transform)

            offset_trajectory_point.velocities.append(vel.twist)
            offset_trajectory.points.append(offset_trajectory_point)
        rospy.loginfo("Trajectory ready")
        return offset_trajectory

    def _TOPPRA_interpolation(self, poses, max_vel, max_acc, max_yawrate, max_yawrate_dot, frequency):
        # generates a smooth, interpolated, time-optimal trajectory from an array of poses using TOPPRA package
        num_poses = len(poses.poses)
        rospy.loginfo("Trajectory requested. Interpolating " + str(num_poses) + " poses at " + str(frequency) + "Hz.")
        way_pts = np.zeros((num_poses, 6))
        for i in range(num_poses):
            (roll, pitch, yaw) = euler_from_quaternion([poses.poses[i].orientation.x,
                                                        poses.poses[i].orientation.y,
                                                        poses.poses[i].orientation.z,
                                                        poses.poses[i].orientation.w])
            way_pts[i,:] = [poses.poses[i].position.x, poses.poses[i].position.y, poses.poses[i].position.z, roll, pitch, yaw]
        ss = np.linspace(0, 1, num_poses)
        amax = max_acc
        vmax = max_vel
        max_yawrate = np.deg2rad(max_yawrate)
        max_yawrate_dot = np.deg2rad(max_yawrate_dot)
        vlims = [vmax, vmax, vmax, max_yawrate, max_yawrate, max_yawrate]
        alims = [amax, amax, amax, max_yawrate_dot, max_yawrate_dot, max_yawrate_dot]
        path = ta.SplineInterpolator(ss, way_pts)
        pc_vel = constraint.JointVelocityConstraint(vlims)
        pc_acc = constraint.JointAccelerationConstraint(alims)
        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        traj = instance.compute_trajectory(0, 0)
        n_samples = int(traj.duration * frequency)
        ts_sample = np.linspace(0, traj.duration, n_samples)
        qs_sample = traj(ts_sample, 0) #position setpoints
        qds_sample = traj(ts_sample, 1) #velocity setpoints
        qdds_sample = traj(ts_sample, 2) #acceleration setpoints
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id = poses.header.frame_id
        trajectory.header.stamp = rospy.Time.now()
        for i in range(n_samples):
            trans = Transform()
            trans.translation.x = qs_sample[i,0]
            trans.translation.y = qs_sample[i,1]
            trans.translation.z = qs_sample[i,2]
            q = quaternion_from_euler(qs_sample[i,3], qs_sample[i,4], qs_sample[i,5])
            trans.rotation.x = q[0]
            trans.rotation.y = q[1]
            trans.rotation.z = q[2]
            trans.rotation.w = q[3]
            vel = Twist()
            vel.linear.x = qds_sample[i,0]
            vel.linear.y = qds_sample[i,1]
            vel.linear.z = qds_sample[i,2]
            vel.angular.x = qds_sample[i,3]
            vel.angular.y = qds_sample[i,4]
            vel.angular.z = qds_sample[i,5]
            accel = Twist()
            accel.linear.x = qdds_sample[i,0]
            accel.linear.y = qdds_sample[i,1]
            accel.linear.z = qdds_sample[i,2]
            accel.angular.x = qdds_sample[i,3]
            accel.angular.y = qdds_sample[i,4]
            accel.angular.z = qdds_sample[i,5]
            trajectory_point = MultiDOFJointTrajectoryPoint()
            trajectory_point.transforms.append(trans)
            trajectory_point.velocities.append(vel)
            trajectory_point.accelerations.append(accel)
            trajectory_point.time_from_start = rospy.Duration(i / frequency)
            trajectory.points.append(trajectory_point)
        rospy.loginfo("Trajectory ready")
        return trajectory
    
    def publish_viz_trajectory(self, publisher):
        # function to convert a MultiDOFJointTrajectory message to a Path message for visualisation in rviz
        if not (isinstance(self.trajectory, MultiDOFJointTrajectory) and isinstance(publisher, rospy.Publisher)):
            rospy.logerr("Incorrect input types")
        else:
            viz_path = Path()
            viz_path.header.frame_id = self.trajectory.header.frame_id
            viz_path.header.stamp = rospy.Time.now()
            for i in range(len(self.trajectory.points)):
                pose = PoseStamped()
                pose.header.frame_id = self.trajectory.header.frame_id
                pose.header.stamp = viz_path.header.stamp
                pose.pose.position.x = self.trajectory.points[i].transforms[0].translation.x
                pose.pose.position.y = self.trajectory.points[i].transforms[0].translation.y
                pose.pose.position.z = self.trajectory.points[i].transforms[0].translation.z
                pose.pose.orientation.x = self.trajectory.points[i].transforms[0].rotation.x
                pose.pose.orientation.y = self.trajectory.points[i].transforms[0].rotation.y
                pose.pose.orientation.z = self.trajectory.points[i].transforms[0].rotation.z
                pose.pose.orientation.w = self.trajectory.points[i].transforms[0].rotation.w
                viz_path.poses.append(pose)
            publisher.publish(viz_path)

    def reset(self):
        self.restart()
        header = Header(stamp=rospy.Time.now(), frame_id = self.trajectory.header.frame_id)
        self.trajectory = MultiDOFJointTrajectory(header=header)
        self.start_time = rospy.Time.now()

    def restart(self):
        self.i = 0
        self.complete = False
        self.started = False

    def trajectoryPoint2Pose(self, point=MultiDOFJointTrajectoryPoint()):
        pose = PoseStamped()
        pose.pose.position = point.transforms[0].translation
        pose.pose.orientation = point.transforms[0].rotation
        return pose

    def do_transform_twist(self, twist, transform):
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
            twist_transformed = TwistStamped()
            twist_transformed.twist.linear = wrench_transformed.wrench.force
            twist_transformed.twist.angular = wrench_transformed.wrench.torque
            return twist_transformed
        else:
            rospy.logerr("Incorrect types used in do_transform_twist()")
            rospy.logerr("Must be Twist()/TwistStamped() and Transform()/TransformStamped()")

    def do_transform_transform(self, transform1, transform2):
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