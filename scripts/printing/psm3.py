#! /usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import tf2_geometry_msgs
from std_msgs.msg import String, Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Transform, TransformStamped, WrenchStamped, Twist, PoseArray, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Path
from mavros_msgs.msg import State, ExtendedState
from vertical_aam.srv import *
from transitions import Machine
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class printStateMachine(object):
    states = ['Ground', 'Takeoff', 'Scan', 'Print', 'Land', 'Manual', 'Loiter']

    transitions = [
        {'trigger': 'startTakeoff',     'source': ['Ground', 'Manual'],             'dest': 'Takeoff',  'before':   'on_startTakeoff'},
        {'trigger': 'startScan',        'source': ['Takeoff', 'Manual', 'Print'],   'dest': 'Scan',     'before':   'on_startScan'},
        {'trigger': 'startPrint',       'source': 'Loiter',                         'dest': 'Print',    'before':   'on_startPrint'},
        {'trigger': 'startLoiter',      'source': 'Scan',                           'dest': 'Loiter',   'before':   'on_startLoiter'},
        {'trigger': 'startLanding',     'source': '*',                              'dest': 'Land'},
        {'trigger': 'finishLanding',    'source': ['Land', 'Manual'],               'dest': 'Ground'},
        {'trigger': 'manualTakeover',   'source': '*',                              'dest': 'Manual',   'before':   'on_manualTakeover'},
        {'trigger': 'switchToGround',   'source': ['Manual', 'Landing'],            'dest': 'Ground'                                   }
        ]
    
    def __init__(self):
        # get config parameters from parameter server
        self.rate = rospy.get_param('/print_planner/setpoint_rate')
        self.tol_speed = rospy.get_param('/print_planner/tol_speed')
        self.takeoff_hgt = rospy.get_param('/print_planner/tol_height')   
        self.max_vel_move = rospy.get_param('/print_planner/transition_vel')
        self.max_acc_move = rospy.get_param('/print_planner/transition_max_accel')
        self.max_yawrate = rospy.get_param('/print_planner/max_yawrate')
        self.max_yawrate_dot = rospy.get_param('/print_planner/max_yawrate_dot')  
        self.scan_hgt = 4.0           

        self.yaw = 0.0
        self.tooltip_state = "RETRACTED"
        self.tooltip_pose = PoseStamped()
        self.tooltip_pose.header.frame_id = "map"
        self.tooltip_twist = TwistStamped()
        self.tooltip_twist.header.frame_id = "map"
        
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.velocity = TwistStamped()
        self.velocity.header.frame_id = "map"
        self.acceleration = TwistStamped()
        self.acceleration.header.frame_id = "map"

        self.trajectory = trajectory()
        self.operator_confirmed = False


        # initiate state machine model with states and transitions listed above
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial = 'Ground')#, on_exception='on_exception')

        # publisher for on-board position controller
        self.sp_position_pub = rospy.Publisher(
            '/setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.sp_vel_pub = rospy.Publisher(
            '/setpoint/vel', TwistStamped, queue_size=1, tcp_nodelay=True)

        # publish current state for debugging
        self.pub_drone_state = rospy.Publisher(
            '/printer/state',  String, queue_size=1, tcp_nodelay=True)

        # publishers to manipulator
        self.pub_tooltip_state = rospy.Publisher(
            '/manipulator/state',  String, queue_size=1, tcp_nodelay=True)
        self.pub_tooltip_pose = rospy.Publisher(
            '/tooltip_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_tooltip_twist = rospy.Publisher(
            '/tooltip_setpoint/velocity', TwistStamped, queue_size=1, tcp_nodelay=True) 

        # vizualisation publishers
        self.traj_viz_pub = rospy.Publisher(
            '/printer/trajectory', Path, queue_size=1)    

        # drone state subscriber
        state_sub = rospy.Subscriber(
            '/mavros/state', State, self._state_cb, queue_size=5, tcp_nodelay=True)
        ext_state_sub = rospy.Subscriber(
            '/mavros/extended_state', ExtendedState, self._ext_state_cb, queue_size=5, tcp_nodelay=True)
        local_position_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self._local_pos_cb, queue_size=1, tcp_nodelay=True)
        local_velocity_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_body', TwistStamped, self._local_vel_cb, queue_size=1, tcp_nodelay=True)
        
        authorisation_service = rospy.Service('start_layer', Empty, self.authorisation_srv)

        # wait for drone to come online
        rospy.wait_for_message('/mavros/state', State)
        rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        rospy.wait_for_service('generate_layer')
        
        # timer callback to send setpoints at a reasonable rate    
        sp_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self._timer_cb, reset=True)

        # initiate landing position at location where node is started
        self.pad_pose = PoseStamped()
        self.pad_pose = self.local_pose
        self.pad_pose.pose.position.z = self.takeoff_hgt
        rospy.loginfo("Landing site initiated at x=" + str(self.pad_pose.pose.position.x) +
            ", y=" + str(self.pad_pose.pose.position.y) + ".")
        
        self.scan_start = self.pad_pose
        self.scan_start.pose.position.z = self.scan_hgt
        
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(20.0))
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def authorisation_srv(self, req):
        self.operator_confirmed = True
        resp = EmptyResponse()
        return resp

    #--------------------------------------------------------------------------------------------------------------
    #callbacks on state transitions

    def on_startTakeoff(self):
        rospy.loginfo("Takeoff initiated")
        self.pad_pose = PoseStamped()
        self.pad_pose = self.local_pose
        self.pad_pose.pose.position.z = self.takeoff_hgt
        rospy.loginfo("Landing site updated at x=" + str(self.pad_pose.pose.position.x) +
            ", y=" + str(self.pad_pose.pose.position.y) + ".")

    def on_startScan(self):
        # reset aft_pgo_map here
        self.trajectory.reset()
        self.trajectory.transition(self.pose, self.scan_start)
        self.trajectory.pause(self.scan_start, 10)
        self.trajectory.publish_viz_trajectory(self.traj_viz_pub)

    def on_startPrint(self):
        self.trajectory.reset()
        self.trajectory.pause(trajectoryPoint2Pose(self.layer.points[0]), 2)
        self.trajectory.append_traj(self.layer)
        self.trajectory.publish_viz_trajectory(self.traj_viz_pub)

    def on_startLoiter(self):
        def call_slicing_service():
            slice_print = rospy.ServiceProxy('generate_layer', generateLayer)
            req = generateLayerRequest()
            resp = slice_print(req)
            return resp.trajectory
        self.trajectory.reset()
        self.layer = call_slicing_service()
        try:
            tf_map2print = self.tfBuffer.lookup_transform('map', 'print_origin', rospy.Time.now(), timeout=rospy.Duration(10))
            tf_tip2drone = self.tfBuffer.lookup_transform('tooltip_r', 'base_link', rospy.Time.now(), timeout=rospy.Duration(10))
            tf_drone2tip = self.tfBuffer.lookup_transform('base_link', 'tooltip_r', rospy.Time.now(), timeout=rospy.Duration(10))
        except:
            rospy.logerr("Printing plane not initialised!")
        self.layer = self.trajectory.transform_trajectory(self.layer, tf_map2print)
        self.layer = handle_drone_trajectory(self.layer, tf_drone2tip, tf_tip2drone)
        self.trajectory.transition(self.pose, trajectoryPoint2Pose(self.layer.points[0]))
        self.trajectory.publish_viz_trajectory(self.traj_viz_pub)

    def on_manualTakeover(self):
        rospy.loginfo("Manual takeover")

    def on_exception(self):
        self.startLanding()

    #---------------------------------------------------------------------------------------------------------------
    # callbacks to occur on timer event - need to be defined for every state that is called

    def during_Loiter(self):
        if self.operator_confirmed:
            complete, pose, velocity = self.trajectory.follow()
            if not complete:
                self.pose = pose
                self.velocity = velocity
            else:
                self.operator_confirmed = False
                self.startPrint()

    def during_Scan(self):
        self.tooltip_state = "HOME"
        scan_complete, pose, velocity = self.trajectory.follow()
        if not scan_complete:
            self.pose = pose
            self.velocity = velocity
        else:
            self.startLoiter()
    
    def during_Print(self):
        self.tooltip_state = "STAB_6DOF"
        print_complete, pose, velocity = self.trajectory.follow()
        if not print_complete:
            self.pose = pose
            self.velocity = velocity
        else:
            self.startScan()
    
    def during_Takeoff(self):
        self.tooltip_state = "HOME"
        self.velocity.twist.angular = Vector3(0,0,0)
        #increase target z to deined loiter height
        if self.pose.pose.position.z < self.takeoff_hgt:
            self.pose.pose.position.z += self.tol_speed / self.rate
            self.velocity.twist.linear = Vector3(0,0,self.tol_speed)
        else: #when target has reached loiter height and drone knows its flying, move to next state 
            self.pose.pose.position.z = self.takeoff_hgt
            self.velocity.twist.linear = Vector3(0,0,0)
            self.startScan()

    def during_Land(self):
        self.tooltip_state = "HOME"
        self.velocity.twist.angular = Vector3(0,0,0)
        #reduce height of z setpoint until altitude is zero
        if self.pose.pose.position.z > 0 and not (self.mavros_ext_state.landed_state == 1):
            self.pose.pose.position.z += -self.tol_speed / self.rate
            self.velocity.twist.linear = Vector3(0,0,-self.tol_speed)
        else:
            self.switchToGround()

    def during_Manual(self):
        # If flying -> goto home position
        self.pose = self.local_pose
        self.velocity = self.local_velocity
        self.tooltip_state = "STAB_3DOF"
        if self.mavros_ext_state.landed_state == 1:
            self.switchToGround()
        if self.mavros_state.mode == "OFFBOARD":
            self.startScan()
        
    def during_Ground(self):
        # if landed -> takeoff. 
        self.pose = self.local_pose
        self.velocity = self.local_velocity
        self.tooltip_state = "HOME"
        if self.mavros_state.armed:
            self.tooltip_state = "HOME"
            if self.mavros_ext_state.landed_state == 2:
                self.manualTakeover()
            if self.mavros_state.mode == "OFFBOARD":
                self.tooltip_state = "HOME"
                self.startTakeoff()
               
    def during_always(self): #this callback always runs to check if not in offboard mode
        if self.mavros_state.mode != "OFFBOARD" and not (self.state == 'Manual' or self.state == 'Ground'):
                self.manualTakeover()

    #----------------------------------------------------------------------------------------------
    #ros callbacks

    def _timer_cb(self, event): #timer callback runs at specified rate to output setpoints
        self.during_always()
        exec("self.during_" + str(self.state) + "()") #execute the function name corresponding to the current state
        # update time stamps and publish current values of drone and manipulator commands
        self.tooltip_pose.header.stamp = rospy.Time.now()
        self.tooltip_twist.header.stamp = rospy.Time.now()
        self.pose.header.stamp = rospy.Time.now()
        self.velocity.header.stamp = rospy.Time.now()
        self.sp_position_pub.publish(self.pose)
        self.sp_vel_pub.publish(self.velocity)
        self.pub_drone_state.publish(String(str(self.state)))
        self.pub_tooltip_state.publish(String(self.tooltip_state))
        self.pub_tooltip_pose.publish(self.tooltip_pose)
        self.pub_tooltip_twist.publish(self.tooltip_twist)

    def _state_cb(self, state_msg):
        self.mavros_state = state_msg

    def _ext_state_cb(self, ext_state_msg):
        #reference for landed_state:
        # uint8 LANDED_STATE_UNDEFINED = 0
        # uint8 LANDED_STATE_ON_GROUND = 1
        # uint8 LANDED_STATE_IN_AIR = 2
        # uint8 LANDED_STATE_TAKEOFF = 3
        # uint8 LANDED_STATE_LANDING = 4
        self.mavros_ext_state = ext_state_msg

    def _local_pos_cb(self, local_pose_msg):
        self.local_pose = local_pose_msg

    def _local_vel_cb(self, local_vel_msg):
        self.local_velocity = local_vel_msg

    #---------------------



class trajectory:
    def __init__(self, traj_init=MultiDOFJointTrajectory(), frame_id='map'):
        self.trajectory = traj_init
        self.trajectory.header.stamp = rospy.Time.now()
        self.trajectory.header.frame_id = frame_id
        self.start_time = rospy.Time.now()
        self.i = 0

    def follow(self):
        complete = False 
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
            complete = True
            self.i = 0
        return complete, pose, twist
    
    def append_traj(self, trajectory_app=MultiDOFJointTrajectory()):
        if len(self.trajectory.points) != 0:
            last_start_time = self.trajectory.points[-1].time_from_start
            for i in range(len(trajectory_app.points)-1):
                trajectory_app.points[i].time_from_start += last_start_time
                self.trajectory.points.append(trajectory_app.points[i])
        else:
            self.trajectory.points = trajectory_app.points

    def transition(self, start_pose=PoseStamped(), end_pose=PoseStamped(), vel=0.5, acc=10.0, yawrate=100.0):
        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = start_pose.header.frame_id
        poses.poses.append(start_pose.pose)
        poses.poses.append(end_pose.pose)        
        transition_trajectory = self._TOPPRA_interpolation(poses, vel, acc, yawrate, 1000, 30)
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
            traj.points[i].transforms[0] = do_transform_transform(traj.points[i].transforms[0], tf2_transform).transform
            traj.points[i].velocities[0] = do_transform_twist(traj.points[i].velocities[0], tf2_transform).twist
        traj.header.frame_id = tf2_transform.header.frame_id
        return traj

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
        self.i = 0
        header = Header(stamp=rospy.Time.now(), frame_id = self.trajectory.header.frame_id)
        self.trajectory = MultiDOFJointTrajectory(header=header)
        self.start_time = rospy.Time.now()


def trajectoryPoint2Pose(point=MultiDOFJointTrajectoryPoint()):
    pose = PoseStamped()
    pose.pose.position = point.transforms[0].translation
    pose.pose.orientation = point.transforms[0].rotation
    return pose

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
        twist_transformed = TwistStamped()
        twist_transformed.twist.linear = wrench_transformed.wrench.force
        twist_transformed.twist.angular = wrench_transformed.wrench.torque
        return twist_transformed
    else:
        rospy.logerr("Incorrect types used in do_transform_twist()")
        rospy.logerr("Must be Twist()/TwistStamped() and Transform()/TransformStamped()")

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
    
def handle_drone_trajectory(toolpath_trajectory, tf_drone2tip, tf_tip2drone):
    # service generates a drone trajectory offset from a supplied tooltip trajectory. x/y/z and yaw commands
    # are preserved and pitch/roll angles are ignored
    rospy.loginfo("Offset drone trajectory requested")

    tf = tf_tip2drone
    tf_rotation = TransformStamped()
    tf_rotation.header = tf.header
    tf_rotation.child_frame_id = tf.child_frame_id
    tf_rotation.transform.rotation = tf.transform.rotation
    tf_rotation.transform.translation = Vector3(0,0,0)

    tf_inv = tf_drone2tip
    tf_inv_rotation = TransformStamped()
    tf_inv_rotation.header = tf_inv.header
    tf_inv_rotation.child_frame_id = tf_inv.child_frame_id
    tf_inv_rotation.transform.rotation = tf_inv.transform.rotation
    tf_inv_rotation.transform.translation = Vector3(0,0,0)

    drone_trajectory = MultiDOFJointTrajectory()
    drone_trajectory.header.frame_id = toolpath_trajectory.header.frame_id
    drone_trajectory.header.stamp = rospy.Time.now()

    for i in range(len(toolpath_trajectory.points)): 
        traj = do_transform_transform(tf_rotation, toolpath_trajectory.points[i].transforms[0])
        (roll, pitch, yaw) = euler_from_quaternion([traj.transform.rotation.x,
                                                    traj.transform.rotation.y,
                                                    traj.transform.rotation.z,
                                                    traj.transform.rotation.w])
        q = quaternion_from_euler(yaw,0,0,'szyx')
        traj.transform.rotation = Quaternion(q[0],q[1],q[2],q[3])
        traj = do_transform_transform(tf_inv_rotation, traj)
        drone_transform = do_transform_transform(tf, traj)
        
        drone_trajectory_point = MultiDOFJointTrajectoryPoint()
        drone_trajectory_point.time_from_start = toolpath_trajectory.points[i].time_from_start
        drone_trajectory_point.transforms.append(drone_transform.transform)
        drone_trajectory_point.velocities.append(do_transform_twist(toolpath_trajectory.points[i].velocities[0], tf_inv).twist)
        drone_trajectory.points.append(drone_trajectory_point)
    rospy.loginfo("Trajectory ready")
    return drone_trajectory

if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_state_machine', anonymous=True)
    pSM = printStateMachine()
    rospy.spin()