import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion
from mavros_msgs.msg import State, ExtendedState
from vertical_aam.srv import *
from transitions import Machine
from trajectory_handler import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo


class printStateMachine(object):
    states = ['Ground', 'Takeoff', 'Scan', 'Print', 'Land', 'Manual']

    # transitions = [
    #     {'trigger': 'startTakeoff',     'source': ['Ground', 'Manual'],  'dest': 'Takeoff',  'after':   'on_startTakeoff'  },
    #     {'trigger': 'startPrint',       'source': 'Move',                       'dest': 'Print',    'before':  'on_startPrint'    },
    #     {'trigger': 'arriveAtHome',     'source': 'Move',                           'dest': 'Home',     'after':   'on_arriveAtHome'  },
    #     {'trigger': 'goToHome',         'source': ['Takeoff', 'Print', 'Manual'],   'dest': 'Move',     'before':  'on_goToHome'      },
    #     {'trigger': 'goToPrint',        'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPrint'     },
    #     {'trigger': 'goToPad',          'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPad'       },
    #     {'trigger': 'startLanding',     'source': 'Move',                           'dest': 'Landing',  'after':   'on_startLanding'  },
    #     {'trigger': 'manualTakeover',   'source': '*',                              'dest': 'Manual',   'before':  'on_manualTakeover'},
    #     {'trigger': 'switchToGround',   'source': ['Manual', 'Landing'],            'dest': 'Ground'                                  }     
    #     ]

    transitions = [
        {'trigger': 'startTakeoff',     'source': ['Ground', 'Manual'],             'dest': 'Takeoff'},
        {'trigger': 'startScan',        'source': ['Takeoff', 'Manual', 'Print'],   'dest': 'Scan'},
        {'trigger': 'startPrint',       'source': 'Scan',                           'dest': 'Print'},
        {'trigger': 'startLanding',     'source': 'Scan',                           'dest': 'Land'},
        {'trigger': 'finishLanding',    'source': ['Land', 'Manual'],               'dest': 'Ground'},
        {'trigger': 'manualTakeover',   'source': '*',                              'dest': 'Manual'},
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


        # initiate state machine model with states and transitions listed above
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial = 'Ground')

        # initiate trajectoryHandler object instance - used to store and generate new trajectories 
        self.tH_print = trajectoryHandler(
            frequency=self.rate, max_vel=self.max_vel_print, max_acc=self.max_acc_print, 
            max_yawrate=self.max_yawrate, max_yawrate_dot=self.max_yawrate_dot)

        self.tH_move = trajectoryHandler(
            frequency=self.rate, max_vel=self.max_vel_move, max_acc=self.max_acc_move, 
            max_yawrate=self.max_yawrate, max_yawrate_dot=self.max_yawrate_dot)

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

        # drone state subscriber
        state_sub = rospy.Subscriber(
            '/mavros/state', State, self._state_cb, queue_size=5, tcp_nodelay=True)
        ext_state_sub = rospy.Subscriber(
            '/mavros/extended_state', ExtendedState, self._ext_state_cb, queue_size=5, tcp_nodelay=True)
        local_position_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self._local_pos_cb, queue_size=1, tcp_nodelay=True)
        local_velocity_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_body', TwistStamped, self._local_vel_cb, queue_size=1, tcp_nodelay=True)

        # wait for drone to come online
        # rospy.wait_for_message('/mavros/state', State)
        # rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        # rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        # rospy.wait_for_service('fetch_poses')
        # rospy.wait_for_service('get_transformed_trajectory')
        # rospy.wait_for_service('get_TOPPRA_trajectory')
        # rospy.wait_for_service('get_drone_trajectory')
        
        # timer callback to send setpoints at a reasonable rate    
        sp_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self._timer_cb, reset=True)

        # initiate landing position at location where node is started
        self.pad_pose = PoseStamped()
        self.pad_pose = self.local_pose
        self.pad_pose.pose.position.z = self.takeoff_hgt
        rospy.loginfo("Landing site initiated at x=" + str(self.pad_pose.pose.position.x) +
            ", y=" + str(self.pad_pose.pose.position.y) + ".")

    #--------------------------------------------------------------------------------------------------------------
    #callbacks on state transitions

    def on_startTakeoff(self):
        rospy.loginfo("Takeoff initiated")
        self.pad_pose = PoseStamped()
        self.pad_pose = self.local_pose
        self.pad_pose.pose.position.z = self.takeoff_hgt
        rospy.loginfo("Landing site updated at x=" + str(self.pad_pose.pose.position.x) +
            ", y=" + str(self.pad_pose.pose.position.y) + ".")

    def on_arriveAtHome(self):
        print("HOME")
        print("pose-------------")
        print(self.pose)
        # generate trajectory
        if #layer generation succesful:
            rospy.loginfo("starting print layer")
            self.goToPrint()
        else:
            self.goToPad()

    def on_startScan(self):
        rospy.loginfo("Generating trajectory to scan position")
        ####
        self.scan_start = PoseStamped()
        self.scan_start.header.stamp = rospy.Time.now()
        self.scan_start.header.frame_id = 'map'
        self.scan_start.pose.position.x = 0
        self.scan_start.pose.position.y = 0
        self.scan_start.pose.position.z = 5
        self.scan_start.pose.orientation.w = 1
        ####
        trajectory_to_scan = generate_transition(self.pose, self.scan_start)
        scanning_trajectory = generate_pause(self.scan_start, 5)

        self.trajectory = trajectory()
        self.trajectory.append(trajectory_to_scan)
        self.trajectory.append(scanning_trajectory)


        # determine loiter point above print
        # generate transition trajectory
        # generate scanning trajectory
        # restart mapping

    def on_manualTakeover(self):
        rospy.loginfo("Manual takeover")

    #---------------------------------------------------------------------------------------------------------------
    # callbacks to occur on timer event - need to be defined for every state that is called

    def during_Scan(self):
        self.tooltip_state = "HOME"
        # follow scan trajectory
        # when scan quality threshold is met (or maybe just complete trajectory):
        # try:
        # generate trajectory to print
        # generate print trajectory
        complete, self.pose, self.velocity = self.trajectory.follow()
        if complete:
            self.startPrint()
        # else:
        # self.startLanding()

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
            if self.mavros_ext_state.landed_state == 2:
                self.startScan()

    def during_Print(self):
        self.tooltip_state = "STAB_6DOF"
        # follow print trajectory
        # self.pose, self.velocity = self.print_trajectory.follow()
        # self.pose, self.velocity, self.acceleration, self.tooltip_pose, self.tooltip_twist, self.tooltip_accel, complete = self.tH_print.follow_print_trajectory()
        # if complete:
        #     self.layer += 1
        #     self.goToHome()
        self.startLanding()

    def during_Landing(self):
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
            self.goToHome()
        
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
        # print(str(self.state))
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

    #callbacks save topics to current object for use later
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

    def _sp_raw_cb(self, sp_raw_msg):
        self.setpoint_raw = sp_raw_msg


class trajectory:
    def __init__(self, trajectory=MultiDOFJointTrajectory()):
        self.trajectory = trajectory
        self.start_time = rospy.Time.now()
        self.i = 0
    def follow(self):
        if self.i >= len(self.trajectory.points)-1:
            complete = True
        else:
            while (self.trajectory.points[self.i].time_from_start.to_sec() + self.start_time.to_sec()) < rospy.Time.now().to_sec():
                self.i += 1
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
        return complete, pose, twist
    def append(self, trajectory_app=MultiDOFJointTrajectory()):
        last_index = len(self.trajectory.points)-1
        for j in len(trajectory_app.points):
            trajectory_app.points[j].time_from_start += self.trajectory.points[last_index].time_from_start
            self.trajectory.points.append(trajectory_app.points)


def generate_transition(start_pose=PoseStamped(), end_pose=PoseStamped()):
    poses = PoseArray()
    poses.header.stamp = rospy.Time.now()
    poses.header.frame_id = start_pose.header.frame_id
    if start_pose.pose != end_pose.pose:
        if start_pose.header.frame_id != end_pose.header.frame_id:
            rospy.logerr("Cannot interpolate between poses in different reference frames.")
        else:
            poses.poses.append(start_pose.pose)
            poses.poses.append(end_pose.pose)        
            transition_trajectory = _TOPPRA_interpolation(poses)
    return transition_trajectory

def generate_pause(pose=PoseStamped(), time=0):
    trajectory = MultiDOFJointTrajectory()
    trajectory.header.frame_id = pose.header.frame_id
    trajectory.header.stamp = pose.header.stamp
    traj_point_init = MultiDOFJointTrajectoryPoint()
    traj_point_init.transforms[0].translation = pose.pose.position
    traj_point_init.transforms[0].rotation = pose.pose.orientation
    traj_point_init.time_from_start = rospy.Duration(0)
    traj_point_final = traj_point_init
    traj_point_final.time_from_start = rospy.Duration(time)
    trajectory.points.append(traj_point_init)
    trajectory.points.append(traj_point_final)
    return trajectory

def _TOPPRA_interpolation(poses, max_vel=0.15, max_acc=0.5, may_yawrate=1.0, may_yawrate_dot=10.0, frequency=30):
    # generates a smooth, interpolated, time-optimal trajectory from an array of poses using TOPPRA package

    num_poses = len(poses.poses)
    
    rospy.loginfo("Trajectory requested. Interpolating " + str(num_poses) + " poses at " + str(frequency) + "Hz.")

    way_pts = np.zeros((num_poses, 6))
    for i in range(num_poses):
        (roll, pitch, yaw) = euler_from_quaternion([poses[i].orientation.x,
                                                    poses[i].orientation.y,
                                                    poses[i].orientation.z,
                                                    poses[i].orientation.w])
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
    
    
if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_state_machine', anonymous=True)
    pSM = printStateMachine()
    rospy.spin()