#! /usr/bin/env python3
import rospy
import tf2_ros
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest, Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, TransformStamped, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Path
from mavros_msgs.msg import State, ExtendedState
from vertical_aam.srv import *
from transitions import Machine
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory import trajectory

#todo: fix velocity ffs

class printStateMachine(object):
    states = ['Ground', 'Takeoff', 'Scan', 'Print', 'Land', 'Manual', 'Loiter']

    transitions = [
        {'trigger': 'startTakeoff',     'source': ['Ground', 'Manual'],             'dest': 'Takeoff',  'before':   'on_startTakeoff'},
        {'trigger': 'startScan',        'source': ['Takeoff', 'Manual'],            'dest': 'Scan',     'before':   'on_startScan'},
        {'trigger': 'startPrint',       'source': 'Loiter',                         'dest': 'Print',    'before':   'on_startPrint'},
        {'trigger': 'endPrint',         'source': 'Print',                          'dest': 'Scan',     'before':    'on_endPrint'},
        {'trigger': 'startLoiter',      'source': ['Scan', 'Takeoff'],                           'dest': 'Loiter',   'before':   'on_startLoiter'},
        {'trigger': 'startLanding',     'source': '*',                              'dest': 'Land'},
        {'trigger': 'finishLanding',    'source': ['Land', 'Manual'],               'dest': 'Ground'},
        {'trigger': 'manualTakeover',   'source': '*',                              'dest': 'Manual',   'before':   'on_manualTakeover'},
        {'trigger': 'switchToGround',   'source': ['Manual', 'Landing'],            'dest': 'Ground'                                   },
        ]
    
    def __init__(self):
        # get config parameters from parameter server
        self.rate = rospy.get_param('/print_planner/setpoint_rate')
        self.tol_speed = rospy.get_param('/print_planner/tol_speed')
        self.takeoff_hgt = rospy.get_param('/print_planner/tol_height')    
        self.scan_hgt = rospy.get_param('/print_planner/scan_height')  
        self.pause_before_print = 1.0   
        self.scan_time = 5   

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
        self.tooltip_trajectory = trajectory()
        self.operator_confirmed = False

        self.traj_blind = trajectory()


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
            '/printer/drone_trajectory', Path, queue_size=1) 
        self.tip_traj_viz_pub = rospy.Publisher(
            '/printer/tooltip_trajectory', Path, queue_size=1)    

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

        self.scan_start = self.pad_pose
        self.scan_start.pose.position.z = self.scan_hgt
        
        rospy.loginfo("Landing site updated at x=" + str(self.pad_pose.pose.position.x) +
            ", y=" + str(self.pad_pose.pose.position.y) + ".")

    def on_startScan(self):
        # reset aft_pgo_map here
        # self.trajectory.reset()
        # self.trajectory.transition(self.pose, self.scan_start)
        # self.tooltip_trajectory.pause(self.scan_start, self.scan_time)
        # self.trajectory.publish_viz_trajectory(self.traj_viz_pub)
        # call_scan_reset_service()
        pass

    def on_startPrint(self):
        pause_time = self.pause_before_print

        #load tooltip trajectory
        self.tooltip_trajectory.reset()
        self.tooltip_trajectory.pause(self.tooltip_trajectory.trajectoryPoint2Pose(self.tooltip_layer.points[0]), pause_time)
        self.tooltip_trajectory.append_traj(self.tooltip_layer)
        self.tooltip_trajectory.publish_viz_trajectory(self.tip_traj_viz_pub)

        #load drone trajectory
        self.trajectory.reset()
        self.trajectory.pause(self.trajectory.trajectoryPoint2Pose(self.drone_layer.points[0]), pause_time)
        self.trajectory.append_traj(self.drone_layer)
        self.trajectory.publish_viz_trajectory(self.traj_viz_pub)

        #open nozzle
        # call_nozzle_open_service()

    def on_endPrint(self):
        #close nozzle
        # call_nozzle_close_service()
        self.on_startScan()
        
    def on_startLoiter(self):
        # layer = call_slicing_service()
        layer = self.generate_blind_trajectory()

        try:
            tf_map2print = self.tfBuffer.lookup_transform('map', 'print_origin', rospy.Time.now(), timeout=rospy.Duration(20))
            tf_tip2drone = self.tfBuffer.lookup_transform('tooltip_init_r', 'base_link', rospy.Time.now(), timeout=rospy.Duration(20))
            tf_tip2tip = self.tfBuffer.lookup_transform('tooltip_init_r', 'tooltip_init', rospy.Time.now(), timeout=rospy.Duration(20))
        except:
            rospy.logerr("Unable to fetch TFs!")

        layer = self.trajectory.transform_trajectory(layer, tf_map2print)

        self.drone_layer = self.trajectory.offset_trajectory(layer, tf_tip2drone)
        
        self.tooltip_layer = self.trajectory.rotate_trajectory(layer, tf_tip2tip)

        self.trajectory.reset()
        self.trajectory.transition(self.pose, self.trajectory.trajectoryPoint2Pose(self.drone_layer.points[0]))
        self.trajectory.publish_viz_trajectory(self.traj_viz_pub)

    def on_manualTakeover(self):
        rospy.loginfo("Manual takeover")
        call_nozzle_close_service()

    # def on_exception(self):
    #     rospy.logerr('state machine exception!')
    #     self.startLanding()
    #     call_nozzle_close_service()

    #---------------------------------------------------------------------------------------------------------------
    # callbacks to occur on timer event - need to be defined for every state that is called

    def during_Loiter(self):
        self.tooltip_state = "STAB_3DOF"
        if self.operator_confirmed:
            complete, pose, velocity = self.trajectory.follow()
            if not complete:
                self.pose = pose
                self.velocity = velocity
            else:
                self.operator_confirmed = False
                self.startPrint()

    def during_Scan(self):
        # self.tooltip_state = "STAB_3DOF"
        # scan_complete, pose, velocity = self.trajectory.follow()
        # if not scan_complete:
        #     self.pose = pose
        #     self.velocity = velocity
        # else:
        self.startLoiter()
    
    def during_Print(self):
        self.tooltip_state = "STAB_6DOF"
        print_complete, pose, velocity = self.trajectory.follow()
        tooltip_print_complete, tip_pose, tip_velocity = self.tooltip_trajectory.follow()
        if not print_complete:
            self.pose = pose
            self.velocity = velocity
            self.tooltip_pose = tip_pose
            self.tooltip_twist = tip_velocity
        else:
            self.endPrint()
    
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
        
        self.pose.header.stamp = rospy.Time.now()
        self.velocity.header.stamp = rospy.Time.now()

        self.tooltip_pose.header.stamp = rospy.Time.now()
        self.tooltip_twist.header.stamp = rospy.Time.now()
        # self.tooltip_twist =
        # self.tooltip_pose =

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

    def generate_blind_trajectory(self):
        self.traj_blind.reset()

        def generate_pose(x,y,z):
            pose1 = PoseStamped()
            pose1.header.stamp = rospy.Time.now()
            pose1.header.frame_id = "print_origin"
            pose1.pose.position.x = x
            pose1.pose.position.y = y
            pose1.pose.position.z = z
            pose1.pose.orientation.x = 0
            pose1.pose.orientation.y = 0
            pose1.pose.orientation.z = 0
            pose1.pose.orientation.w = 1
            return pose1
        
        pause_time = 30
        
        # pose = generate_pose(0, 0, 0.1)
        # pose = generate_pose(-0.2, 0, 0.1)
        pose = generate_pose(1.5, 0, 0.1)
        # pose = generate_pose(1.5, 0, 0.48)
        # pose = generate_pose(1.3, 0, 0.48)
        # pose = generate_pose(1.1, 0, 0.48)
        # pose = generate_pose(0.9, 0, 0.48)
        # pose = generate_pose(0.7, 0, 0.48)
        # pose = generate_pose(0.5, 0, 0.48)
        # pose = generate_pose(0.3, 0, 0.48)
        # pose = generate_pose(0.1, 0, 0.48)
        # pose = generate_pose(-0.1, 0, 0.48)
        # pose = generate_pose(-0.3, 0, 0.48)

        self.traj_blind.pause(pose, pause_time)
        # self.traj_blind.transition(pose1, pose2)
        # self.traj_blind.pause(pose2, pause_time)
        # self.traj_blind.transition(pose2, pose3)
        # self.traj_blind.pause(pose3, pause_time)
        # self.traj_blind.transition(pose3, pose4)
        # self.traj_blind.pause(pose4, pause_time)
        # self.traj_blind.transition(pose4, pose5)
        # self.traj_blind.pause(pose5, pause_time)
        # self.traj_blind.transition(pose5, pose6)
        # self.traj_blind.pause(pose6, pause_time)
        # self.traj_blind.transition(pose6, pose7)
        # self.traj_blind.pause(pose7, pause_time)
        # self.traj_blind.transition(pose7, pose8)
        # self.traj_blind.pause(pose8, pause_time)
        # self.traj_blind.transition(pose8, pose9)
        # self.traj_blind.pause(pose9, pause_time)
        # self.traj_blind.transition(pose9, pose10)
        # self.traj_blind.pause(pose10, pause_time)
        # self.traj_blind.transition(pose10, pose11)
        # self.traj_blind.pause(pose11, pause_time)
        # self.traj_blind.transition(pose11, pose12)
        # self.traj_blind.pause(pose12, pause_time)
        # self.traj_blind.transition(pose12, pose13)
        # self.traj_blind.pause(pose13, pause_time)

        return self.traj_blind.trajectory


# def call_slicing_service():
#     slice_print = rospy.ServiceProxy('generate_layer', generateLayer)
#     req = generateLayerRequest()
#     resp = slice_print(req)
#     return resp.trajectory

def call_nozzle_open_service():
    try:
        open_nozzle = rospy.ServiceProxy('open_nozzle', Trigger)
        req = TriggerRequest()
        resp = open_nozzle(req)
    except:
        rospy.logwarn("printing hardware not connected")

def call_nozzle_close_service():
    try:
        close_nozzle = rospy.ServiceProxy('close_nozzle', Trigger)
        req = TriggerRequest()
        resp = close_nozzle(req)
    except:
        rospy.logwarn("printing hardware not connected")

def call_scan_reset_service():
    try:
        restart_mapping = rospy.ServiceProxy('restart_mapping', Empty)
        req = EmptyRequest()
        resp = restart_mapping(req)
    except:
        rospy.logwarn("mapping restart unavailable")

if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_state_machine', anonymous=True)
    pSM = printStateMachine()
    rospy.spin()