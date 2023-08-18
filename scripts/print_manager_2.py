import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion
from mavros_msgs.msg import State, ExtendedState
from tricopter.srv import *
from transitions import Machine
from trajectory_handler import *
from misc_functions import *


class printStateMachine(object):
    states = ['Takeoff', 'Landing', 'Home', 'Move', 'Print', 'Ground', 'Manual']

    transitions = [
        {'trigger': 'startTakeoff',     'source': ['Ground', 'Manual'],                         'dest': 'Takeoff',  'after':   'on_startTakeoff'  },
        {'trigger': 'startPrint',       'source': 'Move',                       'dest': 'Print',    'before':  'on_startPrint'    },
        {'trigger': 'arriveAtHome',     'source': 'Move',                           'dest': 'Home',     'after':   'on_arriveAtHome'  },
        {'trigger': 'goToHome',         'source': ['Takeoff', 'Print', 'Manual'],   'dest': 'Move',     'before':  'on_goToHome'      },
        {'trigger': 'goToPrint',        'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPrint'     },
        {'trigger': 'goToPad',          'source': 'Home',                           'dest': 'Move',     'before':  'on_goToPad'       },
        {'trigger': 'startLanding',     'source': 'Move',                           'dest': 'Landing',  'after':   'on_startLanding'  },
        {'trigger': 'manualTakeover',   'source': '*',                              'dest': 'Manual',   'before':  'on_manualTakeover'},
        {'trigger': 'switchToGround',   'source': ['Manual', 'Landing'],            'dest': 'Ground'                                  }     
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
        self.offset = rospy.get_param('/print_planner/offset')

        # initial values
        # self.target = FlatTarget()
        # self.target.header.frame_id = "map"
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
        rospy.wait_for_message('/mavros/state', State)
        rospy.wait_for_message('/mavros/extended_state', ExtendedState)
        rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        rospy.wait_for_service('fetch_poses')
        rospy.wait_for_service('get_transformed_trajectory')
        rospy.wait_for_service('get_TOPPRA_trajectory')
        rospy.wait_for_service('get_drone_trajectory')
        
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
        if self.layer < rospy.get_param(str(self.tH_print.waypoint_prefix) + '/n_layers'):
            rospy.loginfo("Generating trajectory for next layer")
            self.tH_print.generate_print_layer(self.layer)
            rospy.loginfo("starting print")
            self.goToPrint()
        else:
            self.goToPad()
    
    def on_startLanding(self):
        rospy.loginfo("Landing initiated")
   
    def on_goToPad(self):
        print("pose-------------")
        print(self.pose)
        print("pad--------------")
        print(self.pad_pose)
        rospy.loginfo("Generating trajectory to pad")
        self.tH_move.generate_transition(self.pose, self.pad_pose)  
        self.moveCompletionTransition = self.startLanding

    def on_goToPrint(self): 
        rospy.loginfo("Generating trajectory to beginning of print")  
        print("pose-------------")
        print(self.pose)
        self.print_start_pose = self.tH_print.get_print_start_pose()
        self.tH_move.generate_transition(self.pose, self.print_start_pose)
        self.moveCompletionTransition = self.arriveAtPrint

    def on_startPrint(self):
        rospy.loginfo("Printing...")

    def on_goToHome(self):
        rospy.loginfo("Generating trajectory to loiter position")
        print("pose-------------")
        print(self.pose)
        # determine loiter point above print
        if self.layer < rospy.get_param(str(self.tH_print.waypoint_prefix) + '/n_layers'):
            self.home_pose = self.tH_print.get_loiter_point(self.layer, self.offset)
        self.tH_move.generate_transition(self.pose, self.home_pose)
        self.moveCompletionTransition = self.arriveAtHome

    def on_arriveAtPrint(self):
        rospy.loginfo("Arrived at print start - waiting for velocity/position tolerances")

    def on_manualTakeover(self):
        rospy.loginfo("Manual takeover")

    #---------------------------------------------------------------------------------------------------------------
    # callbacks to occur on timer event - need to be defined for every state that is called

    def during_Home(self):
        self.tooltip_state = "HOME"
        self.pose = self.home_pose
        self.velocity.twist.linear = Vector3(0,0,0)
        self.velocity.twist.angular = Vector3(0,0,0)

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
                self.goToHome()

    def during_Move(self):
        self.tooltip_state = "HOME"
        self.pose, self.velocity, self.acceleration, complete = self.tH_move.follow_transition_trajectory()
        if complete:
            self.moveCompletionTransition()

    def during_Print(self):
        self.tooltip_state = "STAB_6DOF"
        self.pose, self.velocity, self.acceleration, self.tooltip_pose, self.tooltip_twist, self.tooltip_accel, complete = self.tH_print.follow_print_trajectory()
        if complete:
            self.layer += 1
            self.goToHome()

    def during_Landing(self):
        self.tooltip_state = "HOME"
        self.velocity.twist.angular = Vector3(0,0,0)
        #reduce height of z setpoint until altitude is zero
        if self.pose.pose.position.z > 0 and not (self.mavros_ext_state.landed_state == 1):
            self.pose.pose.position.z += -self.tol_speed / self.rate
            self.velocity.twist.linear = Vector3(0,0,-self.tol_speed)
        else:
        #make double sure the drone is on the ground by calling the mavros landing service
            call_landing_service()
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
    
    
if __name__ == '__main__':
    # initialize node
    rospy.init_node('print_state_machine', anonymous=True)
    pSM = printStateMachine()
    rospy.spin()