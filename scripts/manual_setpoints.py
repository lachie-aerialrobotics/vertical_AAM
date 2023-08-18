#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float32, String
from controller_msgs.msg import FlatTarget
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, TransformStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from sensor_msgs.msg import Joy
from tricopter.srv import *
from transitions import Machine
from trajectory_handler import *
from misc_functions import *
import tf2_ros
from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R

class joystick:
    def __init__(self):    
        self.yaw_axis = 0.0
        self.throt_axis = 0.0
        self.roll_axis = 0.0
        self.pitch_axis = 0.0
        self.yaw = 0.0

        self.deadzone = 0.05
        
        joy_sub = rospy.Subscriber(
            '/joy', Joy, self._joy_cb, queue_size=5, tcp_nodelay=True)

    def _joy_cb(self, joy_msg):
        if np.abs(joy_msg.axes[0]) > self.deadzone:
            self.yaw_axis = joy_msg.axes[0]
        else: self.yaw_axis = 0.0
        if np.abs(joy_msg.axes[1]) > self.deadzone:
            self.throt_axis = joy_msg.axes[1]
        else: self.throt_axis = 0.0
        if np.abs(joy_msg.axes[3]) > self.deadzone:
            self.roll_axis = joy_msg.axes[3]
        else: self.roll_axis = 0.0
        if np.abs(joy_msg.axes[4]) > self.deadzone:
            self.pitch_axis = joy_msg.axes[4]
        else: self.pitch_axis = 0.0

        
class printStateMachine():
    def __init__(self):
        #init tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.DT_d, self.d_R_t = self._lookup_vec_and_rot('body', 'tooltip_init', Timeout=10)

        self.pose_sp = PoseStamped()
        self.pose_sp.header.frame_id = "map"

        self.tip_pose_sp = PoseStamped()
        self.tip_pose_sp.header.frame_id = "map"

        self.WD_w = np.asarray([0.0, 0.0, 0.0])

        self.yaw = 0.0

        self.max_vel = 0.01
        self.max_yaw_rate = 0.01

        self.rate = rospy.get_param('/print_planner/setpoint_rate')

        self.mavros_state = State()
        self.mavros_state.mode = "MANUAL"
        
        state_sub = rospy.Subscriber(
            '/mavros/state', State, self._state_cb, queue_size=5, tcp_nodelay=True)

        # publishers to geometric controller
        self.geo_pose_pub = rospy.Publisher(
            'reference/flatsetpoint', FlatTarget, queue_size=1, tcp_nodelay=True)
        self.geo_yaw_pub = rospy.Publisher(
            'reference/yaw', Float32, queue_size=1, tcp_nodelay=True)

        # publisher for on-board position controller
        self.sp_position_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=1, tcp_nodelay=True)

        # publishers to manipulator
        self.pub_tooltip_state = rospy.Publisher(
            '/manipulator/state',  String, queue_size=1, tcp_nodelay=True)
        self.pub_tooltip_pose = rospy.Publisher(
            '/tooltip_setpoint/pose', PoseStamped, queue_size=1, tcp_nodelay=True)     
        
        # timer callback to send setpoints at a reasonable rate    
        sp_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self._timer_cb, reset=True)

    #----------------------------------------------------------------------------------------------
    #ros callbacks
    def _timer_cb(self, event): #timer callback runs at specified rate to output setpoints
        
        self.yaw += jS.yaw_axis * self.max_yaw_rate
        yaw_quat = quaternion_from_euler(0, 0, self.yaw)
        yaw_R = R.from_quat(yaw_quat)
        DT_w = yaw_R.apply(self.DT_d)
        inc = yaw_R.apply(np.asarray([jS.pitch_axis * self.max_vel, jS.roll_axis * self.max_vel, jS.throt_axis * self.max_vel]))
        self.WD_w += inc
        WT_w = self.WD_w + DT_w
        w_R_t = yaw_R * self.d_R_t
        w_q_t = R.as_quat(w_R_t)

        # self.WD_w = yaw_R.apply(self.WD_w)
        
        self.pose_sp.header.stamp = rospy.Time.now()
        self.pose_sp.pose.position.x = self.WD_w[0]
        self.pose_sp.pose.position.y = self.WD_w[1]
        self.pose_sp.pose.position.z = self.WD_w[2]
        self.pose_sp.pose.orientation.x = yaw_quat[0]
        self.pose_sp.pose.orientation.y = yaw_quat[1]
        self.pose_sp.pose.orientation.z = yaw_quat[2]
        self.pose_sp.pose.orientation.w = yaw_quat[3]

        self.tip_pose_sp.header.stamp = rospy.Time.now()
        self.tip_pose_sp.pose.position.x = WT_w[0]
        self.tip_pose_sp.pose.position.y = WT_w[1]
        self.tip_pose_sp.pose.position.z = WT_w[2]
        self.tip_pose_sp.pose.orientation.x = w_q_t[0]
        self.tip_pose_sp.pose.orientation.y = w_q_t[1]
        self.tip_pose_sp.pose.orientation.z = w_q_t[2]
        self.tip_pose_sp.pose.orientation.w = w_q_t[3]

        self.sp_position_pub.publish(self.pose_sp)
        self.pub_tooltip_pose.publish(self.tip_pose_sp)
        target, yaw = flat_target_msg_conversion(4, self.pose_sp)
        self.geo_pose_pub.publish(target)
        self.geo_yaw_pub.publish(yaw)
        if self.mavros_state.mode == "OFFBOARD":
            self.pub_tooltip_state.publish(String("STAB_6DOF"))
        else:
            self.pub_tooltip_state.publish(String("STAB_3DOF"))

    def _state_cb(self, state_msg):
        self.mavros_state = state_msg

    def _lookup_vec_and_rot(self, frame_id, child_frame_id, Timeout=0.1):
        tf = self.tfBuffer.lookup_transform(frame_id, child_frame_id, time=rospy.Time.now(), timeout=rospy.Duration(Timeout))       
        translation, rotation = self._msg_to_vec_and_rot(tf)
        return translation, rotation

    def _msg_to_vec_and_rot(self, msg):
        if isinstance(msg, TransformStamped):
            translation = np.asarray([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            quat = np.asarray([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
            rotation = R.from_quat(quat)
        elif isinstance(msg, PoseStamped):
            translation = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            quat = np.asarray([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            rotation = R.from_quat(quat)
        return translation, rotation

if __name__ == '__main__':
    # initialize node
    
    rospy.init_node('print_state_machine', anonymous=True)
    jS = joystick()
    pSM = printStateMachine()
    
    rospy.spin()