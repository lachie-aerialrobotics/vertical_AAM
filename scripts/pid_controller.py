#! /usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from vertical_aam.cfg import PIDConfig

class PIDController:
    def __init__(self):
        # init config server
        srv = Server(PIDConfig, self.config_cb)

        # init stored variables
        self.sp_pose = np.zeros(4)
        self.sp_vel = np.zeros(4)
        self.err = 0.0
        self.i_err = 0.0
        self.t = rospy.Time.now()

        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1, tcp_nodelay=True)
        sub_mav_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.controller_cb, queue_size=1, tcp_nodelay=True)
        sub_sp_pose = rospy.Subscriber('/setpoint/pose', PoseStamped, self.pose_sp_cb, queue_size=1, tcp_nodelay=True)
        sub_sp_vel = rospy.Subscriber('/setpoint/vel', TwistStamped, self.vel_sp_cb, queue_size=1, tcp_nodelay=True)

        rospy.spin()

    def controller_cb(self, mav_pose_msg):
        sp_vel = self.sp_vel
        sp_pose = self.sp_pose
        mav_pose = pose2np(mav_pose_msg)
        vel_cmd = sp_vel + self.pos_PID(mav_pose, sp_pose)
        self.pub_vel.publish(np2twist(vel_cmd))

    def pos_PID(self, mav_pose, sp_pose):
        err = sp_pose - mav_pose
        dt = (rospy.Time.now() - self.t).to_sec()
        self.t = rospy.Time.now()
        if dt == 0:
            d_err = 0
            self.i_err += 0
        else:
            d_err = (err - self.err) / dt
            self.i_err += err * dt
        self.err = err
        vel_cmd = self.P * err + self.I * self.i_err + self.D * d_err
        for i in range(4):
            if vel_cmd[i] > self.v_max[i]:
                vel_cmd[i] = self.v_max[i]
            elif vel_cmd[i] < -self.v_max[i]:
                vel_cmd[i] = -self.v_max[i]
        return vel_cmd
    
    def pose_sp_cb(self, pose_msg):
        self.sp_pose = pose2np(pose_msg)

    def vel_sp_cb(self, vel_msg):
        self.sp_vel = twist2np(vel_msg)
    
    def config_cb(self, config, level):
        self.v_max = np.asarray([config.vmax_xyz, config.vmax_xyz, config.vmax_xyz, config.vmax_yaw])
        self.P = np.asarray([config.P_xy, config.P_xy, config.P_z, config.P_yaw])
        #todo: re-enable and fix I term
        # self.I = np.asarray([config.I_xy, 
        #                      config.I_xy, 
        #                      config.I_z, 
        #                      0])
        self.I = np.zeros(4)
        self.D = np.asarray([config.D_xy, config.D_xy, config.D_z, 0])
        return config

def twist2np(msg):
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    z = msg.twist.linear.z
    yaw = msg.twist.angular.z
    return np.asarray([x, y, z, yaw])

def pose2np(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    q = (msg.pose.orientation.x, 
         msg.pose.orientation.y, 
         msg.pose.orientation.z, 
         msg.pose.orientation.w)
    r, p, yaw = euler_from_quaternion(q)
    return np.asarray([x, y, z, yaw])

def np2twist(arr):
    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.twist.linear.x = arr[0]
    msg.twist.linear.y = arr[1]
    msg.twist.linear.z = arr[2]
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = arr[3]
    return msg
 
if __name__ == '__main__':
    rospy.init_node('pid_controller')
    pC = PIDController()
    