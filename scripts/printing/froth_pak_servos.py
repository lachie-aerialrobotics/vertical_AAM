#! /usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from dynamixel_sdk import *

def init_dynamixels():
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    for i in range(2):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel ID "+i+" has been successfully connected")

def shutdown_dynamixels():
    # Disable Dynamixel Torque
    for i in range(2):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            
    # Close port
    portHandler.closePort()

def set_position(id, goal_position):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


def handle_open_nozzle(req):
    set_position(1, OPEN_POSITION_VALUE)
    resp = TriggerResponse()
    resp.success = True
    resp.message = String(str('Opened nozzle valve'))
    return resp

def handle_close_nozzle(req):
    set_position(1, CLOSED_POSITION_VALUE)
    rospy.sleep(0.5)
    set_position(2, OPEN_POSITION_VALUE)
    rospy.sleep(PURGE_TIME)
    set_position(2, CLOSED_POSITION_VALUE)
    resp = TriggerResponse()
    resp.success = True
    resp.message = String(str('Closed nozzle valve'))
    return resp

def froth_pak_services():
    
    init_dynamixels()
    
    rospy.Service('close_nozzle', Trigger, handle_close_nozzle)
    rospy.Service('open_nozzle', Trigger, handle_open_nozzle)
    rospy.spin()

    shutdown_dynamixels()


if __name__ == "__main__":
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    TORQUE_ENABLE               = 1
    TORQUE_DISABLE              = 0
    PROTOCOL_VERSION            = 2 
    
    rospy.init_node('pgo_services')

    OPEN_POSITION_VALUE         = rospy.get_param('print_servos/open_position')
    CLOSED_POSITION_VALUE       = rospy.get_param('print_servos/closed_position')   
    BAUDRATE                    = rospy.get_param('print_servos/baud')
    DEVICENAME                  = rospy.get_param('print_servos/port')
    PURGE_TIME                  = rospy.get_param('print_servos/purge_time')
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    froth_pak_services()

