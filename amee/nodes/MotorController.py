#!/usr/bin/env python

# MotorController.py
# @author Samuel Wejeus (wejeus@kth.se)

import roslib; roslib.load_manifest('amee')
import rospy
from std_msgs.msg import String, Int32
from amee.msg import Velocity
from robo.msg import Encoder, Motor
from turtlesim.msg import Velocity as V


# Send custom msg: rostopic pub -1 /MotorController/Velocity amee/Velocity -- 1.0 1.0

pubMotor = {}
pubEncoderInterval = {}

RIGHT = -2.0
LEFT = 2.0
FORWARD = 2.0
BACKWARD = -2.0

# In /turtle only one key can be pressed at a time
def determine(linear, angular):
    leftWheel = 0.0
    rightWheel = 0.0

    if linear != 0.0:
        leftWheel = linear
        rightWheel = linear
    else:
        if angular == RIGHT:
            leftWheel = FORWARD
            rightWheel = BACKWARD
        else:
            leftWheel = BACKWARD
            rightWheel = FORWARD
    return (leftWheel, rightWheel)

# TODO: Should this take a (singleshot) velocity change or a vector point to travel to?
def handle_static_change(msg):
    (left, right) = determine(msg.linear, msg.angular)
    rospy.loginfo(rospy.get_name() + " Determined (static) change -> left: %s, right: %s", left, right)
    pubMotor.publish(Motor(left, right))

def handle_keyboard_change(msg):
    (left, right) = determine(msg.linear, msg.angular)
    rospy.loginfo(rospy.get_name() + " Determined (keyboard) change -> left: %s, right: %s", left, right)
    pubMotor.publish(Motor(left, right))

def handle_encoder(msg):
    timestamp = msg.timestamp
    left = msg.left
    right = msg.right
    #rospy.loginfo(rospy.get_name() + " Encoder got: %s %s %s", timestamp, left, right)
    # TODO: Handle trajectory error


if __name__ == '__main__':
    rospy.loginfo(rospy.get_name() + " is starting up...")
    
    try: 
        # Name of this node
        rospy.init_node('MotorController')

        # Init publishers
        pubMotor = rospy.Publisher('/serial/motor_speed', Motor)
        pubEncoderInterval = rospy.Publisher('/serial/encoder_interval', Int32)

        # The TOPIC we want to listen to
        rospy.Subscriber("/MotorController/Velocity", Velocity, handle_static_change)
        rospy.Subscriber("/turtle1/command_velocity", V, handle_keyboard_change)
        rospy.Subscriber("/serial/encoder", Encoder, handle_encoder)
       
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException: pass