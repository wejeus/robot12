#!/usr/bin/env python

# MotorController.py
# @author Samuel Wejeus (wejeus@kth.se)

import roslib; roslib.load_manifest('amee')
import rospy
import math, operator
from std_msgs.msg import String, Int32
from amee.msg import Movement
from robo.msg import Encoder, Motor
from turtlesim.msg import Velocity as V


# Send custom msg in ROS
# rostopic pub -1 <the topic to publish to> <the message type to use> -- <values>
# rostopic pub -1 /MotorController/Movement amee/Movement -- 1.0 1.0

# Node functionality
# Rotate a certain angle A on the spot 
# Move straight a distance D
# Move to a certain position (x,y) wrt to the statring position (forward at the start is the x-direction)
# Control robot using a device such as keyboard or joystick


NODE_NAME = "MotorController"
MOVE_STRAIGHT = 1
MOVE_ROTATE = 2
MOVE_COORDINATE = 3
WHEEL_RADIUS = 0.1
TICKS_REV = 500

mPublisherMotor = {}
mPublisherEncoderInterval = {}
# mEncoderCurrent = (0, 0.0, 0.0)
# mEncoderPrevious  = (0, 0.0, 0.0)


def norm(vector):
    return math.sqrt(sum(map(lambda coord: coord*coord, vector)))

def dot_product(vector1, vector2):
    if len(vector1) != len(vector2):
        raise Exception("Vector size mismatch")
    return sum(map(operator.mul, vector1, vector2))

# Move to a 2 dimensional point
def move_coordinate(point=(0,0)):
    rospy.loginfo(NODE_NAME + ' MOVE_COORDINATE')
    ref_vec = (0,1) # Reference vector, set to coordinate axis in direction of robot
    distance = norm(point)
    angle = math.acos(dot_product(ref_vec, point) / norm(ref_vec)*norm(point))
    move_rotate(angle)
    move_straight(distance)



def move_straight(distance):
    rospy.loginfo(NODE_NAME + ' MOVE_STRAIGHT')
    curDistance = 0.0

    while curDistance < distance:
        error = calc_error()
        C = 2*math.pi*WHEEL_RADIUS
        mPublisherMotor.pub(1.0, 1.0)

    return

def move_rotate(angle):
    rospy.loginfo(NODE_NAME + ' MOVE_ROTATE')
    return

# TODO: Should this take a (singleshot) velocity change or a vector point to travel to?
def handle_static_change(msg):
    if msg.type == MOVE_STRAIGHT:
        move_straight(msg.distance)
    elif msg.type == MOVE_ROTATE:
        move_rotate(msg.angle)
    elif msg.type == MOVE_COORDINATE:
        move_coordinate((msg.x, msg.y))
    else:
        rospy.logwarn(NODE_NAME + ' UNKNOWN_MOVEMENT')

    # (left, right) = determine(msg.linear, msg.angular)
    # rospy.loginfo(rospy.get_name() + " Determined (static) change -> left: %s, right: %s", left, right)
    # pubMotor.publish(Motor(left, right))

def handle_keyboard_change(msg):
    (left, right) = determine(msg.linear, msg.angular)
    rospy.loginfo(rospy.get_name() + " Determined (keyboard) change -> left: %s, right: %s", left, right)
    pubMotor.publish(Motor(left, right))

# Returns (derivLeft, derivRight)
def calc_tic_speed(encoderCurrent, encoderPrevious):
    (curTime, curRight, curLeft) = encoderCurrent
    (prevTime, prevRight, prevLeft) = encoderPrevious
    deltaTime = (curTime - prevTime) * 1000
    return ( deltaTime, (curLeft - prevLeft)/deltaTime, (curRight - prevRight)/deltaTime )


# prevEncoderMean = (0.0,0.0,0.0)
# currEncoderMean = (0.0,0.0,0.0)
# currMesurement = (0.0,0.0,0.0)
# counter = 0
# # TODO INIT> set prev
# init = False 
# allMesurements = (0.0, 0.0, 0.0)

# def update_mean(encoder):
#     global allMesurements, prevEncoderMean, currEncoderMean, counter

#     allMesurements = tuple(map(operator.add, allMesurements, encoder))
#     counter += 1
    
#     if (counter == 10):
#         prevEncoderMean = currEncoderMean
#         currEncoderMean = tuple(map(lambda x : x * 1.0/counter, allMesurements))
#         allMesurements = (0.0,0.0,0.0)
#         init = True
#         counter = 0
        

curEncoder = False
prevEncoder = False
ticsSpeedLeftMax = 500*1.0 # FIXME (tics per revolution times v_max per motor)
ticsSpeedRightMax = 500*1.2 # FIXME (tics per revolution times v_max per motor)

def handle_encoder(msg):
    global curTicSpeed, curEncoder, prevEncoder

    # Handle initial case when ecoders are empty
    if not curEncoder:
        curEncoder = (msg.timestamp, msg.right, msg.left)
        return

    # ----------- Normal workflow ------------

    prevEncoder = curEncoder
    curEncoder = (msg.timestamp, msg.right, msg.left)

    (timestamp, ticsSpeedRight, ticsSpeedLeft) = calc_tic_speed(curEncoder, prevEncoder)

    pwmRight = ticsSpeedRight / ticsSpeedRightMax
    pwmLeft = ticsSpeedLeft / ticsSpeedLeftMax

    rospy.loginfo("DELTA_TIME: %s R: %s L: %s", timestamp, ticsSpeedRight, ticsSpeedLeft)
    rospy.loginfo("PWM_RIGH: %s PWM_LEFT: %s", pwmRight, pwmLeft)


if __name__ == '__main__':
    # Register this node
    rospy.init_node(NODE_NAME)
    rospy.loginfo(NODE_NAME + " is starting up...")
    
    # mEncoderCurrent = (0.0, 0.0, 0.0)
    # mEncoderPrevious  = (0.0, 0.0, 0.0)
    
    try:
        # Init publishers
        mPublisherMotor = rospy.Publisher('/serial/motor_speed', Motor)
        mPublisherEncoderInterval = rospy.Publisher('/serial/encoder_interval', Int32)

        # The TOPIC we want to listen to
        rospy.Subscriber("/MotorController/Movement", Movement, handle_static_change)
        rospy.Subscriber("/turtle1/command_velocity", V, handle_keyboard_change)
        rospy.Subscriber("/serial/encoder", Encoder, handle_encoder)
       
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException: pass