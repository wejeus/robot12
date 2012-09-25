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

mPublisherMotor = None
mPublisherEncoderInterval = None


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
    global mPublisherMotor

    rospy.loginfo(NODE_NAME + ' MOVE_STRAIGHT: %s', distance)
    try:
        if distance > 0.0:
            rospy.loginfo(NODE_NAME + ' 1: %s', distance)
            mPublisherMotor.publish(1.0, 1.0)
        elif distance < 0.0:
            rospy.loginfo(NODE_NAME + ' 2: %s', distance)
            mPublisherMotor.publish(-1.0, -1.0)
        else:
            rospy.loginfo(NODE_NAME + ' 3: %s', distance)
            mPublisherMotor.publish(0.0, 0.0)
    except:
        rospy.loginfo("error")
    # while curDistance < distance:
    #     error = calc_error()
    #     C = 2*math.pi*WHEEL_RADIUS
    #     mPublisherMotor.publish(1.0, 1.0)
    # return

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

# Calculate the tic speed of individual wheels by determining the number of tics 
# that have passed between two mesurement pointss and divides by the time interval
# elapsed between these points to get the tic speed.
# Speed is measured in tics per 100 miliseconds
# TODO Why is this time measurement correct?
def calc_tic_speed(encoderCurrent, encoderPrevious):
    (curTime, curRight, curLeft) = encoderCurrent
    (prevTime, prevRight, prevLeft) = encoderPrevious
    deltaTime = (curTime - prevTime) * 1000
    return deltaTime, (curRight - prevRight)/deltaTime, (curLeft - prevLeft)/deltaTime
        


class Controller:
    def __init__(self):
        self.encoders = (0.0, 0.0, 0.0)
        self.velocity = 0.0
        self.ticSpeedRight = 0.0
        self.ticSpeedLeft = 0.0
        self.ticSpeedRightMax = 500*1.2 # FIXME (tics per revolution times v_max per motor)
        self.ticSpeedLeftMax = 500*1.0 # FIXME (tics per revolution times v_max per motor)

    def set_velocity(self, velocity):
        self.velocity = velocity

    def set_tic_speed(self, right, left):
        self.ticSpeedRight = right
        self.ticSpeedLeft = left

    def determine_next_pwm(self):
        # Check motor error
        # Check IR sensors
        # Test if moving straight?
        # Test if angular movement?
        pwmRight = ticSpeedRight / ticsSpeedRightMax
        pwmLeft = ticSpeedLeft / ticsSpeedLeftMax
        return (pwmRight, pwmLeft)

    def add_encoder(self, encoder):
        self.encoders = encoder

    # Should return a list of encoders history so we can take the average
    # Currently only returns on previous
    def get_encoders(self):
        return self.encoders

    def debug_print(self):
        rospy.loginfo("TIC_SPEED_R: %s TIC_SPEED_L: %s", self.ticSpeedRight, self.ticSpeedLeft)



# Input -> msg(timestamp, right, left) where (left, right) 
# is the number of tics the wheels have rotated (sum of positive and negative direction)
def handle_encoder(msg):
    #rospy.loginfo("RAW_TIME: %s RAW_R: %s RAW_L: %s", msg.timestamp, msg.right, msg.left)
    global controller

    currentEncoder = (msg.timestamp, msg.right, msg.left)
    previousEncoder = controller.get_encoders()
    controller.add_encoder(currentEncoder)

    (diffTime, ticSpeedRight, ticSpeedLeft) = calc_tic_speed(currentEncoder, previousEncoder)

    controller.set_tic_speed(ticSpeedRight, ticSpeedLeft)
    controller.debug_print()



controller = Controller()

if __name__ == '__main__':
    # Register this node
    rospy.init_node(NODE_NAME)
    rospy.loginfo(NODE_NAME + " is starting up...")
    
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