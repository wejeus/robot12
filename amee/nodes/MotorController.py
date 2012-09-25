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
import Helpers


# Send custom msg in ROS
# rostopic pub -1 <the topic to publish to> <the message type to use> -- <values>
# rostopic pub -1 /MotorController/Movement amee/Movement -- 1.0 1.0

NODE_NAME = "MotorController"
MOVE_STRAIGHT = 1
MOVE_ROTATE = 2
MOVE_COORDINATE = 3
WHEEL_RADIUS = 0.1
TICKS_REV = 500
ticSpeedRightMax = 500*1.2 # FIXME (tics per revolution times v_max per motor)
ticSpeedLeftMax = 500*1.0 # FIXME (tics per revolution times v_max per motor)
        

class Controller:
    def __init__(self, publisherMotor, publisherEncoderInterval):
        self.publisherMotor = publisherMotor
        self.publisherEncoderInterval = publisherEncoderInterval
        self.currentEncoder = (0.0, 0.0, 0.0)
        self.previousEncoder = (0.0, 0.0, 0.0)
        self.velocity = 0.0
        self.ticSpeedRight = 0.0
        self.ticSpeedLeft = 0.0

    def determine_next_pwm(self):
        # Check motor error, Check IR sensors, is moving straight? is angular movement?
        maxVelocityRight = 1.0 # What should this be?
        maxVelocityLeft = 1.0 # What should this be?

        # Right motor
        pwmFeedbackRight = self.ticSpeedRight / 500
        pwmRefRight = self.velocity * (1/maxVelocityRight)
        pwmErrorRight = pwmRefRight - pwmFeedbackRight
        controlledPwmRight = pwmRefRight + pwmErrorRight

        # Left motor
        pwmFeedbackLeft = self.ticSpeedLeft / 500
        pwmRefLeft = self.velocity * (1/maxVelocityLeft)
        pwmErrorLeft = pwmRefLeft - pwmFeedbackLeft
        controlledPwmLeft = pwmRefLeft + pwmErrorLeft
        return (controlledPwmRight, controlledPwmLeft)

    def move_straight(self, distance):
        # TODO: Some while loop that checks if we reached target pos
        PR, PL = self.determine_next_pwm()
        self.publisherMotor.publish(1.0, 1.0)


    def move_rotate(self, angle):
        return

    # Calculate the tic speed of individual wheels by determining the number of tics 
    # that have passed between two mesurement pointss and divides by the time interval
    # elapsed between these points to get the tic speed.
    # Speed is measured in tics per 100 miliseconds
    # TODO Why is this time measurement correct?
    # should this be: tics/s = (tic_t - tic_t-1) / dt
    def update_tic_speed(self):
        (curTime, curRight, curLeft) = self.currentEncoder
        (prevTime, prevRight, prevLeft) = self.previousEncoder
        deltaTime = (curTime - prevTime) * 1000
        self.ticSpeedRight = (curRight - prevRight) / deltaTime
        self.ticSpeedLeft = (curLeft - prevLeft) / deltaTime


    # ----------------------- ROS CALLBACKS ----------------------- #

    def handle_static_change(self, msg):
        if msg.type == MOVE_STRAIGHT:
            self.move_straight(msg.distance)
        elif msg.type == MOVE_ROTATE:
            self.move_rotate(msg.angle)
        # elif msg.type == MOVE_COORDINATE:
        #     controller.move_coordinate((msg.x, msg.y))
        else:
            rospy.logwarn(NODE_NAME + ' UNKNOWN_MOVEMENT')

    # Input -> msg(timestamp, right, left) where (left, right) 
    # is the number of tics the wheels have rotated (sum of positive and negative direction)
    def handle_encoder(self, msg):
        self.previousEncoder = self.currentEncoder
        self.currentEncoder = (msg.timestamp, msg.right, msg.left)
        self.update_tic_speed()

        # A little debug..
        PR, PL = self.determine_next_pwm()
        rospy.loginfo("TICSPEED_R: %s TICSPEED_L: %s", self.ticSpeedRight, self.ticSpeedLeft)
        rospy.loginfo("PWM_R: %s PWM_L: %s", PR, PL)


    def handle_keyboard_change(self, msg):
        (left, right) = determine(msg.linear, msg.angular)
        rospy.loginfo(rospy.get_name() + " Determined (keyboard) change -> left: %s, right: %s", left, right)
        pubMotor.publish(Motor(left, right))


controller = None

if __name__ == '__main__':
    # Register this node
    rospy.init_node(NODE_NAME)
    rospy.loginfo(NODE_NAME + " is starting up...")
    
    try:
        # Init publishers
        publisherMotor = rospy.Publisher('/serial/motor_speed', Motor)
        publisherEncoderInterval = rospy.Publisher('/serial/encoder_interval', Int32)

        controller = Controller(publisherMotor, publisherEncoderInterval)

        # The TOPIC we want to listen to
        rospy.Subscriber("/MotorController/Movement", Movement, controller.handle_static_change)
        rospy.Subscriber("/turtle1/command_velocity", V, controller.handle_keyboard_change)
        rospy.Subscriber("/serial/encoder", Encoder, controller.handle_encoder)
       
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException: pass