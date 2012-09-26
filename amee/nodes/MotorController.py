#!/usr/bin/env python

# MotorController.py
# @author Samuel Wejeus (wejeus@kth.se)

import roslib; roslib.load_manifest('amee')
import rospy, math, operator
from std_msgs.msg import String, Int32
from amee.msg import Movement
from robo.msg import Encoder, Motor
from turtlesim.msg import Velocity as V

NODE_NAME = "MotorController"
MOVE_STRAIGHT = 1
MOVE_ROTATE = 2
MOVE_COORDINATE = 3
WHEEL_RADIUS = 0.1

REVOLUTION_PER_SEC_RIGHT = 1.0 # What should this be?
REVOLUTION_PER_SEC_LEFT = 1.0 # What should this be?
TICS_PER_REVOLUTION = 500
ROTATION_SPEED = 0.4

def norm(vector):
    return math.sqrt(sum(map(lambda coord: coord*coord, vector)))

def dot_product(vector1, vector2):
    if len(vector1) != len(vector2):
        raise Exception("Vector size mismatch")
    return sum(map(operator.mul, vector1, vector2))



class Controller:
    def __init__(self, publisherMotor, publisherEncoderInterval):
        self.publisherMotor = publisherMotor
        self.publisherEncoderInterval = publisherEncoderInterval
        self.currentEncoder = (0.0, 0.0, 0.0)
        self.previousEncoder = (0.0, 0.0, 0.0)

        self.motorRight = 0.0
        self.motorLeft = 0.0
        self.ticSpeedRight = 0.0
        self.ticSpeedLeft = 0.0

        self.positionHistory = []

    def get_current_pwm_velocities(self):
        # Check motor error, Check IR sensors, is moving straight? is angular movement?
        pwmRight = self.ticSpeedRight / (REVOLUTION_PER_SEC_RIGHT * TICS_PER_REVOLUTION)
        pwmLeft = self.ticSpeedLeft / (REVOLUTION_PER_SEC_LEFT * TICS_PER_REVOLUTION)
        return (pwmRight, pwmLeft)

    def calc_next_point(self):
        ticLength = 2*math.pi/500
        distanceBetweenWheels = 0.5

        deltaDistanceRight = self.ticSpeedRight * ticLength
        deltaDistanceLeft = self.ticSpeedLeft * ticLength
        deltaDistanceTotal = (deltaDistanceRight + deltaDistanceLeft) / 2

        deltaAngle = (deltaDistanceRight - deltaDistanceLeft) / distanceBetweenWheels

        someAngle = 0.0 # TODO!!! Which angle is this?
        deltaY = deltaDistanceTotal*math.sin(someAngle + deltaAngle/2)
        deltaX = deltaDistanceTotal*math.cos(someAngle + deltaAngle/2)
        
        return (deltaX, deltaY, deltaAngle)

    def get_travelled_distance(self):
        total = (0.0, 0.0, 0.0)
        for point in self.positionHistory:
            total = tuple(map(operator.add, total, point))
        return total
   
    def stop_motors(self):
        self.publisherMotor.publish(0.0, 0.0)

    def move(self, leftVelocity, rightVelocity):
        (currentPWMRight, currentPWMLeft) = self.get_current_pwm_velocities()

        # The velocity that we want
        rightPWMVelocity = rightVelocity / (2.0 * math.pi * WHEEL_RADIUS * REVOLUTION_PER_SEC_RIGHT)
        leftPWMVelocity = leftVelocity / (2.0 * math.pi * WHEEL_RADIUS * REVOLUTION_PER_SEC_LEFT)

        rightError = rightPWMVelocity - currentPWMRight
        leftError = leftPWMVelocity - currentPWMLeft

        self.motorRight = self.motorRight + 0.05 * rightError
        self.motorLeft = self.motorLeft + 0.05 * leftError

        self.publisherMotor.publish(self.motorRight, self.motorLeft)

    def move_straight(self, distance):
        travelledDistance = (0.0, 0.0, 0.0)
        loopRate = rospy.Rate(5)

        self.move(1.0, 1.0) # full speed ahead!

        while travelledDistance[0] < distance:
            rospy.loginfo("distance travelled: %s", travelledDistance[0])

            position = self.calc_next_point()
            self.positionHistory.append(position)
            travelledDistance = self.get_travelled_distance()
            loopRate.sleep() # TODO: How long should we sleep?

        self.stop_motors()
        self.positionHistory = []

    # FIXME Make sure angle is degrees...
    def move_rotate(self, angle):
        degreesToTravel = abs(angle / (180.0 * math.pi * 0.25))
        sign = 1 if angle > 0 else -1
        travelledDistance = (0.0, 0.0, 0.0)
        loopRate = rospy.Rate(5)

        self.move(sign*ROTATION_SPEED, sign*(-ROTATION_SPEED))
        
        while travelledDistance[2] < degreesToTravel:
            rospy.loginfo("degrees rotated: %s", travelledDistance[2])

            position = self.calc_next_point()
            self.positionHistory.append(position)
            travelledDistance = self.get_travelled_distance()
            loopRate.sleep() # TODO: How long should we sleep?

        self.stop_motors()
        self.positionHistory = []

    def move_coordinate(self, x, y):
        ref_vec = (1.0, 0.0) # Reference vector, set to coordinate axis in direction of robot
        point = (2.0, 2.0)
        distance = norm(point)
        angle = math.acos(dot_product(ref_vec, point) / (norm(ref_vec)*norm(point)))
        self.move_rotate(angle)
        self.move_straight(distance)

    # Calculate the tic speed of individual wheels by determining the number of tics 
    # that have passed between two mesurement pointss and divides by the time interval
    # elapsed between these points to get the tic speed.
    # Speed is measured in tics per second
    def update_tic_speed(self):
        (curTime, curRight, curLeft) = self.currentEncoder
        (prevTime, prevRight, prevLeft) = self.previousEncoder
        deltaTime = (curTime - prevTime)
        self.ticSpeedRight = (curRight - prevRight) / deltaTime
        self.ticSpeedLeft = (curLeft - prevLeft) / deltaTime


    # ----------------------- ROS CALLBACKS ----------------------- #

    def handle_static_change(self, msg):
        if msg.type == MOVE_STRAIGHT:
            self.move_straight(msg.distance)
        elif msg.type == MOVE_ROTATE:
            self.move_rotate(msg.angle)
        elif msg.type == MOVE_COORDINATE:
            self.move_coordinate(msg.x, msg.y)
        else:
            rospy.logwarn(NODE_NAME + ' UNKNOWN_MOVEMENT')

    # Input -> msg(timestamp, right, left) where (left, right) 
    # is the number of tics the wheels have rotated (sum of positive and negative direction)
    def handle_encoder(self, msg):
        self.previousEncoder = self.currentEncoder
        self.currentEncoder = (msg.timestamp, msg.right, msg.left)
        self.update_tic_speed()

    def handle_keyboard_change(self, msg):
        if msg.linear != 0.0:
            sign = 1 if msg.linear > 0 else -1
            self.move_straight(sign*0.5) # if up/down -> move 0.5 meters in that direction
        elif msg.angular != 0.0:
            sign = 1 if msg.angular > 0 else -1
            self.move_rotate(sign*10) # If left/righ key -> rotate 10 degrees


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

        rospy.loginfo("... done! Entering spin() loop")

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException: pass