#!/usr/bin/env python

# MotorController.py
# @author Samuel Wejeus (wejeus@kth.se)

import roslib; roslib.load_manifest('amee')
import rospy, math, operator
from std_msgs.msg import String, Int32
from amee.msg import MovementCommand, KeyboardCommand, Velocity, Odometry

NODE_NAME = "MovementControl"

TYPE_MOVE_STRAIGHT = 1
TYPE_MOVE_ROTATE = 2
TYPE_MOVE_COORDINATE = 3

MOVEMENT_SPEED = 0.3
ROTATION_SPEED = 0.3


# TODO: BUGFIX: positive/negative directions/angle is not handle correctly
# TODO: This can overshoot the target position with a small error
# TODO: How long should we sleep in move/rotate? (i.e. find best value)

def norm(vector):
    return math.sqrt(sum(map(lambda coord: coord*coord, vector)))

def dot_product(vector1, vector2):
    if len(vector1) != len(vector2):
        raise Exception("Vector size mismatch")
    return sum(map(operator.mul, vector1, vector2))


class Controller:
    def __init__(self, publisherMotor):
        self.publisherMotor = publisherMotor
        
        # distance/angle is mesured in respect to origin of start of movement
        self.totalDistance = 0.0 # Meters
        self.totalAngle = 0.0    # Degrees

    def stop_motors(self):
        rospy.loginfo("STOPPING MOTORS")
        self.publisherMotor.publish(0.0, 0.0)
        self.totalDistance = 0.0
        self.totalAngle = 0.0

    def move(self, rightVelocity, leftVelocity):
        self.publisherMotor.publish(rightVelocity, leftVelocity)

    def move_straight(self, distance):
        rospy.loginfo("MOVING: %s METRES", distance)
        direction = 1 if distance > 0 else -1
        loopRate = rospy.Rate(5)
        
        lastDistance = self.totalDistance
        traveledDistance = 0

        # full speed ahead!
        self.move(direction*MOVEMENT_SPEED, direction*MOVEMENT_SPEED)
        
        while abs(traveledDistance) < abs(distance - 0.05):
            rospy.loginfo("distance travelled: %s", traveledDistance)
            loopRate.sleep()
            traveledDistance = (self.totalDistance - lastDistance)  

        rospy.loginfo("DONE. MOVED: %s", traveledDistance)    
        self.stop_motors()

    # FIXME Make sure angle is degrees...
    def move_rotate(self, degreesToTravel):
        rospy.loginfo("ROTATING: %s DEGREES", degreesToTravel)
        loopRate = rospy.Rate(5)
        lastAngle = self.totalAngle
        travelledAngle = 0

        # TODO: make K_p tuneable while driving the robot
        # Proportional gain constant (tune this to imropve turn performance)
        K_p = 0.5
        # lower speed as we come closer to "degreesToTravel"
        rotationSpeed = ROTATION_SPEED * K_p * (degreesToTravel - self.totalAngle)
        # saturate speed to ROTATION_SPEED
        rotationSpeed = ROTATION_SPEED if rotationSpeed > ROTATION_SPEED else rotationSpeed 
        self.move(rotationSpeed, -rotationSpeed)
        #self.move(direction*angularVelocity, direction*(-angularVelocity))

        while abs(travelledAngle) < (abs(degreesToTravel) - 0.05):
            rospy.loginfo("degrees rotated: %s", self.totalAngle)
            loopRate.sleep()
            travelledAngle = self.totalAngle - lastAngle
        
        rospy.loginfo("DONE. ROTATED: %s", self.totalAngle)
        self.stop_motors()

    def move_coordinate(self, x, y):
        ref_vec = (1.0, 0.0) # Reference vector, set to coordinate axis in direction of robot
        point = (2.0, 2.0)
        distance = norm(point)
        angle = math.acos(dot_product(ref_vec, point) / (norm(ref_vec)*norm(point)))
        self.move_rotate(angle * (180/math.pi))
        self.move_straight(distance)


    # ----------------------- ROS CALLBACKS ----------------------- #

    def handle_odometry_change(self, msg):
        self.totalDistance = msg.distance
        self.totalAngle = msg.angle
        
    def handle_static_change(self, msg):
        if msg.type == TYPE_MOVE_STRAIGHT:
            self.move_straight(msg.distance)
        elif msg.type == TYPE_MOVE_ROTATE:
            self.move_rotate(msg.angle)
        elif msg.type == TYPE_MOVE_COORDINATE:
            self.move_coordinate(msg.x, msg.y)
        else:
            rospy.logwarn(NODE_NAME + ' UNKNOWN_MOVEMENT')

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
        publisherMotor = rospy.Publisher('/amee/motor_control/set_wheel_velocities', Velocity)
        
        controller = Controller(publisherMotor)

        # The TOPIC we want to listen to
        rospy.Subscriber("/MovementControl/MovementCommand", MovementCommand, controller.handle_static_change)
        rospy.Subscriber("/amee/motor_control/odometry", Odometry, controller.handle_odometry_change)
        rospy.Subscriber("/KeyboardControl/KeyboardCommand", KeyboardCommand, controller.handle_keyboard_change)

        rospy.loginfo("... done! Entering spin() loop")

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException: pass