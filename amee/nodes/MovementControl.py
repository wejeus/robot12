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
        self.travelledDistance = 0.0 # Meters
        self.travelledAngle = 0.0    # Degrees

    def stop_motors(self):
        rospy.loginfo("STOPPING MOTORS")
        self.publisherMotor.publish(0.0, 0.0)
        self.travelledDistance = 0.0
        self.travelledAngle = 0.0

    def move(self, rightVelocity, leftVelocity):
        self.publisherMotor.publish(rightVelocity, leftVelocity)

    def move_straight(self, distance):
        rospy.loginfo("MOVING: %s METRES", distance)
        direction = 1 if distance > 0 else -1
        loopRate = rospy.Rate(5)
        
        # full speed ahead!
        self.move(direction*MOVEMENT_SPEED, direction*MOVEMENT_SPEED)
        
        while self.travelledDistance < abs(distance - 0.05):
            rospy.loginfo("distance travelled: %s", self.travelledDistance)
            loopRate.sleep()

        rospy.loginfo("DONE. MOVED: %s", self.travelledDistance)    
        self.stop_motors()

    # FIXME Make sure angle is degrees...
    def move_rotate(self, angle):
        rospy.loginfo("ROTATING: %s DEGREES", angle)
        degreesToTravel = abs(angle)
        direction = 1 if angle > 0 else -1
        loopRate = rospy.Rate(5)

        self.move(direction*ROTATION_SPEED, direction*(-ROTATION_SPEED))
        
        while abs(self.travelledAngle) < (degreesToTravel - 0.05):
            rospy.loginfo("degrees rotated: %s", self.travelledAngle)
            loopRate.sleep()
        
        rospy.loginfo("DONE. ROTATED: %s", abs(self.travelledAngle))
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
        self.travelledDistance = msg.distance
        self.travelledAngle = msg.angle
        
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