#!/usr/bin/env python

# MotorController.py
# @author Samuel Wejeus (wejeus@kth.se)

import roslib; roslib.load_manifest('amee')
import rospy, math, operator
from std_msgs.msg import String, Int32
from amee.msg import MovementCommand, KeyboardCommand, Velocity, Odometry, IRDistances
#from turtlesim.msg import Velocity as turtleCommand

NODE_NAME = "MovementControl"

TYPE_MOVE_STRAIGHT = 1
TYPE_MOVE_ROTATE = 2
TYPE_MOVE_COORDINATE = 3
TYPE_FOLLOW_WALL = 4
TYPE_STOP_FOLLOW_WALL = 5

MOVEMENT_SPEED = 0.3
MAX_ROTATION_SPEED = 0.1
MIN_ROTATION_SPEED = 0.06


IR_BASE_RIGHT = 0.104

linearSpeed = 0.1
rotationSpeed = 0.05
K_p_1 = 0.03
K_p_2 = 0.1
refDistance = 0.04
noWallDistance = 0.15
wallDistTol = 0.01


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
        self.followWall = False

        # distance/angle is mesured in respect to origin of start of movement
        self.totalDistance = 0.0 # Meters
        self.totalAngle = 0.0    # Degrees
        self.ir_leftBack = 0.0
        self.ir_leftFront = 0.0
        self.ir_rightBack = 0.0
        self.ir_rightFront = 0.0

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

    # totalAngle
    # lastAngle
    #
    def move_rotate(self, degreesToTravel):
        rospy.loginfo("ROTATING: %s DEGREES", degreesToTravel)
        loopRate = rospy.Rate(5)
        lastAngle = self.totalAngle
        travelledAngle = 0
        
        # TODO: add integral control to speed up last part of turn
        # TODO: make K_p tuneable while driving the robot
        # Proportional gain constant (tune this to imropve turn performance)
        K_p = 1.0/200.0 # starts to slow down 20 degrees before final angle
        angleError = degreesToTravel - travelledAngle

        while abs(angleError) > 0.5:            
            # lower speed as we come closer to "degreesToTravel"
            rotationSpeed = K_p * angleError
            # saturate speed to ROTATION_SPEED if too high
            rotationSpeed = math.copysign(MAX_ROTATION_SPEED, rotationSpeed) if abs(rotationSpeed) > MAX_ROTATION_SPEED else rotationSpeed 
            # saturate speed to ROTATION_SPEED if too low
            rotationSpeed = math.copysign(MIN_ROTATION_SPEED, rotationSpeed) if abs(rotationSpeed) < MIN_ROTATION_SPEED else rotationSpeed 
            
            self.move(rotationSpeed, -rotationSpeed)

            rospy.loginfo("degrees rotated: %s", self.totalAngle)
            rospy.loginfo("angle error: %s", angleError)
            loopRate.sleep()
            travelledAngle = self.totalAngle - lastAngle
            angleError = degreesToTravel - travelledAngle

        rospy.loginfo("DONE. ROTATED: %s", travelledAngle)
        rospy.loginfo("DONE. ROTATED TOTAL: %s", self.totalAngle)

        self.stop_motors()

    def move_coordinate(self, x, y):
        ref_vec = (1.0, 0.0) # Reference vector, set to coordinate axis in direction of robot
        point = (x, y)
        distance = norm(point)
        angle = math.acos(dot_product(ref_vec, point) / (norm(ref_vec)*norm(point)))
        self.move_rotate(angle * (180/math.pi))
        self.move_straight(distance)

    def follow_wall(self):
        rospy.loginfo("Following wall")

        self.followWall = True
        loopRate = rospy.Rate(10)

        while self.followWall:
            linearSpeed = rospy.get_param("/linearSpeed") # This updates the rotationSpeed on the ParamServer
            K_p_1 = rospy.get_param("/K_p_1")
            K_p_2 = rospy.get_param("/K_p_2")
            refDistance = rospy.get_param("/refDistance")

            ir_right_mean = (self.ir_rightBack + self.ir_rightFront)/2
            angle_to_wall = math.tan((self.ir_rightBack - self.ir_rightFront) / IR_BASE_RIGHT)
            distance_to_wall = math.cos(angle_to_wall) * ir_right_mean
            
            #if abs(refDistance - distance_to_wall) > noWallDistance:
            #    self.move_straight(0.1)
            #    self.move_rotate(90)
            if abs(refDistance - distance_to_wall) < wallDistTol: # if we're at distance_to_wall +- 5cm 
                error = self.ir_rightBack - self.ir_rightFront
                rotationSpeed = K_p_1 * error # TODO: add integrating control if needed
            else:
                error = refDistance - ir_right_mean
                rotationSpeed = K_p_2 * error # TODO: add integrating control if needed

            self.move(linearSpeed + rotationSpeed, linearSpeed - rotationSpeed)
            loopRate.sleep()

        rospy.loginfo("Stop following wall")
        self.followWall = False
        self.stop_motors()
        

   # ----------------------- ROS CALLBACKS ----------------------- #

    def handle_odometry_change(self, msg):
        self.totalDistance = msg.distance
        self.totalAngle = msg.angle

    def handle_ir_change(self, msg):
        self.ir_leftBack = msg.leftBack
        self.ir_leftFront = msg.leftFront
        self.ir_rightBack = msg.rightBack
        self.ir_rightFront = msg.rightFront
        
    def handle_static_change(self, msg):
        if msg.type == TYPE_MOVE_STRAIGHT:
            self.move_straight(msg.distance)
        elif msg.type == TYPE_MOVE_ROTATE:
            self.move_rotate(msg.angle)
        elif msg.type == TYPE_MOVE_COORDINATE:
            self.move_coordinate(msg.x, msg.y)
        elif msg.type == TYPE_FOLLOW_WALL:
            self.follow_wall()
        elif msg.type == TYPE_STOP_FOLLOW_WALL:
            self.followWall = False
        else:
            rospy.logwarn(NODE_NAME + ' UNKNOWN_MOVEMENT')

    def handle_keyboard_change(self, msg):
        if msg.linear == 2:
            self.move_straight(0.2)
        elif msg.linear == -2:
            self.move_straight(-0.2)
        elif msg.angular == 2:
            self.move_rotate(10)
        elif msg.angular == -2:
            self.move_rotate(-10)


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
        rospy.Subscriber("/amee/sensors/irdistances", IRDistances , controller.handle_ir_change)
        #rospy.Subscriber("/KeyboardControl/KeyboardCommand", KeyboardCommand, controller.handle_keyboard_change)
        #rospy.Subscriber("/turtle1/command_velocity", turtleCommand, controller.handle_keyboard_change)

        # Get/set the default values from ParamServer
        rospy.set_param("/linearSpeed", linearSpeed)
        rospy.set_param("/rotationSpeed", rotationSpeed)
        rospy.set_param("/K_p_1", K_p_1)
        rospy.set_param("/K_p_2", K_p_2)
        rospy.set_param("/refDistance", refDistance)
        rospy.set_param("/noWallDistance", noWallDistance)
        rospy.set_param("/wallDistTol", wallDistTol)

        # linearSpeed = rospy.get_param("/linearSpeed", linearSpeed)
        # rotationSpeed = rospy.get_param("/rotationSpeed", rotationSpeed)
        # K_p_1 = rospy.get_param("/K_p_1")
        # K_p_2 = rospy.get_param("/K_p_2")
        # refDistance = rospy.get_param("/refDistance")
        # noWallDistance = rospy.get_param("/noWallDistance")
        # wallDistTol = rospy.get_param("/wallDistTol")

        rospy.loginfo("... done! Entering spin() loop")

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException: pass
