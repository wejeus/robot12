#!/usr/bin/env python

# MotorController.py
# @author Samuel Wejeus (wejeus@kth.se)

import roslib; roslib.load_manifest('amee')
import rospy, math, operator
from std_msgs.msg import String, Int32
from time import time
from amee.msg import MovementCommand, KeyboardCommand, Velocity, Odometry, IRDistances


NODE_NAME = "Doctor"

INTERVAL = 5

class Controller:
    def __init__(self, publisherMotor):
        self.publisherMotor = publisherMotor
        self.motorVelocities = (0.0, 0.0)
        self.lastMessageTimestamp = time()

    def kill_motors(self):
        if self.motorVelocities != (0.0, 0.0):
            self.publisherMotor.publish(0.0, 0.0)


    def recent_velocity_update(self):
        return (time() - self.lastMessageTimestamp) > INTERVAL


   # ----------------------- ROS CALLBACKS ----------------------- #

    def handle_velocity_update(self, msg):
        self.motorVelocities = (msg.leftVelocity, msg.rightVelocity)
        self.lastMessageTimestamp = time()




controller = None

if __name__ == '__main__':
    # Register this node
    rospy.init_node(NODE_NAME)
    rospy.loginfo(NODE_NAME + " is here, what should I fix?")
    
    try:
        # Init publishers
        publisherMotor = rospy.Publisher('/amee/motor_control/set_wheel_velocities', Velocity)
        
        controller = Controller(publisherMotor)
        loopRate = rospy.Rate(10)

        # The TOPIC we want to listen to
        rospy.Subscriber('/amee/motor_control/set_wheel_velocities', Velocity, controller.handle_velocity_update)
        #rospy.Subscriber("/amee/sensors/irdistances", IRDistances , controller.handle_ir_change)
        

        while not rospy.is_shutdown():
            rospy.spin()
            if not controller.recent_velocity_update():
                controller.kill_motors()
                
            loopRate.sleep(5)

    except rospy.ROSInterruptException: pass
