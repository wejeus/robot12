#!/usr/bin/python

import sys
import commands

print 'Number of arguments:', len(sys.argv), 'arguments.'
print 'Argument List:', str(sys.argv)

if sys.argv[1] == 'forward':
	print "Moving forward 5 meters"
	print commands.getstatusoutput("rostopic pub -1 /MotorController/Movement amee/Movement -- 1 5.0 0.0 0.0 0.0")
elif sys.argv[1] == 'rotate':
	print "Rotation 90 degrees"
	print commands.getstatusoutput("rostopic pub -1 /MotorController/Movement amee/Movement -- 2 0.0 90.0 0.0 0.0")
elif sys.argv[1] == 'point':
	print "Moving to point (2, 2)"
	print commands.getstatusoutput("rostopic pub -1 /MotorController/Movement amee/Movement -- 3 0.0 0.0 2.0 2.0")
elif sys.argv[1] == 'reset':
	print "Reseting motors to (0, 0)"
	print commands.getstatusoutput('rostopic pub -1 /serial/motor_speed robo/Motor -- 0.0 0.0')