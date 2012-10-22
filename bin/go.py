#!/usr/bin/python

import sys
import commands

motorCmd = "rostopic pub -1 /MovementControl/MovementCommand amee/MovementCommand -- %s %s %s %s %s"

if sys.argv[1] == 'forward':
	meters = sys.argv[2]
	print ("Moving forward %s meters", meters)
	print commands.getstatusoutput(motorCmd % (1, meters, 0.0, 0.0, 0.0))
elif sys.argv[1] == 'rotate':
	degrees = sys.argv[2]
	print ("Rotating %s degrees", degrees)
	print commands.getstatusoutput(motorCmd % (2, 0.0, degrees, 0.0, 0.0))
elif sys.argv[1] == 'point':
	(x, y) = (sys.argv[2], sys.argv[3])
	print ("Moving to point (%s, %s)" % (x, y))
	print commands.getstatusoutput(motorCmd % (3, 0.0, 0.0, x, y))
elif sys.argv[1] == 'wall':
	print ("Following wall")
	print commands.getstatusoutput(motorCmd % (4, 0.0, 0.0, 0.0, 0.0))
elif sys.argv[1] == 'reset':
	print "Reseting motors to (0, 0)"
	print commands.getstatusoutput('rostopic pub -1 /serial/motor_speed robo/Motor -- 0.0 0.0')
else:
	print "Unknow command"