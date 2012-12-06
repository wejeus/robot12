This is the source code of group 0.
The project's and robot's name is amee.


To build the project use the provided CMakeLists.txt
The project requires opencv and Eigen to be built.

This archive contains the following subfolders:

	src
	nodes
	msg
	IRSensorCalibration

-----src------
In this folder you find all the cpp-source files of our project.
The code of each ros node we wrote is stored in its own subfolder. For instance the node responsible for reading the ir distances from 
the raw adc values is called IRSensorReader and its source is in the folder src/IRSensorReader.
There are both nodes that are necessary for running amee and some nodes that we only wrote as tools that helped us developing.
The main nodes you need for running amee are:

MotorControl		 	---	controls the motors
IRSensorReader			---	transforms adc values to distances
MovementControl			---	provides higher level movement functionalities
Mapper				---	maps and localizes the robot in a maze. Creates the graph for path planning. Keeps track of unexplored areas.
TagDetector (in folder Camera)	---	Tag detection and classification
StrategyControl			---	Path planning and execution

optional:
PublishAmee			---	convenience tool that publishs certain messages to the different nodes, eg. PublishAmee wall starts wall following.
KeyHandler			--- 	allows to remote control amee
IMUReader			---	reads IMU values

Further there are the sub folders:
	amee			---	created by ros
	Graph			---	graph data structure and path planner used for StrategyControl and Mapper
	Localize		---	contains source files of a not correctly working Kalman filter
	MeasureAmee		---	a test node to check how much the odometry drifts
	NodeKiller		---	tool to kill processes
	SensorSync		--- 	code for synchronizing messages that due to the lack of time wasn't used in the end
	SonarControl		---	a node that moves a servo on which the sonar was mounted and publishes the measured distances for each servo position (not used)


-----nodes------
Contains all the python scripts we wrote.
Apart from our first implementation of MovementControl (the one we used for milestone 0) it contains some scripts we used while developing.
The most important one is MapVisualizer as it visualizes the current map the Mapper node provides and the robot's position in it.

-----msg------
Contains all the messages we used.

-----IRSensorCalibration-----
Contains matlab scripts for the calibration of the IR sensors. We tried calibrating it on two different ways:
first by estimating an exponential function that fits the adc values best and then by linearizing the relationship between distances
and adc values. The latter one is the one we used at the end.
