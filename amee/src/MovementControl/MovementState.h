#ifndef MOVEMENT_STATE_H
#define MOVEMENT_STATE_H

#include "amee/Odometry.h"
#include "amee/IRDistances.h"

namespace amee{

	enum STATE{ MOVE_STOP, MOVE_STRAIGHT, MOVE_ROTATE, MOVE_COORDINATE, MOVE_FOLLOW_WALL, MOVE_LOST};

	struct SensorData {
		Odometry odometry;
		IRDistances irdistances;
		float sonarDistance;
	};

	class MovementState{
	public:
		//Constructors & destructor
	    MovementState(){};
		virtual ~MovementState(){};


		virtual void init(const SensorData& data) = 0;
		virtual bool isRunning() const = 0;
		virtual void doControl(const SensorData& data) = 0;
		
	}; //MovementStates	

}; //namespace amee

#endif
